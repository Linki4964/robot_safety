# robot_safety — 机器人安全状态机监控

面向 TurtleBot3 / Gazebo 的机器人安全监控与（后续）安全状态机。

**当前进度：第一步完成 —— 运行时整体状态监控模块。**

```
src/
├── robot_safety_msgs/         # 自定义接口：状态快照与子结构
│   └── msg/{RobotState,TopicHealth,Odometry,BatteryState,Imu,
│            LaserScan,VelocityCommand,JointState}.msg
└── robot_safety_monitor/      # 监控节点
    ├── robot_safety_monitor/
    │   ├── analyzer.py        # 纯 Python 判定核心（零 ROS 依赖，可单测）
    │   ├── monitor_node.py    # 采集器 + 状态聚合发布节点
    │   └── state_reporter.py  # 命令行状态查看器（独立进程）
    ├── config/monitor_params.yaml
    ├── launch/{monitor,monitor_with_sim}.launch.py
    └── test/test_analyzer.py  # 78 项离线单元测试
```

---

## 1. 这个模块做什么

它以固定频率（默认 10 Hz）发布一条 `RobotState` 快照，回答两个问题：

1. **机器人现在在做什么？** —— 位姿、速度、朝向/倾角、最近障碍距离、关节转动、最近速度指令。
2. **支撑上面这个答案的数据可信吗？** —— 每个话题是否按时到达、里程计协方差是否可信、指令与执行是否一致。

产物话题：`/robot_safety/monitor/state`（`robot_safety_msgs/msg/RobotState`）。

**它不做决策。** 快照里只有状态、告警与 `motion_expected`；"是否必须停机"属于第二步的安全状态机。这样监控本身不会成为安全链路的单点。

---

## 2. 两条时间语义（本模块最关键的设计）

| 时间 | 来源 | 用途 |
|---|---|---|
| `stamp` / `*_stamp_sec` | 消息头时间。Gazebo 下是 `/clock` 仿真时间 | 记录数据"出生"时刻 |
| `wall_time_sec` / `*_wall_time_sec` | 监控进程墙钟 | **看门狗、超时、陈旧判定** |

**陈旧判定一律用墙钟。** 如果拿消息时间戳判超时，仿真暂停（话题彻底不再发布）或回放 rosbag（时间戳一直很旧）都会被误判为"健康"。`TopicHealth` 因此同时给出两种时间，并用 `age_sec`（墙钟）做判定。

同理，**tick 定时器跑墙钟而非 ROS 时间**：若用仿真时间定时器，仿真一暂停监控就停止发布——而那恰恰是最需要它说话的时刻。

---

## 3. 判定分层

`analyzer.py` 里的判定分四层，任一层出问题都会体现在最终 `status`（取**最坏**值）与 `warnings` 里：

| 层 | 检查 | 告警码 |
|---|---|---|
| 1 数据链路 | 每个源是否按时到达 | `ODOM_STALE` / `ODOM_MISSING` / `SCAN_*` / `IMU_*` / `JOINTS_STALE` / `COMMAND_STALE` / `BATTERY_*` |
| 2 物理合理性 | 里程计协方差、机体倾角 | `POSE_UNCERTAIN` / `TWIST_UNCERTAIN` / `TILT_HIGH` / `TILT_CRITICAL` |
| 3 场景安全量 | 激光最近障碍、电量 | `OBSTACLE_NEAR` / `OBSTACLE_CRITICAL` / `BATTERY_LOW` / `BATTERY_CRITICAL` |
| 4 意图 vs 执行 | 指令与实际运动是否一致 | `COMMAND_MISMATCH` / `UNEXPECTED_MOTION` |

状态码：`0 UNKNOWN · 1 OK · 2 STALE · 3 ERROR · 4 CRITICAL`。该数值同时用于消息字段与告警严重度，便于状态机直接比较。

### 三个容易踩的坑（已在实现中处理）

- **协方差哨兵值**：差速底盘里程计对不可观测自由度（z / roll / pitch）填 `1e12`，含义是"该自由度不可观测"，**不是**"极不确定"。若当成真实方差会把每一次正常行驶都误报为 `TWIST_UNCERTAIN`。现只判定可观测的 x / y / yaw。
- **激光无回波**：Gazebo 射线传感器对超出量程的方向返回 `inf`。满视野全是 `inf` 是"前方空阔"的正常信号，不是故障。快照用 `no_return` 显式表达，避免与"传感器坏了"混淆。
- **`consistent` 的三态性**："一致 / 不一致 / 无法判定"被压进一个布尔量会撒谎。因此 `VelocityCommand` 同时给出 `valid`（是否具备判定条件）与 `consistent`。没有里程计时报 `valid=false`，而不是宣称"不一致"。

---

## 4. 构建

```bash
cd ~/robot-safety
source /opt/ros/humble/setup.bash

# 若尚未安装构建工具（本机确实缺失）
sudo apt install -y python3-colcon-common-extensions python3-empy

colcon build --symlink-install
source install/setup.bash
```

---

## 5. 运行

终端 A —— 仿真：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端 B —— 监控 + 状态查看：

```bash
source ~/robot-safety/install/setup.bash
ros2 launch robot_safety_monitor monitor.launch.py autostart_reporter:=true

# 或分开：只跑监控
ros2 run robot_safety_monitor monitor --ros-args --params-file \
  src/robot_safety_monitor/config/monitor_params.yaml

# 另开一个终端看状态
ros2 run robot_safety_monitor state_reporter            # 持续摘要，状态变化时打印详情
ros2 run robot_safety_monitor state_reporter --once -v  # 单次全量快照
```

一条命令同时拉起仿真与监控：

```bash
ros2 launch robot_safety_monitor monitor_with_sim.launch.py
```

原始话题直接观察：

```bash
ros2 topic echo /robot_safety/monitor/state --once
ros2 topic hz   /robot_safety/monitor/state
```

### 让机器人动起来（验证指令/执行为路径）

```bash
ros2 run turtlebot3_teleop teleop_keyboard
# 或直接发一条指令
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

---

## 6. 配置

全部阈值都在 `config/monitor_params.yaml`，可在启动时覆盖，也可运行时用 `ros2 param set` 调整：

```bash
ros2 param set /robot_safety_monitor timeout.odom_sec 0.5
ros2 param set /robot_safety_monitor obstacle.warn_range_m 0.25
```

要点：

- `topic.*`：把某个话题设为 `""` 表示该平台没有此源。
- `battery.monitor`：默认 `false`（Gazebo TurtleBot3 不发布电量）。开启但没有发布者会**一直**报 `BATTERY_STALE`。
- `command.monitor`：默认 `true`。曾经在发布、随后中断的指令方属于真实故障，必须报 `COMMAND_STALE`。只有在"平台确实没有速度指令方"时才设为 `false`，此时该源从健康评估中完全移除。
- `timeout.*`：墙钟秒数。仿真中 `odom/scan/imu` 设为 1.0 s 已是宽松值（实际分别约 30 Hz / 5 Hz / 100 Hz）。

---

## 7. 测试

**离线单元测试**（不需要 ROS 图，78 项）：

```bash
cd src/robot_safety_monitor
PYTHONPATH=$PWD python3 -m pytest test -v
```

覆盖四元数/倾角几何、协方差解析（含 numpy 真值陷阱与哨兵值）、陈旧判定、速率窗口有界性、各阈值边界、指令一致性三态，以及"最坏状态取胜"等聚合语义。

---

## 8. 已实测验证的行为

在 ROS 2 Humble + Gazebo 11 + `turtlebot3_world` 上：

- 静止：`status=OK`、无告警；位姿 `(-2.000, -0.500)` 与生成点一致；`cov=(1e-05, 1e-05, 0.001)`。
- 障碍：324/360 有效回波，最近障碍 0.503 m、前方 1.938 m（spawn 点附近）。
- 运动：下发 `linear.x=0.12` 后实测 `0.12 m/s`，`motion_expected=true`、`consistent=true`，无误报。
- 仿真全部崩溃（gzserver 退出）时：`status=CRITICAL`，`ODOM_MISSING` / `SCAN_MISSING` / `IMU_MISSING`，正确降级而非假装正常。

---

## 9. 下一步（第二步：安全状态机）

状态机的输入字母表已经定型，就是 `/robot_safety/monitor/state`：

- `status` / `source_status[]`：链路健康，决定"能否信任其他字段"。
- `warnings[]` + `warning_severity[]`：事件源，`warning_detail[]` 供人阅读。
- `motion_expected`：区分"该动却没动"与"没让动却动了"。
- `odom` / `scan` / `imu` / `joints` / `command`：连续量，用于趋势与阈值。

建议的后续增量：

1. 状态机的状态集（如 `INIT / NORMAL / DEGRADED / SAFE_STOP / FAULT`）与迁移条件直接引用上表的告警码。
2. 增加 `/cmd_vel` 仲裁与安全停机输出（`safety_gate`），把 `armed` 从只读变为真正的门控。
3. 引入时序/持续性判据（例如"倾角持续 0.5 s"），避免单帧抖动触发停机——`TopicObservation` 的速率窗口已为此提供了模式参考。
4. 接入 rosbag 回放做回归测试。

---

## 10. 已知边界

- 本模块**只观测**，不拦截控制指令；`armed` 目前仅是 "运动被预期且状态 ≤ OK" 的只读指示。
- `front_range` 的扇区半角由 `front_sector_half_angle_rad` 配置（默认 0.35 rad ≈ 20°）。
- 倾角判据依赖 IMU 姿态；若 IMU 不提供 orientation，`tilt` 判据返回 `UNKNOWN` 而**不是** `OK`（缺数据不等于安全）。
- 仿真中激光只有 5 Hz，1 s 超时很宽松；真实硬件上建议按传感器实际频率收紧。
