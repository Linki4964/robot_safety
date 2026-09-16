# 核心异常判据（Core Anomaly Detection Rules）

| 项目 | 内容 |
|---|---|
| 文档编号 | RSS-003 |
| 版本 | v1.0 |
| 范围 | **16 条核心判据** —— 你选定的关键项（含补回的 `MOT-002`） |
| 格式 | 按指定模板：`id / name / category / precondition / input / model / condition / severity / response / diagnostics / validation` |
| 上位文档 | `docs/anomaly_detection_rules.md`（RSS-002，全量 76 条判据） |
| 说明 | 已失效的英文标识符保留英文（便于落代码），必要处配中文说明 |

> **本轮精简说明**：
> - 已补回 `MOT-002`（角速度上限）——与 `MOT-001` 同源，差速底盘最易超限。
> - `CMD-001` 原含"线/角速度超额定"，该内容已由 `MOT-001`/`MOT-002` 承担，故 `CMD-001` 撤销。
> - 其余未选中的判据未删除，仍在 RSS-002 中，按 §4「暂缓项」登记，后续可渐进补入。

---

## 0. 通用参数（Common Parameters）

所有规则共用的物理量与时延。**这些值必须先标定，否则 `COL-005`/`COL-006` 无法正确工作。**

```yaml
common_params:

  # ---- 平台几何与运动学（待实测）----
  robot.radius_m: 0.105          # 外接圆半径
  wheel.separation_m: 0.160      # 轮距 W
  wheel.radius_m: 0.033          # 轮半径
  wheel.max_linear_mps: 0.22     # 单轮最大线速度 = ω_wheel_max · r_wheel

  # ---- 速度上限 ----
  velocity.max_linear_mps: 0.22
  velocity.max_angular_rps: 2.84
  velocity.max_linear_accel: 0.50      # m/s^2
  velocity.max_angular_accel: 3.00     # rad/s^2

  # ---- 停止距离模型（COL-005 / COL-006 的核心，必须实测）----
  stopping.T_response_sec: 0.15        # = T_detect + T_ctrl，待实测
  stopping.A_brake_mps2: 0.50          # 制动减速度，待实测（取最差条件）
  stopping.D_margin_m: 0.05            # 固定安全余量

  # ---- 各源超时（SEN-001）----
  timeout.odom_sec: 0.10
  timeout.scan_sec: 0.60
  timeout.imu_sec: 0.05
  timeout.joint_states_sec: 0.20
  timeout.cmd_vel_sec: 0.50
  timeout.localization_sec: 0.30

  # ---- 看门狗周期 ----
  watchdog.state_machine_ms: 300
  watchdog.safety_gate_ms: 200

  # ---- 指令与仲裁 ----
  arb.window_ms: 200
  command.deadband_linear: 0.02
  command.deadband_angular: 0.10

  # ---- 激光可用性 ----
  scan.min_valid_ratio: 0.30
```

**阈值来源标记**：`manufacturer` 制造商额定 / `measured` 待实测 / `standard` 标准推导。

---

## 1. 运动安全 MOT

### MOT-001 速度上限（线速度）

```yaml
id: MOT-001
name: Speed Limit Exceeded (Linear)
category: MOTION

precondition:
  - velocity_valid == true              # 有可判定的速度值（指令或里程计）

input:
  - cmd_linear_velocity                 # /cmd_vel.linear.x
  - odom_linear_velocity                # /odom.twist.linear 合成速度

model:
  limits:
    v_max_cmd: 0.22                     # m/s，制造商额定
    v_max_actual: 0.30                  # m/s，超出此值判定为空转/失控（S4）
  measured:
    v_cmd: abs(cmd_linear_velocity)
    v_actual: abs(odom_linear_velocity)

condition:
  cmd_exceeded:    v_cmd > v_max_cmd            # 指令超限 → S2
  actual_exceeded: v_actual > v_max_actual      # 实测超限 → S4

severity: S2          # 指令超限；实测超限升为 S4

response:
  action: REJECT_COMMAND_AND_DEGRADE   # 拒绝指令 + 限速，而非静默截断
  latch: false

diagnostics:
  reason: MOT_LIN_VEL_EXCEED           # 指令超限
  escalate: MOT_ACTUAL_VEL_EXCEED      # 实测超限

validation:
  method: boundary_injection_test      # 注入 v=0.23，断言被拒
  measured: true
```

**中文说明**
- **意义**：最基础的边界。区别对待两类超限——**指令超限**说明上层发了不该发的值（S2，拒绝即可）；**实测超限**说明底盘实际跑超了（S4，可能编码器故障或失控）。
- **为什么拒绝而非截断**：静默截断会让上层以为指令已生效，产生"控制器认为在走、实际没走"的隐性不一致，反而掩盖故障。
- **等级差异**：`v_max_actual`（0.30）比 `v_max_cmd`（0.22）宽，因为里程计噪声与控制超调会让实测略高于指令，避免误报。

---

### MOT-002 速度上限（角速度）

```yaml
id: MOT-002
name: Speed Limit Exceeded (Angular)
category: MOTION

precondition:
  - velocity_valid == true

input:
  - cmd_angular_velocity                # /cmd_vel.angular.z
  - odom_angular_velocity               # /odom.twist.angular.z

model:
  limits:
    w_max_cmd: 2.84                     # rad/s，制造商额定
    w_max_actual: 3.50                  # rad/s
  measured:
    w_cmd: abs(cmd_angular_velocity)
    w_actual: abs(odom_angular_velocity)

condition:
  cmd_exceeded:    w_cmd > w_max_cmd
  actual_exceeded: w_actual > w_max_actual

severity: S2

response:
  action: REJECT_COMMAND_AND_DEGRADE
  latch: false

diagnostics:
  reason: MOT_ANG_VEL_EXCEED
  escalate: MOT_ACTUAL_ANG_VEL_EXCEED

validation:
  method: boundary_injection_test      # 注入 ω=3.0，断言被拒
  measured: true
```

**中文说明**
- **意义**：角速度是差速底盘最容易超限的量，且直接对应"原地高速旋转扫倒/夹伤"。**扫掠面积随转速增大**，同样的转速在机器人边缘产生的线速度是 `ω · R_robot`，可能远超直行速度。
- **与 MOT-004 的关系**：本规则管单值上限，`MOT-004` 管线/角耦合后的可达性，两者互补。

---

### MOT-003 加速度上限

```yaml
id: MOT-003
name: Acceleration Limit Exceeded
category: MOTION

precondition:
  - command_available == true
  - previous_command_available == true
  - dt >= 0.02                          # 间隔下限，防除零与单帧尖峰

input:
  - cmd_linear_velocity                 # 当前帧
  - cmd_linear_velocity_prev            # 上一帧
  - cmd_angular_velocity
  - cmd_angular_velocity_prev
  - dt                                  # 两帧实际间隔 [s]

model:
  limits:
    a_max: 0.50                         # m/s^2
    alpha_max: 3.00                     # rad/s^2
  measured:
    a_cmd:     abs(v_now - v_prev) / dt
    alpha_cmd: abs(w_now - w_prev) / dt

condition:
  linear_exceeded:  a_cmd > a_max
  angular_exceeded: alpha_cmd > alpha_max

severity: S2

response:
  action: REJECT_COMMAND_AND_DEGRADE
  latch: false

diagnostics:
  reason: MOT_LIN_ACCEL_EXCEED
  reason_alt: MOT_ANG_ACCEL_EXCEED

validation:
  method: step_injection_test          # 构造 0→0.22 单帧阶跃，断言被拒
  measured: true
```

**中文说明**
- **意义**：限制**突变指令**。一帧内的速度阶跃会让底盘猛冲/急转，造成机械冲击、轮子打滑（打滑后里程计失真，`COL-005` 的停止距离模型随之失效）、货物倾覆。
- **`dt >= 0.02` 的必要性**：若两帧间隔极短（如重复帧、时间戳异常），`Δv/dt` 会算出极大的假加速度。设置间隔下限可抑制这类误报。
- **注意**：本判据用**有限差分**近似，对噪声敏感。若指令噪声大，建议先对指令做滑动平均再差分，否则会频繁误报。

---

### MOT-004 差速运动学约束

```yaml
id: MOT-004
name: Differential Drive Kinematic Violation
category: MOTION

precondition:
  - command_available == true

input:
  - cmd_linear_velocity                 # v
  - cmd_angular_velocity                # ω

model:
  geometry:
    W: 0.160                            # 轮距 [m]
    r_wheel: 0.033                      # 轮半径 [m]
  limits:
    v_wheel_max: 0.22                   # 单轮最大线速度 [m/s]
  derived:
    # 差速底盘左右轮线速度
    v_left:  v - (W / 2) * w
    v_right: v + (W / 2) * w
    v_wheel_required: max(abs(v_left), abs(v_right))

condition:
  v_wheel_required > v_wheel_max        # 组合指令超出单轮能力

severity: S2

response:
  action: REJECT_COMMAND_AND_DEGRADE
  latch: false

diagnostics:
  reason: MOT_TWIST_INFEASIBLE

validation:
  method: infeasible_twist_test         # 注入 v=0.22, ω=2.84（组合不可达），断言被拒
  measured: true
```

**中文说明**
- **意义**：线速度和角速度**各自都不超限**，但组合起来可能超出底盘能力。例如 `v=0.22, ω=2.84` 时，外侧轮需要 `0.22 + 0.08×2.84 ≈ 0.45 m/s`，远超单轮 0.22 m/s 的能力。
- **为什么必须单独判**：控制器会因积分饱和而持续输出该指令，底盘却无法执行——**表现为"指令已下、底盘未动"**，会被 `CMD-011` 误判为机械故障。提前拒绝可避免这个歧义。
- **`W` 与 `r_wheel` 必须实测**：这两个值直接决定判据是否准确。`W` 可用卷尺量轮心距，`r_wheel` 可用"推车一圈看位移"标定。

---

## 2. 碰撞与距离 COL

### COL-005 停止距离不足（核心判据）

```yaml
id: COL-005
name: Insufficient Stopping Distance
category: COLLISION

precondition:
  - robot_motion_expected
  - lidar_health == OK
  - velocity_valid == true

input:
  - obstacle_distance                   # scan.front_range
  - linear_velocity                     # max(|odom.speed|, |cmd.v|)

model:
  required_distance:
    reaction: v * T_response
    braking:  v * v / (2 * A_brake)
    margin:   D_margin
  params:
    T_response: 0.15                    # s，= T_detect + T_ctrl（待实测）
    A_brake:    0.50                    # m/s^2（待实测）
    D_margin:   0.05                    # m

condition:
  obstacle_distance < required_distance

severity: S3

response:
  action: PROTECTIVE_STOP
  latch: false

diagnostics:
  reason: INSUFFICIENT_STOPPING_DISTANCE

validation:
  method: measured_braking_test
  measured: true
```

**中文说明**
- **意义**：把"能不能在碰到之前停下来"变成可核算的判据。这是整个碰撞安全的核心。
- **公式**：`required_distance = v·T_response + v²/(2·A_brake) + D_margin`
  - `v·T_response`：反应期间已经走过的距离（**时延×速度**，不可忽略）
  - `v²/(2·A_brake)`：制动距离，随速度平方增长
  - `D_margin`：固定安全余量
- **当前参数下的实际值**（`v = 0.22 m/s`）：

| 分量 | 值 |
|---|---|
| `0.22 × 0.15` | 0.033 m |
| `0.22² / (2×0.50)` | 0.048 m |
| `D_margin` | 0.050 m |
| **`required_distance`** | **≈ 0.131 m** |

- **⚠ 仿真下的严重问题**：Gazebo 激光仅 **5 Hz**，`T_detect ≈ 0.2 s`，则 `T_response ≈ 0.30 s`，`required_distance ≈ 0.20 m`。若沿用现有 `critical_range_m = 0.15 m`，**判据成立时已经撞上**。必须先按实际频率标定 `T_response`。
- **`latch: false` 的语义**：条件消失即自动恢复。这是有意的——保护停应能自动解除，避免每次接近障碍都需人工复位。
- **`velocity_valid` 前置条件的作用**：取 `max(|odom.speed|, |cmd.v|)` 偏保守。若两者都拿不到，则本判据**无法判定**，此时不应报"距离充足"，而应由 `SEN-001`（超时）接管。

---

### COL-006 动态限速

```yaml
id: COL-006
name: Dynamic Speed Limiting
category: COLLISION

precondition:
  - robot_motion_expected
  - lidar_health == OK
  - col_005_model_valid == true         # 停止距离模型已标定且可用

input:
  - obstacle_distance
  - linear_velocity

model:
  required_distance:                    # 同 COL-005
    reaction: v * T_response
    braking:  v * v / (2 * A_brake)
    margin:   D_margin
  warning_band:
    lower_factor: 1.2                   # × required_distance
    upper_factor: 2.0
  # 反解：给定可用距离，允许的最大速度
  allowed_velocity:
    formula: solve_v(required_distance(v) == obstacle_distance - D_margin)
    clamp: [0, velocity.max_linear_mps]

condition:
  (1.2 * required_distance) < obstacle_distance < (2.0 * required_distance)

severity: S2

response:
  action: LIMIT_VELOCITY                # 压缩速度至 allowed_velocity，非停车
  latch: false
  report_back: true                     # 必须把限速值回传上层

diagnostics:
  reason: SPEED_LIMITED_BY_RANGE

validation:
  method: gradual_approach_test         # 渐进接近障碍，断言速度单调下降
  measured: true
```

**中文说明**
- **意义**：在"距离充足"和"必须停车"之间加一个**过渡带**。直接二值停车会让机器人在阈值附近反复启停，而渐进减速既安全又平顺。
- **与 `COL-005` 的分工**：`COL-005` 是"来不及了，停"；`COL-006` 是"还来得及，但得慢下来"。两者用同一个距离模型，只是系数不同（1.2 / 2.0）。
- **`report_back: true` 是关键**：如果限速只发生在门控内部而不告诉上层，上层会以为指令已生效并继续积分，最终退化成 `CMD-011`（指令—执行不一致）的误报。**限速值必须回传**。
- **恢复时的加速度限制**：距离恢复后不应瞬间恢复到额定速度（会造成突加速），应按 `MOT-003` 的加速度上限逐步恢复。

---

### COL-010 传感器盲区

```yaml
id: COL-010
name: Sensor Blind Direction
category: COLLISION

precondition:
  - lidar_health == OK
  - motion_expected == true

input:
  - cmd_linear_velocity
  - cmd_angular_velocity
  - scan.angle_min
  - scan.angle_max
  - scan.valid_point_count
  - scan.point_count

model:
  coverage:
    # 指令的速度方向（机体坐标系）
    motion_direction: atan2(v, w) 或由 v/ω 合成的运动方向
    required_sector: 运动方向 ± 扇区半角
  validity:
    # 该方向上的有效点比例
    valid_ratio_in_sector: valid_points_in_sector / points_in_sector
  threshold:
    min_valid_ratio: 0.50

condition:
  (required_sector 未被 scan.angle_min..angle_max 覆盖)
  OR (valid_ratio_in_sector < min_valid_ratio)

severity: S2

response:
  action: BLOCK_DIRECTION_AND_DEGRADE   # 禁止朝该方向运动
  latch: false

diagnostics:
  reason: SENSOR_BLIND_DIRECTION

validation:
  method: restricted_fov_test           # 模拟后向运动（激光仅前向180°），断言被拒
  measured: false
```

**中文说明**
- **意义**：**盲区运动 = 不可检测的碰撞**。所有距离判据都隐含一个前提——"这个方向上我确实能看见"。如果指令要求朝传感器看不到的方向移动，`COL-005` 的 `obstacle_distance` 就是无意义的（可能读到 `inf`，被判为"前方空阔"）。
- **典型场景**：激光只有 270° 视野而指令要求侧移/后退；或某一扇区的点全部因反光丢失。
- **与 `COL-004`（无回波语义）的区别**：`no_return` 是**全视野**无回波（可能是正常空阔）；本规则是**特定方向**无有效覆盖，即使其他方向有回波也要拦截。
- **`atan2(v, w)` 的注意**：纯原地旋转（`v=0`）时运动方向不明确，此时应改用 `MOT-011`（旋转空间许可）判定，而不是本规则。

---

### COL-011 有效回波

```yaml
id: COL-011
name: Insufficient Valid Returns
category: COLLISION

precondition:
  - lidar_health == OK

input:
  - scan.valid_point_count
  - scan.point_count

model:
  measured:
    valid_ratio: valid_point_count / point_count
  threshold:
    min_valid_ratio: 0.30               # 低于此值，最近距离不可信

condition:
  valid_ratio < min_valid_ratio

severity: S2

response:
  action: DEGRADE_AND_LIMIT_VELOCITY
  latch: false

diagnostics:
  reason: LOW_VALID_RETURNS

validation:
  method: synthetic_scan_test           # 注入 80% 为 inf 的扫描，断言降级
  measured: false
```

**中文说明**
- **意义**：`obstacle_distance` 取的是"**有效**回波的最小值"。如果大部分方向无有效回波（玻璃、镜面、吸光表面、超量程），这个"最小值"就不能代表真实环境——**看起来空阔，其实是无数据**。
- **与 `COL-004` 的区别**：`no_return`（全部无回波）在源健康时是"前方空阔"的正常信号；但**部分**无回波且比例很低时，说明传感器覆盖不可信。两者必须分开处理，否则会把"传感器坏了"当成"前方空阔"。
- **联动**：本判据触发时应**同时抑制** `COL-005`/`COL-006` 的判定（因为 `obstacle_distance` 已不可信），而非仅降速。这点在实现时容易漏。

---

## 3. 传感器安全 SEN

### SEN-001 数据超时

```yaml
id: SEN-001
name: Data Timeout
category: SENSOR

precondition:
  - source_observed == true             # 至少收到过一次样本
  # 注意：从未收到过样本（observed == false）按 UNKNOWN 处理，其严重度等同 CRITICAL

input:
  - last_wall_time                      # 该源最后一次样本的墙钟时间
  - now_wall                            # 当前墙钟时间
  - timeout_sec                         # 该源配置的超时

model:
  age: now_wall - last_wall_time
  timeouts:
    odom:         0.10                  # s  （3 × 周期，30 Hz）
    scan:         0.60                  # s  （3 × 周期，5 Hz）
    imu:          0.05                  # s  （3 × 周期，100 Hz）
    joint_states: 0.20                  # s  （3 × 周期，30 Hz）
    cmd_vel:      0.50                  # s
    localization: 0.30                  # s  （3 × 周期，10 Hz）
  generic_rule: timeout_sec = max(3 * period, 0.05)

condition:
  age > timeout_sec

severity: S3

response:
  action: PROTECTIVE_STOP
  latch: false

diagnostics:
  reason: <SOURCE>_STALE                # 如 ODOM_STALE / SCAN_STALE / IMU_STALE
  reason_never_seen: <SOURCE>_MISSING

validation:
  method: source_silence_test           # 逐源停掉发布，断言在超时后触发
  measured: true
```

**中文说明**
- **意义**：数据"来没来"。这是最基础的健康判据，现有实现已具备（`*_STALE` / `*_MISSING`）。
- **必须用墙钟（wall clock），不能用消息时间戳**：仿真暂停时话题彻底不发布（时间戳也停），rosbag 回放时时间戳一直很旧。若用消息时间戳判超时，两种情况都会被误判为"健康"。现有实现已正确处理这一点，务必保留。
- **`timeout = 3 × 采样周期` 的依据**：允许丢 2 帧仍算健康，第 3 帧丢失才判定超时。这是"容忍偶发抖动、但不容忍持续中断"的平衡。
- **⚠ 现状与建议值不符**：现有配置 `scan = 1.0 s`（激光 5 Hz，允许丢 5 帧）、`imu = 1.0 s`（IMU 100 Hz，允许丢 100 帧）、`joint_states = 2.0 s`。**这些值过宽**，应按下表收紧。收紧前请确认调度抖动不会造成误报（见 §5 待确认 3）。

| 源 | 建议值 | 现值 | 变更 |
|---|---|---|---|
| odom | 0.10 s | 1.0 s | ✅ 收紧 |
| scan | **0.60 s** | 1.0 s | ✅ 收紧 |
| imu | **0.05 s** | 1.0 s | ✅ 收紧 |
| joint_states | 0.20 s | 2.0 s | ✅ 收紧 |
| cmd_vel | 0.50 s | 1.0 s | ✅ 收紧 |
| localization | 0.30 s | 缺失 | ➕ 新增 |

- **`UNKNOWN` 不等于 `OK`**：从未收到样本时必须按最坏情况处理。缺数据不等于安全——这是安全设计中反复出现的陷阱。

---

### SEN-011 数据非法

```yaml
id: SEN-011
name: Invalid Data Value
category: SENSOR

precondition:
  - source_observed == true

input:
  - 所有安全相关字段                      # 见下方 range 表

model:
  invalid_if:
    - is_nan(value)
    - is_inf(value) 且 非激光预期量程值     # Gazebo 超量程返回 inf 属正常
    - value 超出物理量程
  ranges:
    cmd_linear_velocity:   [-1.0, 1.0]    # m/s
    cmd_angular_velocity:  [-10.0, 10.0]  # rad/s
    odom_speed:            [0, 1.0]       # m/s
    battery_percentage:    [0, 1]
    battery_voltage:       [0, 60]        # V

condition:
  任一安全相关字段为 NaN / inf / 越量程

severity: S3

response:
  action: PROTECTIVE_STOP
  sub_action: 丢弃该帧并按帧连续计数
  escalate: 连续 3 帧非法 → SAFE_STOP
  latch: false

diagnostics:
  reason: INVALID_DATA_VALUE

validation:
  method: nan_injection_test            # 注入 NaN，断言被检出且不进入 OK
  measured: false
```

**中文说明**
- **意义**：这是**最容易被低估**的判据，因为它在正常情况下几乎不触发，但一旦触发后果严重。
- **为什么危险**：`NaN > x` 恒为 `false`。一条含 `NaN` 的速度指令会**静默通过所有上限检查**（因为所有比较都返回 false），然后进入控制器。同样，`inf` 会污染 `min()` 计算——这正是 `COL-005` 里 `obstacle_distance` 的风险来源。**安全检查"通过"了，但通过的原因是数据非法而非数据安全。**
- **激光的 `inf` 必须豁免**：Gazebo 射线传感器对超出量程的方向返回 `inf`，这是**正常信号**（表示该方向空阔），不能当作非法数据。必须区分"预期的 `inf`"与"意外的 `inf`"。
- **三帧升级的依据**：单帧非法可能来自通信位翻转（偶发），连续三帧则表明数据源持续错误。分级可避免单帧抖动导致停机。

---

## 4. 指令与控制 CMD

### CMD-005 控制权冲突

```yaml
id: CMD-005
name: Command Authority Conflict
category: COMMAND

precondition:
  - command_available == true

input:
  - cmd_source_id                       # 指令源标识（节点名 / 话题 / 消息内标识）
  - cmd_linear_velocity
  - cmd_angular_velocity

model:
  arbitration_window:
    T_arb: 0.20                          # s
  nonzero_criterion:                     # 与 MOT-001 的 deadband 一致
    abs(v) > command.deadband_linear: 0.02
    abs(w) > command.deadband_angular: 0.10
  distinct_sources: 在 T_arb 窗口内发出非零指令的独立源数量

condition:
  distinct_sources > 1

severity: S4

response:
  action: SAFE_STOP
  latch: true
  reset: manual                         # 需人工确认恢复单源

diagnostics:
  reason: COMMAND_AUTHORITY_CONFLICT

validation:
  method: dual_source_test              # 同时启动 teleop 与导航，断言冲突检出
  measured: false
```

**中文说明**
- **意义**：多个源同时发指令时，**控制权不明**。两个控制器（如遥操作 + 导航）可能下发相反的指令，底盘在二者之间反复争夺，行为完全不可预测。
- **为什么是 S4（需人工复位）而非 S3**：这不是传感器噪声，而是**控制权归属问题**。在有人介入前，无法判断该听谁的；自动恢复会导致两个源继续冲突。
- **实现要点**：源标识必须**可信且不可伪造**。仅靠话题名不够（任何节点都能发 `/cmd_vel`），建议在消息内或通过独立的指令通道携带源标识。
- **与 `CMD-007` 的区别**：本规则管"**多个源**都在发"，`CMD-007` 管"**唯一源**突然不发了"。

---

### CMD-007 指令看门狗

```yaml
id: CMD-007
name: Command Watchdog Timeout
category: COMMAND

precondition:
  - robot_motion_expected == true       # 只在"应该动"时判定，静止时无需心跳

input:
  - last_command_wall_time
  - now_wall

model:
  silence: now_wall - last_command_wall_time
  timeout:
    T_cmd_wd: 0.50                      # s

condition:
  silence > T_cmd_wd

severity: S3

response:
  action: PROTECTIVE_STOP
  sub_action: 保持最后有效指令 <= 200 ms，随后归零
  latch: false

diagnostics:
  reason: COMMAND_STALE

validation:
  method: command_source_silence_test   # 断开指令源，测量实际停车延迟
  measured: true
```

**中文说明**
- **意义**：指令通道丢失后，若机器人继续按最后一条指令运动，等于**开环盲走**——它不知道前方是否有人、是否该停，而且没有任何新指令能让它停下来（因为通道已断）。
- **`precondition` 为何重要**：如果机器人本就不该动（无指令），那"没有指令"是正常状态，不能报超时。加这个前置条件可避免静止时的持续误报。
- **`保持 ≤ 200 ms` 的作用**：突然归零会造成急停冲击。短暂保持最后指令（200 ms）可让底盘平稳停住，但**必须远短于** `T_cmd_wd`（500 ms），否则又变成盲走。这个窗口需要与 `T_response` 对齐标定。
- **⚠ 现状**：现有 `timeout.cmd_vel_sec = 1.0 s` 偏宽——允许盲走 1 秒，按 0.22 m/s 计就是 0.22 m。建议收紧至 0.50 s。

---

## 5. 定位安全 LOC

### LOC-001 定位跳变

```yaml
id: LOC-001
name: Localization Jump
category: LOCALIZATION

precondition:
  - localization_available == true
  - previous_pose_available == true

input:
  - pose_x
  - pose_y
  - pose_yaw
  - pose_previous
  - dt

model:
  measured:
    dp: sqrt((x - x_prev)^2 + (y - y_prev)^2)
    implied_velocity: dp / dt
    dyaw: abs(yaw - yaw_prev)
  limits:
    max_jump: 0.50                      # m，单帧
    max_implied_velocity: 2.0           # m/s，远高于 v_max，物理上不可能
    max_yaw_jump: 30                    # deg，单帧（物理上界约 16° @10 Hz）

condition:
  (dp > max_jump AND implied_velocity > max_implied_velocity)
  OR (dyaw > max_yaw_jump)

severity: S4

response:
  action: SAFE_STOP
  latch: true
  reset: manual

diagnostics:
  reason: LOCALIZATION_JUMP

validation:
  method: pose_jump_injection_test       # 注入单帧 1.0 m 跳变，断言触发
  measured: false
```

**中文说明**
- **意义**：位姿跳变会让**所有基于地图的判断失效**——路径跟踪、避障、区域限速全部指向错误位置。机器人可能"以为自己在 A 点"而实际在 B 点。
- **为什么用"物理不可能"作为判据**：`max_implied_velocity = 2.0 m/s` 远超 `v_max = 0.22 m/s`。机器人**物理上不可能**在单帧内移动 0.5 m。因此这个判据的误报率极低——它抓的是"物理上不可能的事件"，而非"看起来有点大的值"。
- **偏航跳变同理**：`ω_max = 2.84 rad/s`，10 Hz 下单帧偏航物理上界约 16°，所以 30° 也是物理不可能。
- **双条件（`dp > max_jump AND implied > max_velocity`）的作用**：避免低速时因单帧噪声误报。同时满足两个条件才判定。
- **`latch: true` 的依据**：跳变通常意味着定位器重定位或粒子发散，**不会自己恢复**；且在人工确认前无法判断机器人真实位置。
- **诱因**：AMCL 重定位、粒子收敛、特征稀疏区、传感器错位。

---

### LOC-005 定位超时

```yaml
id: LOC-005
name: Localization Timeout
category: LOCALIZATION

precondition:
  - localization_observed == true

input:
  - last_localization_wall_time
  - now_wall

model:
  silence: now_wall - last_localization_wall_time
  timeout:
    T_loc: 0.30                         # s = 3 × 周期（AMCL 10 Hz）

condition:
  silence > T_loc

severity: S3

response:
  action: PROTECTIVE_STOP
  latch: false

diagnostics:
  reason: LOCALIZATION_TIMEOUT

validation:
  method: localization_silence_test      # 停掉定位节点，断言保护停
  measured: true
```

**中文说明**
- **意义**：无定位则无法保证路径与避障正确。对 AMR，定位是**安全相关输入**而非仅任务相关。
- **⚠ 与 `SEN-001` 的可能重叠**：如果定位话题已在 `SEN-001` 的监控源清单里，本规则就是重复判定。**建议二选一**：要么把定位源纳入 `SEN-001` 统一管理，要么保留本规则（语义更明确，且可单独配置更严的超时）。见 §5 待确认 1。
- **`3 × 周期` 的依据**：同 `SEN-001`。AMCL 10 Hz → 0.30 s。**不要沿用 1.0 s 这类宽松值**，否则机器人已盲走 0.3 m 才报超时。
- **注意与 `LOC-001` 的区别**：`LOC-005` 是定位**不再更新**（时间维度），`LOC-001` 是定位**突然跳到别处**（数值维度），`LOC-007`（未选中）是定位**持续发布但数值冻结**——三者是不同失效模式，不可互相替代。

---

## 6. 系统完整性 SYS

### SYS-003 安全状态机 heartbeat

```yaml
id: SYS-003
name: Safety State Machine Heartbeat Loss
category: SYSTEM

precondition:
  - watchdog_enabled == true
  # 监控对象：安全状态机主循环

input:
  - state_machine_heartbeat_wall_time
  - now_wall

model:
  staleness: now_wall - state_machine_heartbeat_wall_time
  timeout:
    T_wd_sm: 0.30                       # s
  also_detect:
    - 状态机进程退出（process exit 事件）

condition:
  staleness > T_wd_sm
  OR 状态机进程退出

severity: S4

response:
  action: PROTECTIVE_STOP                # 由独立组件执行，非状态机自身
  trigger_source: external_watchdog
  latch: false

diagnostics:
  reason: STATE_MACHINE_HEARTBEAT_LOST

validation:
  method: sigstop_process_test           # SIGSTOP 状态机进程，断言看门狗触发
  measured: true
```

**中文说明**
- **意义**：安全状态机是**决策者**。它一旦卡死，所有高级保护（保护停、限速、降级）全部失效——而且**没有任何告警**，因为告警也是它发的。这是最危险的失效：系统"看起来很安静"，实际已失去保护。
- **⚠ 架构红线（务必注意）**：**看门狗不能放在被监控的组件内部**。如果状态机自己检查自己的心跳，它卡死时看门狗也一起卡死，等于没有。`trigger_source: external_watchdog` 就是为此标注——看门狗必须由**独立组件**实现（建议放在 Safety Gate 或独立的进程/硬件看门狗中）。
- **`T_wd_sm = 0.30 s` 的依据**：约为控制时延 `T_ctrl`（0.10 s）的 3 倍，容忍偶发调度延迟，但不容忍持续停滞。
- **与 `SYS-005` 的分工**：`SYS-003` 监控**决策层**（状态机）是否活着；`SYS-005` 监控**执行层**（门控）是否活着。两者机制相同、对象不同，都需要独立看门狗。

---

### SYS-005 Safety Gate heartbeat

```yaml
id: SYS-005
name: Safety Gate Heartbeat Loss
category: SYSTEM

precondition:
  - watchdog_enabled == true
  # 监控对象：Safety Gate（唯一 /cmd_vel 出口仲裁器）

input:
  - safety_gate_heartbeat_wall_time
  - now_wall

model:
  staleness: now_wall - safety_gate_heartbeat_wall_time
  timeout:
    T_gate: 0.20                        # s
  also_detect:
    - 门控进程停滞或退出
    - 门控未在 T_gate 内更新输出

condition:
  staleness > T_gate
  OR 门控未更新输出

severity: S4

response:
  action: REMOVE_MOTOR_POWER             # 由驱动侧独立看门狗执行
  trigger_source: drive_side_watchdog    # 不依赖任何上位软件存活
  latch: false

diagnostics:
  reason: SAFETY_GATE_HEARTBEAT_LOST

validation:
  method: kill_gate_process_test         # kill 门控进程，断言电机失能
  measured: true
```

**中文说明**
- **意义**：Safety Gate 是**最后防线**——所有指令的唯一出口仲裁器。如果它卡死且无看门狗，指令流会中断但电机可能保持最后状态（**持续运动**），这是最坏的结果。
- **⚠ 关键要求**：`action: REMOVE_MOTOR_POWER` 必须由**驱动侧独立看门狗**执行，**不能依赖任何上位软件**。如果门控卡死而响应逻辑也跑在上位机上，响应同样不会发生。
- **`T_gate = 0.20 s` 比 `T_wd_sm = 0.30 s` 更严**：因为门控离执行器更近，它的停滞直接意味着电机失去控制。越靠近执行器，超时应越短。
- **为什么对"未更新输出"也要判**：进程可能还活着但陷入死循环，此时心跳线程若独立仍会跳（假活）。必须同时检查**输出是否真的在更新**——这是"活性"与"进度"的区别。

---

## 7. 依赖关系与实施顺序

> 下表列出**存在硬依赖**的规则（共 6 条）。未列出的规则（`MOT-003` `CMD-005` `LOC-001` `COL-006` 等）无硬依赖，可独立实现。

```yaml
dependencies:

  COL-005:
    requires: [SEN-001, SEN-011, COL-011]
    reason: "obstacle_distance 可信的前提是扫描健康且数值合法"

  COL-006:
    requires: [COL-005]
    reason: "共用停止距离模型；COL-005 未标定时 COL-006 无意义"

  COL-010:
    requires: [SEN-001]
    reason: "视场覆盖信息来自 scan，需扫描健康"

  MOT-004:
    requires: [MOT-001, MOT-002]
    reason: "先判单值上限，再判耦合可达性"

  CMD-007:
    requires: [SEN-001]
    reason: "指令超时与源超时使用同一墙钟机制"

  SYS-003:
    requires: [SYS-005]
    reason: "状态机看门狗需由门控（或独立组件）承载，避免自我监控"

implementation_order:
  phase_1_foundation:
    - SEN-001        # 一切判据的前提：数据可信
    - SEN-011
    - SYS-005        # 最后防线先建，否则后面所有规则都无兜底
    - SYS-003
  phase_2_motion:
    - MOT-001
    - MOT-002
    - MOT-003
    - MOT-004
  phase_3_collision:
    - COL-011
    - COL-010
    - COL-005        # 需先标定 T_response / A_brake
    - COL-006
  phase_4_command:
    - CMD-007
    - CMD-005
  phase_5_localization:
    - LOC-005
    - LOC-001
```

**实施顺序的中文说明**
- **先建 `SYS-005`（Gate 看门狗）**：它是唯一能兜住其他所有规则失效的机制。没有它，后续规则的正确性无法被保证。
- **`COL-005` 必须最后做**：它依赖 `T_response` 与 `A_brake` 的实测值。未标定前实现它，只会得到一个数值错误的判据。
- **`SEN-001` 最先做**：所有距离类判据都建立在"数据可信"之上。

---

## 8. 暂缓项（本轮未选中，仍在 RSS-002）

```yaml
deferred_rules:
  MOT:
    - MOT-005   # jerk 限制（条件项）
    - MOT-009   # 未武装指令
    - MOT-010   # 倾角
    - MOT-011   # 旋转空间许可
    - MOT-012   # 该动没动
  COL:
    - COL-001   # 接近告警
    - COL-002   # 障碍临界距离
    - COL-003   # 全向最小距离
    - COL-008   # 距离突变
    - COL-012   # 残余速度确认
    - COL-013   # 人员检测
    - COL-014   # SSM
  SEN:
    - SEN-003   # 数值卡死
    - SEN-004   # 激光帧冻结
    - SEN-005   # IMU 零方差
    - SEN-007   # 协方差
    - SEN-008   # 偏航交叉校验
    - SEN-009   # 轮速交叉校验
    - SEN-010   # 激光运动一致性
    - SEN-012   # 时钟合理性
    - SEN-013   # 采样率
    - SEN-014   # 电池
  CMD:
    - CMD-004   # 指令时间戳
    - CMD-006   # 控制权声明
    - CMD-010   # 指令泛洪
    - CMD-011   # 该动没动
    - CMD-012   # 不该动却动
  LOC:
    - LOC-002   # 偏航跳变
    - LOC-003   # 协方差
    - LOC-004   # 数值合法性
    - LOC-006   # 定位—里程计偏差
    - LOC-007   # 位姿冻结
    - LOC-008   # 置信度
    - LOC-009   # 长期漂移
  COM:
    - COM-001 ~ COM-010
  SYS:
    - SYS-001   # 掉电
    - SYS-002   # 急停回路
    - SYS-004   # 监控节点看门狗
    - SYS-006 ~ SYS-018

  note: >
    未选中不等于不重要。以下 3 条在风险评估中优先级很高，
    建议在核心 16 条稳定后尽快补入。
```

**中文说明：暂缓项中建议优先补回的 3 条**

| 规则 | 为什么重要 |
|---|---|
| `COL-002` 障碍临界距离 | 与 `COL-005` 同源；`COL-005` 用模型算，`COL-002` 用固定阈值兜底。两者互为冗余 |
| `SEN-003` 数值卡死 | 传感器**持续发布错误数据**比丢失数据更危险——`SEN-001` 完全抓不到它 |
| `SYS-002` 急停回路 | 纯硬件兜底。所有软件判据都失效时，它是唯一剩下的保护 |

---

## 9. 待确认问题

| # | 问题 | 影响 |
|---|---|---|
| 1 | `LOC-005` 与 `SEN-001` 是否重复？定位话题是否已纳入 `SEN-001` 的监控源清单？ | 若是，需二选一，避免同一超时判两次 |
| 2 | `T_response` 与 `A_brake` 能否实测？ | **不能实测则 `COL-005`/`COL-006` 无法实现**（这是硬依赖） |
| 3 | 收紧 `SEN-001` 超时值后，调度抖动会不会造成误报？需实测抖动分布 | 决定超时值能否收紧 |
| 4 | `CMD-005` 的指令源标识如何获取且可靠？仅靠话题名不足 | 决定本判据能否实现 |
| 5 | `MOT-003` 差分是否需先做滑动平均？指令噪声水平如何？ | 决定加速度判据是否频繁误报 |
| 6 | 激光视野角是多少？是否有后向/侧向覆盖？ | 决定 `COL-010` 的实际触发频率 |

---

*RSS-003 · v1.0 · 16 条核心判据*
