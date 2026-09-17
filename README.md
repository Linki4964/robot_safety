# robot_safety — 机器人安全监控与门控

## 一、项目简介

本项目是一个面向 TurtleBot3 / Gazebo 的机器人安全栈，参考标准 ISO 13849（安全检测标准设计参考第 5 章、第 6 章，验证测试参考第 10 章），涵盖**观测、裁决、强制执行**三个部分：观测层（`monitor`）以 10 Hz 发布 `RobotState` 快照，判断各话题数据是否可信、物理量是否合理；裁决层（`motion_safety`）识别速度、加速度与差速运动学超限，只识别、只告警，不做任何处置；门控层（`safety_gate`）在 `/cmd_vel` 路径上强制执行裁决结果，只做原样放行、发布零速、拒绝三件事之一，只减不增。

## 二、项目演示

![robot safety demo](TMP_RECORDING.gif)

录屏内容为 Gazebo 中的 TurtleBot3（burger）及其 360° 激光扫描可视化：机器人静止停在生成点，蓝色射线为其激光测距结果，全部延伸至量程上限而无任何回波被障碍物截断，对应"前方空阔、无障碍"的正常状态。此时监控的判定为 `status=OK` 且无告警，位姿与生成点一致。

## 三、启动方法

```bash
cd ~/robot-safety
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash
```

```bash
# 终端 A：仿真
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 B：监控 + 门控 + 状态显示
ros2 launch robot_safety_monitor safety_gate.launch.py
```

```bash
ros2 run robot_safety_monitor state_reporter             # 持续摘要，状态跳变时打印详情
ros2 run robot_safety_monitor state_reporter --once -v   # 单次全量快照
```
