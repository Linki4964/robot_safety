# Copyright 2026 robot-safety maintainer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Runtime state monitor node: subscribes to the robot topics, judges the data
in :mod:`robot_safety_monitor.analyzer`, and publishes one aggregated
``RobotState`` snapshot per tick.

Design notes that matter for the safety state machine built on top of this:

* **One collector per topic.** Each collector owns exactly one subscription and
  converts a raw message into the ROS-independent sample dataclass the analyser
  consumes. Adding a new monitored topic means adding one collector class and
  one line in :meth:`StatePublisher.build_collectors` -- not editing a large
  callback.
* **Wall clock for watchdogs, ROS time for stamps.** See the module docstring
  of :mod:`robot_safety_monitor.analyzer` for why this separation is required.
* **The node never decides policy.** It reports status, findings and
  ``motion_expected``. Whether the robot must stop is the business of the step-2
  safety state machine, which consumes ``/robot_safety/monitor/state``.
"""

from __future__ import annotations

import math
import time
import traceback
from typing import Dict, List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.clock import Clock as ClockSource, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import BatteryState as BatteryStateMsg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import JointState as JointStateMsg
from sensor_msgs.msg import LaserScan as LaserScanMsg

from robot_safety_msgs.msg import (
    BatteryState,
    Imu,
    JointState,
    LaserScan,
    Odometry,
    RobotState,
    TopicHealth,
    VelocityCommand,
)

from .analyzer import (
    ALL_SOURCES,
    SOURCE_BATTERY,
    SOURCE_COMMAND,
    SOURCE_IMU,
    SOURCE_JOINTS,
    SOURCE_ODOM,
    SOURCE_SCAN,
    Analyzer,
    AttitudeSample,
    BatterySample,
    CommandSample,
    Config,
    JointSample,
    MonitorState,
    MotionSample,
    ObservationTracker,
    RangeSample,
    quaternion_to_roll_pitch,
    quaternion_to_yaw,
    tilt_angle_deg,
    variance_or_negative_one,
)

SOFTWARE_VERSION = "0.1.0"

# Sensor data is streamed, not latched: a late subscriber wants the newest
# sample, so keep a shallow queue and best-effort delivery.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Diagonal slots of a twist covariance that a planar differential-drive base can
# actually observe: linear x, linear y, angular z. Slots 2, 3 and 4 are z, roll
# and pitch, which odometry marks with an "unobserved" sentinel.
OBSERVABLE_TWIST_AXES = (0, 1, 5)


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #
def _stamp_to_sec(stamp) -> float:
    """Convert a ``builtin_interfaces/Time`` to float seconds without raising.

    ``stamp`` may be ``None`` when a publisher left the header unset, which is
    common on hand-rolled test publishers, so this must never throw.
    """
    if stamp is None:
        return 0.0
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return 0.0


def _finite(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _as_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


class _SimTimeSubscriber:
    """Receives ``/clock`` explicitly, with a QoS that actually matches.

    Gazebo publishes ``/clock`` as BEST_EFFORT, while ``create_subscription``
    defaults to RELIABLE for non-sensor types. Those are incompatible in Humble,
    so a node relying on ``use_sim_time`` alone can silently never receive
    simulated time -- and therefore never fire its ROS-time timer at all, which
    looks exactly like a monitor that is running but reporting nothing. Creating
    the subscription here with the matching profile removes that failure mode.
    """

    def __init__(self, node: Node, topic: str = "/clock"):
        self._node = node
        self.active = False
        self.last_stamp = 0.0
        self.last_wall_time = 0.0
        self._subscription = node.create_subscription(
            ClockMsg, topic, self._on_clock, SENSOR_QOS
        )

    def _on_clock(self, msg: ClockMsg) -> None:
        self.active = True
        self.last_stamp = _stamp_to_sec(msg.clock)
        self.last_wall_time = time.time()

    def age_sec(self) -> float:
        """Wall-clock age of the newest /clock sample; ``inf`` when never seen."""
        if not self.active:
            return math.inf
        return max(0.0, time.time() - self.last_wall_time)


# --------------------------------------------------------------------------- #
# Collector base
# --------------------------------------------------------------------------- #
class Collector:
    """Base class for per-topic collectors.

    Subclasses implement :meth:`build_sample`, which turns the latest raw
    message into the analyser's sample dataclass. Subscribing, stamping and
    observation bookkeeping are handled here so subclasses stay small.
    """

    source: str = ""

    def __init__(
        self,
        node: Node,
        tracker: ObservationTracker,
        config: Config,
        topic: str,
        publish_wall_time: bool = True,
    ):
        self.node = node
        self.tracker = tracker
        self.config = config
        self.topic = topic

        # A collector whose topic is disabled still needs a tracker record so
        # the source shows up as ERROR rather than silently vanishing from the
        # snapshot (an unseen-but-expected source must be visible).
        self.observation = tracker.register(
            self.source, topic, self.timeout_sec(config)
        )
        self.subscription = None
        self.last_message = None
        self.last_stamp = 0.0

        if topic:
            self.subscription = node.create_subscription(
                self.msg_type,
                topic,
                self._on_message,
                SENSOR_QOS,
            )

    # -- to be provided by subclasses ---------------------------------------
    @property
    def msg_type(self):
        raise NotImplementedError

    def timeout_sec(self, config: Config) -> float:
        raise NotImplementedError

    def build_sample(self, msg) -> object:  # pragma: no cover - interface only
        raise NotImplementedError

    def apply_sample(self, state: MonitorState, sample) -> None:
        raise NotImplementedError

    # -- plumbing -----------------------------------------------------------
    def _on_message(self, msg) -> None:
        wall = time.time()
        self.last_message = msg
        header = getattr(msg, "header", None)
        self.last_stamp = _stamp_to_sec(getattr(header, "stamp", None)) if header is not None else 0.0
        self.tracker.observe(self.source, wall, self.last_stamp)
        try:
            self.tracker.set_payload(self.source, **self.derived_payload(msg))
        except Exception as exc:  # noqa: BLE001 - a bad message must not kill the monitor
            self.node.get_logger().warn(
                "failed to derive payload for %s: %s" % (self.source, exc)
            )

    def derived_payload(self, msg) -> Dict[str, object]:
        """Optional extra values exposed for logging / fine-grained checks."""
        return {}

    def collect(self, state: MonitorState) -> None:
        """Attach the latest sample (when present) to ``state``."""
        if self.last_message is None:
            return
        try:
            self.apply_sample(state, self.build_sample(self.last_message))
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                "failed to process %s message: %s" % (self.source, exc)
            )

    def last_wall_time(self) -> float:
        return (
            self.observation.last_wall_time
            if self.observation.last_wall_time is not None
            else 0.0
        )


class OdometryCollector(Collector):
    source = SOURCE_ODOM
    msg_type = OdometryMsg

    def timeout_sec(self, config: Config) -> float:
        return config.odom_timeout_sec

    def derived_payload(self, msg) -> Dict[str, object]:
        return {
            "x": _as_float(msg.pose.pose.position.x),
            "y": _as_float(msg.pose.pose.position.y),
        }

    def build_sample(self, msg: OdometryMsg) -> MotionSample:
        pose = msg.pose.pose
        twist = msg.twist.twist

        quat = pose.orientation
        yaw = quaternion_to_yaw(
            _as_float(quat.x), _as_float(quat.y), _as_float(quat.z), _as_float(quat.w, 1.0)
        )

        vx = _as_float(twist.linear.x)
        vy = _as_float(twist.linear.y)
        vz = _as_float(twist.linear.z)

        # Generated messages expose covariance as numpy.ndarray or array.array,
        # so never use truthiness on them: `not array` is ambiguous for numpy
        # and silently wrong for array.array. Only None means "absent".
        pose_cov = msg.pose.covariance
        twist_cov = msg.twist.covariance
        if pose_cov is None:
            pose_cov = []
        if twist_cov is None:
            twist_cov = []

        # A planar diff-drive base observes only x, y and yaw. The remaining
        # diagonal entries carry the "not observed" sentinel, so they are read
        # but filtered out by variance_or_negative_one rather than counted.
        twist_observable = [
            variance_or_negative_one(twist_cov, index) for index in OBSERVABLE_TWIST_AXES
        ]
        twist_usable = [value for value in twist_observable if value >= 0.0]
        twist_cov_max = max(twist_usable) if twist_usable else -1.0

        return MotionSample(
            available=True,
            x=_as_float(pose.position.x),
            y=_as_float(pose.position.y),
            z=_as_float(pose.position.z),
            yaw_rad=yaw,
            vx=vx,
            vy=vy,
            vz=vz,
            speed_mps=math.sqrt(vx * vx + vy * vy + vz * vz),
            yaw_rate_rps=_as_float(twist.angular.z),
            pose_cov_xx=variance_or_negative_one(pose_cov, 0),
            pose_cov_yy=variance_or_negative_one(pose_cov, 1),
            pose_cov_yawyaw=variance_or_negative_one(pose_cov, 5),
            twist_cov_max=twist_cov_max,
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: MotionSample) -> None:
        state.odom = sample


class LaserScanCollector(Collector):
    source = SOURCE_SCAN
    msg_type = LaserScanMsg

    def __init__(self, *args, front_half_angle_rad: float = 0.35, **kwargs):
        # The frontal sector is where a differential-drive robot is about to go,
        # so a near reading there matters more than one at the side.
        self.front_half_angle_rad = float(front_half_angle_rad)
        super().__init__(*args, **kwargs)

    def timeout_sec(self, config: Config) -> float:
        return config.scan_timeout_sec

    def derived_payload(self, msg) -> Dict[str, object]:
        return {"point_count": len(msg.ranges)}

    def build_sample(self, msg: LaserScanMsg) -> RangeSample:
        angle_min = _as_float(msg.angle_min)
        angle_increment = _as_float(msg.angle_increment)
        range_min = _as_float(msg.range_min)
        range_max = _as_float(msg.range_max)

        closest = math.inf
        front = math.inf
        front_angle = 0.0
        valid = 0

        for index, raw in enumerate(msg.ranges):
            value = _as_float(raw, default=float("nan"))
            if not _finite(value):
                continue
            if value < range_min or value > range_max:
                continue
            valid += 1
            if value < closest:
                closest = value
            angle = angle_min + index * angle_increment
            if abs(angle) <= self.front_half_angle_rad and value < front:
                front = value
                front_angle = angle

        return RangeSample(
            available=True,
            angle_min=angle_min,
            angle_max=_as_float(msg.angle_max),
            angle_increment=angle_increment,
            range_min=range_min,
            range_max=range_max,
            closest_range=closest,
            front_range=front,
            front_angle=front_angle,
            valid_point_count=valid,
            point_count=len(msg.ranges),
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: RangeSample) -> None:
        state.scan = sample


class ImuCollector(Collector):
    source = SOURCE_IMU
    msg_type = ImuMsg

    def timeout_sec(self, config: Config) -> float:
        return config.imu_timeout_sec

    def derived_payload(self, msg) -> Dict[str, object]:
        return {"lin_acc_z": _as_float(msg.linear_acceleration.z)}

    def build_sample(self, msg: ImuMsg) -> AttitudeSample:
        orientation = msg.orientation
        qx = _as_float(orientation.x)
        qy = _as_float(orientation.y)
        qz = _as_float(orientation.z)
        qw = _as_float(orientation.w)

        # A quaternion of all zeros is the conventional "no estimate" value.
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        orientation_available = norm > 1e-6
        roll = pitch = yaw = 0.0
        tilt = 0.0
        if orientation_available:
            roll, pitch = quaternion_to_roll_pitch(qx, qy, qz, qw)
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            tilt = tilt_angle_deg(roll, pitch)

        return AttitudeSample(
            available=True,
            orientation_available=orientation_available,
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            tilt_deg=tilt,
            ang_vel_x=_as_float(msg.angular_velocity.x),
            ang_vel_y=_as_float(msg.angular_velocity.y),
            ang_vel_z=_as_float(msg.angular_velocity.z),
            lin_acc_x=_as_float(msg.linear_acceleration.x),
            lin_acc_y=_as_float(msg.linear_acceleration.y),
            lin_acc_z=_as_float(msg.linear_acceleration.z),
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: AttitudeSample) -> None:
        state.imu = sample


class JointStateCollector(Collector):
    source = SOURCE_JOINTS
    msg_type = JointStateMsg

    def timeout_sec(self, config: Config) -> float:
        return config.joints_timeout_sec

    def derived_payload(self, msg) -> Dict[str, object]:
        return {"joint_count": len(msg.name)}

    def build_sample(self, msg: JointStateMsg) -> JointSample:
        names: Sequence[str] = msg.name or []
        velocities: Sequence[float] = msg.velocity or []

        moving: List[str] = []
        max_abs = 0.0
        total_abs = 0.0
        for index, name in enumerate(names):
            if index >= len(velocities):
                break
            velocity = abs(_as_float(velocities[index]))
            total_abs += velocity
            if velocity > max_abs:
                max_abs = velocity
            if velocity > self.config.joint_velocity_threshold:
                moving.append(str(name))

        return JointSample(
            available=True,
            joint_count=len(names),
            moving_joints=moving,
            max_abs_velocity=max_abs,
            total_abs_velocity=total_abs,
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: JointSample) -> None:
        state.joints = sample


class VelocityCommandCollector(Collector):
    source = SOURCE_COMMAND
    msg_type = Twist

    def timeout_sec(self, config: Config) -> float:
        return config.command_timeout_sec

    def build_sample(self, msg: Twist) -> CommandSample:
        linear = msg.linear
        angular = msg.angular
        linear_x = _as_float(linear.x)
        linear_y = _as_float(linear.y)
        linear_z = _as_float(linear.z)
        angular_x = _as_float(angular.x)
        angular_y = _as_float(angular.y)
        angular_z = _as_float(angular.z)

        magnitude = max(
            abs(linear_x), abs(linear_y), abs(linear_z),
            abs(angular_x), abs(angular_y), abs(angular_z),
        )
        return CommandSample(
            available=True,
            fresh=False,  # filled in by the publisher, which owns the clock
            stop_command=magnitude < self.config.command_motion_threshold,
            linear_x=linear_x,
            linear_y=linear_y,
            linear_z=linear_z,
            angular_x=angular_x,
            angular_y=angular_y,
            angular_z=angular_z,
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: CommandSample) -> None:
        state.command = sample


class BatteryCollector(Collector):
    source = SOURCE_BATTERY
    msg_type = BatteryStateMsg

    def timeout_sec(self, config: Config) -> float:
        return config.battery_timeout_sec

    def build_sample(self, msg: BatteryStateMsg) -> BatterySample:
        percentage = _as_float(msg.percentage, default=-1.0)
        percentage = max(-1.0, min(1.0, percentage))
        return BatterySample(
            available=True,
            voltage_v=_as_float(msg.voltage),
            percentage=percentage,
            temperature_c=_as_float(msg.temperature),
            charging=bool(getattr(msg, "power_supply_status", 0) == 1),
            power_supply_status=int(getattr(msg, "power_supply_status", 0)),
            last_stamp_sec=self.last_stamp,
            last_wall_time_sec=self.last_wall_time(),
        )

    def apply_sample(self, state: MonitorState, sample: BatterySample) -> None:
        state.battery = sample


# --------------------------------------------------------------------------- #
# State publisher
# --------------------------------------------------------------------------- #
class StatePublisher(Node):
    """Aggregates robot telemetry into periodic ``RobotState`` snapshots."""

    def __init__(self) -> None:
        super().__init__("robot_safety_monitor")

        self.config = self._declare_config()
        self.tracker = ObservationTracker(
            self.config, rate_window_sec=self.config.rate_window_sec
        )
        self.analyzer = Analyzer(self.config, self.tracker)

        self.robot_id = self.declare_parameter("robot_id", "turtlebot3").value
        self.world_name = self.declare_parameter("world_name", "").value
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self.publish_rate_hz = max(0.5, min(100.0, self.publish_rate_hz))
        self.expect_sim_time = bool(
            self.declare_parameter("expect_sim_time", True).value
        )

        topics = {
            SOURCE_ODOM: str(self.declare_parameter("topic.odom", "/odom").value),
            SOURCE_SCAN: str(self.declare_parameter("topic.scan", "/scan").value),
            SOURCE_IMU: str(self.declare_parameter("topic.imu", "/imu").value),
            SOURCE_JOINTS: str(
                self.declare_parameter("topic.joint_states", "/joint_states").value
            ),
            SOURCE_COMMAND: str(
                self.declare_parameter("topic.cmd_vel", "/cmd_vel").value
            ),
            SOURCE_BATTERY: str(self.declare_parameter("topic.battery", "").value),
        }
        front_half_angle = float(
            self.declare_parameter("front_sector_half_angle_rad", 0.35).value
        )

        self.publisher = self.create_publisher(
            RobotState, "robot_safety/monitor/state", QoSProfile(depth=10)
        )

        self.collectors: Dict[str, Collector] = {}
        self._add_collector(
            OdometryCollector(self, self.tracker, self.config, topics[SOURCE_ODOM])
        )
        self._add_collector(
            LaserScanCollector(
                self,
                self.tracker,
                self.config,
                topics[SOURCE_SCAN],
                front_half_angle_rad=front_half_angle,
            )
        )
        self._add_collector(
            ImuCollector(self, self.tracker, self.config, topics[SOURCE_IMU])
        )
        self._add_collector(
            JointStateCollector(
                self, self.tracker, self.config, topics[SOURCE_JOINTS]
            )
        )
        # Command monitoring is opt-out. By default the source is watched, so a
        # commander that was running and then died is reported as COMMAND_STALE:
        # that is a real fault the safety layer must see. Set command.monitor
        # to false only on a platform that genuinely has no velocity commander,
        # where the source is then excluded from the assessment entirely.
        if self.config.monitor_command:
            self._add_collector(
                VelocityCommandCollector(
                    self, self.tracker, self.config, topics[SOURCE_COMMAND]
                )
            )
        else:
            self.tracker.declare_not_monitored(
                SOURCE_COMMAND, self.config.command_timeout_sec
            )
        # Battery is optional: monitoring it is opt-in because the Gazebo
        # TurtleBot3 publishes no battery topic, and an unmonitored source must
        # not show up as a permanent failure.
        if self.config.monitor_battery or topics[SOURCE_BATTERY]:
            self._add_collector(
                BatteryCollector(
                    self, self.tracker, self.config, topics[SOURCE_BATTERY]
                )
            )
        else:
            self.tracker.declare_not_monitored(
                SOURCE_BATTERY, self.config.battery_timeout_sec
            )

        self.start_wall = time.time()
        self.tick_count = 0
        self.last_tick_wall = self.start_wall
        self.tick_period = 1.0 / self.publish_rate_hz

        # Created before the timer so the first ticks already have a stamp. The
        # timer is intentionally driven by wall time even under Gazebo: with a
        # ROS-time timer the monitor would stop publishing whenever the simulator
        # pauses, which would hide exactly the failure it exists to report.
        self.sim_clock = _SimTimeSubscriber(self)

        # The tick loop runs on wall time even when the node uses simulated time.
        # A ROS-time timer would freeze whenever the simulator pauses, so the
        # monitor would stop reporting precisely when the robot's state becomes
        # unknown -- the one moment its output matters most.
        wall_clock = ClockSource(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(self.tick_period, self.tick, clock=wall_clock)

        self.get_logger().info(
            "runtime state monitor up | robot=%s sim_time=%s rate=%.1fHz "
            "publishing %s"
            % (
                self.robot_id,
                self.expect_sim_time,
                self.publish_rate_hz,
                self.publisher.topic_name,
            )
        )

    # -- setup helpers ------------------------------------------------------
    def _add_collector(self, collector: Collector) -> None:
        self.collectors[collector.source] = collector
        if collector.subscription is None:
            self.get_logger().warn(
                "source %s has no topic configured; it will be reported as "
                "missing" % collector.source
            )
        else:
            self.get_logger().debug(
                "monitoring %s on %s" % (collector.source, collector.topic)
            )

    def _declare_config(self) -> Config:
        """Build the analyzer config from ROS parameters.

        Parameter names are dotted and mirror the dataclass fields, so a
        threshold can be retuned from a launch file or ``ros2 param set``
        without touching code.
        """
        defaults = Config()
        config = Config(
            odom_timeout_sec=float(
                self.declare_parameter("timeout.odom_sec", defaults.odom_timeout_sec).value
            ),
            scan_timeout_sec=float(
                self.declare_parameter("timeout.scan_sec", defaults.scan_timeout_sec).value
            ),
            imu_timeout_sec=float(
                self.declare_parameter("timeout.imu_sec", defaults.imu_timeout_sec).value
            ),
            joints_timeout_sec=float(
                self.declare_parameter(
                    "timeout.joint_states_sec", defaults.joints_timeout_sec
                ).value
            ),
            command_timeout_sec=float(
                self.declare_parameter(
                    "timeout.cmd_vel_sec", defaults.command_timeout_sec
                ).value
            ),
            battery_timeout_sec=float(
                self.declare_parameter(
                    "timeout.battery_sec", defaults.battery_timeout_sec
                ).value
            ),
            battery_warn_fraction=float(
                self.declare_parameter(
                    "battery.warn_fraction", defaults.battery_warn_fraction
                ).value
            ),
            battery_critical_fraction=float(
                self.declare_parameter(
                    "battery.critical_fraction", defaults.battery_critical_fraction
                ).value
            ),
            tilt_warn_deg=float(
                self.declare_parameter("tilt.warn_deg", defaults.tilt_warn_deg).value
            ),
            tilt_critical_deg=float(
                self.declare_parameter(
                    "tilt.critical_deg", defaults.tilt_critical_deg
                ).value
            ),
            obstacle_warn_range_m=float(
                self.declare_parameter(
                    "obstacle.warn_range_m", defaults.obstacle_warn_range_m
                ).value
            ),
            obstacle_critical_range_m=float(
                self.declare_parameter(
                    "obstacle.critical_range_m", defaults.obstacle_critical_range_m
                ).value
            ),
            pose_cov_warn=float(
                self.declare_parameter("odom.pose_cov_warn", defaults.pose_cov_warn).value
            ),
            twist_cov_warn=float(
                self.declare_parameter("odom.twist_cov_warn", defaults.twist_cov_warn).value
            ),
            command_deadband_linear=float(
                self.declare_parameter(
                    "command.deadband_linear", defaults.command_deadband_linear
                ).value
            ),
            command_deadband_angular=float(
                self.declare_parameter(
                    "command.deadband_angular", defaults.command_deadband_angular
                ).value
            ),
            command_tolerance_fraction=float(
                self.declare_parameter(
                    "command.tolerance_fraction", defaults.command_tolerance_fraction
                ).value
            ),
            command_motion_threshold=float(
                self.declare_parameter(
                    "command.motion_threshold", defaults.command_motion_threshold
                ).value
            ),
            joint_velocity_threshold=float(
                self.declare_parameter(
                    "joints.velocity_threshold", defaults.joint_velocity_threshold
                ).value
            ),
            monitor_battery=bool(
                self.declare_parameter("battery.monitor", defaults.monitor_battery).value
            ),
            monitor_command=bool(
                self.declare_parameter("command.monitor", defaults.monitor_command).value
            ),
            rate_window_sec=float(
                self.declare_parameter("rate_window_sec", defaults.rate_window_sec).value
            ),
        )
        return config

    # -- time ---------------------------------------------------------------
    def ros_now_sec(self) -> float:
        """Current ROS time in seconds (simulated time under Gazebo).

        Falls back to the newest ``/clock`` value when the node clock has not
        started. That matters at startup: with ``use_sim_time`` the node clock
        stays at zero until the first ``/clock`` arrives, and a zero stamp in the
        snapshot would be indistinguishable from an epoch timestamp.
        """
        stamp = _stamp_to_sec(self.get_clock().now().to_msg())
        if stamp > 0.0:
            return stamp
        return float(self.sim_clock.last_stamp)

    def sim_time_active(self) -> bool:
        """True once ROS time is advancing, from the node clock or from /clock."""
        return self.ros_now_sec() > 0.0

    # -- main loop ----------------------------------------------------------
    def tick(self) -> None:
        wall = time.time()
        self.tick_count += 1
        period = wall - self.last_tick_wall
        self.last_tick_wall = wall

        state = MonitorState(now_wall=wall, now_stamp=self.ros_now_sec())
        for collector in self.collectors.values():
            collector.collect(state)

        # Command freshness is a wall-clock property, so it is computed here
        # rather than inside the collector callback.
        if state.command.available:
            command_observation = self.tracker.get(SOURCE_COMMAND)
            state.command.fresh = bool(
                command_observation is not None and command_observation.fresh(wall)
            )

        assessment = self.analyzer.assess(state, now=wall)
        motion_expected = self.analyzer.motion_expected(state)

        message = self.build_message(
            state, assessment, motion_expected, period, wall
        )
        self.publisher.publish(message)

    # -- message construction ----------------------------------------------
    def build_message(
        self,
        state: MonitorState,
        assessment,
        motion_expected: bool,
        period: float,
        wall: float,
    ) -> RobotState:
        message = RobotState()

        # Use the same time source as the rest of the snapshot, including the
        # /clock fallback, so the published stamp always matches the sensor
        # stamps that were judged.
        stamp = self.get_clock().now().to_msg()
        stamp_sec = self.ros_now_sec()
        stamp.sec = int(stamp_sec)
        stamp.nanosec = int(round((stamp_sec - int(stamp_sec)) * 1e9))
        if stamp.nanosec >= 1000000000:
            stamp.sec += 1
            stamp.nanosec -= 1000000000
        message.header.stamp = stamp
        message.header.frame_id = "odom"

        message.robot_id = str(self.robot_id)
        message.world_name = str(self.world_name)
        message.monitor_node = self.get_fully_qualified_name()
        message.software_version = SOFTWARE_VERSION

        message.wall_time_sec = float(wall)
        message.uptime_sec = float(wall - self.start_wall)
        message.tick_period_sec = float(period)
        message.tick_count = int(self.tick_count)

        message.status = int(assessment.status)

        # Per-source arrays are emitted in a stable order so consumers can index
        # them, and so a diff of two snapshots is readable.
        ordered_sources = self._ordered_source_names()
        for name in ordered_sources:
            message.source_name.append(name)
            message.source_status.append(
                int(assessment.source_status.get(name, 0))
            )
            message.source_reason.append(
                str(assessment.source_reason.get(name, ""))
            )

        for finding in assessment.findings:
            message.warnings.append(finding.code)
            message.warning_severity.append(int(finding.severity))
            message.warning_detail.append(finding.detail)

        message.motion_expected = bool(motion_expected)
        message.armed = bool(motion_expected and assessment.status <= 1)

        self._fill_odom(message.odom, state.odom)
        self._fill_battery(message.battery, state.battery)
        self._fill_imu(message.imu, state.imu)
        self._fill_scan(message.scan, state.scan)
        self._fill_joints(message.joints, state.joints)
        self._fill_command(message.command, state.command)
        return message

    def _ordered_source_names(self) -> List[str]:
        """Monitored sources in a stable order, for the parallel arrays.

        Sources the platform declared as not monitored are excluded: they are
        neither healthy nor failing, and including them would force a consumer to
        special-case entries that are permanently ERROR.
        """
        monitored = set(self.tracker.monitored_names())
        ordered = [name for name in ALL_SOURCES if name in monitored]
        ordered.extend(
            name for name in self.tracker.names()
            if name in monitored and name not in ordered
        )
        return ordered

    # Each _fill_* mirrors the corresponding .msg definition field by field.
    def _fill_odom(self, out: Odometry, sample: MotionSample) -> None:
        out.available = bool(sample.available)
        out.x = float(sample.x)
        out.y = float(sample.y)
        out.z = float(sample.z)
        out.yaw_rad = float(sample.yaw_rad)
        out.vx = float(sample.vx)
        out.vy = float(sample.vy)
        out.vz = float(sample.vz)
        out.speed_mps = float(sample.speed_mps)
        out.yaw_rate_rps = float(sample.yaw_rate_rps)
        out.pose_cov_xx = float(sample.pose_cov_xx)
        out.pose_cov_yy = float(sample.pose_cov_yy)
        out.pose_cov_yawyaw = float(sample.pose_cov_yawyaw)
        out.twist_cov_max = float(sample.twist_cov_max)
        out.pose_valid = bool(
            sample.available
            and (sample.pose_cov_xx < 0.0 or sample.pose_cov_xx < self.config.pose_cov_warn)
            and (sample.pose_cov_yy < 0.0 or sample.pose_cov_yy < self.config.pose_cov_warn)
        )
        out.twist_valid = bool(
            sample.available
            and (sample.twist_cov_max < 0.0 or sample.twist_cov_max < self.config.twist_cov_warn)
        )
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)

    def _fill_battery(self, out: BatteryState, sample: BatterySample) -> None:
        out.available = bool(sample.available)
        out.voltage_v = float(sample.voltage_v)
        out.percentage = float(sample.percentage)
        out.temperature_c = float(sample.temperature_c)
        out.charging = bool(sample.charging)
        out.power_supply_status = int(sample.power_supply_status)
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)

    def _fill_imu(self, out: Imu, sample: AttitudeSample) -> None:
        out.available = bool(sample.available)
        out.orientation_available = bool(sample.orientation_available)
        out.roll_rad = float(sample.roll_rad)
        out.pitch_rad = float(sample.pitch_rad)
        out.yaw_rad = float(sample.yaw_rad)
        out.tilt_deg = float(sample.tilt_deg)
        out.ang_vel_x = float(sample.ang_vel_x)
        out.ang_vel_y = float(sample.ang_vel_y)
        out.ang_vel_z = float(sample.ang_vel_z)
        out.lin_acc_x = float(sample.lin_acc_x)
        out.lin_acc_y = float(sample.lin_acc_y)
        out.lin_acc_z = float(sample.lin_acc_z)
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)

    def _fill_scan(self, out: LaserScan, sample: RangeSample) -> None:
        out.available = bool(sample.available)
        out.angle_min = float(sample.angle_min)
        out.angle_max = float(sample.angle_max)
        out.angle_increment = float(sample.angle_increment)
        out.range_min = float(sample.range_min)
        out.range_max = float(sample.range_max)
        # An infinite closest range means "nothing in range"; encode it as the
        # sentinel -1 so downstream comparisons cannot accidentally treat
        # infinity as a tiny distance.
        out.closest_range = (
            float(sample.closest_range)
            if _finite(sample.closest_range)
            else -1.0
        )
        out.front_range = (
            float(sample.front_range) if _finite(sample.front_range) else -1.0
        )
        out.front_angle = float(sample.front_angle)
        out.valid_point_count = int(sample.valid_point_count)
        out.point_count = int(sample.point_count)
        closest = sample.closest_range
        out.no_return = bool(sample.available and sample.valid_point_count == 0)
        out.obstacle_detected = bool(
            sample.available
            and _finite(closest)
            and closest <= self.config.obstacle_warn_range_m
        )
        out.obstacle_critical = bool(
            sample.available
            and _finite(closest)
            and closest <= self.config.obstacle_critical_range_m
        )
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)

    def _fill_joints(self, out: JointState, sample: JointSample) -> None:
        out.available = bool(sample.available)
        out.joint_count = int(sample.joint_count)
        out.moving_joints = [str(name) for name in sample.moving_joints]
        out.moving_joint_count = len(sample.moving_joints)
        out.max_abs_velocity = float(sample.max_abs_velocity)
        out.total_abs_velocity = float(sample.total_abs_velocity)
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)

    def _fill_command(self, out: VelocityCommand, sample: CommandSample) -> None:
        out.available = bool(sample.available)
        out.fresh = bool(sample.fresh)
        out.stop_command = bool(sample.stop_command)
        out.linear_x = float(sample.linear_x)
        out.linear_y = float(sample.linear_y)
        out.linear_z = float(sample.linear_z)
        out.angular_x = float(sample.angular_x)
        out.angular_y = float(sample.angular_y)
        out.angular_z = float(sample.angular_z)
        out.valid = bool(sample.consistency_checked)
        out.consistent = bool(sample.consistency_checked and sample.consistent)
        out.deviation = float(sample.deviation)
        out.last_stamp_sec = float(sample.last_stamp_sec)
        out.last_wall_time_sec = float(sample.last_wall_time_sec)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the monitor until shutdown.

    ``ExternalShutdownException`` is what rclpy raises when the context is shut
    down from outside the spin loop, which is exactly what happens on SIGINT or
    SIGTERM from ``ros2 launch``. Treating it as an expected exit keeps shutdown
    clean instead of printing a traceback that looks like a crash.
    """
    rclpy.init(args=argv)
    node = StatePublisher()
    exit_code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001 - report and fail loudly, but exit cleanly
        node.get_logger().fatal("monitor terminated by an unexpected error")
        traceback.print_exc()
        exit_code = 1
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
