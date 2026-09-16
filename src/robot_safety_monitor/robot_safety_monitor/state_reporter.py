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

"""Console reader for ``/robot_safety/monitor/state``.

Kept separate from the monitor node on purpose: the monitor must keep running
and publishing even if a human-facing consumer dies or hangs, and an operator
must be able to watch the state without becoming part of the safety path.

Usage::

    ros2 run robot_safety_monitor state_reporter            # continuous
    ros2 run robot_safety_monitor state_reporter --once     # single snapshot
    ros2 run robot_safety_monitor state_reporter --verbose  # full detail
"""

from __future__ import annotations

import sys
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from robot_safety_msgs.msg import RobotState
from robot_safety_monitor.analyzer import status_name

STATUS_WIDTH = 8


def format_summary(msg: RobotState, age: float) -> str:
    """One-line status suitable for a terminal strip or a log line."""
    flags: List[str] = []
    if msg.command.available and msg.command.fresh:
        flags.append("cmd")
    if msg.motion_expected:
        flags.append("moving")
    if msg.scan.obstacle_critical:
        flags.append("obstacle!")
    elif msg.scan.obstacle_detected:
        flags.append("obstacle")
    if msg.imu.available and msg.imu.tilt_deg >= 15.0:
        flags.append("tilt")
    flag_text = (" [" + ",".join(flags) + "]") if flags else ""

    return (
        "state=%-*s pos=(%+.2f,%+.2f) yaw=%+.1fdeg v=%.2fm/s w=%+.2frad/s "
        "front=%.2fm age=%.2fs%s"
        % (
            STATUS_WIDTH,
            status_name(msg.status),
            msg.odom.x,
            msg.odom.y,
            msg.odom.yaw_rad * 57.29577951308232,
            msg.odom.speed_mps,
            msg.odom.yaw_rate_rps,
            msg.scan.front_range,
            age,
            flag_text,
        )
    )


def format_detail(msg: RobotState, wall_now: float) -> str:
    """Multi-line report: identity, per-source health, findings, sample data."""
    lines: List[str] = []
    lines.append("=" * 78)
    lines.append("robot=%s world=%s" % (msg.robot_id, msg.world_name or "-"))
    lines.append(
        "status=%s uptime=%.1fs ticks=%d period=%.3fs wall_age=%.2fs"
        % (
            status_name(msg.status),
            msg.uptime_sec,
            msg.tick_count,
            msg.tick_period_sec,
            max(0.0, wall_now - msg.wall_time_sec),
        )
    )
    lines.append(
        "motion_expected=%s armed=%s sim_stamp=%.3f"
        % (msg.motion_expected, msg.armed, float(msg.header.stamp.sec)
           + float(msg.header.stamp.nanosec) * 1e-9)
    )

    lines.append("-" * 78)
    lines.append("sources:")
    for index, name in enumerate(msg.source_name):
        status = msg.source_status[index] if index < len(msg.source_status) else 0
        reason = msg.source_reason[index] if index < len(msg.source_reason) else ""
        line = "  %-12s %-*s" % (name, STATUS_WIDTH, status_name(status))
        if reason:
            line += "  %s" % reason
        lines.append(line)

    if msg.warnings:
        lines.append("-" * 78)
        lines.append("findings:")
        for index, code in enumerate(msg.warnings):
            severity = (
                msg.warning_severity[index] if index < len(msg.warning_severity) else 0
            )
            detail = msg.warning_detail[index] if index < len(msg.warning_detail) else ""
            lines.append("  [%-*s] %-20s %s" % (STATUS_WIDTH, status_name(severity), code, detail))
    else:
        lines.append("-" * 78)
        lines.append("findings: none")

    lines.append("-" * 78)
    lines.append("sample data:")
    lines.append(
        "  odom     available=%s pose=(%.3f, %.3f, %.3f) yaw=%.3f "
        "twist=(%.3f, %.3f, %.3f) speed=%.3f yaw_rate=%.3f cov=(%.2g, %.2g, %.2g)"
        % (
            msg.odom.available,
            msg.odom.x, msg.odom.y, msg.odom.z, msg.odom.yaw_rad,
            msg.odom.vx, msg.odom.vy, msg.odom.vz,
            msg.odom.speed_mps, msg.odom.yaw_rate_rps,
            msg.odom.pose_cov_xx, msg.odom.pose_cov_yy, msg.odom.pose_cov_yawyaw,
        )
    )
    lines.append(
        "  scan     valid=%d/%d closest=%.3f front=%.3f (%.1fdeg) limits=[%.2f, %.2f]"
        % (
            msg.scan.valid_point_count, msg.scan.point_count,
            msg.scan.closest_range, msg.scan.front_range,
            msg.scan.front_angle * 57.29577951308232,
            msg.scan.range_min, msg.scan.range_max,
        )
    )
    lines.append(
        "  imu      orientation=%s roll=%.1fdeg pitch=%.1fdeg tilt=%.2fdeg "
        "gyro=(%.3f, %.3f, %.3f) acc=(%.3f, %.3f, %.3f)"
        % (
            msg.imu.orientation_available,
            msg.imu.roll_rad * 57.29577951308232,
            msg.imu.pitch_rad * 57.29577951308232,
            msg.imu.tilt_deg,
            msg.imu.ang_vel_x, msg.imu.ang_vel_y, msg.imu.ang_vel_z,
            msg.imu.lin_acc_x, msg.imu.lin_acc_y, msg.imu.lin_acc_z,
        )
    )
    lines.append(
        "  joints   count=%d moving=%d (%s) max|v|=%.4f"
        % (
            msg.joints.joint_count, msg.joints.moving_joint_count,
            ",".join(msg.joints.moving_joints) or "-",
            msg.joints.max_abs_velocity,
        )
    )
    lines.append(
        "  command  available=%s fresh=%s stop=%s linear=(%.3f, %.3f, %.3f) "
        "angular=(%.3f, %.3f, %.3f) valid=%s consistent=%s deviation=%.3f"
        % (
            msg.command.available, msg.command.fresh, msg.command.stop_command,
            msg.command.linear_x, msg.command.linear_y, msg.command.linear_z,
            msg.command.angular_x, msg.command.angular_y, msg.command.angular_z,
            msg.command.valid, msg.command.consistent, msg.command.deviation,
        )
    )
    lines.append(
        "  battery  available=%s voltage=%.2fV percent=%.1f%% charging=%s"
        % (
            msg.battery.available, msg.battery.voltage_v,
            msg.battery.percentage * 100.0, msg.battery.charging,
        )
    )
    return "\n".join(lines)


class StateReporter(Node):
    """Prints monitor snapshots to stdout."""

    def __init__(self, once: bool = False, verbose: bool = False) -> None:
        super().__init__("robot_safety_state_reporter")
        self.once = bool(once)
        self.verbose = bool(verbose)
        self.received = 0
        self.last_status: Optional[int] = None

        topic = str(
            self.declare_parameter("topic", "/robot_safety/monitor/state").value
        )
        summary_period = float(
            self.declare_parameter("summary_period_sec", 1.0).value
        )
        self.summary_period = max(0.0, summary_period)
        self.last_summary_wall = 0.0

        self.subscription = self.create_subscription(
            RobotState,
            topic,
            self.on_state,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.get_logger().info("watching %s" % topic)

    def on_state(self, msg: RobotState) -> None:
        wall_now = time.time()
        self.received += 1

        # A status transition always prints in full; that is the event an
        # operator actually cares about.
        transition = msg.status != self.last_status
        if self.verbose or transition:
            print(format_detail(msg, wall_now), flush=True)
            self.last_status = msg.status
            self.last_summary_wall = wall_now
        elif self.summary_period > 0.0 and (
            wall_now - self.last_summary_wall >= self.summary_period
        ):
            print(format_summary(msg, max(0.0, wall_now - msg.wall_time_sec)), flush=True)
            self.last_summary_wall = wall_now

        if self.once:
            raise SystemExit(0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    once = "--once" in args
    verbose = "--verbose" in args or "-v" in args
    filtered = [a for a in args if a not in ("--once", "--verbose", "-v")]

    rclpy.init(args=[sys.argv[0]] + filtered if filtered else None)
    node = StateReporter(once=once, verbose=verbose)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        # ExternalShutdownException is the normal exit path when the context is
        # shut down from outside the spin loop (SIGINT/SIGTERM).
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
