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

"""Safety gate node: the sole arbiter of the velocity command stream.

Architecture (RSS-001 §3.1, layer 3). The monitor identifies a violation; this
node prevents it from reaching the motors. It is a separate process on purpose --
folding enforcement into the monitor would mean one crash removes both the
detection and the protection, violating INV-1.

Behaviour:

* passes the incoming command unchanged while every condition is clear,
* publishes an explicit zero command (not silence) when it refuses, because
  publishing nothing leaves the base holding its last velocity,
* stops defensively if the monitor stops reporting, or if the command source
  goes quiet, so the base never executes a stale command open loop.

The gate keeps publishing ``SafetyGateState`` at a fixed rate whether it is
passing or refusing. That stream is the heartbeat required by RSS-003 SYS-005:
its absence is what tells a downstream watchdog that the gate itself has stalled.

Wiring note. The gate reads ``/cmd_vel_cmd`` and drives ``/cmd_vel``, which is
where the robot's drive plugin listens. The two must differ -- see the topic
wiring block in :meth:`__init__` for why an in-place rewrite cannot work -- so
command sources must publish to ``/cmd_vel_cmd``. The stock TurtleBot3 teleop can
be pointed there with ``-r cmd_vel:=cmd_vel_cmd``.
"""

from __future__ import annotations

import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock as ClockSource, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from robot_safety_msgs.msg import RobotState, SafetyGateState

from .gate_policy import (
    GATE_PASS,
    STATE_NAMES,
    GateConfig,
    GatePolicy,
)

SOFTWARE_VERSION = "0.1.0"

# Commands are streamed and latency-sensitive: keep the newest, tolerate loss.
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return 0.0


class SafetyGateNode(Node):
    """Refuses unsafe velocity commands and reports why."""

    def __init__(self) -> None:
        super().__init__("robot_safety_gate")

        # -- parameters ----------------------------------------------------
        # ------------------------------------------------------------------ #
        # Topic wiring
        # ------------------------------------------------------------------ #
        # The robot's drive plugin takes its command topic from the SDF and
        # cannot be remapped from the command line, so the gate has to publish
        # where the robot already listens. It therefore cannot also read that
        # topic: it would receive its own filtered output, latch on the stop it
        # just published, and no real command would ever get through.
        #
        # Hence three distinct roles:
        #   topic.cmd_vel_in          where commands come from (teleop, planner)
        #   topic.actuator_cmd_vel    where the robot listens (gated output)
        #   topic.cmd_vel_out         optional mirror of the gated stream, for
        #                             logging or an external monitor
        #
        # topic.actuator_cmd_vel MUST differ from topic.cmd_vel_in; the gate
        # refuses to start otherwise rather than silently becoming a pass-through.
        gate_id = str(self.declare_parameter("gate_id", "safety_gate").value)
        monitor_topic = str(
            self.declare_parameter(
                "topic.monitor_state", "/robot_safety/monitor/state"
            ).value
        )
        monitor_timeout = float(
            self.declare_parameter("gate.monitor_timeout_sec", 0.20).value
        )
        command_timeout = float(
            self.declare_parameter("gate.command_timeout_sec", 0.50).value
        )
        block_level = int(self.declare_parameter("gate.block_level", 2).value)
        block_on_motion_status = bool(
            self.declare_parameter("gate.block_on_motion_status", True).value
        )
        pass_clear_frames = int(
            self.declare_parameter("gate.pass_clear_frames", 3).value
        )
        publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 20.0).value
        )
        publish_rate_hz = max(1.0, min(100.0, publish_rate_hz))

        # The gate reads AND writes the topic the robot listens to. Reading a
        # separate intent topic was safer in principle but failed in practice:
        # anything published straight to /cmd_vel -- which is what a teleop, a
        # script or a test naturally does -- bypassed the interlock entirely,
        # and a real 2.9 rad/s command drove the base with no alert raised.
        # Sitting directly in the path is what makes the gate authoritative.
        #
        # The cost is self-echo: the gate hears the stop it just published. It is
        # filtered by value and time in _on_command, because an unfiltered echo
        # resets the cached command to zero and no real instruction can get in.
        actuator_topic = str(
            self.declare_parameter("topic.actuator_cmd_vel", "/cmd_vel").value
        )
        input_topic = actuator_topic
        # Audit mirror is opt-in; an empty value disables it.
        audit_topic = str(
            self.declare_parameter("topic.cmd_vel_audit", "").value
        )

        self.gate_id = gate_id
        self._monitor_topic = monitor_topic
        self.publish_rate_hz = publish_rate_hz
        self.config = GateConfig(
            monitor_timeout_sec=monitor_timeout,
            command_timeout_sec=command_timeout,
            block_level=block_level,
            block_on_motion_status=block_on_motion_status,
            pass_clear_frames=pass_clear_frames,
        )
        self.policy = GatePolicy(self.config)

        # Cached verdicts from the newest monitor snapshot. The command and the
        # measured twist come from the monitor's own snapshot because the raw
        # /cmd_vel topic cannot be subscribed to here (see the wiring note).
        self._motion_alerts: Tuple[Tuple[str, str, int], ...] = ()
        self._motion_status = 0
        self._monitor_command: Optional[Tuple[float, float]] = None
        self._monitor_measured: Optional[Tuple[float, float]] = None
        # Last command this gate published, for echo suppression.
        self._last_published: Optional[Tuple[float, float]] = None
        self._last_published_wall = 0.0

        # -- interfaces ----------------------------------------------------
        # QoS must match the drive plugin's subscriber: RELIABLE/volatile/depth 1
        # is what the stock TurtleBot3 plugin subscribes with, and a best-effort
        # publisher would be incompatible and silently deliver nothing.
        ACTUATOR_QOS = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._actuator_topic = actuator_topic
        self.publisher = self.create_publisher(Twist, actuator_topic, ACTUATOR_QOS)
        self._audit_topic = audit_topic
        self.audit_publisher = None
        if audit_topic:
            self.audit_publisher = self.create_publisher(
                Twist, audit_topic, QoSProfile(depth=10)
            )
        self.report_publisher = self.create_publisher(
            SafetyGateState, "robot_safety/gate/state", QoSProfile(depth=10)
        )
        self.create_subscription(
            Twist, input_topic, self._on_command, COMMAND_QOS
        )
        self.create_subscription(
            RobotState, monitor_topic, self._on_monitor_state, QoSProfile(depth=10)
        )

        # The loop runs on wall time even under a simulated clock: a gate that
        # stops ticking when the simulator pauses is a gate that stops protecting
        # exactly when the robot's state becomes unknown.
        wall_clock = ClockSource(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self.tick, clock=wall_clock
        )

        # If someone else is already publishing to the output topic, commands can
        # reach the motors while bypassing this gate. Say so loudly at startup
        # rather than letting the operator assume the gate is authoritative.
        self._bypass_check_timer = self.create_timer(
            2.0, self._check_bypass, clock=wall_clock
        )
        self._warned_bypass = False

        # A drive plugin holds the last command it received indefinitely, so a
        # previous gate that died without stopping leaves the robot moving. Clear
        # the actuator immediately on startup rather than waiting for the first
        # timer tick, and say so, because a robot that starts moving on its own
        # is indistinguishable from a real runaway if nobody knows why.
        self._publish_stop(reason="gate startup: clearing any latched command")

        self.get_logger().info(
            "safety gate up | in-place on %s | audit %s | monitor %s | "
            "block at S%d | command idle stop after %.2fs"
            % (
                actuator_topic, audit_topic or "(off)",
                monitor_topic, block_level, command_timeout,
            )
        )

    def _publish_stop(self, reason: str = "") -> None:
        """Send an explicit zero command to every output this gate owns."""
        stop = Twist()
        try:
            self._publish(stop)
        except Exception:  # noqa: BLE001 - a shutdown path must not raise
            return
        if reason:
            self.get_logger().info(reason)

    def _check_bypass(self) -> None:
        """Warn once if another publisher shares the output topic.

        The gate can only guarantee commands it sees. A second publisher on the
        same topic (a stray teleop, a leftover test publisher) can drive the robot
        around this node, and the operator must know that the interlock is not
        authoritative rather than assume it is.
        """
        if self._warned_bypass:
            return
        try:
            publishers = self.count_publishers(self._output_topic)
        except Exception:  # noqa: BLE001 - introspection must not kill the gate
            return
        if publishers > 1:
            self._warned_bypass = True
            self.get_logger().warn(
                "BYPASS POSSIBLE: %d publishers on %s; this gate can only "
                "restrain commands it receives. Stop other publishers for the "
                "interlock to be authoritative."
                % (publishers, self._actuator_topic)
            )

    # -- inputs ------------------------------------------------------------
    def _publish(self, command: Twist) -> None:
        """Publish to the actuator topic, remembering it so the echo is ignored."""
        self.publisher.publish(command)
        self._last_published = (float(command.linear.x), float(command.angular.z))
        self._last_published_wall = time.time()
        if self.audit_publisher is not None:
            self.audit_publisher.publish(command)

    def _on_command(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        now = time.time()

        # Ignore our own output coming back. Both the value and a tight time
        # window must match: value alone would swallow a genuine identical
        # command, and time alone would swallow anything sent right after a tick.
        last = self._last_published
        if (
            last is not None
            and linear == last[0]
            and angular == last[1]
            and (now - self._last_published_wall) <= self.config.echo_window_sec
        ):
            return

        self.policy.note_command(linear, angular, now)

    def _on_monitor_state(self, msg: RobotState) -> None:
        self.policy.note_monitor(time.time())
        self._motion_alerts = tuple(
            (alert.code, alert.rule_id, int(alert.level))
            for alert in getattr(msg, "motion_alerts", [])
        )
        self._motion_status = int(getattr(msg, "motion_status", 0))
        # Only a *fresh* command may be judged. The monitor keeps publishing the
        # last value it saw even after it expires, which is right for a snapshot
        # ("here is the most recent command") but wrong for an interlock: judging
        # a stale value produced a stale MOT_ANG_VEL_EXCEED that kept the gate
        # blocked after the offending command had already stopped -- verified
        # live, where a cached 2.9 rad/s from a previous test blocked a later
        # compliant run. No fresh command means nothing to judge.
        if msg.command.available and msg.command.fresh:
            self._monitor_command = (
                float(msg.command.linear_x), float(msg.command.angular_z)
            )
        else:
            self._monitor_command = None
        if msg.odom.available:
            self._monitor_measured = (
                float(msg.odom.speed_mps), float(msg.odom.yaw_rate_rps)
            )
        else:
            self._monitor_measured = None
        # Source-health findings are gated too: an S2-or-worse data problem means
        # the monitor cannot vouch for the robot, so motion is refused while it
        # persists. This is what makes the gate protective against the analyzer's
        # existing COMMAND_MISMATCH / UNEXPECTED_MOTION findings as well.
        for index, code in enumerate(msg.warnings):
            severity = (
                msg.warning_severity[index]
                if index < len(msg.warning_severity) else 0
            )
            self._motion_alerts = self._motion_alerts + ((code, "", int(severity)),)

    # -- main loop ---------------------------------------------------------
    def tick(self) -> None:
        wall = time.time()
        # Only judge a command while the *gate* still considers the intent stream
        # live. The monitor keeps publishing the last command it saw after that
        # command has expired, which is correct for a snapshot but wrong for an
        # interlock: judging it produces a phantom MOT violation that keeps the
        # gate reporting "motion blocked" long after the command stopped, and
        # misdescribes a normal idle as a safety fault.
        intent_live = (
            self.policy.last_command_wall is not None
            and self.policy.command_age(wall) <= self.config.command_timeout_sec
        )
        # Judge the command THIS gate received, not the one the monitor reports.
        # The monitor reads the same topic the gate writes, so after the first
        # tick its "command" is the gate's own gated output -- often the stop the
        # gate just published -- and using it made the gate compare zero against
        # zero, find no violation, and (before that) refuse a perfectly legal
        # command. The gate is the authority on what was requested; the monitor
        # is the authority on what the robot actually did.
        command_pair = (
            (self.policy.last_command_linear, self.policy.last_command_angular)
            if intent_live else None
        )
        measured_pair = self._monitor_measured if intent_live else None
        decision = self.policy.decide(
            now_wall=wall,
            motion_alerts=self._motion_alerts,
            motion_status=self._motion_status,
            command_linear=command_pair[0] if command_pair else None,
            command_angular=command_pair[1] if command_pair else None,
            measured_linear=measured_pair[0] if measured_pair else None,
            measured_angular=measured_pair[1] if measured_pair else None,
        )

        command = Twist()
        command.linear.x = float(decision.output_linear)
        command.angular.z = float(decision.output_angular)
        self._publish(command)

        self._publish_report(decision, wall, command)

    # -- reporting ---------------------------------------------------------
    def _publish_report(self, decision, wall: float, command: Twist) -> None:
        report = SafetyGateState()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = "base_footprint"
        report.gate_id = self.gate_id
        report.state = int(decision.state)
        report.state_name = STATE_NAMES.get(int(decision.state), "UNKNOWN")
        report.input_linear = float(decision.input_linear)
        report.input_angular = float(decision.input_angular)
        report.output_linear = float(decision.output_linear)
        report.output_angular = float(decision.output_angular)
        report.intercepted = bool(decision.intercepted)
        report.reason = decision.reason
        report.detail = decision.detail
        report.blocked_codes = list(decision.blocked_codes)
        report.watchdog_tripped = bool(decision.watchdog_tripped)
        report.monitor_age_sec = float(self.policy.monitor_age(wall))
        report.command_age_sec = float(self.policy.command_age(wall))
        report.pass_count = int(self.policy.passes)
        report.block_count = int(self.policy.blocks)
        report.stamp_sec = float(_stamp_to_sec(report.header.stamp))
        report.wall_time_sec = float(wall)
        self.report_publisher.publish(report)


def main(argv: Optional[Sequence[str]] = None) -> int:
    rclpy.init(args=argv)
    node = SafetyGateNode()
    exit_code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Last act before dying: publish one explicit stop so a base that was
        # mid-command does not keep executing it after the gate disappears.
        try:
            node.policy.note_command(0.0, 0.0, time.time())
            stop = Twist()
            node.publisher.publish(stop)
        except Exception:  # noqa: BLE001 - shutdown must not raise
            pass
        try:
            node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
