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

import os
import sys
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from robot_safety_msgs.msg import RobotState
from robot_safety_monitor.analyzer import CRITICAL, ERROR, status_name
from robot_safety_monitor.motion_safety import LEVEL_NAMES
from robot_safety_monitor.rule_catalog import describe_rule, rule_name_zh

STATUS_WIDTH = 8

# --------------------------------------------------------------------------- #
# Colour
# --------------------------------------------------------------------------- #
# ANSI colours, enabled only when stdout is a terminal. A status stream is
# routinely redirected to a log file, and escape codes in a log are noise that
# breaks grep; disabling automatically is safer than trusting the caller to pass
# --no-color. Set ROBOT_SAFETY_COLOR=always/never to override either way.
ANSI_RESET = "\033[0m"
ANSI_BOLD = "\033[1m"
ANSI_RED = "\033[31m"
ANSI_YELLOW = "\033[33m"

_COLOR_DECISION = None


def color_enabled() -> bool:
    global _COLOR_DECISION
    if _COLOR_DECISION is None:
        override = os.environ.get("ROBOT_SAFETY_COLOR", "").strip().lower()
        if override in ("always", "1", "yes", "true"):
            _COLOR_DECISION = True
        elif override in ("never", "0", "no", "false"):
            _COLOR_DECISION = False
        else:
            _COLOR_DECISION = sys.stdout.isatty()
    return _COLOR_DECISION


def paint(text: str, code: str) -> str:
    """Wrap ``text`` in ``code`` when colour is on."""
    if not color_enabled() or not code:
        return text
    return "%s%s%s" % (code, text, ANSI_RESET)


def severity_color(severity: int) -> str:
    """Red for anything that must stop the robot; nothing for a mere notice.

    Both CRITICAL and ERROR mean the robot should not be moving (RSS-001 §4.3:
    S3/S4 are stops, and this build emits its motion violations at S2 which also
    carries a refusal), so both are red. CRITICAL additionally goes bold, because
    "needs a human to reset" and "recoverable" are worth telling apart at a
    glance. A notice (S1) is deliberately left uncoloured: colour that appears on
    everything stops meaning anything.
    """
    if int(severity) >= CRITICAL:
        return ANSI_BOLD + ANSI_RED
    if int(severity) >= ERROR:
        return ANSI_RED
    return ""


def format_nrc(msg: RobotState, limit: int = 4) -> str:
    """Render active fault codes as a compact UDS-style NRC bracket.

    The full detail block is for reading; this is for watching. Like a UDS
    diagnostic trouble-code list it carries only the codes, one token each, so a
    status strip stays one line no matter how many faults are active. Severity is
    deliberately not repeated here -- ``state=`` at the head of the line already
    states the aggregate urgency, and a second severity marker per code would
    triple the width for no new information.

    An alert storm is capped at ``limit`` codes with a ``+N`` tail: the point of
    the line is to make the fault visible, and an unbounded list would defeat
    that by wrapping.
    """
    codes: List[str] = []
    for alert in getattr(msg, "motion_alerts", []):
        if alert.code:
            codes.append(alert.code)
    for code in msg.warnings:
        if code:
            codes.append(code)

    # Deduplicate while preserving the order above (motion-safety first).
    unique = list(dict.fromkeys(codes))
    if not unique:
        return ""

    shown = unique[:limit]
    text = " ".join(shown)
    if len(unique) > limit:
        text += " +%d" % (len(unique) - limit)
    return " [%s]" % text


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
        "front=%.2fm age=%.2fs%s%s"
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
            format_nrc(msg),
        )
    )


def format_codes_line(msg: RobotState) -> str:
    """Minimal watch line: tick, status, then the active fault codes.

    This is the UDS-style diagnostic view -- what a technician wants while
    injecting faults, where the robot's geometry is not the subject. The tick
    counter is used instead of a clock because it is what makes two consecutive
    lines comparable when the point is to see a code appear and clear.
    """
    return "tick=%-6d %-*s%s" % (
        int(msg.tick_count), STATUS_WIDTH, status_name(msg.status), format_nrc(msg)
    )


def spec_level_for_severity(severity: int) -> str:
    """Best-effort spec level for an analyzer finding.

    The motion rules carry a precise S1..S4 level because the specification
    assigns one. The analyzer's findings were written before that vocabulary
    existed, so their level is derived from the severity they already carry.
    This is an honest approximation and is labelled as such by the caller.
    """
    if severity >= CRITICAL:
        return "S3"
    if severity >= ERROR:
        return "S2"
    return "S1"


def collect_faults(msg: RobotState):
    """Return active faults as ``(severity, level, category, rule_id, code, detail)``.

    The monitor carries faults in two places: ``warnings`` for data-health and
    plausibility findings, and ``motion_alerts`` for motion-safety rule
    violations. A report should not make the reader care which shelf a fault was
    stored on, so both are flattened here into one list and sorted by urgency.
    """
    faults = []

    categories = list(getattr(msg, "warning_category", []))
    rules = list(getattr(msg, "warning_rule_id", []))
    for index, code in enumerate(msg.warnings):
        severity = msg.warning_severity[index] if index < len(msg.warning_severity) else 0
        detail = msg.warning_detail[index] if index < len(msg.warning_detail) else ""
        category = categories[index] if index < len(categories) else ""
        rule_id = rules[index] if index < len(rules) else ""
        faults.append(
            (
                int(severity),
                spec_level_for_severity(int(severity)),
                category,
                rule_id,
                code,
                detail,
                None,
                None,
            )
        )

    for alert in getattr(msg, "motion_alerts", []):
        faults.append(
            (
                int(alert.severity),
                LEVEL_NAMES.get(int(alert.level), "S?"),
                getattr(alert, "category", "") or "MOT",
                alert.rule_id,
                alert.code,
                alert.detail,
                float(alert.value),
                float(alert.threshold),
            )
        )

    faults.sort(key=lambda row: (-row[0], row[2], row[4]))
    return faults


def fault_evidence(row) -> str:
    """Condition part of a fault line: the code, then the observed value.

    Rendered as ``CODE: <value> vs limit <limit>`` when the alert carried
    numbers, which mirrors how the manual writes a condition (a stable token plus
    the observed and allowed values). The value is formatted at a fixed three
    decimals rather than with ``%g``: ``%g`` turns a limit of 2.0 into "2", and a
    status line that mixes "2" and "2.85" reads like two different quantities.
    """
    _severity, _level, _category, _rule, code, detail, value, threshold = row
    if value is not None and threshold is not None:
        return "%s: %.3f vs limit %.3f" % (code, value, threshold)
    return "%s: %s" % (code, detail)


def format_fault_lines(msg: RobotState, threshold: int = ERROR) -> List[str]:
    """One line per active fault, in the manual's naming style, coloured.

    These are printed on *every* report while the fault is active, not only when
    the fault set changes: a fault that scrolls past once and never returns is a
    fault an operator will miss. Colour carries the urgency so the line itself
    can stay compact.
    """
    lines: List[str] = []
    for row in collect_faults(msg):
        severity, level, _category, rule_id, code, _detail, _v, _t = row
        if int(severity) < threshold:
            continue
        if int(severity) >= CRITICAL:
            label = "critical"
        else:
            label = "error"
        # Both names are shown: the manual documents each rule in Chinese and
        # gives the English identifier used in code and logs, and an operator
        # needs to be able to match the console line to either.
        rule_text = describe_rule(rule_id, code)
        zh = rule_name_zh(rule_id or "")
        if zh:
            rule_text = "%s %s" % (rule_text, zh)
        rendered = "[%s] %s %s" % (label, rule_text, fault_evidence(row))
        lines.append(paint(rendered, severity_color(int(severity))))
    return lines


def format_fault_section(msg: RobotState) -> List[str]:
    """Render the dedicated fault-code block.

    Every fault is shown with four things an operator needs in order to act:
    *what class* of problem it is (category), *which rule* was violated, *how
    urgent* it is (severity), and *the evidence*. The count by category is
    summarised first so the shape of the problem is visible at a glance without
    reading every row.
    """
    faults = collect_faults(msg)
    lines: List[str] = ["-" * 78]

    if not faults:
        lines.append("fault codes: none")
        return lines

    # Severity tallies. Levels are named by the worst fault present.
    by_severity: dict = {}
    by_category: dict = {}
    for severity, _level, category, _rule, _code, _detail, _v, _t in faults:
        by_severity[severity] = by_severity.get(severity, 0) + 1
        by_category[category or "?"] = by_category.get(category or "?", 0) + 1

    severity_text = " ".join(
        "%s=%d" % (status_name(level), by_severity[level])
        for level in sorted(by_severity, reverse=True)
    )
    category_text = " ".join(
        "%s=%d" % (name, by_category[name]) for name in sorted(by_category)
    )

    lines.append(
        "fault codes: %d active  (%s)" % (len(faults), severity_text)
    )
    lines.append(
        "categories:  %s   [MOT motion | COL collision | SEN sensor | "
        "CMD command | LOC localization | COM comms | SYS system]"
        % category_text
    )
    lines.append("")
    lines.append(
        "  %-9s %-8s %-28s %-9s %s"
        % ("SEVERITY", "CATEGORY", "RULE / CODE", "LEVEL", "EVIDENCE")
    )
    for severity, level, category, rule_id, code, detail, _v, _t in faults:
        rule_text = ("%s %s" % (rule_id, code)) if rule_id else code
        lines.append(
            "  %-9s %-8s %-28s %-9s %s"
            % (status_name(severity), category or "?", rule_text, level, detail)
        )

    # Where the fault came from still matters for triage, so say it plainly
    # rather than leaving the reader to infer it from the array it sat in.
    motion_count = len(getattr(msg, "motion_alerts", []))
    finding_count = len(msg.warnings)
    lines.append("")
    lines.append(
        "  origin:       %d from motion-safety rules, %d from data/plausibility checks"
        % (motion_count, finding_count)
    )
    lines.append(
        "  motion rules evaluated this tick: %d"
        % int(getattr(msg, "motion_rules_evaluated", 0))
    )
    return lines


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

    # Motion-safety violations (RSS-001 §6). Printed separately from the source
    # findings because they answer a different question: the findings say whether
    # the data can be trusted, these say whether the motion violates a rule.
    motion_status = int(getattr(msg, "motion_status", 0))
    lines.append(
        "motion_status=%s rules_evaluated=%d alerts=%d"
        % (status_name(motion_status),
           int(getattr(msg, "motion_rules_evaluated", 0)),
           len(getattr(msg, "motion_alerts", [])))
    )
    for alert in getattr(msg, "motion_alerts", []):
        lines.append(
            "  [S%d/%-8s] %-28s %s"
            % (
                int(alert.level),
                status_name(int(alert.severity)),
                "%s (%s)" % (alert.code, alert.rule_id),
                alert.detail,
            )
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

    lines.extend(format_fault_section(msg))

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

    def __init__(self, once: bool = False, verbose: bool = False,
                 codes_only: bool = False) -> None:
        super().__init__("robot_safety_state_reporter")
        self.once = bool(once)
        self.verbose = bool(verbose)
        self.codes_only = bool(codes_only)
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
        self.last_fault_wall = 0.0

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
        # Transition is judged on the aggregate status only. Including the alert
        # set here looked helpful but was not: an alert such as
        # MOT_LIN_ACCEL_EXCEED is present in exactly one snapshot, so every
        # flicker printed a full detail block and drowned the output. Active
        # codes are reported continuously by the NRC tail on the summary line
        # instead, and -v remains available when the full block is wanted.
        transition = msg.status != self.last_status

        # The watch mode is throttled like the summary line. Without this it
        # printed once per snapshot (10 Hz), which is a flood rather than a
        # status strip, and a flood is how real alarms get scrolled away.
        codes_due = self.summary_period <= 0.0 or (
            wall_now - self.last_summary_wall >= self.summary_period
        )

        # Fault annunciation is throttled to the same cadence as the summary.
        # Printing it per message looked correct in a 2-second test and produced
        # 228 lines in 12 seconds in practice, which is the alarm fatigue problem
        # this project explicitly sets out to avoid (RSS-003 SEN-006). A fault is
        # repeated once per period until it clears: often enough to be seen,
        # rarely enough to stay readable.
        announce_faults = transition or codes_due

        if self.codes_only:
            if transition or codes_due:
                print(format_codes_line(msg), flush=True)
                self.last_summary_wall = wall_now
                self.last_status = msg.status
        elif self.verbose or transition:
            print(format_detail(msg, wall_now), flush=True)
            self.last_status = msg.status
            self.last_summary_wall = wall_now
        elif self.summary_period > 0.0 and (
            wall_now - self.last_summary_wall >= self.summary_period
        ):
            print(format_summary(msg, max(0.0, wall_now - msg.wall_time_sec)), flush=True)
            self.last_summary_wall = wall_now

        if announce_faults:
            for line in format_fault_lines(msg):
                print(line, flush=True)
            self.last_fault_wall = wall_now

        if self.once:
            raise SystemExit(0)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    once = "--once" in args
    verbose = "--verbose" in args or "-v" in args
    codes_only = "--codes" in args
    if "--no-color" in args:
        os.environ["ROBOT_SAFETY_COLOR"] = "never"
    filtered = [
        a for a in args
        if a not in ("--once", "--verbose", "-v", "--codes", "--no-color")
    ]

    rclpy.init(args=[sys.argv[0]] + filtered if filtered else None)
    node = StateReporter(once=once, verbose=verbose, codes_only=codes_only)
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
