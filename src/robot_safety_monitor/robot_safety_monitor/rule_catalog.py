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

"""Human-readable rule names, taken verbatim from the rule manual.

The manual (``docs/core_detection_rules.md``) names every rule, and an operator
reading a console should see that name rather than an internal code. Names live
in one table here instead of being duplicated into each alert site, so a name
change is a one-line edit and the wording always matches the document the rule
came from.

Completeness is checked by a unit test against the rules this build actually
emits, so a rule cannot be added without a name to show for it.
"""

from __future__ import annotations

from typing import Dict, Optional

# --------------------------------------------------------------------------- #
# Rule names, exactly as written in the manual's section headings
# --------------------------------------------------------------------------- #
RULE_NAMES: Dict[str, str] = {
    # --- MOT: motion safety -------------------------------------------------
    "MOT-001": "Speed Limit Exceeded (Linear)",
    "MOT-002": "Speed Limit Exceeded (Angular)",
    "MOT-003": "Acceleration Limit Exceeded",
    "MOT-004": "Differential Drive Kinematic Violation",
    # --- COL: collision and distance ----------------------------------------
    "COL-001": "Insufficient Stopping Distance",
    "COL-006": "Dynamic Speed Limiting",
    "COL-010": "Sensor Blind Direction",
    "COL-011": "Low Valid Return Ratio",
    # --- SEN: sensor --------------------------------------------------------
    "SEN-001": "Sensor Data Timeout",
    "SEN-011": "Invalid Data Value",
    "SEN-014": "Battery / Power Implausible",
    # --- CMD: command and control -------------------------------------------
    "CMD-005": "Command Authority Conflict",
    "CMD-007": "Command Watchdog Timeout",
    "CMD-012": "Unexpected Motion",
    # --- LOC: localization --------------------------------------------------
    "LOC-001": "Localization Jump",
    "LOC-005": "Localization Timeout",
    # --- SYS: system integrity ----------------------------------------------
    "SYS-003": "Safety State Machine Heartbeat",
    "SYS-005": "Safety Gate Heartbeat Loss",
}

# --------------------------------------------------------------------------- #
# Alert code -> rule
# --------------------------------------------------------------------------- #
# Most motion alerts are named ``MOT_*`` after their rule, but several analyzer
# codes carry no usable rule prefix (``ODOM_STALE``, ``COMMAND_MISMATCH``, ...)
# and have to be mapped explicitly. Entries marked "legacy" are codes this build
# still emits for rules the manual registers as deferred; they are kept so an
# operator never sees an unnamed fault.
CODE_TO_RULE: Dict[str, str] = {
    # MOT-001
    "MOT_LIN_VEL_EXCEED": "MOT-001",
    "MOT_ACTUAL_VEL_EXCEED": "MOT-001",
    # MOT-002
    "MOT_ANG_VEL_EXCEED": "MOT-002",
    "MOT_ACTUAL_ANG_VEL_EXCEED": "MOT-002",
    # MOT-003
    "MOT_LIN_ACCEL_EXCEED": "MOT-003",
    "MOT_ANG_ACCEL_EXCEED": "MOT-003",
    # MOT-004
    "MOT_TWIST_INFEASIBLE": "MOT-004",
    # CMD-007
    "COMMAND_STALE": "CMD-007",
    # SEN-001
    "ODOM_STALE": "SEN-001",
    "ODOM_MISSING": "SEN-001",
    "SCAN_STALE": "SEN-001",
    "SCAN_MISSING": "SEN-001",
    "IMU_STALE": "SEN-001",
    "IMU_MISSING": "SEN-001",
    "JOINTS_STALE": "SEN-001",
    "BATTERY_STALE": "SEN-001",
    # SEN-011
    "POSE_UNCERTAIN": "SEN-011",
    "TWIST_UNCERTAIN": "SEN-011",
    # SEN-014
    "BATTERY_LOW": "SEN-014",
    "BATTERY_CRITICAL": "SEN-014",
    # COL-001
    "OBSTACLE_NEAR": "COL-001",
    "OBSTACLE_CRITICAL": "COL-001",
    # legacy: rules the manual defers, still emitted by this build
    "TILT_HIGH": "MOT-010",
    "TILT_CRITICAL": "MOT-010",
    "COMMAND_MISMATCH": "CMD-011",
    "UNEXPECTED_MOTION": "CMD-012",
}

# Names for the legacy codes above, so they are not left bare when they fire.
LEGACY_RULE_NAMES: Dict[str, str] = {
    "MOT-010": "Tilt Limit Exceeded (deferred rule)",
    "CMD-011": "Command/Execution Mismatch (deferred rule)",
}

# --------------------------------------------------------------------------- #
# Gate reason -> rule
# --------------------------------------------------------------------------- #
GATE_REASON_TO_RULE: Dict[str, str] = {
    "SAFETY_GATE_MOTION_BLOCKED": "",          # rule comes from the alert itself
    "SAFETY_GATE_MOTION_STATUS_BLOCKED": "",
    # The gate losing its input is the heartbeat loss of SYS-005. SYS-004 is the
    # same idea but is not one of the rules in core_detection_rules.md, so the
    # report names the rule the manual actually defines.
    "SAFETY_GATE_MONITOR_STALE": "SYS-005",
    "SAFETY_GATE_NO_MONITOR_STATE": "SYS-005",
    "SAFETY_GATE_COMMAND_STALE": "CMD-007",
    "SAFETY_GATE_NO_COMMAND": "CMD-007",
}


# Chinese names, verbatim from the manual's section headings. The manual is
# written in Chinese, so a Chinese-speaking operator should see the same wording
# the rule is documented under rather than a translation invented at display time.
RULE_NAMES_ZH: Dict[str, str] = {
    "MOT-001": "速度上限（线速度）",
    "MOT-002": "速度上限（角速度）",
    "MOT-003": "加速度上限",
    "MOT-004": "差速运动学约束",
    "COL-001": "停止距离不足",
    "COL-006": "动态限速",
    "COL-010": "传感器盲区",
    "COL-011": "有效回波",
    "SEN-001": "数据超时",
    "SEN-011": "数据非法",
    "SEN-014": "电池与电源合理性",
    "CMD-005": "控制权冲突",
    "CMD-007": "指令看门狗",
    "CMD-012": "异常运动",
    "LOC-001": "定位跳变",
    "LOC-005": "定位超时",
    "SYS-003": "安全状态机 heartbeat",
    "SYS-005": "Safety Gate heartbeat",
}

LEGACY_RULE_NAMES_ZH: Dict[str, str] = {
    "MOT-010": "倾角超限（暂缓项）",
    "CMD-011": "指令与执行不一致（暂缓项）",
}


def rule_name_zh(rule_id: str) -> Optional[str]:
    """Return the manual's Chinese name for ``rule_id``, or ``None``."""
    if not rule_id:
        return None
    return RULE_NAMES_ZH.get(rule_id) or LEGACY_RULE_NAMES_ZH.get(rule_id)


def rule_name(rule_id: str) -> Optional[str]:
    """Return the manual's name for ``rule_id``, or ``None`` if unknown."""
    if not rule_id:
        return None
    return RULE_NAMES.get(rule_id) or LEGACY_RULE_NAMES.get(rule_id)


def rule_for_code(code: str) -> str:
    """Return the manual rule id behind an alert code (``""`` when unknown)."""
    if not code:
        return ""
    return CODE_TO_RULE.get(code, "")


def describe_rule(rule_id: str, fallback_code: str = "") -> str:
    """Render ``[RULE] [Name]`` for an alert, degrading gracefully.

    Never returns an empty string: a fault with no known rule still shows its
    code, because an unnamed alarm is worse than an unfamiliar one.
    """
    resolved = rule_id or rule_for_code(fallback_code)
    name = rule_name(resolved)
    if resolved and name:
        return "[%s] [%s]" % (resolved, name)
    if resolved:
        return "[%s]" % resolved
    if fallback_code:
        return "[%s]" % fallback_code
    return "[UNCLASSIFIED]"
