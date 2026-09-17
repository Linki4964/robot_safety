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

"""Tests for rule naming and the console fault lines.

An operator acts on what the console says, so a fault that appears with the
wrong name, or with no name at all, is a defect even when the judgement behind it
was correct.
"""

import pytest

from robot_safety_monitor import analyzer as az
from robot_safety_monitor import motion_safety as ms
from robot_safety_monitor import rule_catalog as rc


class TestRuleNames:
    def test_motion_rules_use_the_manual_names(self):
        assert rc.rule_name("MOT-001") == "Speed Limit Exceeded (Linear)"
        assert rc.rule_name("MOT-002") == "Speed Limit Exceeded (Angular)"
        assert rc.rule_name("MOT-003") == "Acceleration Limit Exceeded"
        assert rc.rule_name("MOT-004") == "Differential Drive Kinematic Violation"

    def test_unknown_rule_has_no_name(self):
        assert rc.rule_name("MOT-999") is None
        assert rc.rule_name("") is None

    def test_describe_rule_renders_rule_and_name(self):
        assert rc.describe_rule("MOT-002") == (
            "[MOT-002] [Speed Limit Exceeded (Angular)]"
        )

    def test_describe_rule_never_returns_empty(self):
        # An unnamed alarm is worse than an unfamiliar one, so every path must
        # produce something an operator can read.
        assert rc.describe_rule("MOT-002", "") != ""
        assert rc.describe_rule("", "SOMETHING_ODD") == "[SOMETHING_ODD]"
        assert rc.describe_rule("", "") == "[UNCLASSIFIED]"

    def test_describe_rule_resolves_legacy_codes(self):
        assert "Command/Execution Mismatch" in rc.describe_rule("", "COMMAND_MISMATCH")
        assert "Tilt Limit Exceeded" in rc.describe_rule("", "TILT_CRITICAL")


class TestCoverageGuards:
    """Every code this build can emit must resolve to a name."""

    def test_every_analyzer_code_has_a_rule_and_name(self):
        codes = [
            value for name, value in vars(az).items()
            if name.startswith("CODE_") and isinstance(value, str)
        ]
        assert codes, "code discovery is broken"
        for code in codes:
            rule = rc.rule_for_code(code)
            assert rule, "%s has no rule mapping" % code
            assert rc.rule_name(rule), "%s (rule %s) has no name" % (code, rule)

    def test_every_motion_alert_code_has_a_rule_and_name(self):
        codes = [
            value for name, value in vars(ms).items()
            if name.startswith("CODE_") and isinstance(value, str)
        ]
        assert codes, "code discovery is broken"
        for code in codes:
            rule = rc.rule_for_code(code)
            assert rule, "%s has no rule mapping" % code
            assert rc.rule_name(rule), "%s (rule %s) has no name" % (code, rule)

    def test_gate_reasons_that_map_to_a_rule_have_names(self):
        for reason, rule in rc.GATE_REASON_TO_RULE.items():
            if not rule:
                continue  # rule comes from the alert itself
            assert rc.rule_name(rule), "%s -> %s has no name" % (reason, rule)


# --------------------------------------------------------------------------- #
# Console rendering. Needs real messages, so it skips without a built workspace.
# --------------------------------------------------------------------------- #
pytest.importorskip("robot_safety_msgs.msg")
from robot_safety_msgs.msg import MotionAlert, RobotState  # noqa: E402

from robot_safety_monitor import state_reporter as sr  # noqa: E402


def make_state(motion_alerts=(), warnings=(), warnings_severity=(), rule_ids=()):
    msg = RobotState()
    for code, rule_id, level, severity in motion_alerts:
        alert = MotionAlert()
        alert.code = code
        alert.rule_id = rule_id
        alert.category = "MOT"
        alert.level = level
        alert.severity = severity
        alert.value = 2.85
        alert.threshold = 2.00
        alert.detail = "commanded angular speed 2.850 rad/s exceeds 2.00"
        msg.motion_alerts.append(alert)
    for index, code in enumerate(warnings):
        msg.warnings.append(code)
        msg.warning_severity.append(
            warnings_severity[index] if index < len(warnings_severity) else 3
        )
        msg.warning_detail.append("detail for %s" % code)
        msg.warning_rule_id.append(rule_ids[index] if index < len(rule_ids) else "")
        msg.warning_category.append("SEN")
    return msg


class TestFaultLines:
    def setup_method(self):
        # Force colour off so assertions compare plain text.
        sr._COLOR_DECISION = False

    def test_line_follows_the_manual_naming_style(self):
        msg = make_state(motion_alerts=[
            ("MOT_ANG_VEL_EXCEED", "MOT-002", 2, az.ERROR),
        ])
        lines = sr.format_fault_lines(msg)
        assert len(lines) == 1
        line = lines[0]
        assert line.startswith("[error]")
        assert "[MOT-002]" in line
        assert "[Speed Limit Exceeded (Angular)]" in line
        assert "MOT_ANG_VEL_EXCEED" in line
        # Fixed three-decimal formatting, so a limit of 2.0 does not render as "2".
        assert "2.850" in line and "2.000" in line
        assert "vs limit" in line

    def test_critical_uses_the_critical_label(self):
        msg = make_state(motion_alerts=[("MOT_ACTUAL_VEL_EXCEED", "MOT-001", 4, az.CRITICAL)])
        assert sr.format_fault_lines(msg)[0].startswith("[critical]")

    def test_notices_are_not_printed_by_default(self):
        msg = make_state(motion_alerts=[("MOT_JERK_HIGH", "MOT-005", 1, az.UNKNOWN)])
        assert sr.format_fault_lines(msg) == []

    def test_findings_are_rendered_too(self):
        msg = make_state(warnings=["ODOM_STALE"], warnings_severity=[az.ERROR])
        lines = sr.format_fault_lines(msg)
        assert lines and "ODOM_STALE" in lines[0]

    def test_no_faults_means_no_lines(self):
        assert sr.format_fault_lines(make_state()) == []

    def test_red_is_applied_to_errors(self):
        # An ERROR means the robot should not be moving, so it is red, not
        # yellow: a refusal and a caution must not look alike.
        sr._COLOR_DECISION = True
        try:
            msg = make_state(motion_alerts=[("MOT_ANG_VEL_EXCEED", "MOT-002", 2, az.ERROR)])
            line = sr.format_fault_lines(msg)[0]
            assert sr.ANSI_RED in line
            assert line.endswith(sr.ANSI_RESET)
        finally:
            sr._COLOR_DECISION = False

    def test_chinese_rule_name_is_included(self):
        msg = make_state(motion_alerts=[("MOT_ANG_VEL_EXCEED", "MOT-002", 2, az.ERROR)])
        line = sr.format_fault_lines(msg)[0]
        assert "速度上限（角速度）" in line

    def test_every_named_rule_has_a_chinese_name(self):
        # The manual is Chinese; a rule with only an English name would leave a
        # Chinese-speaking operator reading a translation nobody reviewed.
        for rule_id in rc.RULE_NAMES:
            assert rc.rule_name_zh(rule_id), "%s has no Chinese name" % rule_id

    def test_critical_is_bold_red(self):
        sr._COLOR_DECISION = True
        try:
            msg = make_state(
                motion_alerts=[("MOT_ACTUAL_VEL_EXCEED", "MOT-001", 4, az.CRITICAL)]
            )
            line = sr.format_fault_lines(msg)[0]
            assert sr.ANSI_BOLD in line and sr.ANSI_RED in line
        finally:
            sr._COLOR_DECISION = False

    def test_colour_can_be_forced_off_by_environment(self, monkeypatch):
        monkeypatch.setenv("ROBOT_SAFETY_COLOR", "never")
        sr._COLOR_DECISION = None
        try:
            assert sr.color_enabled() is False
            assert sr.paint("x", sr.ANSI_RED) == "x"
        finally:
            sr._COLOR_DECISION = None
