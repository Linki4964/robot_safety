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

"""Unit tests for the motion-safety rules (RSS-003 §1, MOT-001 .. MOT-004).

These run without a ROS graph:

    cd src/robot_safety_monitor && python3 -m pytest test -v

The tests are written against the rule text rather than the implementation, so
each one names the rule it pins down. The boundary cases matter more than the
obvious ones: a limit check is only correct if it excludes exactly the values the
spec excludes.
"""

import pytest

from robot_safety_monitor import analyzer as az
from robot_safety_monitor import motion_safety as ms


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def make_config(**overrides):
    config = ms.MotionSafetyConfig()
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


def make_state(
    cmd_linear=0.0,
    cmd_angular=0.0,
    command_available=True,
    command_fresh=True,
    speed=0.0,
    yaw_rate=0.0,
    odom_available=True,
    now_wall=100.0,
    now_stamp=50.0,
):
    """A MonitorState carrying just the quantities the MOT rules read."""
    state = az.MonitorState(now_wall=now_wall, now_stamp=now_stamp)
    state.command = az.CommandSample(
        available=command_available,
        fresh=command_fresh,
        linear_x=cmd_linear,
        angular_z=cmd_angular,
    )
    state.odom = az.MotionSample(
        available=odom_available,
        speed_mps=speed,
        yaw_rate_rps=yaw_rate,
    )
    return state


def codes(alerts):
    return [alert.code for alert in alerts]


def find(alerts, code):
    for alert in alerts:
        if alert.code == code:
            return alert
    return None


def run(monitor, state):
    return monitor.evaluate(state, now_wall=state.now_wall, now_stamp=state.now_stamp)


# --------------------------------------------------------------------------- #
# MOT-001 linear speed limit
# --------------------------------------------------------------------------- #
class TestMot001LinearSpeed:
    def test_within_rated_limit_is_quiet(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.20, speed=0.20))
        assert alerts == []

    def test_command_above_rated_is_s2(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.23))
        alert = find(alerts, ms.CODE_LIN_VEL_EXCEED)
        assert alert is not None
        assert alert.rule_id == "MOT-001"
        assert alert.level == ms.LEVEL_S2
        assert alert.severity == az.ERROR
        assert alert.value == pytest.approx(0.23)
        assert alert.threshold == pytest.approx(0.22)

    def test_boundary_exactly_at_limit_is_not_exceeded(self):
        # The spec writes the condition as `> v_max_cmd`, so equality passes.
        monitor = ms.MotionSafetyMonitor(make_config())
        assert run(monitor, make_state(cmd_linear=0.22)) == []

    def test_measured_above_safe_stop_is_s4(self):
        # A measured overspeed is a different, more urgent failure than a bad
        # command: the base is physically doing something unsafe.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.0, speed=0.35))
        alert = find(alerts, ms.CODE_ACTUAL_VEL_EXCEED)
        assert alert is not None
        assert alert.rule_id == "MOT-001"
        assert alert.level == ms.LEVEL_S4
        assert alert.severity == az.CRITICAL
        assert alert.threshold == pytest.approx(0.30)

    def test_measured_between_command_and_actual_limit_is_tolerated(self):
        # The two limits are deliberately different: 0.25 m/s is above the
        # commanded rating but below the measured one, which is exactly the
        # overshoot band that must not raise an emergency.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.20, speed=0.25))
        assert ms.CODE_ACTUAL_VEL_EXCEED not in codes(alerts)

    def test_measured_overspeed_is_judged_without_any_command(self):
        # The measured half of the rule must work with no command at all;
        # otherwise a runaway base looks quiet whenever nobody is commanding.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(
            monitor,
            make_state(command_available=False, speed=0.40),
        )
        assert ms.CODE_ACTUAL_VEL_EXCEED in codes(alerts)

    def test_stale_command_is_not_judged(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.50, command_fresh=False))
        assert ms.CODE_LIN_VEL_EXCEED not in codes(alerts)


# --------------------------------------------------------------------------- #
# MOT-002 angular speed limit
# --------------------------------------------------------------------------- #
class TestMot002AngularSpeed:
    def test_command_above_rated_is_s2(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_angular=3.0))
        alert = find(alerts, ms.CODE_ANG_VEL_EXCEED)
        assert alert is not None
        assert alert.rule_id == "MOT-002"
        assert alert.level == ms.LEVEL_S2
        assert alert.threshold == pytest.approx(2.84)

    def test_boundary_exactly_at_limit_is_not_exceeded(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_angular=2.84))
        assert ms.CODE_ANG_VEL_EXCEED not in codes(alerts)

    def test_measured_above_safe_stop_is_s4(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_angular=0.0, yaw_rate=3.9))
        alert = find(alerts, ms.CODE_ACTUAL_ANG_VEL_EXCEED)
        assert alert is not None
        assert alert.level == ms.LEVEL_S4
        assert alert.threshold == pytest.approx(3.50)

    def test_spin_is_judged_even_while_not_moving_linearly(self):
        # In-place rotation is the case this rule exists for: linear speed is
        # zero, so nothing else in the suite would notice.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_angular=2.90, speed=0.0))
        assert ms.CODE_ANG_VEL_EXCEED in codes(alerts)


# --------------------------------------------------------------------------- #
# MOT-003 acceleration limit
# --------------------------------------------------------------------------- #
class TestMot003Acceleration:
    def test_step_from_standstill_to_rated_is_flagged(self):
        # 0 -> 0.22 m/s in one 50 ms frame is 4.4 m/s^2, far above 0.5.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=100.05))
        alert = find(alerts, ms.CODE_LIN_ACCEL_EXCEED)
        assert alert is not None
        assert alert.rule_id == "MOT-003"
        assert alert.level == ms.LEVEL_S2
        assert alert.value == pytest.approx(0.22 / 0.05, rel=1e-6)

    def test_gradual_ramp_is_quiet(self):
        # 0.22 m/s achieved over 1 s is 0.22 m/s^2, comfortably inside 0.5.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=101.0))
        assert ms.CODE_LIN_ACCEL_EXCEED not in codes(alerts)

    def test_held_command_is_not_a_step(self):
        # The same command held over time has a zero difference between
        # consecutive frames, so it must not look like a step. Note this is not
        # the same as "repeats are skipped": frames are recorded, and it is the
        # zero delta that keeps it quiet.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.10, now_wall=100.0))
        run(monitor, make_state(cmd_linear=0.10, now_wall=100.05))
        alerts = run(monitor, make_state(cmd_linear=0.10, now_wall=100.10))
        assert ms.CODE_LIN_ACCEL_EXCEED not in codes(alerts)

    def test_step_is_still_detected_after_a_long_hold(self):
        # Regression: a command that holds a value and then steps must be
        # differenced over the *frame* interval, not over the whole hold.
        # Skipping repeated frames made this step look like 0.22 m/s over 1.6 s
        # (0.14 m/s^2) instead of over 50 ms (4.4 m/s^2), so the rule missed a
        # real violation. This test pins the fix.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        for step in range(1, 16):                  # hold zero for ~1.5 s
            run(monitor, make_state(cmd_linear=0.0, now_wall=100.0 + 0.1 * step))
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=101.6))
        alert = find(alerts, ms.CODE_LIN_ACCEL_EXCEED)
        assert alert is not None, "step after a hold must still be detected"
        assert alert.value == pytest.approx(0.22 / 0.1, rel=1e-6)

    def test_held_command_keeps_the_acceleration_baseline_fresh(self):
        # Companion to the regression above: the interval used for the step is
        # the gap to the immediately preceding frame, which is why holding a
        # value must keep appending frames.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.05, now_wall=200.0))
        run(monitor, make_state(cmd_linear=0.05, now_wall=200.1))
        assert monitor.previous_command.wall == pytest.approx(200.1)

    def test_angular_acceleration_is_flagged_separately(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_angular=0.0, now_wall=100.0))
        # 0 -> 0.5 rad/s in 50 ms = 10 rad/s^2, above the 3.0 limit.
        alerts = run(monitor, make_state(cmd_angular=0.5, now_wall=100.05))
        alert = find(alerts, ms.CODE_ANG_ACCEL_EXCEED)
        assert alert is not None
        assert alert.rule_id == "MOT-003"
        assert alert.value == pytest.approx(10.0, rel=1e-3)

    def test_interval_floor_suppresses_phantom_acceleration(self):
        # Two frames 1 ms apart would imply 220 m/s^2 from an ordinary 0.22 m/s
        # step. The spec's dt floor exists exactly to prevent that misfire.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=100.001))
        alert = find(alerts, ms.CODE_LIN_ACCEL_EXCEED)
        assert alert is not None
        # Floor is 0.02 s, so the reported rate is 11 m/s^2, not 220.
        assert alert.value == pytest.approx(0.22 / 0.02, rel=1e-6)

    def test_first_command_alone_raises_no_acceleration_alert(self):
        # Precondition requires a previous command; one sample is not a change.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=100.0))
        assert ms.CODE_LIN_ACCEL_EXCEED not in codes(alerts)

    def test_acceleration_needs_a_fresh_command(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        alerts = run(
            monitor, make_state(cmd_linear=0.22, now_wall=100.02, command_fresh=False)
        )
        assert ms.CODE_LIN_ACCEL_EXCEED not in codes(alerts)

    def test_step_across_a_silence_is_not_differenced(self):
        # Regression: when the command source goes quiet, the two frames either
        # side of the silence are not one continuous intent. Differencing them
        # across a 0.5 s silence gave 0.22/0.5 = 0.44 m/s^2 -- just under the
        # 0.5 limit -- so a real step command was silently swallowed. The rule
        # must report "not measurable" rather than a reassuring small number.
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        # The stream dies; the previous frame goes stale.
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.6, command_fresh=False))
        # The stream resumes with a full step to the rated speed.
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=100.7))
        assert ms.CODE_LIN_ACCEL_EXCEED not in codes(alerts)
        assert monitor.last_rules_evaluated == 5, (
            "MOT-003 must be reported as not judged, not as judged-and-fine"
        )

    def test_gap_shorter_than_timeout_is_still_judged(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.0, now_wall=100.0))
        alerts = run(monitor, make_state(cmd_linear=0.22, now_wall=100.4))
        alert = find(alerts, ms.CODE_LIN_ACCEL_EXCEED)
        assert alert is not None
        assert alert.value == pytest.approx(0.22 / 0.4, rel=1e-6)


# --------------------------------------------------------------------------- #
# MOT-004 differential-drive kinematic constraint
# --------------------------------------------------------------------------- #
class TestMot004DifferentialKinematics:
    def test_wheel_speeds_follow_the_spec_relation(self):
        config = make_config()
        left, right = ms.wheel_speeds(0.10, 1.0, config)
        assert left == pytest.approx(0.10 - 0.08 * 1.0)
        assert right == pytest.approx(0.10 + 0.08 * 1.0)

    def test_rated_speed_is_reachable_when_driving_straight(self):
        config = make_config()
        reachable, required = ms.twist_is_reachable(0.22, 0.0, config)
        assert reachable
        assert required == pytest.approx(0.22)

    def test_spec_example_combination_is_infeasible(self):
        # The spec's own example: v=0.22 and w=2.84 each pass MOT-001/002, but
        # together they need ~0.45 m/s from the outer wheel against a 0.22 m/s
        # limit. This is the rule's reason for existing.
        config = make_config()
        reachable, required = ms.twist_is_reachable(0.22, 2.84, config)
        assert not reachable
        assert required == pytest.approx(0.22 + 0.08 * 2.84, rel=1e-6)

    def test_reports_the_required_wheel_speed_and_limit(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.22, cmd_angular=2.84))
        alert = find(alerts, ms.CODE_TWIST_INFEASIBLE)
        assert alert is not None
        assert alert.rule_id == "MOT-004"
        assert alert.level == ms.LEVEL_S2
        assert alert.threshold == pytest.approx(0.22)
        assert alert.value > alert.threshold

    def test_slow_spin_in_place_is_reachable(self):
        # A modest in-place rotation needs only (W/2)*w from each wheel.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.0, cmd_angular=1.0))
        assert ms.CODE_TWIST_INFEASIBLE not in codes(alerts)

    def test_both_axes_inside_their_limits_still_trips(self):
        # Guards the point of the rule: neither MOT-001 nor MOT-002 fires here.
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(monitor, make_state(cmd_linear=0.22, cmd_angular=2.84))
        assert ms.CODE_LIN_VEL_EXCEED not in codes(alerts)
        assert ms.CODE_ANG_VEL_EXCEED not in codes(alerts)
        assert ms.CODE_TWIST_INFEASIBLE in codes(alerts)

    def test_not_judged_without_a_fresh_command(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(
            monitor,
            make_state(cmd_linear=0.22, cmd_angular=2.84, command_fresh=False),
        )
        assert ms.CODE_TWIST_INFEASIBLE not in codes(alerts)


# --------------------------------------------------------------------------- #
# aggregation and bookkeeping
# --------------------------------------------------------------------------- #
class TestAggregation:
    def test_no_alerts_means_ok(self):
        assert ms.MotionSafetyMonitor.worst_severity([]) == az.OK

    def test_worst_severity_wins(self):
        low = ms.MotionAlert(
            code="A", rule_id="MOT-001", level=ms.LEVEL_S2, detail=""
        )
        high = ms.MotionAlert(
            code="B", rule_id="MOT-001", level=ms.LEVEL_S4, detail=""
        )
        assert ms.MotionSafetyMonitor.worst_severity([low, high]) == az.CRITICAL

    def test_level_to_severity_mapping(self):
        # A notice must not degrade the aggregate verdict: S1 reports UNKNOWN,
        # which is the "no impact" level, not STALE.
        assert ms.level_to_severity(ms.LEVEL_S1) == az.UNKNOWN
        assert ms.level_to_severity(ms.LEVEL_S2) == az.ERROR
        assert ms.level_to_severity(ms.LEVEL_S3) == az.CRITICAL
        assert ms.level_to_severity(ms.LEVEL_S4) == az.CRITICAL
        # Level itself stays distinct even where severity collapses.
        assert ms.LEVEL_S3 != ms.LEVEL_S4

    def test_alert_carries_evidence_and_timestamps(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        alerts = run(
            monitor, make_state(cmd_linear=0.23, now_wall=100.0, now_stamp=7.5)
        )
        alert = alerts[0]
        assert alert.detail
        assert alert.stamp_sec == pytest.approx(7.5)
        assert alert.wall_time_sec == pytest.approx(100.0)

    def test_rules_evaluated_counts_only_judged_checks(self):
        # With nothing available, no rule can be judged and the count is zero,
        # so a consumer can tell "nothing fired" from "nothing was checked".
        monitor = ms.MotionSafetyMonitor(make_config())
        run(
            monitor,
            make_state(command_available=False, command_fresh=False, odom_available=False),
        )
        assert monitor.last_rules_evaluated == 0

    def test_rules_evaluated_counts_the_available_checks(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.05))
        # MOT-001 command + MOT-001 measured + MOT-002 command + MOT-002 measured
        # + MOT-004 == 5; MOT-003 has no previous sample yet.
        assert monitor.last_rules_evaluated == 5

    def test_reset_clears_history(self):
        monitor = ms.MotionSafetyMonitor(make_config())
        run(monitor, make_state(cmd_linear=0.10, now_wall=100.0))
        assert monitor.previous_command is not None
        monitor.reset()
        assert monitor.previous_command is None
