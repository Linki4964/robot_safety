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

"""Unit tests for the ROS-independent monitor core.

These run without a ROS graph:

    cd src/robot_safety_monitor && python3 -m pytest test -v

They cover the judgements that decide whether the robot is considered safe, so
they are deliberately about *semantics* (what conclusion is drawn at time t),
not about message plumbing.
"""

import math

import pytest

from robot_safety_monitor import analyzer as az


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def make_tracker(config=None, sources=None):
    """Tracker with the canonical sources registered at a 1 s timeout."""
    config = config or az.Config()
    tracker = az.ObservationTracker(config)
    for name in sources or (
        az.SOURCE_ODOM,
        az.SOURCE_SCAN,
        az.SOURCE_IMU,
        az.SOURCE_JOINTS,
        az.SOURCE_COMMAND,
    ):
        timeout = {
            az.SOURCE_ODOM: config.odom_timeout_sec,
            az.SOURCE_SCAN: config.scan_timeout_sec,
            az.SOURCE_IMU: config.imu_timeout_sec,
            az.SOURCE_JOINTS: config.joints_timeout_sec,
            az.SOURCE_COMMAND: config.command_timeout_sec,
            az.SOURCE_BATTERY: config.battery_timeout_sec,
        }[name]
        tracker.register(name, "/" + name, timeout)
    return tracker


def healthy_state(now=100.0, **overrides):
    """A MonitorState that should assess as OK, for tests to perturb."""
    state = az.MonitorState(now_wall=now)
    state.odom = az.MotionSample(
        available=True,
        x=1.0,
        y=2.0,
        yaw_rad=0.5,
        vx=0.10,
        speed_mps=0.10,
        pose_cov_xx=1e-5,
        pose_cov_yy=1e-5,
        pose_cov_yawyaw=1e-5,
        twist_cov_max=1e-5,
    )
    state.scan = az.RangeSample(
        available=True,
        range_min=0.12,
        range_max=3.5,
        closest_range=1.20,
        front_range=1.50,
        valid_point_count=350,
        point_count=360,
    )
    state.imu = az.AttitudeSample(
        available=True, orientation_available=True, tilt_deg=0.4
    )
    state.joints = az.JointSample(available=True, joint_count=2, max_abs_velocity=3.0)
    state.command = az.CommandSample(
        available=True, fresh=True, stop_command=False, linear_x=0.10
    )
    for key, value in overrides.items():
        setattr(state, key, value)
    return state


def observed_tracker(now=100.0, config=None):
    """Tracker whose sources have all just delivered a sample."""
    tracker = make_tracker(config)
    for name in tracker.names():
        tracker.observe(name, now)
    return tracker


# --------------------------------------------------------------------------- #
# geometry
# --------------------------------------------------------------------------- #
class TestGeometry:
    def test_yaw_identity_is_zero(self):
        assert az.quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

    def test_yaw_ninety_degrees(self):
        half = math.sqrt(0.5)
        assert az.quaternion_to_yaw(0.0, 0.0, half, half) == pytest.approx(
            math.pi / 2.0
        )

    def test_yaw_is_wrapped_into_range(self):
        # 270 degrees must come back as -90 degrees, not +270.
        half = math.sqrt(0.5)
        yaw = az.quaternion_to_yaw(0.0, 0.0, -half, half)
        assert yaw == pytest.approx(-math.pi / 2.0)
        assert -math.pi < yaw <= math.pi

    def test_normalize_angle_handles_pi(self):
        assert az.normalize_angle(math.pi) == pytest.approx(math.pi)
        assert az.normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
        assert az.normalize_angle(-3.0 * math.pi) == pytest.approx(math.pi)

    def test_level_attitude_has_no_tilt(self):
        assert az.tilt_angle_deg(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)

    def test_tilt_ignores_yaw(self):
        # A robot rotated purely about z is still level.
        assert az.tilt_angle_deg(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)

    def test_tilt_detects_roll(self):
        assert az.tilt_angle_deg(math.radians(30.0), 0.0) == pytest.approx(
            30.0, abs=1e-6
        )

    def test_tilt_detects_pitch(self):
        assert az.tilt_angle_deg(0.0, math.radians(20.0)) == pytest.approx(
            20.0, abs=1e-6
        )

    def test_large_roll_can_exceed_ninety(self):
        # Past 90 degrees the robot is on its side / upside down.
        assert az.tilt_angle_deg(math.radians(120.0), 0.0) == pytest.approx(
            120.0, abs=1e-6
        )


# --------------------------------------------------------------------------- #
# covariance helpers
# --------------------------------------------------------------------------- #
class TestCovariance:
    def test_diagonal_of_complete_matrix(self):
        covariance = [float(i) for i in range(36)]
        diagonal = az.covariance_diagonal(covariance)
        assert diagonal == [0.0, 7.0, 14.0, 21.0, 28.0, 35.0]

    def test_empty_covariance_is_not_zero_variance(self):
        # No covariance information must be distinguishable from "perfect".
        assert az.covariance_diagonal([]) == []

    def test_ragged_covariance_is_rejected(self):
        assert az.covariance_diagonal([1.0, 2.0, 3.0]) == []

    def test_missing_element_reports_negative_one(self):
        assert az.variance_or_negative_one([], 0) == -1.0

    def test_nan_element_reports_negative_one(self):
        matrix = [float("nan")] + [0.0] * 35
        assert az.variance_or_negative_one(matrix, 0) == -1.0

    def test_truthiness_is_never_evaluated_on_covariance(self):
        # Regression guard: generated ROS messages expose covariance as a numpy
        # array, where `if not covariance` raises ValueError, and as
        # array.array, where an empty one is falsy. The helpers must therefore
        # probe with len() and compare against None, never with truthiness.
        class NoTruthiness(list):
            def __bool__(self):
                raise AssertionError("covariance truthiness must not be evaluated")

            __nonzero__ = __bool__

        matrix = NoTruthiness([0.0] * 36)
        matrix[0] = 1e-5
        matrix[7] = 2e-5
        matrix[35] = 3e-5
        assert az.covariance_diagonal(matrix) == [1e-5, 2e-5, 0.0, 0.0, 0.0, 3e-5]
        # Diagonal slot 5 is the yaw variance; a 6x6 covariance has slots 0..5.
        assert az.variance_or_negative_one(matrix, 5) == pytest.approx(3e-5)
        assert az.variance_or_negative_one(matrix, 1) == pytest.approx(2e-5)
        # Out-of-range and empty inputs are "unknown", not zero.
        assert az.variance_or_negative_one(matrix, 6) == -1.0
        assert az.variance_or_negative_one(NoTruthiness(), 0) == -1.0


# --------------------------------------------------------------------------- #
# staleness / observation tracking
# --------------------------------------------------------------------------- #
class TestObservationTracker:
    def test_unobserved_required_source_is_critical(self):
        tracker = make_tracker()
        status, reason = tracker.health(100.0)[az.SOURCE_ODOM]
        assert status == az.CRITICAL
        assert "no sample" in reason

    def test_unobserved_optional_source_is_error(self):
        tracker = make_tracker()
        status, _ = tracker.health(100.0)[az.SOURCE_IMU]
        assert status == az.ERROR

    def test_fresh_sample_is_ok(self):
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0)
        assert tracker.health(100.5)[az.SOURCE_ODOM][0] == az.OK

    def test_sample_older_than_timeout_is_stale(self):
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0)
        status, reason = tracker.health(102.0)[az.SOURCE_ODOM]
        assert status == az.STALE
        assert "timeout" in reason

    def test_staleness_uses_wall_clock_not_message_stamp(self):
        # A frozen simulator keeps stamping old message times; only the
        # wall-clock arrival time can reveal that nothing is arriving.
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0, stamp=50.0)
        assert tracker.health(100.2)[az.SOURCE_ODOM][0] == az.OK
        assert tracker.health(101.5)[az.SOURCE_ODOM][0] == az.STALE

    def test_message_stamp_is_recorded(self):
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0, stamp=42.5)
        assert tracker.require(az.SOURCE_ODOM).last_stamp == pytest.approx(42.5)

    def test_zero_stamp_does_not_overwrite(self):
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0, stamp=42.5)
        tracker.observe(az.SOURCE_ODOM, 100.1, stamp=0.0)
        assert tracker.require(az.SOURCE_ODOM).last_stamp == pytest.approx(42.5)

    def test_message_count_increments(self):
        tracker = make_tracker()
        for index in range(5):
            tracker.observe(az.SOURCE_ODOM, 100.0 + 0.1 * index)
        assert tracker.require(az.SOURCE_ODOM).message_count == 5

    def test_rate_estimation(self):
        tracker = make_tracker()
        for index in range(11):
            tracker.observe(az.SOURCE_ODOM, 100.0 + 0.1 * index)
        rate = tracker.require(az.SOURCE_ODOM).measured_rate(101.0, 5.0)
        assert rate == pytest.approx(10.0, rel=0.05)

    def test_rate_unknown_with_single_sample(self):
        tracker = make_tracker()
        tracker.observe(az.SOURCE_ODOM, 100.0)
        assert tracker.require(az.SOURCE_ODOM).measured_rate(100.1, 5.0) == -1.0

    def test_rate_window_is_bounded(self):
        # 200 samples at 20 Hz span 10 s, twice the 5 s rate window. The buffer
        # must hold the window plus one anchor sample, not everything ever seen.
        tracker = make_tracker()
        for index in range(200):
            tracker.observe(az.SOURCE_ODOM, 100.0 + 0.05 * index)
        observation = tracker.require(az.SOURCE_ODOM)
        assert observation.message_count == 200
        assert len(observation.rate_samples) <= 5.0 / 0.05 + 2

    def test_rate_window_stays_bounded_across_rate_and_duration(self):
        # The bound must not drift with either the sample rate or the run
        # length; a slow leak here would grow the monitor's memory forever.
        for period, steps in ((0.05, 2000), (0.1, 2000), (0.01, 5000)):
            tracker = make_tracker()
            for index in range(steps):
                tracker.observe(az.SOURCE_ODOM, 100.0 + period * index)
            observation = tracker.require(az.SOURCE_ODOM)
            assert len(observation.rate_samples) <= 5.0 / period + 2, period

    def test_rate_remains_accurate_after_long_run(self):
        tracker = make_tracker()
        period = 0.05
        steps = 4000
        for index in range(steps):
            tracker.observe(az.SOURCE_ODOM, 100.0 + period * index)
        observation = tracker.require(az.SOURCE_ODOM)
        now = 100.0 + period * (steps - 1)
        assert observation.measured_rate(now, 5.0) == pytest.approx(1.0 / period, rel=0.02)

    def test_explicit_required_flag_overrides_config(self):
        tracker = make_tracker()
        tracker.register(az.SOURCE_IMU, "/imu", 1.0, required=True)
        assert tracker.health(100.0)[az.SOURCE_IMU][0] == az.CRITICAL

    def test_unregistered_source_raises(self):
        tracker = make_tracker()
        with pytest.raises(KeyError):
            tracker.observe("laser", 100.0)

    def test_not_monitored_source_is_excluded_from_health(self):
        # A platform with no battery topic declares the source absent; that must
        # not be reported as a failure, or the snapshot would never be OK.
        tracker = make_tracker()
        tracker.declare_not_monitored(az.SOURCE_BATTERY, 5.0)
        for name in (az.SOURCE_ODOM, az.SOURCE_SCAN):
            tracker.observe(name, 100.0)
        health = tracker.health(100.0)
        assert az.SOURCE_BATTERY not in health
        assert az.SOURCE_BATTERY not in tracker.monitored_names()
        assert az.SOURCE_ODOM in health

    def test_not_monitored_source_still_raises_when_observed(self):
        # The record still exists, so a stray observation is a caller bug, not
        # a silently accepted event.
        tracker = make_tracker()
        tracker.declare_not_monitored(az.SOURCE_BATTERY, 5.0)
        assert tracker.require(az.SOURCE_BATTERY).monitored is False


# --------------------------------------------------------------------------- #
# threshold judgements
# --------------------------------------------------------------------------- #
class TestTiltEvaluation:
    def test_level_is_ok(self):
        status, findings = az.evaluate_tilt(2.0, az.Config())
        assert status == az.OK
        assert findings == []

    def test_warning_band(self):
        status, findings = az.evaluate_tilt(20.0, az.Config())
        assert status == az.ERROR
        assert findings[0].code == az.CODE_TILT_HIGH

    def test_critical_band(self):
        status, findings = az.evaluate_tilt(60.0, az.Config())
        assert status == az.CRITICAL
        assert findings[0].code == az.CODE_TILT_CRITICAL

    def test_boundary_is_inclusive(self):
        assert az.evaluate_tilt(15.0, az.Config())[0] == az.ERROR
        assert az.evaluate_tilt(45.0, az.Config())[0] == az.CRITICAL

    def test_no_attitude_yields_unknown_not_ok(self):
        # Absence of an attitude estimate must not be silently reported as safe.
        status, findings = az.evaluate_tilt(None, az.Config())
        assert status == az.UNKNOWN
        assert findings == []


class TestObstacleEvaluation:
    def test_clear_space_is_ok(self):
        status, findings = az.evaluate_obstacle(1.0, az.Config())
        assert status == az.OK
        assert findings == []

    def test_near_obstacle_warns(self):
        status, findings = az.evaluate_obstacle(0.25, az.Config())
        assert status == az.ERROR
        assert findings[0].code == az.CODE_OBSTACLE_NEAR

    def test_very_near_obstacle_is_critical(self):
        status, findings = az.evaluate_obstacle(0.10, az.Config())
        assert status == az.CRITICAL
        assert findings[0].code == az.CODE_OBSTACLE_CRITICAL

    def test_infinite_range_means_clear(self):
        status, _ = az.evaluate_obstacle(math.inf, az.Config())
        assert status == az.OK

    def test_negative_sentinel_means_unknown(self):
        # The publisher writes -1 when no valid return exists at all.
        status, _ = az.evaluate_obstacle(-1.0, az.Config())
        assert status == az.UNKNOWN

    def test_nan_range_means_unknown(self):
        status, _ = az.evaluate_obstacle(float("nan"), az.Config())
        assert status == az.UNKNOWN


class TestBatteryEvaluation:
    def test_full_battery_is_ok(self):
        status, _ = az.evaluate_battery(0.9, True, az.Config())
        assert status == az.OK

    def test_low_battery_warns(self):
        status, findings = az.evaluate_battery(0.25, True, az.Config())
        assert status == az.ERROR
        assert findings[0].code == az.CODE_BATTERY_LOW

    def test_empty_battery_is_critical(self):
        status, findings = az.evaluate_battery(0.10, True, az.Config())
        assert status == az.CRITICAL
        assert findings[0].code == az.CODE_BATTERY_CRITICAL

    def test_unavailable_battery_is_unknown_not_ok(self):
        status, _ = az.evaluate_battery(-1.0, False, az.Config())
        assert status == az.UNKNOWN


class TestPoseHealth:
    def test_small_covariance_is_ok(self):
        status, _ = az.evaluate_pose_health(1e-5, 1e-5, 1e-5, az.Config())
        assert status == az.OK

    def test_large_position_covariance_is_flagged(self):
        status, findings = az.evaluate_pose_health(0.5, 1e-5, 1e-5, az.Config())
        assert status == az.ERROR
        assert findings[0].code == az.CODE_POSE_UNCERTAIN

    def test_large_twist_covariance_is_flagged(self):
        status, findings = az.evaluate_pose_health(1e-5, 1e-5, 0.5, az.Config())
        assert status == az.ERROR
        assert findings[0].code == az.CODE_TWIST_UNCERTAIN

    def test_unknown_covariance_is_not_flagged(self):
        status, _ = az.evaluate_pose_health(-1.0, -1.0, -1.0, az.Config())
        assert status == az.OK

    def test_unobserved_dof_sentinel_is_filtered_out(self):
        # Diff-drive odometry writes 1e12 for z/roll/pitch. That means "this
        # degree of freedom is not observed", not "the estimate is terrible",
        # so it must never raise POSE_UNCERTAIN or TWIST_UNCERTAIN.
        assert az.covariance_is_sentinel(1.0e12)
        assert az.variance_or_negative_one([1.0e12] * 36, 5) == -1.0
        status, findings = az.evaluate_pose_health(-1.0, -1.0, -1.0, az.Config())
        assert status == az.OK
        assert findings == []

    def test_nan_and_infinite_covariance_are_ignored(self):
        status, findings = az.evaluate_pose_health(
            float("nan"), float("inf"), float("inf"), az.Config()
        )
        assert status == az.OK
        assert findings == []

    def test_negative_covariance_is_treated_as_unusable(self):
        assert az.variance_or_negative_one([-1.0] * 36, 0) == -1.0


# --------------------------------------------------------------------------- #
# intent vs execution
# --------------------------------------------------------------------------- #
class TestCommandConsistency:
    def test_matching_command_and_motion_is_ok(self):
        status, findings, expected, _ = az.evaluate_command_consistency(
            0.10, 0.0, True, 0.10, 0.0, az.Config()
        )
        assert status == az.OK
        assert expected is True
        assert findings == []

    def test_command_without_motion_is_a_mismatch(self):
        # Stuck wheel, unpowered motor, or an e-stop the controller ignores.
        status, findings, expected, deviation = az.evaluate_command_consistency(
            0.20, 0.0, True, 0.0, 0.0, az.Config()
        )
        assert status == az.ERROR
        assert expected is True
        assert findings[0].code == az.CODE_COMMAND_MISMATCH
        assert deviation == pytest.approx(0.20)

    def test_odometry_lag_within_tolerance_is_ok(self):
        # Simulation odometry always lags a fresh command by a few ticks; a
        # small shortfall must not raise a finding.
        status, _, _, _ = az.evaluate_command_consistency(
            0.20, 0.0, True, 0.15, 0.0, az.Config()
        )
        assert status == az.OK

    def test_motion_without_any_command_is_flagged(self):
        status, findings, expected, _ = az.evaluate_command_consistency(
            None, None, False, 0.30, 0.0, az.Config()
        )
        assert status == az.ERROR
        assert expected is False
        assert findings[0].code == az.CODE_UNEXPECTED_MOTION

    def test_motion_after_command_expired_is_flagged(self):
        status, findings, _, _ = az.evaluate_command_consistency(
            0.10, 0.0, False, 0.30, 0.0, az.Config()
        )
        assert status == az.ERROR
        assert findings[0].code == az.CODE_UNEXPECTED_MOTION

    def test_stop_command_with_motion_is_flagged(self):
        status, findings, expected, _ = az.evaluate_command_consistency(
            0.0, 0.0, True, 0.25, 0.0, az.Config()
        )
        assert status == az.ERROR
        assert expected is False
        assert findings[0].code == az.CODE_UNEXPECTED_MOTION

    def test_stopped_robot_with_stop_command_is_ok(self):
        status, findings, _, _ = az.evaluate_command_consistency(
            0.0, 0.0, True, 0.0, 0.0, az.Config()
        )
        assert status == az.OK
        assert findings == []

    def test_rotation_is_compared_too(self):
        status, findings, _, _ = az.evaluate_command_consistency(
            0.0, 0.50, True, 0.0, 0.0, az.Config()
        )
        assert status == az.ERROR
        assert findings[0].code == az.CODE_COMMAND_MISMATCH

    def test_stale_command_without_motion_is_not_a_finding(self):
        # A parked robot whose teleop node stopped publishing is normal.
        status, findings, _, _ = az.evaluate_command_consistency(
            None, None, False, 0.0, 0.0, az.Config()
        )
        assert status == az.OK
        assert findings == []

    def test_missing_odometry_makes_consistency_unjudgeable(self):
        # With no odometry there is nothing to compare against. Reporting
        # UNEXPECTED_MOTION or COMMAND_MISMATCH here would invent a fault whose
        # real cause (missing odometry) is already reported by the health layer.
        status, findings, expected, _ = az.evaluate_command_consistency(
            0.30, 0.0, True, None, None, az.Config(), odom_available=False
        )
        assert status == az.UNKNOWN
        assert findings == []
        assert expected is False

    def test_missing_command_makes_consistency_unjudgeable(self):
        # No command has ever arrived. The "no fresh command" branch must not be
        # reached, or the monitor would report a verdict of *agreement* for a
        # comparison it never performed.
        status, findings, expected, _ = az.evaluate_command_consistency(
            None, None, False, 0.0, 0.0, az.Config(), command_available=False
        )
        assert status == az.UNKNOWN
        assert findings == []
        assert expected is False


# --------------------------------------------------------------------------- #
# end-to-end assessment
# --------------------------------------------------------------------------- #
class TestAnalyzerAssessment:
    def test_fully_healthy_robot_is_ok(self):
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(healthy_state())
        assert assessment.status == az.OK
        assert assessment.findings == []

    def test_all_sources_reported_in_assessment(self):
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(healthy_state())
        for name in tracker.names():
            assert name in assessment.source_status

    def test_one_degraded_subsystem_cannot_be_hidden(self):
        # Everything is healthy except one genuinely dangerous quantity.
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        state.imu.tilt_deg = 80.0
        assessment = analyzer.assess(state)
        assert assessment.status == az.CRITICAL
        assert assessment.has(az.CODE_TILT_CRITICAL)

    def test_stale_odometry_makes_the_snapshot_not_ok(self):
        config = az.Config()
        tracker = observed_tracker(now=100.0)
        analyzer = az.Analyzer(config, tracker)
        # Judge two seconds later: no source delivered anything since.
        assessment = analyzer.assess(healthy_state(now=102.0), now=102.0)
        assert assessment.status == az.STALE
        assert assessment.has(az.CODE_ODOM_STALE)
        assert assessment.has(az.CODE_SCAN_STALE)

    def test_never_seen_source_uses_absence_code(self):
        config = az.Config()
        tracker = make_tracker(config)
        tracker.observe(az.SOURCE_SCAN, 100.0)
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(healthy_state(now=100.0), now=100.0)
        assert assessment.has(az.CODE_ODOM_MISSING)
        # Scan is present, so it must not be reported as missing.
        assert not assessment.has(az.CODE_SCAN_MISSING)

    def test_optional_source_going_quiet_is_stale_not_missing(self):
        config = az.Config()
        tracker = make_tracker(config)
        for name in (az.SOURCE_ODOM, az.SOURCE_SCAN, az.SOURCE_IMU):
            tracker.observe(name, 100.0)
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(healthy_state(now=102.0), now=102.0)
        assert assessment.has(az.CODE_IMU_STALE)
        assert not assessment.has(az.CODE_IMU_MISSING)

    def test_severity_of_worst_finding_wins(self):
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        state.scan.closest_range = 0.10  # critical
        state.imu.tilt_deg = 20.0        # warning only
        assessment = analyzer.assess(state)
        assert assessment.status == az.CRITICAL
        assert assessment.has(az.CODE_OBSTACLE_CRITICAL)
        assert assessment.has(az.CODE_TILT_HIGH)

    def test_command_consistency_is_written_back_into_state(self):
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        state.command.linear_x = 0.30
        state.odom.speed_mps = 0.0
        analyzer.assess(state)
        assert state.command.consistency_checked is True
        assert state.command.consistent is False
        assert state.command.deviation == pytest.approx(0.30)

    def test_consistency_not_claimed_when_odometry_is_missing(self):
        # The three-state collapse must not report agreement it never checked.
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        state.odom = az.MotionSample(available=False)
        assessment = analyzer.assess(state)
        assert state.command.consistency_checked is False
        assert state.command.consistent is False
        assert 'COMMAND_MISMATCH' not in assessment.codes()
        assert 'UNEXPECTED_MOTION' not in assessment.codes()

    def test_consistency_not_claimed_when_command_is_missing(self):
        # Same rule from the other side: no intent data, no verdict.
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        state.command = az.CommandSample(available=False)
        assessment = analyzer.assess(state)
        assert state.command.consistency_checked is False
        assert state.command.consistent is False
        assert 'UNEXPECTED_MOTION' not in assessment.codes()
        assert 'COMMAND_MISMATCH' not in assessment.codes()

    def test_motion_expected_reflects_fresh_nonzero_command(self):
        config = az.Config()
        tracker = observed_tracker()
        analyzer = az.Analyzer(config, tracker)
        state = healthy_state()
        assert analyzer.motion_expected(state) is True

        state.command.fresh = False
        assert analyzer.motion_expected(state) is False

        state.command.fresh = True
        state.command.linear_x = 0.0
        assert analyzer.motion_expected(state) is False

    def test_no_data_at_all_is_critical(self):
        # Nothing has ever arrived: the monitor must not claim the robot is OK.
        config = az.Config()
        tracker = make_tracker(config)
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(az.MonitorState(now_wall=100.0))
        assert assessment.status == az.CRITICAL
        assert assessment.has(az.CODE_ODOM_MISSING)
        assert assessment.has(az.CODE_SCAN_MISSING)

    def test_not_monitored_sources_do_not_degrade_the_assessment(self):
        config = az.Config()
        tracker = observed_tracker()
        tracker.declare_not_monitored(az.SOURCE_BATTERY, 5.0)
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(healthy_state(), now=100.0)
        assert assessment.status == az.OK
        assert az.SOURCE_BATTERY not in assessment.source_status
        assert not assessment.has(az.CODE_BATTERY_STALE)

    def test_worst_status_helper(self):
        assert az.worst_status([]) == az.UNKNOWN
        assert az.worst_status([az.OK, az.STALE]) == az.STALE
        assert az.worst_status([az.OK, az.CRITICAL, az.ERROR]) == az.CRITICAL

    def test_status_names_are_total(self):
        for code in (az.UNKNOWN, az.OK, az.STALE, az.ERROR, az.CRITICAL):
            assert az.status_name(code) in {"UNKNOWN", "OK", "STALE", "ERROR", "CRITICAL"}
        assert az.status_name(99).startswith("UNKNOWN")
