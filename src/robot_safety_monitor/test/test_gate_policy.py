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

"""Unit tests for the safety gate policy.

The gate is the last component between a command and the motors, so these tests
are about the properties that matter for a fail-safe interlock rather than about
happy paths: it must stop when it cannot know, it must remove authority but never
add it, and it must not chatter.
"""

import pytest

from robot_safety_monitor import gate_policy as gp


def make_policy(**overrides):
    config = gp.GateConfig()
    for key, value in overrides.items():
        setattr(config, key, value)
    return gp.GatePolicy(config)


def armed_policy(now=100.0, linear=0.10, angular=0.0, **overrides):
    """A policy that has seen a fresh monitor snapshot and a fresh command."""
    policy = make_policy(**overrides)
    policy.note_monitor(now)
    policy.note_command(linear, angular, now)
    return policy


def tick(policy, now_wall, motion_alerts=(), motion_status=0):
    """Advance one gate tick with a fresh monitor heartbeat.

    The real monitor publishes at 10 Hz, so a test that advances time without
    refreshing the heartbeat is testing the monitor watchdog by accident rather
    than the behaviour it means to exercise.
    """
    policy.note_monitor(now_wall)
    return policy.decide(
        now_wall=now_wall, motion_alerts=motion_alerts, motion_status=motion_status
    )


# --------------------------------------------------------------------------- #
# Fail-safe behaviour: stop when you cannot know
# --------------------------------------------------------------------------- #
class TestFailSafe:
    def test_stops_when_no_monitor_has_ever_reported(self):
        # The gate has no verdict to act on, so it must not invent a pass.
        policy = make_policy()
        decision = policy.decide(now_wall=100.0)
        assert decision.state == gp.GATE_HOLDING
        assert decision.reason == gp.REASON_NO_MONITOR
        assert decision.watchdog_tripped is True
        assert not decision.passing

    def test_stops_when_the_monitor_goes_silent(self):
        # "Monitor died" must never be interpreted as "everything is fine".
        policy = armed_policy(now=100.0)
        decision = policy.decide(now_wall=100.0 + 0.25)
        assert decision.state == gp.GATE_WATCHDOG
        assert decision.reason == gp.REASON_MONITOR_STALE
        assert decision.watchdog_tripped is True

    def test_monitor_watchdog_boundary(self):
        # Ages are chosen to avoid the float representation trap: 100.2 - 100.0
        # is 0.19999999999998863, so a literal "exactly at the limit" assertion
        # would be testing float rounding rather than the watchdog.
        just_inside = armed_policy(now=100.0, monitor_timeout_sec=0.20)
        assert just_inside.decide(now_wall=100.15).passing

        past_limit = armed_policy(now=100.0, monitor_timeout_sec=0.20)
        assert past_limit.decide(now_wall=100.25).state == gp.GATE_WATCHDOG

    def test_idle_command_source_stops_without_alarming(self):
        # A quiet command source is normal operation -- releasing the teleop key,
        # finishing a goal, no controller connected. The gate must still stop the
        # robot (a drive plugin holds its last command indefinitely), but it must
        # not report a fault or trip a watchdog: nothing is wrong.
        policy = armed_policy(now=100.0, command_timeout_sec=0.50)
        policy.note_monitor(100.6)              # monitor is alive; only the
        decision = policy.decide(now_wall=100.6)  # command stream went quiet
        assert decision.state == gp.GATE_HOLDING
        assert decision.reason == gp.REASON_COMMAND_IDLE
        assert decision.watchdog_tripped is False
        assert decision.output_linear == 0.0

    def test_command_idle_still_reported_when_legacy_flag_set(self):
        # Platforms that still want the old fault semantics can ask for them.
        policy = armed_policy(
            now=100.0, command_timeout_sec=0.50, command_timeout_is_fault=True
        )
        policy.note_monitor(100.6)
        decision = policy.decide(now_wall=100.6)
        assert decision.watchdog_tripped is True

    def test_holds_when_no_command_has_ever_arrived(self):
        policy = make_policy()
        policy.note_monitor(100.0)
        decision = policy.decide(now_wall=100.0)
        assert decision.state == gp.GATE_HOLDING
        assert decision.reason == gp.REASON_NO_COMMAND
        # Holding is a stop, but not a watchdog trip: nobody has commanded yet.
        assert decision.watchdog_tripped is False


# --------------------------------------------------------------------------- #
# Authority is only ever removed
# --------------------------------------------------------------------------- #
class TestNeverAmplifies:
    def test_passes_a_clean_command_unchanged(self):
        policy = armed_policy(now=100.0, linear=0.10, angular=0.50)
        decision = policy.decide(now_wall=100.0)
        assert decision.state == gp.GATE_PASS
        assert decision.output_linear == pytest.approx(0.10)
        assert decision.output_angular == pytest.approx(0.50)
        assert decision.command_changed() is False

    def test_blocking_outputs_zero_and_keeps_the_input_visible(self):
        policy = armed_policy(now=100.0, linear=0.30)
        decision = policy.decide(
            now_wall=100.0, motion_alerts=(("MOT_LIN_VEL_EXCEED", "MOT-001", 2),)
        )
        assert decision.output_linear == 0.0
        assert decision.output_angular == 0.0
        # The refused command is reported so a log can show what was asked for.
        assert decision.input_linear == pytest.approx(0.30)
        assert decision.command_changed() is True

    def test_gate_never_increases_a_command(self):
        # Property check over a spread of inputs and faults: whatever the gate
        # outputs must never exceed what came in, in either axis.
        for linear in (-0.3, -0.1, 0.0, 0.1, 0.3):
            for angular in (-3.0, 0.0, 3.0):
                policy = armed_policy(now=100.0, linear=linear, angular=angular)
                decision = policy.decide(now_wall=100.0)
                assert abs(decision.output_linear) <= abs(linear) + 1e-9
                assert abs(decision.output_angular) <= abs(angular) + 1e-9

    def test_stop_is_published_not_silence(self):
        # Publishing nothing would leave the base holding its last velocity.
        policy = armed_policy(now=100.0, linear=0.20)
        decision = policy.decide(
            now_wall=100.0, motion_alerts=(("MOT_TWIST_INFEASIBLE", "MOT-004", 2),)
        )
        assert decision.output_linear == 0.0
        assert decision.intercepted is True


# --------------------------------------------------------------------------- #
# Motion-safety blocking
# --------------------------------------------------------------------------- #
class TestMotionBlocking:
    def test_blocks_at_the_configured_level(self):
        policy = armed_policy(now=100.0, block_level=2)
        decision = policy.decide(
            now_wall=100.0, motion_alerts=(("MOT_ANG_VEL_EXCEED", "MOT-002", 2),)
        )
        assert decision.state == gp.GATE_BLOCKED
        assert decision.reason == gp.REASON_MOTION_ALERT
        assert decision.blocked_codes == ("MOT_ANG_VEL_EXCEED",)

    def test_notice_below_the_block_level_passes(self):
        # S1 is "log only" per RSS-001 §4.3; blocking on it would make the gate
        # stricter than policy allows without the operator asking.
        policy = armed_policy(now=100.0, block_level=2)
        decision = policy.decide(
            now_wall=100.0, motion_alerts=(("MOT_JERK_HIGH", "MOT-005", 1),)
        )
        assert decision.passing

    def test_raises_to_protective_stop_when_level_is_lowered(self):
        policy = armed_policy(now=100.0, block_level=1)
        decision = policy.decide(
            now_wall=100.0, motion_alerts=(("MOT_JERK_HIGH", "MOT-005", 1),)
        )
        assert decision.state == gp.GATE_BLOCKED

    def test_block_is_cleared_once_the_alert_disappears(self):
        policy = armed_policy(now=100.0, pass_clear_frames=1)
        tick(policy, 100.0, (("MOT_LIN_VEL_EXCEED", "MOT-001", 2),))
        assert tick(policy, 100.1).passing
        assert tick(policy, 100.2).passing

    def test_all_blocking_codes_are_reported(self):
        policy = armed_policy(now=100.0)
        decision = policy.decide(
            now_wall=100.0,
            motion_alerts=(
                ("MOT_LIN_VEL_EXCEED", "MOT-001", 2),
                ("MOT_TWIST_INFEASIBLE", "MOT-004", 2),
            ),
        )
        assert set(decision.blocked_codes) == {
            "MOT_LIN_VEL_EXCEED", "MOT_TWIST_INFEASIBLE"
        }
        assert "MOT-001" in decision.detail and "MOT-004" in decision.detail

    def test_motion_status_can_block_on_its_own(self):
        policy = armed_policy(now=100.0, block_on_motion_status=True)
        decision = policy.decide(now_wall=100.0, motion_status=3)
        assert decision.state == gp.GATE_BLOCKED
        assert decision.reason == gp.REASON_MOTION_DEGRADED

    def test_motion_status_blocking_can_be_disabled(self):
        policy = armed_policy(now=100.0, block_on_motion_status=False)
        assert policy.decide(now_wall=100.0, motion_status=4).passing


# --------------------------------------------------------------------------- #
# Debounce and bookkeeping
# --------------------------------------------------------------------------- #
class TestDebounceAndCounters:
    def test_authority_is_not_restored_immediately(self):
        # A command stream that flickers between allowed and refused is worse
        # for the drivetrain than one that stays stopped, so clearing must be
        # held for a few ticks.
        policy = armed_policy(now=100.0, pass_clear_frames=3)
        tick(policy, 100.0, (("MOT_LIN_VEL_EXCEED", "MOT-001", 2),))
        assert not tick(policy, 100.1).passing     # clear frame 1
        assert not tick(policy, 100.2).passing     # clear frame 2
        assert tick(policy, 100.3).passing         # clear frame 3 -> restored

    def test_new_fault_resets_the_clear_counter(self):
        policy = armed_policy(now=100.0, pass_clear_frames=3)
        tick(policy, 100.0, (("MOT_LIN_VEL_EXCEED", "MOT-001", 2),))
        tick(policy, 100.1)                     # 1 clear frame
        tick(policy, 100.2, (("MOT_LIN_VEL_EXCEED", "MOT-001", 2),))  # reset
        assert not tick(policy, 100.3).passing  # counting again from 1

    def test_counters_track_pass_and_block(self):
        policy = armed_policy(now=100.0, pass_clear_frames=1)
        tick(policy, 100.0)
        tick(policy, 100.1, (("MOT_LIN_VEL_EXCEED", "MOT-001", 2),))
        assert policy.passes == 1
        assert policy.blocks == 1

    def test_reset_forgets_everything(self):
        policy = armed_policy(now=100.0)
        tick(policy, 100.0)
        policy.reset()
        assert policy.last_monitor_wall is None
        assert policy.last_command_wall is None
        assert policy.decide(now_wall=100.0).state == gp.GATE_HOLDING

    def test_state_names_cover_all_states(self):
        for state in (gp.GATE_PASS, gp.GATE_BLOCKED, gp.GATE_WATCHDOG, gp.GATE_HOLDING):
            assert gp.STATE_NAMES[state]

    def test_age_is_infinite_before_the_first_sample(self):
        policy = make_policy()
        assert policy.monitor_age(100.0) == float("inf")
        assert policy.command_age(100.0) == float("inf")
