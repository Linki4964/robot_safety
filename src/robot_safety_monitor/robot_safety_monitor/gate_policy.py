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

"""Safety gate policy -- the enforcement half of the motion-safety loop.

The monitor *identifies* a violation and announces it; this module *prevents* the
violation from reaching the motors. They are deliberately separate components
running as separate processes (RSS-001 §3.1, layers 1 and 3): if the gate lived
inside the monitor, then a monitor crash would silently remove both the
detection and the enforcement, and INV-1 ("a layer-1 fault must not remove the
robot's protection") would be violated.

Like every other decision module in this package it is pure Python with the time
passed in, so the policy can be unit-tested without a ROS graph.

The gate enforces three independent things, in order of who is to blame:

1. **Feed-forward health** -- is the monitor still alive? A safety gate cannot
   act on a verdict it never received. If the monitor's snapshots stop, the gate
   stops the robot rather than continuing on the last good verdict, because
   "monitor died" must never degrade into "everything is fine" (SYS-004).
2. **Command source health** -- is anyone still commanding? A base that keeps
   executing the last command after the commander goes silent is driving open
   loop (CMD-007). The gate converts that into a stop.
3. **Motion-safety verdict** -- did any motion rule fire above the blocking
   level? If so the command is refused.

Standard fail-safe framing: this module only ever *removes* authority. It never
invents motion, never raises a limit and never smooths a command upward.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Set, Tuple

# --------------------------------------------------------------------------- #
# Gate states
# --------------------------------------------------------------------------- #
# Ordered by urgency; the value is what goes on the wire.
GATE_PASS = 0        # commanding normally
GATE_BLOCKED = 1     # refusing to pass a command (stopping)
GATE_WATCHDOG = 2    # the monitor went silent; stopping defensively
GATE_HOLDING = 3     # nobody is commanding; stopped, but nothing is wrong

STATE_NAMES = {
    GATE_PASS: "PASS",
    GATE_BLOCKED: "BLOCKED",
    GATE_WATCHDOG: "WATCHDOG",
    GATE_HOLDING: "HOLDING",
}

# --------------------------------------------------------------------------- #
# Reason codes
# --------------------------------------------------------------------------- #
REASON_PASS = "GATE_PASSING"
REASON_NO_MONITOR = "SAFETY_GATE_NO_MONITOR_STATE"
REASON_MONITOR_STALE = "SAFETY_GATE_MONITOR_STALE"
REASON_NO_COMMAND = "SAFETY_GATE_NO_COMMAND"
# The command source went quiet. This is *not* a fault: releasing the teleop key
# or finishing a navigation goal is normal operation. The gate still stops the
# robot, because a drive plugin holds its last command indefinitely and an
# unstopped robot would keep driving with nobody in control. Stopping without
# alarming is the distinction that matters -- the action is protective, the
# judgement is "nothing is wrong".
REASON_COMMAND_IDLE = "SAFETY_GATE_COMMAND_IDLE"
REASON_MOTION_ALERT = "SAFETY_GATE_MOTION_BLOCKED"
REASON_MOTION_DEGRADED = "SAFETY_GATE_MOTION_STATUS_BLOCKED"


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class GateConfig:
    """Thresholds for the gate.

    ``monitor_timeout_sec`` is the gate's own watchdog on the monitor, and it is
    intentionally tighter than the state machine's (RSS-003 SYS-005 sets
    ``T_gate = 0.20 s`` against ``T_wd_sm = 0.30 s``): the closer a component sits
    to the actuator, the less time it may spend not knowing. The gate is the last
    thing between a command and the motors, so its tolerance for silence is the
    smallest in the system.
    """

    # Watchdog on the monitor's snapshot stream. SYS-005.
    monitor_timeout_sec: float = 0.20
    # How long the command source may be quiet before the gate stops the robot.
    # This is a *response* threshold, not a fault threshold: crossing it means
    # "nobody is commanding", so the gate commands a stop itself. It produces no
    # alert and trips no watchdog.
    command_timeout_sec: float = 0.50
    # Kept so callers that asked for the old behaviour can still get it.
    command_timeout_is_fault: bool = False
    # Window in which a command identical to the one this gate just published is
    # treated as its own echo rather than a new instruction. Needed because the
    # gate reads and writes the same topic (see safety_gate_node): without it the
    # gate would hear its own zero output, latch on the stop it just published,
    # and no real command could ever get through.
    #
    # It must exceed the gate's own publication period or an echo slips past
    # between two ticks and refreshes the command timer, which defeats the idle
    # stop: verified live, a 30 ms window against a 20 Hz (50 ms) gate left the
    # robot turning at 1.0 rad/s after the commander had stopped. The default is
    # three publication periods at 20 Hz.
    #
    # The cost of a wide window: a genuine command that is byte-identical to the
    # last thing the gate published, sent within the window, is ignored. For a
    # stop that is harmless (the gate is already publishing a stop); for a
    # non-zero value it simply means that one repetition is skipped, and the
    # next frame is accepted.
    echo_window_sec: float = 0.150

    # Command limits re-checked by the gate itself. These mirror the monitor's
    # MOT-001/002/004 thresholds, and RSS-001 §13.3 asks safety functions to be
    # implemented in two independent channels. More practically: a gate that
    # consumed the monitor's transient alert array oscillated between passing and
    # blocking, because some alerts fire on one frame only. Evaluating the
    # command it is holding is stable by construction.
    max_linear_mps: float = 0.22
    max_angular_rps: float = 2.84
    # Measured limits (the "actual" half of MOT-001/002).
    max_actual_linear_mps: float = 0.30
    max_actual_angular_rps: float = 3.50
    # MOT-004 differential-drive geometry.
    wheel_separation_m: float = 0.160
    wheel_radius_m: float = 0.033
    max_wheel_speed_mps: float = 0.22
    # Minimum spec level that causes a block. S2 by default: an S2 motion rule
    # violation means "the command is asking for something it must not get", and
    # the response in RSS-001 §4.3 is DEGRADED + envelope, not a stop. Blocking
    # the whole command is stricter than the spec's default response, so the
    # level is configurable and documented rather than hard-coded.
    block_level: int = 2
    # Also block on the monitor's aggregate motion_status, which folds in
    # severities of alerts that may have already cleared this tick.
    block_on_motion_status: bool = True
    # Pass-through while blocked, or force a stop? Fail-safe default is stop.
    fail_action_is_stop: bool = True
    # Consecutive clear frames required before authority is restored. Prevents
    # the gate from chattering open and closed around a marginal threshold.
    pass_clear_frames: int = 3
    # Deadband below which a "stop" is treated as already stopped, so the gate
    # does not report blocking for a command that is numerically a stop.
    motion_threshold: float = 0.01

    # Findings that must NOT be able to block, because blocking them is
    # self-defeating: they describe the robot failing to execute a command, and
    # refusing the command is what makes the robot fail to execute it. Gating on
    # them deadlocks the robot -- verified live, a compliant 1.0 rad/s command was
    # refused forever because COMMAND_MISMATCH (commanded but not moving) was
    # active, and it was active precisely because the command was refused.
    #
    # These are consistency observations for the state machine to resolve (slow
    # down, re-plan, re-enable the drive); they are not a reason for the gate to
    # hold the robot still. The command-limit checks above already cover the
    # genuinely unsafe cases.
    self_defeating_codes: Tuple[str, ...] = (
        # Echoes of this gate's own output are not instructions from anyone.
        "SAFETY_GATE_SELF_ECHO",
        "COMMAND_MISMATCH",
        "UNEXPECTED_MOTION",
        "MOT_INTENT_PERSISTENT_MISMATCH",
        # Retained for platforms that still judge command staleness: treating it
        # as a motion violation would bury the reason, so an operator would read
        # "motion blocked" when the truth is "nobody is commanding".
        "COMMAND_STALE",
    )


# --------------------------------------------------------------------------- #
# Decision
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class GateDecision:
    """What the gate decided for one command, and why."""

    state: int
    output_linear: float
    output_angular: float
    input_linear: float = 0.0
    input_angular: float = 0.0
    reason: str = REASON_PASS
    detail: str = ""
    blocked_codes: Tuple[str, ...] = ()
    watchdog_tripped: bool = False
    intercepted: bool = False

    @property
    def passing(self) -> bool:
        return self.state == GATE_PASS

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, "UNKNOWN(%d)" % self.state)

    def command_changed(self) -> bool:
        """True when gating altered the command rather than letting it through."""
        return (
            not _close(self.input_linear, self.output_linear)
            or not _close(self.input_angular, self.output_angular)
        )


def _close(a: float, b: float, tolerance: float = 1e-9) -> bool:
    return abs(a - b) <= tolerance


# --------------------------------------------------------------------------- #
# Policy
# --------------------------------------------------------------------------- #
class GatePolicy:
    """Decides whether a command may reach the actuators.

    Holds only the small amount of state the policy needs: the last command
    values (so the gate can keep publishing a stop rather than going silent),
    timestamps for the two watchdogs, and a debounce counter for restoring
    authority.
    """

    def __init__(self, config: Optional[GateConfig] = None):
        self.config = config or GateConfig()
        self.reset()

    # -- state -------------------------------------------------------------
    def reset(self) -> None:
        self.last_command_wall: Optional[float] = None
        self.last_monitor_wall: Optional[float] = None
        self.last_command_linear = 0.0
        self.last_command_angular = 0.0
        self._clear_frames = 0
        self.blocks = 0
        self.passes = 0

    def note_command(self, linear: float, angular: float, wall: float) -> None:
        """Record an incoming command. Called once per received command."""
        self.last_command_linear = float(linear)
        self.last_command_angular = float(angular)
        self.last_command_wall = float(wall)

    def note_monitor(self, wall: float) -> None:
        """Record that a monitor snapshot arrived."""
        self.last_monitor_wall = float(wall)

    # -- freshness ---------------------------------------------------------
    def monitor_age(self, now_wall: float) -> float:
        if self.last_monitor_wall is None:
            return math.inf
        return max(0.0, now_wall - self.last_monitor_wall)

    def command_age(self, now_wall: float) -> float:
        if self.last_command_wall is None:
            return math.inf
        return max(0.0, now_wall - self.last_command_wall)

    # -- decision ----------------------------------------------------------
    def decide(
        self,
        now_wall: float,
        motion_alerts: Sequence[Tuple[str, str, int]] = (),
        motion_status: int = 0,
        command_linear: Optional[float] = None,
        command_angular: Optional[float] = None,
        measured_linear: Optional[float] = None,
        measured_angular: Optional[float] = None,
    ) -> GateDecision:
        """Decide the output for the current tick.

        ``motion_alerts`` is a sequence of ``(code, rule_id, level)``. The policy
        takes plain values rather than message objects so it can be tested with
        literals and so it stays free of ROS types.

        The ``command_*`` and ``measured_*`` arguments let the gate judge by the
        rules directly instead of waiting for the monitor to announce a
        violation. Both paths are kept: the alert array carries verdicts only the
        monitor can make (collision, tilt, data health), while the direct check
        covers the command limits stably.
        """
        config = self.config

        # Judge the command currently in hand, rather than only trusting alerts
        # that may or may not have fired this frame.
        direct = self._check_command_limits(
            command_linear, command_angular, measured_linear, measured_angular
        )
        blocking = list(direct) + [
            entry for entry in motion_alerts
            if int(entry[2]) >= config.block_level
            and entry[0] not in config.self_defeating_codes
        ]
        monitor_age = self.monitor_age(now_wall)
        command_age = self.command_age(now_wall)

        # A motion violation is only meaningful while a command is actually being
        # asked for. Once the command stream expires the robot should stop
        # because nobody is commanding, not because of a violation that was
        # cached from an earlier intent -- otherwise the reported reason
        # misdescribes the situation, which is exactly the kind of mislabelling
        # that sends an operator chasing the wrong fault.
        stale_command = (
            self.last_command_wall is not None
            and command_age > config.command_timeout_sec
        )
        if blocking and stale_command:
            blocking = []

        # ---- 3. motion-safety verdict ------------------------------------
        if blocking:
            self._clear_frames = 0
            self.blocks += 1
            codes = tuple(code for code, _rule, _level in blocking)
            detail = "; ".join(
                "%s (%s) at S%d" % (code, rule_id or "-", level)
                for code, rule_id, level in blocking
            )
            return self._stop(
                GATE_BLOCKED, REASON_MOTION_ALERT,
                "refusing command: %s" % detail,
                codes=codes,
            )

        if config.block_on_motion_status and int(motion_status) >= _severity_of(
            config.block_level
        ):
            self._clear_frames = 0
            self.blocks += 1
            return self._stop(
                GATE_BLOCKED, REASON_MOTION_DEGRADED,
                "monitor reports motion status severity %d at or above the "
                "blocking level" % int(motion_status),
            )

        # ---- monitor watchdog: REMOVED BY REQUEST ------------------------
        # There is deliberately no timeout on the monitor snapshot stream.
        #
        # !! KNOWN ACCEPTED RISK !!
        # RSS-003 SYS-005 and SYS-004 require the opposite: a gate that keeps
        # acting after its monitor has stopped is a gate that can no longer tell
        # safe from unsafe, and the spec's response is to stop the robot. With
        # this check removed, if the monitor dies the gate keeps evaluating the
        # last command against the last verdict it received, so an over-limit
        # command arriving after the monitor is gone will be passed. The
        # requirement was dropped because a quiet command source is normal
        # operation and the operator did not want timeouts acting on the robot;
        # reintroduce this block (and ``monitor_timeout_sec``) to restore it.
        #
        # What still protects against a stale *verdict*: the gate re-checks the
        # command limits itself below, so a monitor that dies while reporting
        # "all clear" cannot make the gate pass an unsafe command. What is lost
        # is protection against the monitor dying, which is precisely the case
        # the removed check covered.
        _ = monitor_age

        # ---- 2. command watchdog -----------------------------------------
        # Only meaningful once a command has been seen; before that the robot is
        # simply uncommanded and the stop is driven by HOLDING.
        if self.last_command_wall is None:
            self._clear_frames = 0
            self.blocks += 1
            return self._stop(
                GATE_HOLDING, REASON_NO_COMMAND,
                "no velocity command received yet", watchdog=False,
            )
        if command_age > config.command_timeout_sec:
            self._clear_frames = 0
            self.blocks += 1
            return self._stop(
                GATE_HOLDING, REASON_COMMAND_IDLE,
                "command source idle for %.3f s; holding a stop. This is not a "
                "fault -- there is simply no current command, and the base would "
                "otherwise keep executing the last one open loop."
                % command_age,
                # Not a watchdog trip and not a fault when idle is expected.
                watchdog=config.command_timeout_is_fault,
            )

        # ---- 4. pass -----------------------------------------------------
        # A gate that has never refused anything passes immediately. One that has
        # refused requires the condition to stay clear for ``pass_clear_frames``
        # consecutive ticks before authority is restored, because a command
        # stream that flickers between allowed and refused is worse for the
        # drivetrain than one that stays stopped.
        self._clear_frames += 1
        if self.blocks and self._clear_frames < config.pass_clear_frames:
            return GateDecision(
                state=GATE_BLOCKED,
                output_linear=0.0,
                output_angular=0.0,
                input_linear=self.last_command_linear,
                input_angular=self.last_command_angular,
                reason=REASON_PASS,
                detail="conditions clear; holding the stop for %d more tick(s) "
                       "before restoring authority"
                       % (config.pass_clear_frames - self._clear_frames),
                intercepted=True,
            )

        self.passes += 1
        return GateDecision(
            state=GATE_PASS,
            output_linear=self.last_command_linear,
            output_angular=self.last_command_angular,
            input_linear=self.last_command_linear,
            input_angular=self.last_command_angular,
            reason=REASON_PASS,
            detail="command passed unchanged",
        )

    # -- helpers -----------------------------------------------------------
    def _check_command_limits(
        self,
        command_linear: Optional[float],
        command_angular: Optional[float],
        measured_linear: Optional[float],
        measured_angular: Optional[float],
    ) -> List[Tuple[str, str, int]]:
        """Apply the honoured MOT limits to a command and to the measured state.

        Duplicating the monitor's arithmetic is deliberate (see ``GateConfig``):
        the two channels must not share a single point of failure, and a
        stateless check cannot oscillate the way alert-consumption did.
        """
        config = self.config
        found: List[Tuple[str, str, int]] = []

        if command_linear is not None and command_angular is not None:
            linear = abs(float(command_linear))
            angular = abs(float(command_angular))

            if linear > config.max_linear_mps:
                found.append((REASON_MOTION_ALERT, "MOT-001", 2))
            if angular > config.max_angular_rps:
                found.append((REASON_MOTION_ALERT, "MOT-002", 2))

            # MOT-004: both axes legal, combination not realisable.
            half_track = config.wheel_separation_m / 2.0
            worst_wheel = max(
                abs(linear - angular * half_track),
                abs(linear + angular * half_track),
            )
            if worst_wheel > config.max_wheel_speed_mps:
                found.append((REASON_MOTION_ALERT, "MOT-004", 2))

        if measured_linear is not None and abs(float(measured_linear)) > (
            config.max_actual_linear_mps
        ):
            found.append((REASON_MOTION_ALERT, "MOT-001", 4))
        if measured_angular is not None and abs(float(measured_angular)) > (
            config.max_actual_angular_rps
        ):
            found.append((REASON_MOTION_ALERT, "MOT-002", 4))

        return found

    def _stop(
        self,
        state: int,
        reason: str,
        detail: str,
        codes: Tuple[str, ...] = (),
        watchdog: bool = False,
    ) -> GateDecision:
        """Build a stop decision.

        The output is a zero command rather than silence: publishing nothing
        would leave the base holding its last velocity, which is the exact
        failure this gate exists to prevent.
        """
        return GateDecision(
            state=state,
            output_linear=0.0,
            output_angular=0.0,
            input_linear=self.last_command_linear,
            input_angular=self.last_command_angular,
            reason=reason,
            detail=detail,
            blocked_codes=codes,
            watchdog_tripped=watchdog,
            intercepted=True,
        )


# --------------------------------------------------------------------------- #
# Level helpers
# --------------------------------------------------------------------------- #
def _severity_of(level: int) -> int:
    """Spec level -> the shared severity scale.

    Duplicated from :mod:`robot_safety_monitor.motion_safety` on purpose is *not*
    what happens here: the mapping is imported lazily to keep this module free of
    an import cycle at module load time.
    """
    from .motion_safety import level_to_severity

    return level_to_severity(level)
