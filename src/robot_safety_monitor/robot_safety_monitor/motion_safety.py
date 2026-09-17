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

"""Motion-safety rule evaluation -- RSS-003 §1, category MOT.

Scope, stated plainly: this module **identifies** motion-safety violations and
raises an alert for each. It does **not** act on them. No command is rejected,
clamped or zeroed here. Acting is the job of the safety state machine (layer 2)
and the gate (layer 3) of the architecture in RSS-001 §3.1, and keeping that
separation is what stops the observation layer from becoming a single point of
failure.

Implemented rules, exactly the four the specification selects for this phase:

=========== ========================================================= ==========
Rule        Judgement                                                 Severity
=========== ========================================================= ==========
MOT-001     commanded linear speed > v_max_cmd                       S2
            measured linear speed  > v_max_actual                    S4
MOT-002     commanded angular speed > w_max_cmd                      S2
            measured angular speed  > w_max_actual                   S4
MOT-003     commanded |delta v|/dt > a_max, |delta w|/dt > alpha_max S2
MOT-004     commanded twist needs a wheel speed above the wheel limit S2
=========== ========================================================= ==========

The command/measured split in MOT-001/002 is the point of those rules, not an
implementation detail. A command over the rated value means the layer above is
asking for something it should not (reject it). A *measured* value over the
limit means the base is physically doing something unsafe -- encoder fault, a
runaway controller -- and that is a different, more urgent failure, hence S4.
The measured limit is deliberately looser than the commanded one so that
odometry noise and control overshoot do not raise a false emergency.

Rules explicitly deferred by RSS-003 §8 and therefore NOT implemented here:
``MOT-005`` (jerk), ``MOT-009`` (unarmed command), ``MOT-010`` (tilt),
``MOT-011`` (rotation clearance), ``MOT-012`` (intent not realised).

Two of those deserve a note because their *effect* is already present elsewhere:

* ``MOT-010`` tilt is already annunciated by :mod:`robot_safety_monitor.analyzer`
  as ``TILT_HIGH`` / ``TILT_CRITICAL`` (RSS-001 §15.1 lists it as implemented).
  Re-emitting it here would produce two alerts for one fact.
* ``MOT-012`` intent/execution mismatch is already annunciated by the analyzer as
  ``COMMAND_MISMATCH`` / ``UNEXPECTED_MOTION``.

Like :mod:`robot_safety_monitor.analyzer`, this module has no ROS dependency and
takes the current time as an argument, so it is unit-testable without a graph and
reusable by the state machine.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

from .analyzer import (
    CATEGORY_MOT,
    CRITICAL,
    ERROR,
    OK,
    STALE,
    UNKNOWN,
    MonitorState,
    worst_status,
)

# --------------------------------------------------------------------------- #
# Alert codes (RSS-003 §1 `diagnostics` fields; naming per RSS-001 appendix B)
# --------------------------------------------------------------------------- #
CODE_LIN_VEL_EXCEED = "MOT_LIN_VEL_EXCEED"                 # MOT-001, command
CODE_ACTUAL_VEL_EXCEED = "MOT_ACTUAL_VEL_EXCEED"           # MOT-001, measured
CODE_ANG_VEL_EXCEED = "MOT_ANG_VEL_EXCEED"                 # MOT-002, command
CODE_ACTUAL_ANG_VEL_EXCEED = "MOT_ACTUAL_ANG_VEL_EXCEED"   # MOT-002, measured
CODE_LIN_ACCEL_EXCEED = "MOT_LIN_ACCEL_EXCEED"             # MOT-003
CODE_ANG_ACCEL_EXCEED = "MOT_ANG_ACCEL_EXCEED"             # MOT-003
CODE_TWIST_INFEASIBLE = "MOT_TWIST_INFEASIBLE"             # MOT-004

# Spec levels (RSS-001 §4.3).
LEVEL_S1 = 1
LEVEL_S2 = 2
LEVEL_S3 = 3
LEVEL_S4 = 4

LEVEL_NAMES = {
    LEVEL_S1: "S1",
    LEVEL_S2: "S2",
    LEVEL_S3: "S3",
    LEVEL_S4: "S4",
}

_LEVEL_TO_SEVERITY = {
    # S1 is a notice: RSS-001 §4.3 gives it "target state: unchanged", so it must
    # not drag the aggregate verdict away from OK. UNKNOWN is the value for "no
    # impact", which is exactly what a notice has; using a non-OK level here
    # would make every informational alert look like a degradation.
    LEVEL_S1: UNKNOWN,
    LEVEL_S2: ERROR,      # warning: DEGRADED + speed envelope
    LEVEL_S3: CRITICAL,   # protective stop
    LEVEL_S4: CRITICAL,   # safe/emergency stop
}


def level_to_severity(level: int) -> int:
    """Map a spec level S1..S4 onto the shared status scale.

    Both S3 and S4 become CRITICAL. The distinction between a recoverable
    protective stop and a latched safe stop is a state-machine concern, and the
    alert's ``level`` field preserves it for whoever needs it.
    """
    return _LEVEL_TO_SEVERITY.get(int(level), UNKNOWN)


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class MotionSafetyConfig:
    """Thresholds for MOT-001 .. MOT-004.

    Every field carries its calibration source, because RSS-001 §14.2 requires
    each threshold to trace to one of: standard, manufacturer data, measurement,
    or risk assessment. The values here are TurtleBot3 Burger manufacturer
    ratings.

    RSS-003 §0 marks the geometry (``wheel_separation_m``, ``wheel_radius_m``)
    as ``measured``: they feed MOT-004 directly, so an inaccurate pair makes the
    reachability verdict wrong in both directions. Measure them before relying
    on MOT-004.
    """

    # -- MOT-001 / MOT-002 limits (manufacturer rating) -----------------------
    max_linear_mps: float = 0.22          # v_max_cmd
    max_angular_rps: float = 2.84         # w_max_cmd
    # Measured-value limits. Looser than the commanded limits on purpose: their
    # job is to catch a physically runaway base, not control overshoot.
    max_actual_linear_mps: float = 0.30   # v_max_actual -> S4
    max_actual_angular_rps: float = 3.50  # w_max_actual -> S4

    # -- MOT-003 acceleration limits -----------------------------------------
    max_linear_accel_mps2: float = 0.50
    max_angular_accel_rps2: float = 3.00
    # Interval floor for the finite difference. Without it, two command frames
    # that happen to arrive almost together produce a huge phantom acceleration
    # -- the spec calls this out explicitly as the reason for the floor.
    min_accel_dt_sec: float = 0.020

    # -- MOT-004 differential-drive geometry (must be measured) ---------------
    wheel_separation_m: float = 0.160
    wheel_radius_m: float = 0.033
    max_wheel_speed_mps: float = 0.22

    # -- command freshness ----------------------------------------------------
    # MOT-001..004 are all statements about the *commanded* value, so they only
    # make sense while a fresh command exists. The timeout matches
    # ``timeout.cmd_vel_sec`` (RSS-003 §0 puts it at 0.50 s).
    command_timeout_sec: float = 0.50
    command_motion_threshold: float = 0.01


# --------------------------------------------------------------------------- #
# Alerts
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class MotionAlert:
    """One triggered MOT rule.

    ``value`` and ``threshold`` carry the evidence, so a log line or a UI can
    show *why* the rule fired without re-deriving it from the raw topics.
    """

    code: str
    rule_id: str
    level: int
    detail: str
    value: float = 0.0
    threshold: float = 0.0
    escalated: bool = False
    stamp_sec: float = 0.0
    wall_time_sec: float = 0.0
    # Every rule in this module belongs to the motion-safety category, but the
    # field is carried explicitly so a report can group MOT alerts and analyzer
    # findings through one uniform path instead of special-casing their origin.
    category: str = CATEGORY_MOT

    @property
    def severity(self) -> int:
        return level_to_severity(self.level)


# --------------------------------------------------------------------------- #
# Inputs
# --------------------------------------------------------------------------- #
@dataclass
class MotionInputs:
    """The subset of the snapshot the MOT rules reason about.

    Decoupling the rules from :class:`MonitorState` keeps them testable with
    hand-written numbers and makes their required inputs explicit.
    """

    command_available: bool = False
    command_fresh: bool = False
    cmd_linear: float = 0.0
    cmd_angular: float = 0.0
    odom_available: bool = False
    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0

    @classmethod
    def from_state(cls, state: MonitorState) -> "MotionInputs":
        command = state.command
        odom = state.odom
        return cls(
            command_available=command.available,
            command_fresh=command.fresh,
            cmd_linear=command.linear_x,
            cmd_angular=command.angular_z,
            odom_available=odom.available,
            speed_mps=odom.speed_mps,
            yaw_rate_rps=odom.yaw_rate_rps,
        )


# --------------------------------------------------------------------------- #
# MOT-004: differential-drive kinematics
# --------------------------------------------------------------------------- #
def wheel_speeds(
    linear_mps: float, angular_rps: float, config: MotionSafetyConfig
) -> Tuple[float, float]:
    """Return ``(v_left, v_right)`` wheel linear speeds for a body twist.

    Straight from RSS-003 MOT-004 ``derived``:
    ``v_left = v - (W/2)*w``, ``v_right = v + (W/2)*w``.
    """
    half_track = config.wheel_separation_m / 2.0
    return (
        linear_mps - angular_rps * half_track,
        linear_mps + angular_rps * half_track,
    )


def required_wheel_speed(
    linear_mps: float, angular_rps: float, config: MotionSafetyConfig
) -> float:
    """Largest wheel linear speed the commanded twist demands."""
    left, right = wheel_speeds(linear_mps, angular_rps, config)
    return max(abs(left), abs(right))


def twist_is_reachable(
    linear_mps: float, angular_rps: float, config: MotionSafetyConfig
) -> Tuple[bool, float]:
    """Judge whether a command lies inside the platform's reachable speed set.

    Returns ``(reachable, required_wheel_speed)``.

    This is the rule's whole point: ``v`` and ``w`` can each be inside their own
    limit while their *combination* is impossible. At 0.22 m/s and 2.84 rad/s the
    outer wheel would have to turn at about 0.45 m/s against a 0.22 m/s wheel
    rating. The danger is not the impossible motion itself but the ambiguity it
    creates: the controller integrates against a target the base cannot reach, so
    "command sent, base not moving" appears -- which then looks like a mechanical
    fault to a mismatch detector.
    """
    required = required_wheel_speed(linear_mps, angular_rps, config)
    return required <= config.max_wheel_speed_mps, required


# --------------------------------------------------------------------------- #
# Evaluator
# --------------------------------------------------------------------------- #
@dataclass
class _CommandSample:
    linear: float
    angular: float
    stamp: float
    wall: float


# --------------------------------------------------------------------------- #
# Command-level limit evaluation (shared by the monitor and the gate)
# --------------------------------------------------------------------------- #
def command_limit_alerts(
    linear: float,
    angular: float,
    config: MotionSafetyConfig,
) -> List[Tuple[str, str, int]]:
    """Stateless check of one command against MOT-001/002 (command half) and MOT-004.

    Returns ``(code, rule_id, level)`` triples. Being stateless is the point: a
    gate must be able to ask "is this command acceptable" of the command it is
    holding, on every tick, without depending on whether a transient monitor
    alert happened to fire on that particular frame. An earlier gate design
    consumed the monitor's alert array directly and oscillated between passing
    and blocking, because MOT_TWIST_INFEASIBLE fires on the frame the command
    arrives and clears on the next one.

    The monitor calls this too, so both components judge by the same rules.
    """
    alerts: List[Tuple[str, str, int]] = []
    linear = abs(float(linear))
    angular = abs(float(angular))

    if linear > config.max_linear_mps:
        alerts.append((CODE_LIN_VEL_EXCEED, "MOT-001", LEVEL_S2))
    if angular > config.max_angular_rps:
        alerts.append((CODE_ANG_VEL_EXCEED, "MOT-002", LEVEL_S2))

    reachable, _required = twist_is_reachable(linear, angular, config)
    if not reachable:
        alerts.append((CODE_TWIST_INFEASIBLE, "MOT-004", LEVEL_S2))

    return alerts


class MotionSafetyMonitor:
    """Stateful evaluator for MOT-001 .. MOT-004.

    State is needed only by MOT-003, which is about the *change* between
    consecutive commands. Keeping it here leaves :class:`MonitorState` a pure
    value object.
    """

    def __init__(self, config: Optional[MotionSafetyConfig] = None):
        self.config = config or MotionSafetyConfig()
        # Bounded history: MOT-003 needs only the previous sample, but a small
        # window leaves room for the smoothing RSS-003 §9 item 5 asks about
        # without a redesign.
        self._history: Deque[_CommandSample] = deque(maxlen=16)
        self.last_rules_evaluated = 0
        # Last values seen by MOT-003, for diagnostics. Not used by any rule.
        self.last_dt: Optional[float] = None
        self.last_accel: Optional[Tuple[float, float]] = None

    # -- diagnostics -------------------------------------------------------
    def reset(self) -> None:
        """Forget command history. Used by tests and after a restart."""
        self._history.clear()

    @property
    def previous_command(self) -> Optional[_CommandSample]:
        """The last distinct command seen, or ``None``."""
        return self._history[-1] if self._history else None

    # -- helpers -----------------------------------------------------------
    def _observe_command(
        self, inputs: MotionInputs, now_wall: float, now_stamp: float
    ) -> None:
        """Record this frame's command so the next tick can difference against it.

        Every frame is recorded, including frames that repeat the previous value.
        That is what the spec's difference quotient assumes: it is defined over
        two consecutive command frames using their real interval. Skipping
        repeats would mean measuring a step against the last *change*, so a
        command that held a value for a while and then stepped would be
        differenced over the whole hold and the acceleration would be diluted by
        an order of magnitude -- a missed violation, which is the worst kind of
        bug for a safety monitor.
        """
        if not (inputs.command_available and inputs.command_fresh):
            return
        self._history.append(
            _CommandSample(
                linear=inputs.cmd_linear,
                angular=inputs.cmd_angular,
                stamp=now_stamp,
                wall=now_wall,
            )
        )

    def _delta_t(self) -> Optional[float]:
        """Interval between the last two command frames, or ``None``.

        The wall clock is preferred because it is monotonic and unaffected by a
        simulator pause; the message stamp is the fallback. Either way the value
        is floored at ``min_accel_dt_sec`` so a near-zero interval cannot
        manufacture an enormous acceleration.

        Returns ``None`` when the gap exceeds the command timeout. Such a gap
        means the command source went quiet, so the two frames do not belong to
        one continuous intent and differencing them yields a meaningless rate.
        That meaningless rate is usually *small*, which silently swallows a real
        step command issued when the stream resumes: a 0 -> 0.22 m/s step
        measured across a 0.5 s silence reads as 0.44 m/s^2 and stays just under
        the 0.5 m/s^2 limit. Not measurable is reported as not measurable.
        """
        if len(self._history) < 2:
            return None
        newest = self._history[-1]
        previous = self._history[-2]

        dt = newest.wall - previous.wall
        if not math.isfinite(dt) or dt <= 0.0:
            dt = newest.stamp - previous.stamp
        if not math.isfinite(dt) or dt <= 0.0:
            return None
        if dt > self.config.command_timeout_sec:
            return None
        if dt < self.config.min_accel_dt_sec:
            # A near-zero interval would imply an absurd rate from an ordinary
            # command; the spec's floor exists exactly to suppress that.
            dt = self.config.min_accel_dt_sec
        return dt

    # -- main entry point --------------------------------------------------
    def evaluate(
        self,
        state: MonitorState,
        now_wall: float,
        now_stamp: Optional[float] = None,
    ) -> List[MotionAlert]:
        """Evaluate MOT-001 .. MOT-004 and return the alerts that fired.

        An empty list means "no rule fired", not "the robot is definitely safe":
        rules whose inputs are missing are skipped, and the caller can see how
        many were actually judged through :attr:`last_rules_evaluated`.
        """
        config = self.config
        now_stamp = state.now_stamp if now_stamp is None else now_stamp
        inputs = MotionInputs.from_state(state)

        alerts: List[MotionAlert] = []

        def emit(
            code: str,
            rule_id: str,
            level: int,
            detail: str,
            value: float,
            threshold: float,
        ) -> None:
            alerts.append(
                MotionAlert(
                    code=code,
                    rule_id=rule_id,
                    level=level,
                    detail=detail,
                    value=value,
                    threshold=threshold,
                    stamp_sec=now_stamp,
                    wall_time_sec=now_wall,
                )
            )

        self._observe_command(inputs, now_wall, now_stamp)
        command_usable = inputs.command_available and inputs.command_fresh
        evaluated = 0

        # ---- MOT-001: linear speed limit ---------------------------------
        # Command and measured are separate judgements by design; see the module
        # docstring for why they carry different severities.
        linear_cmd = abs(inputs.cmd_linear)
        if command_usable:
            evaluated += 1
            if linear_cmd > config.max_linear_mps:
                emit(
                    CODE_LIN_VEL_EXCEED, "MOT-001", LEVEL_S2,
                    "commanded linear speed %.3f m/s exceeds the rated %.3f m/s"
                    % (linear_cmd, config.max_linear_mps),
                    linear_cmd, config.max_linear_mps,
                )
        if inputs.odom_available:
            evaluated += 1
            linear_actual = abs(inputs.speed_mps)
            if linear_actual > config.max_actual_linear_mps:
                emit(
                    CODE_ACTUAL_VEL_EXCEED, "MOT-001", LEVEL_S4,
                    "measured linear speed %.3f m/s exceeds the safe-stop limit "
                    "%.3f m/s" % (linear_actual, config.max_actual_linear_mps),
                    linear_actual, config.max_actual_linear_mps,
                )

        # ---- MOT-002: angular speed limit --------------------------------
        angular_cmd = abs(inputs.cmd_angular)
        if command_usable:
            evaluated += 1
            if angular_cmd > config.max_angular_rps:
                emit(
                    CODE_ANG_VEL_EXCEED, "MOT-002", LEVEL_S2,
                    "commanded angular speed %.3f rad/s exceeds the rated "
                    "%.3f rad/s" % (angular_cmd, config.max_angular_rps),
                    angular_cmd, config.max_angular_rps,
                )
        if inputs.odom_available:
            evaluated += 1
            angular_actual = abs(inputs.yaw_rate_rps)
            if angular_actual > config.max_actual_angular_rps:
                emit(
                    CODE_ACTUAL_ANG_VEL_EXCEED, "MOT-002", LEVEL_S4,
                    "measured angular speed %.3f rad/s exceeds the safe-stop "
                    "limit %.3f rad/s"
                    % (angular_actual, config.max_actual_angular_rps),
                    angular_actual, config.max_actual_angular_rps,
                )

        # ---- MOT-003: acceleration limit ---------------------------------
        # Preconditions per spec: both commands available, and dt >= the floor.
        dt = self._delta_t()
        self.last_dt = dt
        if command_usable and dt is not None:
            previous = self._history[-2]
            current = self._history[-1]
            evaluated += 1

            linear_accel = abs(current.linear - previous.linear) / dt
            self.last_accel = (linear_accel, abs(current.angular - previous.angular) / dt)
            if linear_accel > config.max_linear_accel_mps2:
                emit(
                    CODE_LIN_ACCEL_EXCEED, "MOT-003", LEVEL_S2,
                    "commanded linear acceleration %.3f m/s^2 exceeds %.3f m/s^2 "
                    "(%.3f -> %.3f m/s over %.3f s)"
                    % (
                        linear_accel, config.max_linear_accel_mps2,
                        previous.linear, current.linear, dt,
                    ),
                    linear_accel, config.max_linear_accel_mps2,
                )

            angular_accel = abs(current.angular - previous.angular) / dt
            if angular_accel > config.max_angular_accel_rps2:
                emit(
                    CODE_ANG_ACCEL_EXCEED, "MOT-003", LEVEL_S2,
                    "commanded angular acceleration %.3f rad/s^2 exceeds "
                    "%.3f rad/s^2 (%.3f -> %.3f rad/s over %.3f s)"
                    % (
                        angular_accel, config.max_angular_accel_rps2,
                        previous.angular, current.angular, dt,
                    ),
                    angular_accel, config.max_angular_accel_rps2,
                )

        # ---- MOT-004: differential-drive reachability --------------------
        # Depends on MOT-001/002 per RSS-003 §7: the single-axis limits are
        # judged first, and this rule catches combinations they both pass.
        if command_usable:
            evaluated += 1
            reachable, required = twist_is_reachable(
                inputs.cmd_linear, inputs.cmd_angular, config
            )
            if not reachable:
                emit(
                    CODE_TWIST_INFEASIBLE, "MOT-004", LEVEL_S2,
                    "command (v=%.3f m/s, w=%.3f rad/s) needs a wheel speed of "
                    "%.3f m/s, above the %.3f m/s single-wheel limit"
                    % (
                        inputs.cmd_linear, inputs.cmd_angular,
                        required, config.max_wheel_speed_mps,
                    ),
                    required, config.max_wheel_speed_mps,
                )

        self.last_rules_evaluated = evaluated
        return alerts

    # -- aggregation -------------------------------------------------------
    @staticmethod
    def worst_severity(alerts: List[MotionAlert]) -> int:
        """Worst severity among the alerts, or ``OK`` when there are none."""
        if not alerts:
            return OK
        return worst_status(alert.severity for alert in alerts)
