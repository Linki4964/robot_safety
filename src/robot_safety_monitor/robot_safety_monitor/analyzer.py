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

"""ROS-independent core of the runtime safety monitor.

Everything in this module is plain Python: no ``rclpy``, no message types, no
wall-clock or ROS-time lookups. Callers pass the current wall time in
explicitly. That has three consequences the project depends on:

1. The judgement logic can be unit tested without a running ROS graph.
2. The safety state machine of step 2 can reuse the same predicates instead of
   re-deriving them from raw topics.
3. Time is a parameter, not an ambient fact, so "what does the monitor conclude
   at time t" is reproducible.

Two clocks appear throughout, and conflating them is the classic bug in a
simulation-based monitor:

``stamp``
    ROS time carried by the message header. Under Gazebo this is simulated
    time, which can run faster than wall time, or stop entirely.
``wall_time``
    The monitor's monotonic-ish wall clock reading when the sample arrived.

Staleness, watchdogs and timeout checks always use ``wall_time``. A paused
simulator stops publishing, and only wall-clock arrival time reveals that.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------- #
# Status levels
# --------------------------------------------------------------------------- #
# Plain integers rather than an Enum: these values travel through ROS messages
# as uint8, and keeping them as ints removes a conversion step in the node.
UNKNOWN = 0
OK = 1
STALE = 2
ERROR = 3
CRITICAL = 4

STATUS_NAMES = {
    UNKNOWN: "UNKNOWN",
    OK: "OK",
    STALE: "STALE",
    ERROR: "ERROR",
    CRITICAL: "CRITICAL",
}

# Canonical source identifiers. The node reuses these as its tracking keys and
# the state machine will match on them, so they are part of the module API.
SOURCE_ODOM = "odom"
SOURCE_SCAN = "scan"
SOURCE_IMU = "imu"
SOURCE_JOINTS = "joint_states"
SOURCE_COMMAND = "command"
SOURCE_BATTERY = "battery"

ALL_SOURCES: Tuple[str, ...] = (
    SOURCE_ODOM,
    SOURCE_SCAN,
    SOURCE_IMU,
    SOURCE_JOINTS,
    SOURCE_COMMAND,
)


def status_name(status: int) -> str:
    """Return the printable name of a status code, tolerating unknown values."""
    return STATUS_NAMES.get(int(status), "UNKNOWN(%d)" % int(status))


def worst_status(statuses: Iterable[int]) -> int:
    """Return the highest-urgency status in ``statuses`` (UNKNOWN when empty)."""
    worst = UNKNOWN
    for status in statuses:
        if int(status) > worst:
            worst = int(status)
    return worst


# --------------------------------------------------------------------------- #
# Quaternion / geometry helpers
# --------------------------------------------------------------------------- #
def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to the half-open interval ``(-pi, pi]``."""
    if not math.isfinite(angle):
        return 0.0
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw (rotation about z) from a unit quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


def quaternion_to_roll_pitch(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float]:
    """Extract roll and pitch from a unit quaternion, both wrapped to (-pi, pi]."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return normalize_angle(roll), normalize_angle(pitch)


def tilt_angle_deg(roll: float, pitch: float) -> float:
    """Angle in degrees between the body z axis and the world z axis.

    Yaw is deliberately excluded: a robot rotated in place is level, and only
    roll/pitch indicate climbing, tipping or a fall. ``acos(cos(pitch)cos(roll))``
    is the exact angle between the two axes for the ZYX convention.
    """
    cos_tilt = math.cos(pitch) * math.cos(roll)
    cos_tilt = max(-1.0, min(1.0, cos_tilt))
    return math.degrees(math.acos(cos_tilt))


# Covariance values at or above this magnitude are treated as "this degree of
# freedom is not observed" rather than as a very large uncertainty. Diff-drive
# odometry uses exactly this convention: the Gazebo plugin publishes 1e12 for z,
# roll and pitch, which must not be reported as a degraded estimate.
UNOBSERVED_COVARIANCE = 1.0e9


def covariance_diagonal(covariance: Sequence[float]) -> List[float]:
    """Extract the diagonal of a row-major 6x6 (or NxN) covariance matrix.

    Returns an empty list unless the input really is a complete square matrix,
    so callers can treat "no covariance information" as a distinct case rather
    than as zeros.

    The length is probed with ``len()`` and an explicit ``None`` check rather
    than a truthiness test: generated ROS messages expose this field as a numpy
    array (or ``array.array``), and evaluating ``not covariance`` on a
    multi-element numpy array raises ``ValueError``.
    """
    if covariance is None:
        return []
    size = len(covariance)
    dimension = int(round(math.sqrt(size))) if size > 0 else 0
    if dimension == 0 or dimension * dimension != size:
        return []
    return [float(covariance[i * dimension + i]) for i in range(dimension)]


def variance_or_negative_one(covariance: Sequence[float], index: int) -> float:
    """Return covariance diagonal element ``index``, or -1 when unusable.

    ``-1`` covers three distinct situations that the monitor treats alike:
    the element is absent, it is not finite, or it carries the "unobserved"
    sentinel. Callers that need to distinguish them can read
    :func:`covariance_diagonal` directly.

    Note the loop variable in the helper below is deliberately not named
    ``index``: shadowing this argument would silently clamp every request to the
    last diagonal slot.
    """
    diagonal = covariance_diagonal(covariance)
    if index < 0 or index >= len(diagonal):
        return -1.0
    value = diagonal[index]
    if not math.isfinite(value) or value < 0.0:
        return -1.0
    if value >= UNOBSERVED_COVARIANCE:
        return -1.0
    return value


def covariance_is_sentinel(value: float) -> bool:
    """True when ``value`` is the 'degree of freedom not observed' sentinel."""
    return math.isfinite(value) and value >= UNOBSERVED_COVARIANCE


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class Config:
    """All monitor thresholds. Every value is overridable from ROS parameters."""

    # --- staleness watchdogs, wall-clock seconds -----------------------------
    odom_timeout_sec: float = 1.0
    scan_timeout_sec: float = 1.0
    imu_timeout_sec: float = 1.0
    joints_timeout_sec: float = 2.0
    command_timeout_sec: float = 1.0
    battery_timeout_sec: float = 5.0

    # --- sample rate estimation ---------------------------------------------
    rate_window_sec: float = 5.0

    # --- required vs optional sources ---------------------------------------
    # A required source that never appears is a CRITICAL finding; an optional
    # one is an ERROR at most. This keeps the monitor usable on platforms that
    # legitimately lack, say, an IMU or a battery topic.
    required_sources: Tuple[str, ...] = (SOURCE_ODOM, SOURCE_SCAN)
    optional_sources: Tuple[str, ...] = (
        SOURCE_IMU,
        SOURCE_JOINTS,
        SOURCE_COMMAND,
        SOURCE_BATTERY,
    )
    monitor_battery: bool = False
    # Command monitoring is on by default, because a commander that was
    # publishing and then stopped is a genuine fault that the safety layer must
    # see. Note the consequence: with no publisher on /cmd_vel at all, the
    # snapshot carries a permanent COMMAND_STALE finding. That is the honest
    # reading ("nobody is commanding this robot"), and the brief documents how to
    # silence it on a platform that has no commander by design.
    monitor_command: bool = True

    # --- battery thresholds --------------------------------------------------
    battery_warn_fraction: float = 0.30
    battery_critical_fraction: float = 0.15

    # --- attitude thresholds -------------------------------------------------
    tilt_warn_deg: float = 15.0
    tilt_critical_deg: float = 45.0

    # --- obstacle thresholds (lidar) ----------------------------------------
    obstacle_warn_range_m: float = 0.30
    obstacle_critical_range_m: float = 0.15

    # --- odometry health -----------------------------------------------------
    pose_cov_warn: float = 5.0e-2
    twist_cov_warn: float = 5.0e-2

    # --- intent vs execution -------------------------------------------------
    # Absolute slack in m/s (and rad/s) before a command/motion mismatch counts.
    command_deadband_linear: float = 0.02
    command_deadband_angular: float = 0.10
    # Fraction of the commanded magnitude that may be missing before it counts.
    command_tolerance_fraction: float = 0.5
    # Minimum |commanded| to consider the robot "asked to move".
    command_motion_threshold: float = 0.01

    # --- joint activity ------------------------------------------------------
    joint_velocity_threshold: float = 1.0e-3

    # --- motion safety (RSS-003 §1, category MOT) ----------------------------
    # Read by robot_safety_monitor.motion_safety. They live here with the other
    # thresholds so there is a single place to calibrate, and so the rule module
    # can import them without a circular dependency.
    #
    # RSS-003 scopes this phase to MOT-001..MOT-004 only; the remaining MOT rules
    # are listed as deferred in that document's §8.
    mot_max_linear_mps: float = 0.22            # MOT-001 v_max_cmd (manufacturer)
    mot_max_angular_rps: float = 2.84           # MOT-002 w_max_cmd (manufacturer)
    # Measured-value limits: looser than the commanded ones, because their job is
    # to catch a physically runaway base rather than control overshoot.
    mot_max_actual_linear_mps: float = 0.30     # MOT-001 v_max_actual -> S4
    mot_max_actual_angular_rps: float = 3.50    # MOT-002 w_max_actual -> S4
    mot_max_linear_accel_mps2: float = 0.50     # MOT-003 a_max
    mot_max_angular_accel_rps2: float = 3.00    # MOT-003 alpha_max
    mot_min_accel_dt_sec: float = 0.020         # MOT-003 differencing floor
    mot_wheel_separation_m: float = 0.160       # MOT-004 W (must be measured)
    mot_wheel_radius_m: float = 0.033           # MOT-004 r_wheel (must be measured)
    mot_max_wheel_speed_mps: float = 0.22       # MOT-004 v_wheel_max
    mot_command_timeout_sec: float = 0.50       # command freshness for the rules

    def is_required(self, source: str) -> bool:
        """True when ``source`` missing is a CRITICAL rather than an ERROR."""
        return source in self.required_sources

    def motion_safety(self):
        """Build the MOT rule configuration from these thresholds."""
        from .motion_safety import MotionSafetyConfig

        return MotionSafetyConfig(
            max_linear_mps=self.mot_max_linear_mps,
            max_angular_rps=self.mot_max_angular_rps,
            max_actual_linear_mps=self.mot_max_actual_linear_mps,
            max_actual_angular_rps=self.mot_max_actual_angular_rps,
            max_linear_accel_mps2=self.mot_max_linear_accel_mps2,
            max_angular_accel_rps2=self.mot_max_angular_accel_rps2,
            min_accel_dt_sec=self.mot_min_accel_dt_sec,
            wheel_separation_m=self.mot_wheel_separation_m,
            wheel_radius_m=self.mot_wheel_radius_m,
            max_wheel_speed_mps=self.mot_max_wheel_speed_mps,
            command_timeout_sec=self.mot_command_timeout_sec,
            command_motion_threshold=self.command_motion_threshold,
        )


# --------------------------------------------------------------------------- #
# Observation tracking (staleness + measured rate)
# --------------------------------------------------------------------------- #
# Tolerance used when deciding whether a rate sample sits outside the sampling
# window. Chosen far above float64 representation error for elapsed times and
# far below any meaningful sampling interval.
_ANCHOR_EPSILON = 1.0e-9


def _sample_time(sample: Tuple[float, int]) -> float:
    """Sort key for rate samples: the wall-clock time of the sample."""
    return sample[0]


@dataclass
class TopicObservation:
    """Wall-clock evidence about one data source."""

    name: str
    topic: str = ""
    timeout_sec: float = 1.0
    required: bool = False
    # False when the platform has declared that this source is deliberately not
    # monitored (no battery topic, no commander). Such a source is neither
    # healthy nor failing, so it is excluded from health() entirely: reporting
    # it as ERROR would hold the whole snapshot at ERROR forever.
    monitored: bool = True

    observed: bool = False
    message_count: int = 0

    last_wall_time: Optional[float] = None
    last_stamp: float = 0.0
    last_payload: Dict[str, object] = field(default_factory=dict)

    # Sliding window of (wall_time, message_count) used for rate estimation.
    rate_samples: List[Tuple[float, int]] = field(default_factory=list)

    def age(self, now: float) -> float:
        """Wall-clock seconds since the last sample; ``inf`` when never seen."""
        if self.last_wall_time is None:
            return math.inf
        return max(0.0, now - self.last_wall_time)

    def fresh(self, now: float) -> bool:
        """True when a sample arrived within ``timeout_sec``."""
        return self.observed and self.age(now) <= self.timeout_sec

    def measured_rate(self, now: float, window_sec: float) -> float:
        """Observed arrival rate in Hz, or -1.0 when not yet measurable."""
        if len(self.rate_samples) < 2:
            return -1.0
        oldest_t, oldest_c = self.rate_samples[0]
        elapsed = now - oldest_t
        if elapsed < min(0.5, max(window_sec * 0.2, 1e-3)):
            return -1.0
        return (self.message_count - oldest_c) / elapsed

    def status(self, now: float) -> Tuple[int, str]:
        """Return ``(status, reason)`` for this source at wall time ``now``."""
        if not self.observed:
            reason = (
                "no sample received on %s" % (self.topic or self.name)
                if self.required
                else "optional source %s not present" % self.name
            )
            return (CRITICAL if self.required else ERROR), reason
        if self.fresh(now):            return OK, ""
        return STALE, "no sample for %.2fs (timeout %.2fs) on %s" % (
            self.age(now),
            self.timeout_sec,
            self.topic or self.name,
        )


class ObservationTracker:
    """Registry of :class:`TopicObservation` keyed by canonical source name.

    Also tracks a single optional "unregistered" bucket for topics the user
    configured that the monitor does not model in detail; those still need a
    watchdog so a missing aux topic is visible.
    """

    def __init__(self, config: Config, rate_window_sec: Optional[float] = None):
        self._config = config
        self._rate_window = (
            float(rate_window_sec)
            if rate_window_sec is not None
            else float(config.rate_window_sec)
        )
        self._observations: Dict[str, TopicObservation] = {}
        self._registration_order: List[str] = []

    # -- registration -------------------------------------------------------
    def register(
        self,
        name: str,
        topic: str,
        timeout_sec: float,
        required: Optional[bool] = None,
    ) -> TopicObservation:
        """Create (or replace) the tracking record for ``name``."""
        if required is None:
            required = self._config.is_required(name)
        observation = TopicObservation(
            name=name,
            topic=topic,
            timeout_sec=float(timeout_sec),
            required=bool(required),
        )
        if name not in self._observations:
            self._registration_order.append(name)
        self._observations[name] = observation
        return observation

    def get(self, name: str) -> Optional[TopicObservation]:
        return self._observations.get(name)

    def require(self, name: str) -> TopicObservation:
        observation = self._observations.get(name)
        if observation is None:
            raise KeyError("source %r is not registered with the tracker" % name)
        return observation

    def names(self) -> List[str]:
        return list(self._registration_order)

    def declare_not_monitored(self, name: str, timeout_sec: float) -> TopicObservation:
        """Record that ``name`` exists but is deliberately not monitored.

        The record keeps the source's identity and timeout for reporting, but
        :meth:`health` skips it, so "we chose not to watch this" is never
        confused with "this broke".
        """
        observation = self.register(name, "", timeout_sec)
        observation.monitored = False
        return observation

    def monitored_names(self) -> List[str]:
        """Registered sources that participate in the health assessment."""
        return [
            name
            for name in self._registration_order
            if self._observations[name].monitored
        ]

    # -- updates ------------------------------------------------------------
    def observe(
        self, name: str, wall_time: float, stamp: float = 0.0
    ) -> TopicObservation:
        """Record that a sample for ``name`` arrived now."""
        observation = self.require(name)
        observation.observed = True
        observation.message_count += 1
        observation.last_wall_time = float(wall_time)
        if stamp and math.isfinite(stamp):
            observation.last_stamp = float(stamp)

        samples = observation.rate_samples
        samples.append((float(wall_time), observation.message_count))
        cutoff = float(wall_time) - self._rate_window
        # Keep every sample inside the window plus exactly one older anchor, so
        # the rate spanning the window boundary stays computable. ``samples`` is
        # time-ordered, so the window start is found by binary search.
        #
        # The anchor test uses a tolerance because message times are floats:
        # a sample whose nominal time equals the cutoff can land a few ULP
        # below it, and without the tolerance the anchor would sometimes be
        # retained and sometimes not, letting the list creep upward forever.
        first_inside = bisect.bisect_left(samples, cutoff, key=_sample_time)
        if first_inside > 0:
            anchor_time = samples[first_inside - 1][0]
            if anchor_time < cutoff - _ANCHOR_EPSILON:
                keep_from = first_inside - 1
            else:
                keep_from = first_inside
            if keep_from > 0:
                del samples[:keep_from]
        return observation

    def set_payload(self, name: str, **values: object) -> None:
        """Attach derived values (position, ranges, ...) to a source record."""
        observation = self.require(name)
        observation.last_payload.update(values)

    def payload(self, name: str, key: str, default: object = None) -> object:
        observation = self._observations.get(name)
        if observation is None:
            return default
        return observation.last_payload.get(key, default)

    # -- assessment ---------------------------------------------------------
    def health(self, now: float) -> Dict[str, Tuple[int, str]]:
        """Return ``{source: (status, reason)}`` for every monitored source."""
        return {
            name: self._observations[name].status(now)
            for name in self.monitored_names()
        }

    def statuses(self, now: float) -> Dict[str, int]:
        return {
            name: status for name, (status, _reason) in self.health(now).items()
        }


# --------------------------------------------------------------------------- #
# Findings
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Finding:
    """One active safety-relevant judgement.

    ``severity`` uses the same codes as the status levels so a consumer can
    compare findings and statuses directly. ``code`` is the stable identifier
    that the step-2 state machine will switch on; ``detail`` is for humans.
    """

    code: str
    severity: int
    detail: str


# Finding codes, grouped by the layer of the robot they belong to.
CODE_ODOM_STALE = "ODOM_STALE"
CODE_ODOM_MISSING = "ODOM_MISSING"
CODE_SCAN_STALE = "SCAN_STALE"
CODE_SCAN_MISSING = "SCAN_MISSING"
CODE_IMU_STALE = "IMU_STALE"
CODE_IMU_MISSING = "IMU_MISSING"
CODE_JOINTS_STALE = "JOINTS_STALE"
CODE_COMMAND_STALE = "COMMAND_STALE"
CODE_BATTERY_STALE = "BATTERY_STALE"
CODE_BATTERY_LOW = "BATTERY_LOW"
CODE_BATTERY_CRITICAL = "BATTERY_CRITICAL"
CODE_TILT_HIGH = "TILT_HIGH"
CODE_TILT_CRITICAL = "TILT_CRITICAL"
CODE_OBSTACLE_NEAR = "OBSTACLE_NEAR"
CODE_OBSTACLE_CRITICAL = "OBSTACLE_CRITICAL"
CODE_POSE_UNCERTAIN = "POSE_UNCERTAIN"
CODE_TWIST_UNCERTAIN = "TWIST_UNCERTAIN"
CODE_COMMAND_MISMATCH = "COMMAND_MISMATCH"
CODE_UNEXPECTED_MOTION = "UNEXPECTED_MOTION"


@dataclass
class Assessment:
    """Result of judging one snapshot."""

    status: int = UNKNOWN
    findings: List[Finding] = field(default_factory=list)
    source_status: Dict[str, int] = field(default_factory=dict)
    source_reason: Dict[str, str] = field(default_factory=dict)

    @property
    def warnings(self) -> List[str]:
        return [finding.code for finding in self.findings]

    def codes(self) -> List[str]:
        return [finding.code for finding in self.findings]

    def has(self, code: str) -> bool:
        return any(finding.code == code for finding in self.findings)

    def severity_of(self, code: str) -> Optional[int]:
        for finding in self.findings:
            if finding.code == code:
                return finding.severity
        return None


# --------------------------------------------------------------------------- #
# Threshold evaluation helpers
# --------------------------------------------------------------------------- #
def _evaluate_high(
    value: Optional[float],
    warn: float,
    critical: float,
    warn_code: str,
    critical_code: str,
    label: str,
    unit: str,
) -> Tuple[int, List[Finding]]:
    """Judge a value where *large* is bad (tilt, obstacle proximity is separate)."""
    if value is None or not math.isfinite(value):
        return UNKNOWN, []
    if value >= critical:
        return CRITICAL, [
            Finding(
                critical_code,
                CRITICAL,
                "%s %.3f%s exceeds critical limit %.3f%s"
                % (label, value, unit, critical, unit),
            )
        ]
    if value >= warn:
        return ERROR, [
            Finding(
                warn_code,
                ERROR,
                "%s %.3f%s exceeds warning limit %.3f%s"
                % (label, value, unit, warn, unit),
            )
        ]
    return OK, []


def evaluate_tilt(
    tilt_deg: Optional[float], config: Config
) -> Tuple[int, List[Finding]]:
    """Judge body tilt; returns ``UNKNOWN`` when no attitude is available."""
    return _evaluate_high(
        tilt_deg,
        config.tilt_warn_deg,
        config.tilt_critical_deg,
        CODE_TILT_HIGH,
        CODE_TILT_CRITICAL,
        "tilt",
        "deg",
    )


def evaluate_obstacle(
    closest_range: Optional[float], config: Config
) -> Tuple[int, List[Finding]]:
    """Judge lidar proximity; *small* range is bad, so the comparison inverts.

    An infinite range means every beam returned nothing inside the sensor's
    maximum range, i.e. the space ahead is wide open. That is the safest
    possible reading, not an unknown one, so it must assess as OK rather than
    being folded into the "no data" case. The ``-1`` sentinel used for an
    unknown range is excluded explicitly for the same reason.
    """
    if closest_range is None or math.isnan(closest_range):
        return UNKNOWN, []
    if closest_range < 0.0:
        # Sentinel written by the publisher when no valid return exists.
        return UNKNOWN, []
    if math.isinf(closest_range):
        return OK, []
    if closest_range <= config.obstacle_critical_range_m:
        return CRITICAL, [
            Finding(
                CODE_OBSTACLE_CRITICAL,
                CRITICAL,
                "obstacle at %.3fm, inside critical bound %.3fm"
                % (closest_range, config.obstacle_critical_range_m),
            )
        ]
    if closest_range <= config.obstacle_warn_range_m:
        return ERROR, [
            Finding(
                CODE_OBSTACLE_NEAR,
                ERROR,
                "obstacle at %.3fm, inside warning bound %.3fm"
                % (closest_range, config.obstacle_warn_range_m),
            )
        ]
    return OK, []


def evaluate_battery(
    percentage: Optional[float],
    available: bool,
    config: Config,
) -> Tuple[int, List[Finding]]:
    """Judge state of charge. Unknown charge is not treated as a fault."""
    if not available or percentage is None or not math.isfinite(percentage):
        return UNKNOWN, []
    if percentage < 0.0:
        return UNKNOWN, []
    if percentage <= config.battery_critical_fraction:
        return CRITICAL, [
            Finding(
                CODE_BATTERY_CRITICAL,
                CRITICAL,
                "battery at %.1f%%, at or below critical %.1f%%"
                % (percentage * 100.0, config.battery_critical_fraction * 100.0),
            )
        ]
    if percentage <= config.battery_warn_fraction:
        return ERROR, [
            Finding(
                CODE_BATTERY_LOW,
                ERROR,
                "battery at %.1f%%, at or below warning %.1f%%"
                % (percentage * 100.0, config.battery_warn_fraction * 100.0),
            )
        ]
    return OK, []


def evaluate_pose_health(
    pose_cov_xx: float,
    pose_cov_yy: float,
    twist_cov_max: float,
    config: Config,
) -> Tuple[int, List[Finding]]:
    """Judge whether the odometry estimate is still trustworthy.

    A rising position covariance means the filter is drifting or was reset;
    acting on a pose the estimator no longer believes is a safety problem even
    though every topic is perfectly timely.

    Only *observable* degrees of freedom are judged. A diff-drive base cannot
    observe z, roll or pitch, and odometry marks those with a sentinel that is
    explicitly not a variance; treating it as one would flag every healthy run.
    """
    findings: List[Finding] = []
    worst = OK

    usable = [
        value
        for value in (pose_cov_xx, pose_cov_yy)
        if math.isfinite(value) and 0.0 <= value < UNOBSERVED_COVARIANCE
    ]
    if usable:
        largest_pose = max(usable)
        if largest_pose >= config.pose_cov_warn:
            findings.append(
                Finding(
                    CODE_POSE_UNCERTAIN,
                    ERROR,
                    "odom pose variance %.3g >= %.3g m^2"
                    % (largest_pose, config.pose_cov_warn),
                )
            )
            worst = ERROR

    twist_usable = (
        math.isfinite(twist_cov_max)
        and 0.0 <= twist_cov_max < UNOBSERVED_COVARIANCE
    )
    if twist_usable and twist_cov_max >= config.twist_cov_warn:
        findings.append(
            Finding(
                CODE_TWIST_UNCERTAIN,
                ERROR,
                "odom twist variance %.3g >= %.3g"
                % (twist_cov_max, config.twist_cov_warn),
            )
        )
        worst = ERROR
    return worst, findings


def evaluate_command_consistency(
    command_linear_x: Optional[float],
    command_angular_z: Optional[float],
    command_fresh: bool,
    actual_speed: Optional[float],
    actual_yaw_rate: Optional[float],
    config: Config,
    odom_available: bool = True,
    command_available: bool = True,
) -> Tuple[int, List[Finding], bool, float]:
    """Compare commanded intent with executed motion.

    Returns ``(status, findings, motion_expected, deviation)``.

    Three distinguishable situations, all of which the state machine needs:

    * command is fresh and non-zero, motion roughly matches  -> OK
    * command is fresh and non-zero, no motion               -> COMMAND_MISMATCH
      (stuck wheel, unpowered motor, e-stop, planner not connected to the base)
    * command is stale or zero, yet the robot moves          -> UNEXPECTED_MOTION
      (something other than the intended controller is driving the base)

    The comparison is intentionally conservative: it fires only when the
    mismatch is larger than both an absolute deadband and a fraction of the
    commanded magnitude, because odometry in simulation always lags a command
    by a few ticks.

    Two availability flags keep the verdict honest when an input is absent:

    ``odom_available``
        without execution data, a zero-looking velocity would be read as "the
        robot is disobeying" when there is nothing to compare against.
    ``command_available``
        without intent data, the "no fresh command" branch would otherwise report
        a verdict of *agreement* for a comparison that never happened.

    In either case the function returns ``UNKNOWN`` and raises no finding: the
    absent source is already reported by the staleness layer, and inventing a
    second, misleading finding here would bury the real cause.
    """
    if not odom_available or not command_available:
        return UNKNOWN, [], False, 0.0
    if actual_speed is None or not math.isfinite(actual_speed):
        return UNKNOWN, [], False, 0.0

    linear = 0.0 if command_linear_x is None else float(command_linear_x)
    angular = 0.0 if command_angular_z is None else float(command_angular_z)
    command_magnitude = max(abs(linear), abs(angular))
    motion_expected = bool(command_fresh) and command_magnitude >= (
        config.command_motion_threshold
    )

    actual_magnitude = max(
        abs(actual_speed),
        0.0 if actual_yaw_rate is None else abs(float(actual_yaw_rate)),
    )
    deviation = command_magnitude - actual_magnitude

    if not command_fresh:
        # No recent intent at all: any significant motion is unexplained.
        if actual_magnitude > config.command_deadband_angular:
            return (
                ERROR,
                [
                    Finding(
                        CODE_UNEXPECTED_MOTION,
                        ERROR,
                        "robot moving at %.3f while no fresh velocity command "
                        "is present" % actual_magnitude,
                    )
                ],
                motion_expected,
                0.0,
            )
        return OK, [], motion_expected, 0.0

    if not motion_expected:
        # Fresh stop command. Being stopped is fine; creeping is not.
        if actual_magnitude > max(
            config.command_deadband_linear, config.command_deadband_angular
        ):
            return (
                ERROR,
                [
                    Finding(
                        CODE_UNEXPECTED_MOTION,
                        ERROR,
                        "stop command active but robot still moving at %.3f"
                        % actual_magnitude,
                    )
                ],
                motion_expected,
                0.0,
            )
        return OK, [], motion_expected, 0.0

    tolerance = max(
        config.command_deadband_linear,
        config.command_tolerance_fraction * command_magnitude,
    )
    if deviation > tolerance:
        return (
            ERROR,
            [
                Finding(
                    CODE_COMMAND_MISMATCH,
                    ERROR,
                    "commanded magnitude %.3f but executed %.3f "
                    "(missing %.3f, tolerance %.3f)"
                    % (command_magnitude, actual_magnitude, deviation, tolerance),
                )
            ],
            motion_expected,
            deviation,
        )
    return OK, [], motion_expected, deviation


# --------------------------------------------------------------------------- #
# Snapshot and top-level assessment
# --------------------------------------------------------------------------- #
@dataclass
class MotionSample:
    """Latest odometry-derived motion, as seen by the analyser."""

    available: bool = False
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_rad: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    pose_cov_xx: float = -1.0
    pose_cov_yy: float = -1.0
    pose_cov_yawyaw: float = -1.0
    twist_cov_max: float = -1.0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class AttitudeSample:
    """Latest IMU-derived attitude."""

    available: bool = False
    orientation_available: bool = False
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    tilt_deg: float = 0.0
    ang_vel_x: float = 0.0
    ang_vel_y: float = 0.0
    ang_vel_z: float = 0.0
    lin_acc_x: float = 0.0
    lin_acc_y: float = 0.0
    lin_acc_z: float = 0.0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class RangeSample:
    """Latest lidar summary."""

    available: bool = False
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    closest_range: float = math.inf
    front_range: float = math.inf
    front_angle: float = 0.0
    valid_point_count: int = 0
    point_count: int = 0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class BatterySample:
    """Latest battery reading."""

    available: bool = False
    voltage_v: float = 0.0
    percentage: float = -1.0
    temperature_c: float = 0.0
    charging: bool = False
    power_supply_status: int = 0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class JointSample:
    """Latest joint-state summary."""

    available: bool = False
    joint_count: int = 0
    moving_joints: List[str] = field(default_factory=list)
    max_abs_velocity: float = 0.0
    total_abs_velocity: float = 0.0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class CommandSample:
    """Latest velocity command."""

    available: bool = False
    fresh: bool = False
    stop_command: bool = True
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0
    # False when consistency could not be judged (no odometry yet); distinguishes
    # "agrees" from "not checked", which a lone boolean cannot express.
    consistency_checked: bool = False
    consistent: bool = False
    deviation: float = 0.0
    last_stamp_sec: float = 0.0
    last_wall_time_sec: float = 0.0


@dataclass
class MonitorState:
    """Everything the monitor knows at one instant, before it is judged.

    This is the single object the step-2 safety state machine will be handed.
    It is frame- and ROS-independent on purpose.
    """

    now_wall: float = 0.0
    now_stamp: float = 0.0
    odom: MotionSample = field(default_factory=MotionSample)
    imu: AttitudeSample = field(default_factory=AttitudeSample)
    scan: RangeSample = field(default_factory=RangeSample)
    battery: BatterySample = field(default_factory=BatterySample)
    joints: JointSample = field(default_factory=JointSample)
    command: CommandSample = field(default_factory=CommandSample)


class Analyzer:
    """Judges :class:`MonitorState` snapshots into :class:`Assessment` results."""

    def __init__(self, config: Config, tracker: ObservationTracker):
        self._config = config
        self._tracker = tracker

    @property
    def config(self) -> Config:
        return self._config

    @property
    def tracker(self) -> ObservationTracker:
        return self._tracker

    def assess(self, state: MonitorState, now: Optional[float] = None) -> Assessment:
        """Judge ``state`` and return the overall assessment.

        The result is the *worst* status found, and the union of all findings,
        so a single degraded subsystem can never be hidden by several healthy
        ones. That is the right default for safety monitoring.
        """
        now = state.now_wall if now is None else float(now)
        assessment = Assessment()

        # Layer 1: is the data itself arriving? -----------------------------
        health = self._tracker.health(now)
        statuses: List[int] = []
        for name, (status, reason) in health.items():
            assessment.source_status[name] = status
            assessment.source_reason[name] = reason
            statuses.append(status)

            if status in (OK, UNKNOWN):
                continue

            # Two failure shapes need distinct codes, because the recovery
            # action differs: a source that never appeared is a configuration
            # or bring-up problem, while a source that went quiet mid-run is a
            # runtime failure (crashed driver, dropped link, frozen simulator).
            observation = self._tracker.get(name)
            never_seen = observation is not None and not observation.observed
            if never_seen:
                code = ABSENCE_CODES.get(name, _STALENESS_CODE.get(name, name))
            else:
                code = _STALENESS_CODE.get(name, name)
            assessment.findings.append(
                Finding(code, status, reason or "%s not healthy" % name)
            )

        # Layer 2: does the data describe a physically sane robot? ----------
        if state.odom.available:
            pose_status, pose_findings = evaluate_pose_health(
                state.odom.pose_cov_xx,
                state.odom.pose_cov_yy,
                state.odom.twist_cov_max,
                self._config,
            )
            statuses.append(pose_status)
            assessment.findings.extend(pose_findings)

        if state.imu.available and state.imu.orientation_available:
            tilt_status, tilt_findings = evaluate_tilt(
                state.imu.tilt_deg, self._config
            )
            statuses.append(tilt_status)
            assessment.findings.extend(tilt_findings)

        # Layer 3: safety-relevant scene facts ------------------------------
        if state.scan.available:
            obstacle_status, obstacle_findings = evaluate_obstacle(
                state.scan.closest_range, self._config
            )
            statuses.append(obstacle_status)
            assessment.findings.extend(obstacle_findings)

        battery_status, battery_findings = evaluate_battery(
            state.battery.percentage, state.battery.available, self._config
        )
        statuses.append(battery_status)
        assessment.findings.extend(battery_findings)

        # Layer 4: does intent match execution? -----------------------------
        command_fresh = state.command.fresh and state.command.available
        consistency_status, consistency_findings, motion_expected, deviation = (
            evaluate_command_consistency(
                state.command.linear_x if state.command.available else None,
                state.command.angular_z if state.command.available else None,
                command_fresh,
                state.odom.speed_mps if state.odom.available else None,
                state.odom.yaw_rate_rps if state.odom.available else None,
                self._config,
                odom_available=state.odom.available,
                command_available=state.command.available,
            )
        )
        assessment.findings.extend(consistency_findings)
        if consistency_status != UNKNOWN:
            statuses.append(consistency_status)
        # ``consistent`` must not claim agreement that was never checked: it is a
        # three-state fact (agrees / disagrees / not judgeable) collapsed onto a
        # boolean plus ``valid`` in the message.
        state.command.consistency_checked = consistency_status != UNKNOWN
        state.command.consistent = consistency_status == OK
        state.command.deviation = deviation

        # Overall ------------------------------------------------------------
        assessment.status = worst_status(statuses)
        # A finding outranks the aggregate status if it is more severe.
        assessment.status = max(
            assessment.status,
            worst_status(f.severity for f in assessment.findings),
        )
        return assessment

    def motion_expected(self, state: MonitorState) -> bool:
        """True when a fresh non-trivial velocity command is active."""
        if not (state.command.available and state.command.fresh):
            return False
        return max(
            abs(state.command.linear_x), abs(state.command.angular_z)
        ) >= self._config.command_motion_threshold


# Maps a source name to the finding code used when it is unhealthy. Keeping
# this table next to the codes makes the set of staleness findings auditable.
_STALENESS_CODE: Dict[str, str] = {
    SOURCE_ODOM: CODE_ODOM_STALE,
    SOURCE_SCAN: CODE_SCAN_STALE,
    SOURCE_IMU: CODE_IMU_STALE,
    SOURCE_JOINTS: CODE_JOINTS_STALE,
    SOURCE_COMMAND: CODE_COMMAND_STALE,
    SOURCE_BATTERY: CODE_BATTERY_STALE,
}

# Codes that mean "this source never showed up at all" rather than "it went
# quiet". Derived for readability at call sites.
ABSENCE_CODES: Dict[str, str] = {
    SOURCE_ODOM: CODE_ODOM_MISSING,
    SOURCE_SCAN: CODE_SCAN_MISSING,
    SOURCE_IMU: CODE_IMU_MISSING,
}
