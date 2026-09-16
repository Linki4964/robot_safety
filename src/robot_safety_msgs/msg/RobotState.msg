# Aggregated runtime state snapshot of the monitored robot.
#
# This is step 1 of the safety state-machine project: a single, self-describing
# message that captures "what is the robot doing and is the data behind that
# answer trustworthy" at one instant. The safety state machine (step 2) will
# consume exactly this message as its input alphabet.
#
# Time semantics (important):
#   * header.stamp is the ROS time of the monitor tick. Under Gazebo it is the
#     simulated time from /clock.
#   * wall_time_sec is the monitor's own wall-clock time. Watchdogs and data-age
#     computations use wall time, so a paused or slow simulator cannot make
#     stale data look fresh.
#   * Each nested *State message carries both clocks for its own last sample.

std_msgs/Header header

# ------------------------------- identity ----------------------------------
string robot_id              # logical robot name, e.g. "turtlebot3_burger"
string world_name            # Gazebo world / scenario label, empty if unknown
string monitor_node          # fully qualified monitor node name
string software_version      # monitor module version

# -------------------------------- timing -----------------------------------
float64 wall_time_sec        # monitor wall clock (unix epoch seconds)
float64 uptime_sec           # seconds since the monitor started
float64 tick_period_sec      # measured period of this snapshot stream
uint32 tick_count            # number of snapshots published so far

# --------------------------- overall assessment -----------------------------
# Constant / value conventions are shared with the safety state machine.
uint8 status                 # highest-urgency status over all checks
uint8[] source_status        # per topic status, parallel to source_name
string[] source_name         # monitored topic / source identifiers
string[] source_reason       # human readable reason when a source is not OK
string[] warnings            # active finding identifiers, e.g. "ODOM_STALE"
uint8[] warning_severity     # severity per warning, parallel to warnings
string[] warning_detail      # human readable explanation per warning
bool motion_expected         # fresh velocity command within command_timeout
bool armed                   # motion is allowed by the monitor (step 2 owns policy)

# --------------------------- motion safety (MOT) ----------------------------
# Motion-safety rule violations, per RSS-001 §6. Kept in their own array rather
# than merged into `warnings` so that a consumer can distinguish "the robot's
# data is unhealthy" from "the robot's motion violates a safety rule" -- the two
# call for different responses and the safety state machine needs both.
MotionAlert[] motion_alerts
uint8 motion_status          # worst motion-safety severity, on the TopicHealth scale
uint16 motion_rules_evaluated  # number of MOT rules that could be judged this tick

# ------------------------------- sensor data --------------------------------
Odometry odom                # wheel odometry: pose, velocity, TF state
BatteryState battery         # battery, `available` is false when not monitored
Imu imu                      # inertial unit
LaserScan scan               # 2-D lidar
JointState joints            # wheel / actuator joint states
VelocityCommand command      # last commanded velocity
