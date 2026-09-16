# Pose and velocity derived from wheel odometry (/odom), in the odom frame.
#
# yaw_rad, speed_mps and yaw_rate_rps are redundant with the raw quaternion and
# twist on purpose: the safety state machine should not have to redo frame math,
# and the monitor already validates that the redundancy is consistent.

bool available               # an odometry sample was received

float64 x                    # position x [m]
float64 y                    # position y [m]
float64 z                    # position z [m]
float64 yaw_rad              # heading, normalised to (-pi, pi]
float64 vx                   # body linear velocity x [m/s]
float64 vy                   # body linear velocity y [m/s]
float64 vz                   # body linear velocity z [m/s]
float64 speed_mps            # sqrt(vx^2 + vy^2 + vz^2)
float64 yaw_rate_rps         # body angular velocity z [rad/s]

float64 pose_cov_xx          # position variance x [m^2] from the covariance matrix
float64 pose_cov_yy          # position variance y [m^2]
float64 pose_cov_yawyaw      # yaw variance [rad^2], -1 when not provided
float64 twist_cov_max        # largest *observable* twist variance, -1 when none
# Note on covariances: diff-drive odometry publishes a sentinel of 1e12 (or
# larger) for degrees of freedom it does not observe -- z, roll and pitch. Those
# entries mean "no information", not "very uncertain", and are excluded from
# twist_cov_max and from the validity flags below. Only x, y and yaw are judged.

bool pose_valid              # observable pose covariance within the configured bound
bool twist_valid             # observable twist covariance within the configured bound

float64 last_stamp_sec       # header stamp of the last odometry sample
float64 last_wall_time_sec   # wall-clock arrival time of the last sample
