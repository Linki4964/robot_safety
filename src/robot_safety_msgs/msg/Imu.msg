# Inertial data (/imu). Roll and pitch are the safety-relevant part for a
# differential-drive ground robot: a persistent tilt means the robot may be
# climbing an obstacle, tipping, or already fallen.

bool available

float64 roll_rad             # rotation about x, normalised to (-pi, pi]
float64 pitch_rad            # rotation about y, normalised to (-pi, pi]
float64 yaw_rad              # rotation about z, yaw only (unreliable on mag-less IMUs)
float64 tilt_deg             # angle between the body z axis and world z [deg]

float64 ang_vel_x            # [rad/s]
float64 ang_vel_y            # [rad/s]
float64 ang_vel_z            # [rad/s]
float64 lin_acc_x            # [m/s^2]
float64 lin_acc_y            # [m/s^2]
float64 lin_acc_z            # [m/s^2]

bool orientation_available   # sensor provided a valid quaternion

float64 last_stamp_sec
float64 last_wall_time_sec
