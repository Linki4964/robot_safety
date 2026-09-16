# Summary of /joint_states. Wheel / actuator motion is the ground truth for
# "is the drivetrain actually turning", independent of odometry integration.

bool available

uint32 joint_count           # number of named joints in the message
uint32 moving_joint_count    # joints whose |velocity| exceeds the threshold
string[] moving_joints       # names of those joints
float64 max_abs_velocity     # largest |velocity| over all joints [rad/s or m/s]
float64 total_abs_velocity   # sum of |velocity|, a coarse drivetrain activity level

float64 last_stamp_sec
float64 last_wall_time_sec
