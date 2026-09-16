# Last velocity command observed on /cmd_vel (whatever node published it).
#
# This is the *intent* side of the intent/execution comparison: the monitor uses
# it to decide whether the robot is expected to be moving, and whether the
# executed (odometry) motion agrees with the command.

bool available               # a command was received since the monitor started
bool fresh                   # command received within command_timeout_sec
bool stop_command            # command is (numerically) a zero-velocity stop

float64 linear_x             # [m/s]
float64 linear_y             # [m/s]
float64 linear_z             # [m/s]
float64 angular_x            # [rad/s]
float64 angular_y            # [rad/s]
float64 angular_z            # [rad/s]

bool valid                   # there is enough data to judge consistency at all
bool consistent              # executed motion agrees with the command
float64 deviation            # |commanded| - |executed| magnitude, see monitor docs

float64 last_stamp_sec
float64 last_wall_time_sec
