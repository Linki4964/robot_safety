# 2-D lidar summary (/scan). The full point cloud is intentionally not copied
# into the state snapshot: the state machine only needs the derived obstacle
# picture, and keeping the snapshot small keeps it publishable at 10 Hz.

bool available

float64 angle_min            # [rad]
float64 angle_max            # [rad]
float64 angle_increment      # [rad]
float64 range_min            # sensor blind range [m]
float64 range_max            # [m]

float64 closest_range        # minimum finite valid range over the whole field [m]
float64 front_range          # minimum valid range inside the frontal sector [m]
float64 front_angle          # bearing of front_range [rad], 0 when unknown
uint32 valid_point_count     # returns with range_min <= r <= range_max
uint32 point_count           # total returns in the message

# All returns beyond range_max (which a Gazebo ray sensor reports as inf) is the
# normal signature of clear space, not of a broken sensor. Consumers must read
# no_return together with the source health: a healthy source with no_return set
# means "nothing detected", while a stale source means "we do not know".
bool no_return               # no valid return in any direction

bool obstacle_detected       # closest_range within the configured warning bound
bool obstacle_critical       # closest_range within the configured critical bound

float64 last_stamp_sec
float64 last_wall_time_sec
