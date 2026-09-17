# One triggered motion-safety rule (spec RSS-001 §6, category MOT).
#
# The monitor's job for these rules is *identification and annunciation only*:
# this message says "a rule fired, here is the evidence". It is deliberately NOT
# an action. Deciding to slow down or stop is the responsibility of the safety
# state machine (layer 2) and the gate (layer 3); keeping that split is what
# stops the observation layer from becoming a single point of failure.

std_msgs/Header header

string code                  # stable alert identifier, e.g. "MOT_LIN_VEL_EXCEED"
string rule_id               # spec rule that fired, e.g. "MOT-001"
string category              # spec category: MOT/COL/SEN/CMD/LOC/COM/SYS
uint8 level                  # spec level S1..S4, see constants below
uint8 severity               # same level mapped onto TopicHealth's constants
string detail                # human readable evidence, including the numbers

float64 value                # the measured quantity that tripped the rule
float64 threshold            # the limit it was compared against
bool escalated               # true when a time/count escalation raised the level

float64 stamp_sec            # ROS/sim time of this evaluation
float64 wall_time_sec        # wall time of this evaluation (see analyzer docstring)

# Spec levels (RSS-001 §4.3). `level` keeps all four distinct; `severity` is the
# projection onto the TopicHealth scale so a consumer can compare an alert with a
# source status without a lookup table. S3 and S4 both project to CRITICAL: the
# difference between a recoverable protective stop and a latched safe stop is a
# state-machine concern, and collapsing it would silently lose meaning.
uint8 S1 = 1                 # notice: log only, no degradation  -> severity ERROR
uint8 S2 = 2                 # warning: DEGRADED + speed envelope -> severity ERROR
uint8 S3 = 3                 # protective stop (auto-recoverable) -> severity CRITICAL
uint8 S4 = 4                 # safe/emergency stop (needs reset)  -> severity CRITICAL
