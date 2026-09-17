# State of the motion-safety gate -- the sole arbiter of the command stream.
#
# The gate sits between whoever commands the robot and the motors. It only ever
# removes authority: it passes a command unchanged, or it publishes a stop. It
# never raises a limit, smooths a command upward, or invents motion.
#
# Published continuously, including while passing, so an external watchdog can
# tell "the gate is alive and passing" from "the gate is alive and refusing" from
# "the gate has stopped publishing" -- the last of which is the dangerous case
# (RSS-003 SYS-005: a stalled gate must not leave the motors holding their last
# command).

std_msgs/Header header

string gate_id               # identity of the gate instance
uint8 state                  # PASS/BLOCKED/WATCHDOG/HOLDING, see constants
string state_name            # printable state, so a log needs no lookup table

float64 input_linear         # command received, before gating (m/s)
float64 input_angular        # command received, before gating (rad/s)
float64 output_linear        # command published, after gating (m/s)
float64 output_angular       # command published, after gating (rad/s)
bool intercepted             # true when the gate changed or refused the command

string reason                # stable reason code, e.g. SAFETY_GATE_MOTION_BLOCKED
string detail                # human readable explanation with the numbers
string[] blocked_codes       # fault codes that caused the refusal, if any

bool watchdog_tripped        # true when a watchdog (monitor or command) fired
float64 monitor_age_sec      # wall-clock age of the newest monitor snapshot
float64 command_age_sec      # wall-clock age of the newest incoming command

uint64 pass_count            # commands passed since start
uint64 block_count           # commands refused/stopped since start
float64 stamp_sec            # ROS/sim time of this report
float64 wall_time_sec        # wall time of this report

uint8 GATE_PASS = 0          # commanding normally
uint8 GATE_BLOCKED = 1       # refusing a command (motion-safety violation)
uint8 GATE_WATCHDOG = 2      # monitor or command stream lost; stopping defensively
uint8 GATE_HOLDING = 3       # no command has ever arrived; stopping
