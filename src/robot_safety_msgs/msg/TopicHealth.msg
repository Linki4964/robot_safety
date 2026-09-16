# Health record for one monitored data source (topic, clock, or pseudo source).
#
# A source is "OK" while samples keep arriving faster than its timeout. This is
# deliberately based on wall-clock arrival time, never on the message stamp:
# a frozen simulator keeps stamping nothing at all, and a bag replay keeps
# producing old stamps, both of which must be caught here.

# --------------------------- status constants -------------------------------
uint8 UNKNOWN = 0            # never received a sample (yet)
uint8 OK = 1                 # samples arriving within the timeout
uint8 STALE = 2              # no fresh sample inside the timeout
uint8 ERROR = 3              # samples arriving but unusable
uint8 CRITICAL = 4           # safety-relevant failure

string source                # identifier, matching RobotState.source_name
string topic                 # resolved topic name, empty when not a topic

bool observed                # at least one sample received since start
bool fresh                   # last sample arrived within the timeout
uint8 status                 # one of the constants above

float64 age_sec              # wall-clock age of the last sample
float64 timeout_sec          # configured staleness timeout
float64 measured_rate_hz     # observed arrival rate
uint32 message_count         # total samples received

float64 last_stamp_sec       # header stamp of the last sample (ROS/sim time)
float64 last_wall_time_sec   # wall-clock arrival time of the last sample
string reason                # explanation when status is not OK
