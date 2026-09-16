# Battery / power source state. Set available=false when no battery topic is
# monitored (e.g. the Gazebo TurtleBot3 does not publish one), so downstream
# consumers can distinguish "healthy" from "unknown".

bool available

float64 voltage_v            # volts
float64 percentage           # state of charge in [0, 1], -1 when unknown
float64 temperature_c        # degrees Celsius
bool charging                # charger connected / charging
uint8 power_supply_status    # raw sensor_msgs/BatteryState status code

float64 last_stamp_sec
float64 last_wall_time_sec
