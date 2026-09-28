# SafetyAlert.msg
# Failsafe events, geofence breaches, SLAM loss triggers

std_msgs/Header header

uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_CRITICAL = 2
uint8 SEVERITY_EMERGENCY = 3

uint8 severity
string alert_code        # e.g., SLAM_TRACKING_LOST, LOW_BATTERY, GEOFENCE_BREACH, MAVLINK_TIMEOUT
string description
string suggested_action  # e.g., SWITCH_TO_HOVER, RETURN_TO_HOME, IMMEDIATE_LAND
float32 time_since_event_sec
