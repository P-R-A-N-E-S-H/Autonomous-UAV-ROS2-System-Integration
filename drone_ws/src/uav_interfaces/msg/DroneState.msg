# DroneState.msg
# Real-time state of the UAV System Orchestrator

std_msgs/Header header

# Current Finite State Machine Mode
# INIT=0, PREARM=1, ARMED=2, TAKEOFF=3, HOVER=4, EXPLORATION=5, 
# SEMANTIC_GOAL_NAV=6, WAYPOINT_TRACKING=7, RTH=8, LANDING=9, EMERGENCY_HOLD=10
uint8 current_state
string state_name

# MAVROS Flight Controller Connection State
bool is_connected
bool is_armed
bool is_guided
string mavros_mode

# Position & Orientation (Fused in Global/Local frame)
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 linear_velocity

# System Status
float32 battery_percentage
float32 battery_voltage
float32 flight_time_seconds
bool gps_denied_mode
bool slam_healthy
string active_mission_id
