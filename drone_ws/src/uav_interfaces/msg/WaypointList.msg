# WaypointList.msg
# Coordinated 3D waypoint queue for multi-trajectory navigation

std_msgs/Header header

string mission_id
geometry_msgs/PoseStamped[] waypoints
float32[] target_velocities
float32[] dwell_times_sec
bool is_looping
