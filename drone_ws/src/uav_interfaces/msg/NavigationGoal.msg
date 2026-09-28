# NavigationGoal.msg
# Semantic & Geometric Navigation Goal dispatched to System Orchestrator

std_msgs/Header header

string goal_id
string natural_language_prompt     # Originating prompt from Member 1 (TravelUAV / SmolVLM)
string target_semantic_class       # e.g., "red_fire_hydrant", "survivor_backpack", "landing_pad"
geometry_msgs/PoseStamped target_pose
float32 acceptance_radius_meters   # Threshold distance to declare goal reached
float32 max_velocity_mps
float32 timeout_seconds
bool require_visual_verification
