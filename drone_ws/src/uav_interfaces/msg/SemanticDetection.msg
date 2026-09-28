# SemanticDetection.msg
# Grounded visual detection from Member 1 / Sensor streams

std_msgs/Header header

string class_label
float32 confidence
geometry_msgs/Point position_in_camera_frame
geometry_msgs/Point position_in_map_frame
float32[4] bounding_box_2d # [ymin, xmin, ymax, xmax] normalized
sensor_msgs/Image snapshot
