# SystemHealth.msg
# Real-time QoS, Topic Frequency, Latency and Node Heartbeat Diagnostics

std_msgs/Header header

string node_name
float32 cpu_usage_percent
float32 memory_usage_mb
float32 loop_frequency_hz
float32 topic_latency_ms
uint32 dropped_packets
bool all_nodes_alive
string[] active_nodes
string[] degraded_nodes
string status_level # OK, WARN, CRITICAL
