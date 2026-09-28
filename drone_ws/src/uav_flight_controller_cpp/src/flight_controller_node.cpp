/**
 * @file flight_controller_node.cpp
 * @brief High-Rate Deterministic C++ ROS 2 Node implementation for MAVROS setpoint streaming.
 * @author Member 3 (ROS 2 & System Integration Engineer)
 */

#include "uav_flight_controller_cpp/flight_controller_node.hpp"

namespace uav_flight_controller
{

FlightControllerNode::FlightControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("uav_flight_controller_cpp", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing High-Rate C++ Flight Controller Node [Member 3]...");

  // Declare Parameters
  this->declare_parameter<double>("stream_frequency_hz", 100.0);
  this->declare_parameter<double>("max_velocity", 2.0);
  this->declare_parameter<double>("max_acceleration", 1.2);

  stream_frequency_hz_ = this->get_parameter("stream_frequency_hz").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();

  // Custom Flight-Critical QoS Profile (Reliable, Volatile, Depth 10)
  rclcpp::QoS qos_flight(10);
  qos_flight.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_flight.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Sensor QoS Profile (Best Effort, Depth 5)
  rclcpp::QoS qos_sensor(5);
  qos_sensor.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_sensor.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // Initialize Publishers
  pub_mavros_setpoint_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mavros/setpoint_position/local", qos_flight);

  pub_velocity_override_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/setpoint_velocity/cmd_vel", qos_flight);

  pub_safety_alert_ = this->create_publisher<uav_interfaces::msg::SafetyAlert>(
    "/safety/failsafe_trigger", qos_flight);

  // Initialize Subscriptions
  sub_drone_state_ = this->create_subscription<uav_interfaces::msg::DroneState>(
    "/orchestrator/drone_state", qos_flight,
    std::bind(&FlightControllerNode::on_drone_state_received, this, std::placeholders::_1));

  sub_target_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/orchestrator/target_setpoint", qos_flight,
    std::bind(&FlightControllerNode::on_target_pose_received, this, std::placeholders::_1));

  sub_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "/mavros/battery", qos_sensor,
    std::bind(&FlightControllerNode::on_battery_received, this, std::placeholders::_1));

  // Initialize 100Hz Real-Time Setpoint Streaming Timer
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / stream_frequency_hz_));
  setpoint_timer_ = this->create_wall_timer(
    period_ms, std::bind(&FlightControllerNode::on_setpoint_timer_tick, this));

  // Diagnostics Timer (1Hz)
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&FlightControllerNode::on_diagnostics_timer_tick, this));

  RCLCPP_INFO(this->get_logger(), "C++ Flight Controller streaming setpoints at %.1f Hz.", stream_frequency_hz_);
}

void FlightControllerNode::on_drone_state_received(const uav_interfaces::msg::DroneState::SharedPtr msg)
{
  is_armed_ = msg->is_armed;
  is_guided_ = msg->is_guided;
  current_pose_.position = msg->position;
  current_pose_.orientation = msg->orientation;
}

void FlightControllerNode::on_target_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_pose_ = msg->pose;
}

void FlightControllerNode::on_battery_received(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  current_battery_pct_ = msg->percentage * 100.0;
}

void FlightControllerNode::on_setpoint_timer_tick()
{
  // Continuously publish setpoints to keep autopilot watchdog alive
  auto setpoint_msg = geometry_msgs::msg::PoseStamped();
  setpoint_msg.header.stamp = this->now();
  setpoint_msg.header.frame_id = "map";
  setpoint_msg.pose = target_pose_;

  pub_mavros_setpoint_->publish(setpoint_msg);
  setpoints_published_count_++;
}

void FlightControllerNode::on_diagnostics_timer_tick()
{
  RCLCPP_DEBUG(this->get_logger(), "Published %lu setpoints. Armed: %d, Bat: %.1f%%",
    setpoints_published_count_, is_armed_, current_battery_pct_);
}

}  // namespace uav_flight_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uav_flight_controller::FlightControllerNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_flight_controller::FlightControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
