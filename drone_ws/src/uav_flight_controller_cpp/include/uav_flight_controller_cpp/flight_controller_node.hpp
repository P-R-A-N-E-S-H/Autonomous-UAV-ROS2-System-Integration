/**
 * @file flight_controller_node.hpp
 * @brief High-Rate Deterministic C++ ROS 2 Node for Low-Jitter Flight Setpoint Streaming.
 * @author Member 3 (ROS 2 & System Integration Engineer)
 */

#ifndef UAV_FLIGHT_CONTROLLER_CPP__FLIGHT_CONTROLLER_NODE_HPP_
#define UAV_FLIGHT_CONTROLLER_CPP__FLIGHT_CONTROLLER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "uav_interfaces/msg/drone_state.hpp"
#include "uav_interfaces/msg/safety_alert.hpp"

namespace uav_flight_controller
{

class FlightControllerNode : public rclcpp::Node
{
public:
  explicit FlightControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FlightControllerNode() = default;

private:
  // Timer Callbacks
  void on_setpoint_timer_tick();
  void on_diagnostics_timer_tick();

  // Subscription Callbacks
  void on_drone_state_received(const uav_interfaces::msg::DroneState::SharedPtr msg);
  void on_target_pose_received(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_battery_received(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // ROS 2 Communication Objects
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_mavros_setpoint_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_override_;
  rclcpp::Publisher<uav_interfaces::msg::SafetyAlert>::SharedPtr pub_safety_alert_;

  rclcpp::Subscription<uav_interfaces::msg::DroneState>::SharedPtr sub_drone_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery_;

  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Flight State Variables (Atomic & Low Latency)
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose target_pose_;
  double current_battery_pct_{100.0};
  bool is_armed_{false};
  bool is_guided_{false};
  uint64_t setpoints_published_count_{0};
  double stream_frequency_hz_{100.0};
  double max_velocity_{2.0};
  double max_acceleration_{1.2};
};

}  // namespace uav_flight_controller

#endif  // UAV_FLIGHT_CONTROLLER_CPP__FLIGHT_CONTROLLER_NODE_HPP_
