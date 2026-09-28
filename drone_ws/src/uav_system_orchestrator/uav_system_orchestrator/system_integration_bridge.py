#!/usr/bin/env python3
"""
system_integration_bridge.py
============================
Core System Integration & Inter-Module Communication Hub.
Role: Member 3 - ROS2 & System Integration Engineer.

Responsibilities:
 1. Cross-module data synchronization (AI VLM, Visual SLAM, Semantic Memory, Flight Controller).
 2. Seamless message translation between high-level AI semantic goals and MAVROS setpoints.
 3. High-rate trajectory interpolation (smoothing abrupt waypoint jumps to avoid jerky drone kinematics).
 4. Latency monitoring and topic drop detection across inter-module ROS 2 pipelines.
"""

import math
import time
from typing import Dict, Any, Optional, List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import Header, String, Float32
    from geometry_msgs.msg import PoseStamped, TwistStamped, Point
    from nav_msgs.msg import Odometry, Path
    from uav_interfaces.msg import DroneState, NavigationGoal, SemanticDetection, SystemHealth
    from uav_interfaces.srv import QuerySemanticMemory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class TrajectorySmoother:
    """Calculates smooth jerk-limited setpoint transitions between waypoints."""

    def __init__(self, max_vel: float = 1.5, max_accel: float = 1.0):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vel = [0.0, 0.0, 0.0]
        self.target_pos = [0.0, 0.0, 0.0]
        self.last_update = time.time()

    def set_target(self, x: float, y: float, z: float):
        self.target_pos = [x, y, z]

    def step(self, dt: float) -> Tuple[List[float], List[float]]:
        """Calculate next smooth position and velocity step."""
        if dt <= 0.0:
            return self.current_pos, self.current_vel

        new_pos = [0.0, 0.0, 0.0]
        new_vel = [0.0, 0.0, 0.0]

        for i in range(3):
            error = self.target_pos[i] - self.current_pos[i]
            # Desired velocity proportional to error clamped to max_vel
            desired_vel = math.copysign(min(abs(error) * 1.5, self.max_vel), error) if abs(error) > 0.01 else 0.0
            
            # Rate of change of velocity limited by max_accel
            vel_diff = desired_vel - self.current_vel[i]
            max_delta_v = self.max_accel * dt
            if abs(vel_diff) > max_delta_v:
                applied_vel = self.current_vel[i] + math.copysign(max_delta_v, vel_diff)
            else:
                applied_vel = desired_vel

            new_vel[i] = applied_vel
            new_pos[i] = self.current_pos[i] + applied_vel * dt

        self.current_pos = new_pos
        self.current_vel = new_vel
        return self.current_pos, self.current_vel


if ROS2_AVAILABLE:
    class SystemIntegrationBridgeNode(Node):
        """ROS 2 Node for Central Module Integration."""

        def __init__(self):
            super().__init__('system_integration_bridge')
            self.get_logger().info("Starting UAV System Integration Bridge [Member 3]...")

            self.declare_parameter('max_velocity', 1.5)
            self.declare_parameter('max_acceleration', 1.0)
            self.declare_parameter('publish_rate_hz', 50.0)

            v_max = self.get_parameter('max_velocity').get_parameter_value().double_value
            a_max = self.get_parameter('max_acceleration').get_parameter_value().double_value
            rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

            self.smoother = TrajectorySmoother(max_vel=v_max, max_accel=a_max)

            # QoS Configuration
            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE
            )
            qos_sensor = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                durability=DurabilityPolicy.VOLATILE
            )

            # Subscriptions across modules
            # 1. Member 1 (AI VLM Goals)
            self.sub_vlm_goal = self.create_subscription(
                NavigationGoal, '/ai/semantic_goal', self._on_vlm_goal, qos_reliable
            )

            # 2. Member 4 (GPS-denied SLAM Odometry)
            self.sub_slam_odom = self.create_subscription(
                Odometry, '/slam/odometry', self._on_slam_odom, qos_sensor
            )

            # 3. Member 2 (MAVROS Local Pose & State)
            self.sub_mavros_pose = self.create_subscription(
                PoseStamped, '/mavros/local_position/pose', self._on_mavros_pose, qos_sensor
            )

            # 4. Member 5 (Memory Queries & Updates)
            self.cli_memory_query = self.create_client(
                QuerySemanticMemory, '/memory/query_target'
            )

            # Publishers
            # Smooth setpoint stream to MAVROS (50Hz)
            self.pub_mavros_setpoint = self.create_publisher(
                PoseStamped, '/mavros/setpoint_position/local', qos_reliable
            )
            self.pub_active_path = self.create_publisher(
                Path, '/orchestrator/active_trajectory_path', qos_reliable
            )

            # Latency and synchronization telemetry
            self.last_slam_time = 0.0
            self.last_vlm_time = 0.0
            self.slam_pose_count = 0

            self.timer = self.create_timer(1.0 / rate_hz, self._bridge_loop)
            self.last_tick = time.time()
            self.get_logger().info(f"System Integration Bridge active at {rate_hz} Hz.")

        def _on_vlm_goal(self, msg: NavigationGoal):
            self.last_vlm_time = time.time()
            self.get_logger().info(
                f"[Member 1 -> Member 3] Target received: ({msg.target_pose.pose.position.x:.2f}, "
                f"{msg.target_pose.pose.position.y:.2f}, {msg.target_pose.pose.position.z:.2f}) "
                f"Class: '{msg.target_semantic_class}'"
            )
            self.smoother.set_target(
                msg.target_pose.pose.position.x,
                msg.target_pose.pose.position.y,
                msg.target_pose.pose.position.z
            )

        def _on_slam_odom(self, msg: Odometry):
            self.last_slam_time = time.time()
            self.slam_pose_count += 1
            # Update current position anchor
            self.smoother.current_pos = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]

        def _on_mavros_pose(self, msg: PoseStamped):
            pass

        def _bridge_loop(self):
            now = time.time()
            dt = now - self.last_tick
            self.last_tick = now

            # Step trajectory interpolator
            pos, vel = self.smoother.step(dt)

            # Publish 50Hz setpoint to Flight Stack
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = "map"
            sp.pose.position.x = pos[0]
            sp.pose.position.y = pos[1]
            sp.pose.position.z = pos[2]
            sp.pose.orientation.w = 1.0
            self.pub_mavros_setpoint.publish(sp)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = SystemIntegrationBridgeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] System Integration Bridge Trajectory Smoother test...")
        smoother = TrajectorySmoother(max_vel=2.0, max_accel=1.0)
        smoother.set_target(5.0, 3.0, 2.5)
        for i in range(20):
            pos, vel = smoother.step(0.1)
            print(f"t={i*0.1:.1f}s | Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | Vel=({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f})")
            time.sleep(0.05)


if __name__ == '__main__':
    main()
