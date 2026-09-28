#!/usr/bin/env python3
"""
flight_stack_mission_executor.py
================================
Autonomous Waypoint & Flight Stack Mission Executor for Gazebo & ArduPilot SITL.
Author: Member 2 (Simulation & Flight Stack Engineer)

Responsibilities:
 1. Command autonomous Takeoff, Hover Station-Keeping, Waypoint Missions, and Landing.
 2. Monitor ArduPilot EKF3 status and Visual Odometry alignment.
 3. Interface with QGroundControl mission plans and MAVLink telemetry.
"""

import time
import math
from typing import List, Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import BatteryState, Imu
    from uav_interfaces.msg import DroneState, WaypointList
    from uav_interfaces.srv import SetFlightMode
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class FlightStackMissionLogic:
    """Manages flight mission execution and waypoint indexing."""

    def __init__(self, cruise_speed: float = 1.8):
        self.cruise_speed = cruise_speed
        self.waypoints: List[Dict[str, float]] = []
        self.current_wp_idx = 0
        self.is_mission_active = False
        self.current_pose = [0.0, 0.0, 0.0]
        self.acceptance_radius = 0.35
        self.dwell_start_time = 0.0
        self.is_dwelling = False

    def load_waypoints(self, wp_list: List[Dict[str, float]]):
        self.waypoints = wp_list
        self.current_wp_idx = 0
        self.is_mission_active = len(wp_list) > 0

    def step(self) -> Optional[Dict[str, Any]]:
        if not self.is_mission_active or self.current_wp_idx >= len(self.waypoints):
            return None

        target = self.waypoints[self.current_wp_idx]
        dx = target["x"] - self.current_pose[0]
        dy = target["y"] - self.current_pose[1]
        dz = target["z"] - self.current_pose[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        now = time.time()
        if dist < self.acceptance_radius:
            if not self.is_dwelling:
                self.is_dwelling = True
                self.dwell_start_time = now

            dwell_req = target.get("dwell_time", 2.0)
            if (now - self.dwell_start_time) >= dwell_req:
                self.is_dwelling = False
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    self.is_mission_active = False
                    return {"type": "MISSION_COMPLETE"}

        return {
            "type": "NAVIGATE_TO_WAYPOINT",
            "index": self.current_wp_idx,
            "target": target,
            "distance_remaining": dist,
            "is_dwelling": self.is_dwelling
        }


if ROS2_AVAILABLE:
    class FlightStackMissionExecutorNode(Node):
        def __init__(self):
            super().__init__('flight_stack_mission_executor')
            self.get_logger().info("Starting Member 2 [Flight Stack Mission Executor] Node...")

            self.logic = FlightStackMissionLogic(cruise_speed=1.8)

            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Publishers to MAVROS
            self.pub_setpoint = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_reliable)

            # Subscriptions
            self.sub_pose = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, 10)
            self.sub_wps = self.create_subscription(WaypointList, '/flight_stack/mission_queue', self._on_waypoints, qos_reliable)

            self.timer = self.create_timer(0.05, self._executor_loop) # 20Hz loop
            self.get_logger().info("Flight Stack Mission Executor operational at 20Hz.")

        def _on_pose(self, msg: PoseStamped):
            self.logic.current_pose = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]

        def _on_waypoints(self, msg: WaypointList):
            wp_list = []
            for i, p in enumerate(msg.waypoints):
                wp_list.append({
                    "x": p.pose.position.x,
                    "y": p.pose.position.y,
                    "z": p.pose.position.z,
                    "dwell_time": msg.dwell_times_sec[i] if i < len(msg.dwell_times_sec) else 2.0
                })
            self.logic.load_waypoints(wp_list)
            self.get_logger().info(f"[Member 2 Flight Stack] Loaded {len(wp_list)} autonomous mission waypoints.")

        def _executor_loop(self):
            cmd = self.logic.step()
            if cmd and cmd.get("type") == "NAVIGATE_TO_WAYPOINT":
                target = cmd["target"]
                sp = PoseStamped()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.header.frame_id = "map"
                sp.pose.position.x = float(target["x"])
                sp.pose.position.y = float(target["y"])
                sp.pose.position.z = float(target["z"])
                sp.pose.orientation.w = 1.0
                self.pub_setpoint.publish(sp)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = FlightStackMissionExecutorNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Flight Stack Mission Logic Test...")
        logic = FlightStackMissionLogic()
        logic.load_waypoints([{"x": 2.0, "y": 2.0, "z": 2.5, "dwell_time": 0.2}])
        logic.current_pose = [2.0, 2.0, 2.5]
        cmd = logic.step()
        print(f"Step Output: {cmd}")


if __name__ == '__main__':
    main()
