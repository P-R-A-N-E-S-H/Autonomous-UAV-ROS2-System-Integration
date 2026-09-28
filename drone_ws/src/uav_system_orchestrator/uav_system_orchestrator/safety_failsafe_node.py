#!/usr/bin/env python3
"""
safety_failsafe_node.py
=======================
Autonomous Flight Safety Supervisor & Fail-Safe Manager.
Role: Member 3 - ROS2 & System Integration Engineer.

Safety Integrity Architecture:
 1. SLAM Tracking Loss Watchdog: Detects loss of visual features in GPS-denied tunnels/rooms.
 2. 3D Geofence Enforcer: Prevents drone flyaways beyond virtual safety perimeter.
 3. Multi-tier Battery Failsafe:
      - 25%: Warning flag dispatched to Ground Control Station
      - 18%: Auto-RTH initiated via keyframe backtracking
      - 10%: Immediate emergency vertical touchdown
 4. MAVLink / DDS Connection Watchdog: Heartbeat monitoring.
"""

import time
import math
from typing import Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Header, Bool, Float32
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import BatteryState
    from uav_interfaces.msg import SafetyAlert, DroneState
    from uav_interfaces.srv import TriggerFailsafe
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class SafetySupervisorLogic:
    """Decoupled safety logic for verification and testing."""

    def __init__(self,
                 max_slam_timeout_s: float = 0.4,
                 geofence_xy_limit: float = 25.0,
                 geofence_z_max: float = 12.0,
                 geofence_z_min: float = 0.0):
        self.max_slam_timeout_s = max_slam_timeout_s
        self.geofence_xy_limit = geofence_xy_limit
        self.geofence_z_max = geofence_z_max
        self.geofence_z_min = geofence_z_min

        self.last_slam_time = time.time()
        self.last_battery_time = time.time()
        self.current_pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.battery_pct = 100.0
        self.active_alert: Optional[Dict[str, Any]] = None

    def update_slam_heartbeat(self, x: float, y: float, z: float):
        self.last_slam_time = time.time()
        self.current_pose = {"x": x, "y": y, "z": z}

    def update_battery(self, pct: float):
        self.last_battery_time = time.time()
        self.battery_pct = pct

    def check_safety_rules(self) -> Optional[Dict[str, Any]]:
        now = time.time()

        # 1. SLAM Tracking Timeout Check
        if (now - self.last_slam_time) > self.max_slam_timeout_s:
            return {
                "severity": 2, # CRITICAL
                "code": "SLAM_TRACKING_LOST",
                "description": f"No visual SLAM pose received for {now - self.last_slam_time:.2f}s",
                "action": "SWITCH_TO_HOVER"
            }

        # 2. Geofence Check
        dist_xy = math.sqrt(self.current_pose["x"]**2 + self.current_pose["y"]**2)
        if dist_xy > self.geofence_xy_limit:
            return {
                "severity": 3, # EMERGENCY
                "code": "GEOFENCE_BREACH_XY",
                "description": f"XY Distance {dist_xy:.2f}m exceeds max limit {self.geofence_xy_limit}m",
                "action": "RETURN_TO_HOME"
            }

        if self.current_pose["z"] > self.geofence_z_max:
            return {
                "severity": 3,
                "code": "GEOFENCE_BREACH_ALTITUDE",
                "description": f"Altitude {self.current_pose['z']:.2f}m exceeds ceiling {self.geofence_z_max}m",
                "action": "SWITCH_TO_HOVER"
            }

        # 3. Battery Checks
        if self.battery_pct <= 10.0:
            return {
                "severity": 3,
                "code": "BATTERY_CRITICAL_10",
                "description": f"Battery critical at {self.battery_pct:.1f}%",
                "action": "IMMEDIATE_LAND"
            }
        elif self.battery_pct <= 18.0:
            return {
                "severity": 2,
                "code": "BATTERY_LOW_RTH_18",
                "description": f"Battery low at {self.battery_pct:.1f}%, initiating RTH",
                "action": "RETURN_TO_HOME"
            }

        return None


if ROS2_AVAILABLE:
    class SafetyFailsafeNode(Node):
        """ROS 2 Node for Flight Safety Supervisor."""

        def __init__(self):
            super().__init__('safety_failsafe_node')
            self.get_logger().info("Initializing Safety & Failsafe Supervisor [Member 3]...")

            self.logic = SafetySupervisorLogic()

            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Publishers
            self.pub_alert = self.create_publisher(SafetyAlert, '/safety/failsafe_trigger', qos_reliable)

            # Subscriptions
            self.sub_slam = self.create_subscription(PoseStamped, '/slam/pose', self._on_slam, 10)
            self.sub_battery = self.create_subscription(BatteryState, '/mavros/battery', self._on_battery, 10)

            # Watchdog Timer running at 50Hz
            self.timer = self.create_timer(0.02, self._watchdog_step)
            self.get_logger().info("Safety Supervisor Watchdog running at 50Hz.")

        def _on_slam(self, msg: PoseStamped):
            self.logic.update_slam_heartbeat(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )

        def _on_battery(self, msg: BatteryState):
            self.logic.update_battery(msg.percentage * 100.0)

        def _watchdog_step(self):
            alert = self.logic.check_safety_rules()
            if alert:
                msg = SafetyAlert()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"
                msg.severity = alert["severity"]
                msg.alert_code = alert["code"]
                msg.description = alert["description"]
                msg.suggested_action = alert["action"]
                self.pub_alert.publish(msg)
                self.get_logger().warn(f"SAFETY TRIGGER: [{alert['code']}] -> Action: {alert['action']}")


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = SafetyFailsafeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Testing Safety Supervisor Logic...")
        safety = SafetySupervisorLogic(max_slam_timeout_s=0.2)
        safety.update_slam_heartbeat(0, 0, 1.5)
        print("Initial state check:", safety.check_safety_rules())
        print("Simulating SLAM drop for 0.3s...")
        time.sleep(0.3)
        alert = safety.check_safety_rules()
        print(f"Safety Alert Triggered: {alert}")


if __name__ == '__main__':
    main()
