#!/usr/bin/env python3
"""
member5_memory_mock.py
======================
Mock Node for Member 5: Adaptive Semantic Memory & Evaluation Engineer.
Simulates: Semantic database retrieval, landmark spatial persistence, query service.
"""

import time
import math
from typing import Dict, Any, List

try:
    import rclpy
    from rclpy.node import Node
    from uav_interfaces.srv import QuerySemanticMemory
    from uav_interfaces.msg import SemanticDetection
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


DATABASE_LANDMARKS = {
    "medical_supply_crate": {"x": 4.5, "y": 3.2, "z": 2.0, "desc": "Red hard-case medical kit on wooden pallet"},
    "hazmat_container": {"x": -3.8, "y": 6.1, "z": 1.8, "desc": "Yellow chemical drum labeled Class-3 Flammable"},
    "landing_pad_alpha": {"x": 1.2, "y": -4.5, "z": 1.5, "desc": "Elevated aluminum emergency landing zone with high-contrast H"},
    "fire_extinguisher": {"x": 6.0, "y": -1.0, "z": 2.2, "desc": "Wall-mounted CO2 cylinder near emergency exit"}
}


if ROS2_AVAILABLE:
    class Member5MemoryMockNode(Node):
        def __init__(self):
            super().__init__('member5_semantic_memory_node')
            self.get_logger().info("Starting Member 5 [Adaptive Semantic Memory] Mock Node...")

            # Service Server
            self.srv_query = self.create_service(
                QuerySemanticMemory, '/memory/query_target', self._handle_query
            )

            # Subscriptions to new detections to simulate memory updating
            self.sub_detection = self.create_subscription(
                SemanticDetection, '/ai/semantic_detections', self._on_detection, 10
            )

            self.get_logger().info(f"Member 5 Semantic Memory loaded with {len(DATABASE_LANDMARKS)} persistent landmarks.")

        def _handle_query(self, request, response):
            query = request.class_filter.lower()
            self.get_logger().info(f"[Member 5 Memory] Query received for class: '{query}'")

            matched_key = None
            for key in DATABASE_LANDMARKS:
                if key in query or query in key:
                    matched_key = key
                    break

            if matched_key:
                lm = DATABASE_LANDMARKS[matched_key]
                response.found = True
                response.target_pose.pose.position.x = lm["x"]
                response.target_pose.pose.position.y = lm["y"]
                response.target_pose.pose.position.z = lm["z"]
                response.target_pose.pose.orientation.w = 1.0
                response.landmark_description = lm["desc"]
                response.confidence_score = 0.94
                response.observation_count = 5
            else:
                response.found = False
                response.landmark_description = "Target not found in semantic spatial map"
                response.confidence_score = 0.0
                response.observation_count = 0

            return response

        def _on_detection(self, msg: SemanticDetection):
            self.get_logger().info(f"[Member 5 Memory] Updated semantic landmark: {msg.class_label} at ({msg.position_in_map_frame.x:.2f}, {msg.position_in_map_frame.y:.2f})")


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = Member5MemoryMockNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Member 5 Semantic Memory Database Mock initialized.")


if __name__ == '__main__':
    main()
