#!/usr/bin/env python3
"""
member1_vlm_mock.py
===================
Mock Node for Member 1: Vision-Language Navigation Engineer.
Simulates: TravelUAV / SmolVLM / Florence-2 / Grounding DINO natural language goal generation.
"""

import time
import random

try:
    import rclpy
    from rclpy.node import Node
    from uav_interfaces.msg import NavigationGoal, SemanticDetection
    from geometry_msgs.msg import PoseStamped
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


DEMO_PROMPTS = [
    {
        "prompt": "Find the red emergency medical supply crate near warehouse bay 3",
        "class": "medical_supply_crate",
        "target": (4.5, 3.2, 2.0)
    },
    {
        "prompt": "Navigate to the yellow hazardous material container and inspect label",
        "class": "hazmat_container",
        "target": (-3.8, 6.1, 1.8)
    },
    {
        "prompt": "Locate designated emergency survivor landing pad alpha",
        "class": "landing_pad_alpha",
        "target": (1.2, -4.5, 1.5)
    },
    {
        "prompt": "Search for fire extinguisher assembly on north corridor wall",
        "class": "fire_extinguisher",
        "target": (6.0, -1.0, 2.2)
    }
]


if ROS2_AVAILABLE:
    class Member1VLMMockNode(Node):
        def __init__(self):
            super().__init__('member1_vlm_node')
            self.get_logger().info("Starting Member 1 [Vision-Language Navigation] Mock Node...")

            self.pub_goal = self.create_publisher(NavigationGoal, '/ai/semantic_goal', 10)
            self.pub_detection = self.create_publisher(SemanticDetection, '/ai/semantic_detections', 10)

            # Periodically issue realistic mission goals
            self.timer = self.create_timer(15.0, self._dispatch_sample_goal)
            self.goal_idx = 0
            self.get_logger().info("Member 1 VLM ready. Dispatches semantic goals every 15s.")

        def _dispatch_sample_goal(self):
            item = DEMO_PROMPTS[self.goal_idx % len(DEMO_PROMPTS)]
            self.goal_idx += 1

            goal = NavigationGoal()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.goal_id = f"vlm_goal_{int(time.time())}"
            goal.natural_language_prompt = item["prompt"]
            goal.target_semantic_class = item["class"]
            goal.target_pose.pose.position.x = float(item["target"][0])
            goal.target_pose.pose.position.y = float(item["target"][1])
            goal.target_pose.pose.position.z = float(item["target"][2])
            goal.target_pose.pose.orientation.w = 1.0
            goal.acceptance_radius_meters = 0.35
            goal.max_velocity_mps = 1.5
            goal.timeout_seconds = 45.0
            goal.require_visual_verification = True

            self.pub_goal.publish(goal)
            self.get_logger().info(f"[Member 1 VLM] Dispatched: '{item['prompt']}' Target: {item['target']}")


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = Member1VLMMockNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Member 1 VLM Mock test...")
        for p in DEMO_PROMPTS:
            print(f"Goal: '{p['prompt']}' -> Class: {p['class']} Target: {p['target']}")


if __name__ == '__main__':
    main()
