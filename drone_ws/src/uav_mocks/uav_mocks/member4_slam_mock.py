#!/usr/bin/env python3
"""
member4_slam_mock.py
====================
Mock Node for Member 4: GPS-Denied Navigation Engineer.
Simulates: ORB-SLAM3 / RTAB-Map visual odometry, keyframe tracking, and map pose.
"""

import time
import math
import random

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


if ROS2_AVAILABLE:
    class Member4SLAMMockNode(Node):
        def __init__(self):
            super().__init__('member4_orb_slam3_node')
            self.get_logger().info("Starting Member 4 [ORB-SLAM3 / GPS-Denied Navigation] Mock Node...")

            # Publishers
            self.pub_slam_pose = self.create_publisher(PoseStamped, '/slam/pose', 30)
            self.pub_slam_odom = self.create_publisher(Odometry, '/slam/odometry', 30)

            # Subscriptions to ground-truth from sim to add realistic SLAM noise
            self.sub_truth = self.create_subscription(
                PoseStamped, '/mavros/local_position/pose', self._on_truth_pose, 10
            )

            self.current_pose = [0.0, 0.0, 0.0]
            self.is_tracking = True
            self.get_logger().info("Member 4 Visual SLAM active at 30Hz.")

        def _on_truth_pose(self, msg: PoseStamped):
            # Realistic sensor Gaussian noise (0.015m stddev)
            noise_x = random.gauss(0.0, 0.012)
            noise_y = random.gauss(0.0, 0.012)
            noise_z = random.gauss(0.0, 0.008)

            slam_msg = PoseStamped()
            slam_msg.header.stamp = self.get_clock().now().to_msg()
            slam_msg.header.frame_id = "map"
            slam_msg.pose.position.x = msg.pose.position.x + noise_x
            slam_msg.pose.position.y = msg.pose.position.y + noise_y
            slam_msg.pose.position.z = msg.pose.position.z + noise_z
            slam_msg.pose.orientation = msg.pose.orientation

            self.pub_slam_pose.publish(slam_msg)

            odom_msg = Odometry()
            odom_msg.header = slam_msg.header
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose = slam_msg.pose
            self.pub_slam_odom.publish(odom_msg)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = Member4SLAMMockNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Member 4 Visual SLAM Mock initialized.")


if __name__ == '__main__':
    main()
