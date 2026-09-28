#!/usr/bin/env python3
"""
tf_tree_manager.py
==================
Dynamic TF2 Coordinate Frame Transformation & Calibration Broadcaster.
Role: Member 3 - ROS2 & System Integration Engineer.

Coordinate Frame Architecture:
 [map] -> (Visual SLAM World Frame, Member 4)
   └── [odom] -> (Flight Controller EKF Odometry Frame, Member 2)
         └── [base_link] -> (Center of Mass of Quadrotor)
               ├── [camera_link] -> (Physical Camera Mounting Point)
               │     └── [camera_optical_frame] -> (OpenCV Standard Coordinate Frame)
               ├── [imu_link] -> (Inertial Measurement Unit)
               └── [lidar_link] -> (Optional Rangefinder)
"""

import math
import time
from typing import List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def quaternion_from_euler(ai: float, aj: float, ak: float) -> Tuple[float, float, float, float]:
    """Calculate quaternion (x, y, z, w) from euler angles (roll, pitch, yaw)."""
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    x = cj * sc - sj * cs
    y = cj * ss + sj * cc
    z = cj * cs - sj * sc
    w = cj * cc + sj * ss
    return x, y, z, w


if ROS2_AVAILABLE:
    class TFTreeManagerNode(Node):
        """ROS 2 Node for Coordinate Transform Management."""

        def __init__(self):
            super().__init__('tf_tree_manager')
            self.get_logger().info("Initializing Dynamic TF Tree Broadcaster [Member 3]...")

            self.tf_broadcaster = TransformBroadcaster(self)
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)

            # Publish static transforms on startup
            self._publish_static_transforms()

            # Subscribe to SLAM pose to publish dynamic map->odom->base_link
            self.sub_slam = self.create_subscription(
                PoseStamped, '/slam/pose', self._on_slam_pose, 10
            )

            self.get_logger().info("TF Tree Manager active and broadcasting.")

        def _publish_static_transforms(self):
            """Static extrinsics between base_link and drone sensors."""
            now = self.get_clock().now().to_msg()
            transforms: List[TransformStamped] = []

            # base_link -> camera_link (Camera mounted 12cm forward, 5cm up, tilted 15 deg down)
            t_cam = TransformStamped()
            t_cam.header.stamp = now
            t_cam.header.frame_id = 'base_link'
            t_child = 'camera_link'
            t_cam.child_frame_id = t_child
            t_cam.transform.translation.x = 0.12
            t_cam.transform.translation.y = 0.00
            t_cam.transform.translation.z = 0.05
            # Pitch down 15 degrees (-0.2618 rad)
            qx, qy, qz, qw = quaternion_from_euler(0.0, -0.2618, 0.0)
            t_cam.transform.rotation.x = qx
            t_cam.transform.rotation.y = qy
            t_cam.transform.rotation.z = qz
            t_cam.transform.rotation.w = qw
            transforms.append(t_cam)

            # camera_link -> camera_optical_frame (Standard optical rotation: X right, Y down, Z forward)
            t_opt = TransformStamped()
            t_opt.header.stamp = now
            t_opt.header.frame_id = 'camera_link'
            t_opt.child_frame_id = 'camera_optical_frame'
            t_opt.transform.translation.x = 0.0
            t_opt.transform.translation.y = 0.0
            t_opt.transform.translation.z = 0.0
            # Rotate -90 deg yaw, -90 deg pitch
            qx, qy, qz, qw = quaternion_from_euler(-math.pi/2, 0.0, -math.pi/2)
            t_opt.transform.rotation.x = qx
            t_opt.transform.rotation.y = qy
            t_opt.transform.rotation.z = qz
            t_opt.transform.rotation.w = qw
            transforms.append(t_opt)

            # Send static transforms
            self.static_tf_broadcaster.sendTransform(transforms)

        def _on_slam_pose(self, msg: PoseStamped):
            """Broadcast dynamic map -> base_link transform from Member 4 SLAM."""
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'map'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation = msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = TFTreeManagerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] TF Transform Math verification...")
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 4)
        print(f"Yaw 45 deg Quaternion: ({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        print(f"Norm verification: {norm:.6f} (Must be 1.000000)")


if __name__ == '__main__':
    main()
