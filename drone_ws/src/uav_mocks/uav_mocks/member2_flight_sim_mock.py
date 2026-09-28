#!/usr/bin/env python3
"""
member2_flight_sim_mock.py
==========================
Mock Node for Member 2: Simulation & Flight Stack Engineer.
Simulates: Gazebo Harmonic 6-DOF drone dynamics, ArduPilot SITL, MAVROS local pose & battery.
"""

import time
import math
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from sensor_msgs.msg import BatteryState
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class DroneFlightPhysics:
    def __init__(self):
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.target_sp = [0.0, 0.0, 0.0]
        self.battery_pct = 98.5
        self.voltage = 16.6
        self.is_armed = True
        self.mode = "GUIDED"

    def step(self, dt: float):
        # 2nd order closed-loop response modeling motor thrust and drag
        kp = 2.0
        damping = 1.2
        for i in range(3):
            accel = kp * (self.target_sp[i] - self.pos[i]) - damping * self.vel[i]
            self.vel[i] += accel * dt
            # Speed limiter
            speed = abs(self.vel[i])
            if speed > 2.5:
                self.vel[i] = math.copysign(2.5, self.vel[i])
            self.pos[i] += self.vel[i] * dt

        # Slow battery discharge (0.015% per second)
        self.battery_pct = max(5.0, self.battery_pct - 0.015 * dt)
        self.voltage = 13.5 + (self.battery_pct / 100.0) * 3.3


if ROS2_AVAILABLE:
    class Member2FlightSimNode(Node):
        def __init__(self):
            super().__init__('member2_gazebo_sitl_bridge')
            self.get_logger().info("Starting Member 2 [Gazebo Harmonic SITL Sim] Mock Node...")

            self.sim = DroneFlightPhysics()

            # Publishers (MAVROS interface)
            self.pub_local_pose = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 50)
            self.pub_battery = self.create_publisher(BatteryState, '/mavros/battery', 10)

            # Subscriptions
            self.sub_setpoint = self.create_subscription(
                PoseStamped, '/mavros/setpoint_position/local', self._on_setpoint, 10
            )

            self.timer = self.create_timer(0.02, self._sim_tick) # 50Hz physics loop
            self.last_tick = time.time()
            self.get_logger().info("Member 2 Flight Physics active at 50Hz.")

        def _on_setpoint(self, msg: PoseStamped):
            self.sim.target_sp = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]

        def _sim_tick(self):
            now = time.time()
            dt = now - self.last_tick
            self.last_tick = now

            self.sim.step(dt)

            # Publish local position
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "odom"
            pose_msg.pose.position.x = self.sim.pos[0]
            pose_msg.pose.position.y = self.sim.pos[1]
            pose_msg.pose.position.z = self.sim.pos[2]
            pose_msg.pose.orientation.w = 1.0
            self.pub_local_pose.publish(pose_msg)

            # Publish battery state
            bat = BatteryState()
            bat.header.stamp = pose_msg.header.stamp
            bat.voltage = float(self.sim.voltage)
            bat.percentage = float(self.sim.battery_pct / 100.0)
            bat.present = True
            self.pub_battery.publish(bat)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = Member2FlightSimNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Testing Member 2 flight physics...")
        sim = DroneFlightPhysics()
        sim.target_sp = [1.0, 2.0, 2.5]
        for _ in range(10):
            sim.step(0.1)
            print(f"Pos: ({sim.pos[0]:.2f}, {sim.pos[1]:.2f}, {sim.pos[2]:.2f}) Bat: {sim.battery_pct:.1f}%")


if __name__ == '__main__':
    main()
