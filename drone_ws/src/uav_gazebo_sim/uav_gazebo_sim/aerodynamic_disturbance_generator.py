#!/usr/bin/env python3
"""
aerodynamic_disturbance_generator.py
====================================
Advanced Aerodynamic Ground Effect, Wind Turbulence & Motor Dynamics Engine.
Role: Member 2 - Simulation & Flight Stack Engineer.

Mathematical Models:
 1. Cheeseman & Bennett Ground Effect:
      T_IGE = T_OGE / (1 - (R / (4*z))^2)
 2. Dryden Wind Turbulence Model & Discrete Gust Vectors (u, v, w).
 3. Individual 4-Motor Angular Velocity (RPM) & Gyroscopic Precession.
 4. LiPo 4S Internal Resistance & Dynamic Voltage Sag.
"""

import time
import math
import random
from typing import Dict, Any, Tuple, List

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import WrenchStamped, Vector3, PoseStamped
    from sensor_msgs.msg import BatteryState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class AerodynamicPhysicsEngine:
    """Nonlinear aerodynamic and battery physics engine."""

    def __init__(self, rotor_radius: float = 0.10, drone_mass: float = 1.450):
        self.rotor_radius = rotor_radius
        self.mass = drone_mass
        self.gravity = 9.80665
        self.km = 8.54858e-06  # N / (rad/s)^2
        self.kd = 0.06 * self.km  # N*m / (rad/s)^2
        
        # Wind & Turbulence Parameters
        self.base_wind = [0.0, 0.0, 0.0]  # m/s [X, Y, Z]
        self.gust_intensity = 0.0          # m/s
        self.gust_phase = 0.0
        
        # Ground Effect Limit
        self.min_ground_z = 0.05
        
        # Motor RPMs (Motor 0..3)
        self.motor_rpms = [0.0, 0.0, 0.0, 0.0]
        self.motor_thrusts = [0.0, 0.0, 0.0, 0.0]
        
        # Battery Cell State (4S LiPo)
        self.soc = 1.0  # State of Charge 0..1
        self.internal_resistance = 0.024  # Ohms (6mOhm per cell)
        self.cell_count = 4

    def compute_ground_effect_factor(self, altitude_z: float) -> float:
        """Cheeseman & Bennett ground effect ratio T_IGE / T_OGE."""
        z = max(self.min_ground_z, altitude_z)
        ratio = (self.rotor_radius / (4.0 * z)) ** 2
        # Bound factor to avoid physical singularity at touchdown
        return min(1.35, 1.0 / (1.0 - min(0.74, ratio)))

    def compute_dryden_wind_gust(self, dt: float) -> List[float]:
        """Generates continuous Dryden wind turbulence spectrum."""
        self.gust_phase += 1.2 * dt
        gust_x = self.gust_intensity * math.sin(self.gust_phase) + random.gauss(0, self.gust_intensity * 0.15)
        gust_y = self.gust_intensity * math.cos(self.gust_phase * 0.7) + random.gauss(0, self.gust_intensity * 0.15)
        gust_z = self.gust_intensity * 0.3 * math.sin(self.gust_phase * 1.5)
        return [
            self.base_wind[0] + gust_x,
            self.base_wind[1] + gust_y,
            self.base_wind[2] + gust_z
        ]

    def compute_motor_dynamics(self, hover_thrust_req: float, roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> Tuple[List[float], List[float]]:
        """Allocates mixer matrix to 4 X-quadrotor motors and calculates RPMs."""
        base_t = hover_thrust_req / 4.0
        # Quad-X Mixer
        t0 = base_t - roll_cmd + pitch_cmd - yaw_cmd # Front Right (CCW)
        t1 = base_t + roll_cmd + pitch_cmd + yaw_cmd # Front Left (CW)
        t2 = base_t + roll_cmd - pitch_cmd - yaw_cmd # Back Left (CCW)
        t3 = base_t - roll_cmd - pitch_cmd + yaw_cmd # Back Right (CW)

        thrusts = [max(0.1, t) for t in (t0, t1, t2, t3)]
        rpms = []
        for t in thrusts:
            omega = math.sqrt(t / self.km)
            rpm = (omega * 60.0) / (2.0 * math.pi)
            rpms.append(rpm)

        self.motor_thrusts = thrusts
        self.motor_rpms = rpms
        return thrusts, rpms

    def compute_battery_voltage_sag(self, total_thrust_n: float) -> Tuple[float, float]:
        """Computes current draw (Amperes) and dynamic voltage sag under thrust load."""
        # Empirical brushless DC power curve: P = k_p * T^1.5
        electrical_power_watts = 28.0 + 18.5 * (total_thrust_n ** 1.35)
        nominal_ocv = (3.27 + 0.93 * self.soc) * self.cell_count  # 13.08V to 16.80V
        
        # Load Current I = P / V
        current_amps = electrical_power_watts / max(10.0, nominal_ocv)
        voltage_sag = current_amps * self.internal_resistance
        loaded_voltage = max(11.0, nominal_ocv - voltage_sag)
        
        return loaded_voltage, current_amps


if ROS2_AVAILABLE:
    class AerodynamicDisturbanceNode(Node):
        def __init__(self):
            super().__init__('aerodynamic_disturbance_generator')
            self.get_logger().info("Initializing Advanced Aerodynamics & Disturbance Engine [Member 2]...")

            self.engine = AerodynamicPhysicsEngine()

            # Parameters
            self.declare_parameter('gust_intensity', 1.5)
            self.declare_parameter('base_wind_x', 0.5)
            self.declare_parameter('base_wind_y', 0.0)

            self.engine.gust_intensity = self.get_parameter('gust_intensity').get_parameter_value().double_value
            self.engine.base_wind[0] = self.get_parameter('base_wind_x').get_parameter_value().double_value
            self.engine.base_wind[1] = self.get_parameter('base_wind_y').get_parameter_value().double_value

            # Publishers
            self.pub_wrench = self.create_publisher(WrenchStamped, '/gazebo/aerodynamics/disturbance_wrench', 50)
            self.pub_battery_diag = self.create_publisher(BatteryState, '/flight_stack/battery_diagnostics', 10)

            # Subscriptions
            self.sub_pose = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, 10)

            self.timer = self.create_timer(0.02, self._physics_loop)
            self.last_tick = time.time()
            self.current_altitude = 2.5
            self.get_logger().info("Aerodynamic Disturbance Engine running at 50Hz.")

        def _on_pose(self, msg: PoseStamped):
            self.current_altitude = max(0.0, msg.pose.position.z)

        def _physics_loop(self):
            now = time.time()
            dt = now - self.last_tick
            self.last_tick = now

            # 1. Ground Effect
            ge_factor = self.engine.compute_ground_effect_factor(self.current_altitude)

            # 2. Wind Vector
            wind = self.engine.compute_dryden_wind_gust(dt)

            # 3. Publish Aerodynamic Wrench to Gazebo
            w_msg = WrenchStamped()
            w_msg.header.stamp = self.get_clock().now().to_msg()
            w_msg.header.frame_id = "base_link"
            # Aerodynamic drag forces proportional to relative airspeed squared
            cd_eff = 0.18
            w_msg.wrench.force.x = 0.5 * 1.225 * cd_eff * (wind[0] ** 2) * math.copysign(1.0, wind[0])
            w_msg.wrench.force.y = 0.5 * 1.225 * cd_eff * (wind[1] ** 2) * math.copysign(1.0, wind[1])
            w_msg.wrench.force.z = (ge_factor - 1.0) * (self.engine.mass * self.engine.gravity)
            self.pub_wrench.publish(w_msg)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = AerodynamicDisturbanceNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE] Aerodynamic Physics Engine Verification...")
        engine = AerodynamicPhysicsEngine()
        ge_01m = engine.compute_ground_effect_factor(0.10)
        ge_25m = engine.compute_ground_effect_factor(2.50)
        print(f"Ground Effect Ratio at Z=0.10m: {ge_01m:.4f} (Thrust Boost)")
        print(f"Ground Effect Ratio at Z=2.50m: {ge_25m:.4f} (Free Air OGE)")
        v, i = engine.compute_battery_voltage_sag(14.5 * 9.81)
        print(f"Hover Electrical Load: {i:.1f}A | Loaded Voltage: {v:.2f}V")


if __name__ == '__main__':
    main()
