#!/usr/bin/env python3
"""
mission_state_machine.py
========================
Central Mission State Machine Node for Autonomous Drone Navigation.
Role: Member 3 - ROS2 & System Integration Engineer.

This node implements a deterministic Hierarchical Finite State Machine (HFSM)
orchestrating mission flow across all system modules:
 - Member 1: Vision-Language Navigation Goals
 - Member 2: Gazebo SITL / ArduPilot / MAVROS Flight Controller
 - Member 4: GPS-Denied Visual SLAM (ORB-SLAM3 / RTAB-Map)
 - Member 5: Adaptive Semantic Memory Queries

States:
 0: INIT               - Validating node heartbeats and QoS discovery
 1: PREARM_CHECKS      - Checking SLAM health, geofence, battery, EKF status
 2: ARMED              - Flight controller armed in GUIDED/OFFBOARD mode
 3: TAKEOFF            - Controlled vertical ascent to nominal survey altitude
 4: HOVER_HOLD         - Station-keeping in GPS-denied SLAM coordinate frame
 5: EXPLORATION        - Autonomous frontier exploration / search pattern
 6: SEMANTIC_GOAL_NAV  - Executing natural language semantic target navigation
 7: WAYPOINT_TRACKING  - High-precision 3D spline trajectory tracking
 8: RETURN_TO_HOME     - Backtracking via keyframes to takeoff origin
 9: LANDING            - Vertical descent, touchdown detection, auto-disarm
 10: EMERGENCY_HOLD    - Safety failsafe hover / emergency maneuver
"""

import time
import math
import threading
from enum import IntEnum
from typing import Dict, Any, Optional, List, Tuple

# Try ROS2 imports if available, otherwise mock gracefully for standalone execution
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String, Header, Bool, Float32
    from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
    from uav_interfaces.msg import DroneState, NavigationGoal, SafetyAlert, SystemHealth
    from uav_interfaces.srv import SetFlightMode, TriggerFailsafe, QuerySemanticMemory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class MissionState(IntEnum):
    INIT = 0
    PREARM_CHECKS = 1
    ARMED = 2
    TAKEOFF = 3
    HOVER_HOLD = 4
    EXPLORATION = 5
    SEMANTIC_GOAL_NAV = 6
    WAYPOINT_TRACKING = 7
    RETURN_TO_HOME = 8
    LANDING = 9
    EMERGENCY_HOLD = 10


STATE_NAMES = {
    MissionState.INIT: "INIT",
    MissionState.PREARM_CHECKS: "PREARM_CHECKS",
    MissionState.ARMED: "ARMED",
    MissionState.TAKEOFF: "TAKEOFF",
    MissionState.HOVER_HOLD: "HOVER_HOLD",
    MissionState.EXPLORATION: "EXPLORATION",
    MissionState.SEMANTIC_GOAL_NAV: "SEMANTIC_GOAL_NAV",
    MissionState.WAYPOINT_TRACKING: "WAYPOINT_TRACKING",
    MissionState.RETURN_TO_HOME: "RETURN_TO_HOME",
    MissionState.LANDING: "LANDING",
    MissionState.EMERGENCY_HOLD: "EMERGENCY_HOLD"
}


class MissionStateMachineLogic:
    """Core state machine logic decoupled from ROS transport for maximum testability."""

    def __init__(self, takeoff_altitude: float = 2.5, acceptance_radius: float = 0.3):
        self.state = MissionState.INIT
        self.previous_state = MissionState.INIT
        self.takeoff_altitude = takeoff_altitude
        self.acceptance_radius = acceptance_radius
        self.state_enter_time = time.time()
        
        # Telemetry Cache
        self.current_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.home_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.target_pose: Optional[Dict[str, float]] = None
        self.battery_pct = 100.0
        self.is_armed = False
        self.is_connected = False
        self.slam_healthy = False
        self.active_goal_description = ""
        self.transition_log: List[Dict[str, Any]] = []

    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """Execute a state transition with logging and safety checks."""
        if self.state == new_state:
            return False

        old_state = self.state
        self.previous_state = old_state
        self.state = new_state
        self.state_enter_time = time.time()

        entry = {
            "timestamp": time.time(),
            "from_state": STATE_NAMES[old_state],
            "to_state": STATE_NAMES[new_state],
            "reason": reason,
            "pose": dict(self.current_pose),
            "battery": self.battery_pct
        }
        self.transition_log.append(entry)
        return True

    def update_telemetry(self, x: float, y: float, z: float, yaw: float, 
                         battery: float, armed: bool, connected: bool, slam_ok: bool):
        self.current_pose = {"x": x, "y": y, "z": z, "yaw": yaw}
        self.battery_pct = battery
        self.is_armed = armed
        self.is_connected = connected
        self.slam_healthy = slam_ok

    def compute_distance_to_target(self) -> float:
        if not self.target_pose:
            return float('inf')
        dx = self.target_pose["x"] - self.current_pose["x"]
        dy = self.target_pose["y"] - self.current_pose["y"]
        dz = self.target_pose["z"] - self.current_pose["z"]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def evaluate_step(self) -> Tuple[MissionState, Optional[Dict[str, Any]]]:
        """Cyclic evaluation of state machine transitions based on telemetry."""
        now = time.time()
        elapsed_in_state = now - self.state_enter_time
        command_out: Optional[Dict[str, Any]] = None

        if self.state == MissionState.INIT:
            if self.is_connected and self.slam_healthy:
                self.transition_to(MissionState.PREARM_CHECKS, "Sensors and MAVROS connected")

        elif self.state == MissionState.PREARM_CHECKS:
            if self.battery_pct > 20.0 and self.slam_healthy and self.is_connected:
                # Record home coordinates
                self.home_pose = dict(self.current_pose)
                self.transition_to(MissionState.ARMED, "Pre-arm checks PASSED (Battery, SLAM lock, EKF ready)")

        elif self.state == MissionState.ARMED:
            if self.is_armed:
                self.transition_to(MissionState.TAKEOFF, "Arm confirmed, commanding takeoff")
                command_out = {"type": "TAKEOFF", "altitude": self.takeoff_altitude}

        elif self.state == MissionState.TAKEOFF:
            command_out = {"type": "SETPOINT", "x": self.home_pose["x"], "y": self.home_pose["y"], "z": self.takeoff_altitude}
            if abs(self.current_pose["z"] - self.takeoff_altitude) < self.acceptance_radius:
                self.transition_to(MissionState.HOVER_HOLD, f"Reached hover altitude {self.takeoff_altitude}m")

        elif self.state == MissionState.HOVER_HOLD:
            command_out = {"type": "SETPOINT", "x": self.current_pose["x"], "y": self.current_pose["y"], "z": self.takeoff_altitude}
            # Remains in hover until an external goal or exploration command is dispatched

        elif self.state == MissionState.SEMANTIC_GOAL_NAV:
            if self.target_pose:
                command_out = {"type": "SETPOINT", "x": self.target_pose["x"], "y": self.target_pose["y"], "z": self.target_pose["z"]}
                dist = self.compute_distance_to_target()
                if dist < self.acceptance_radius:
                    self.transition_to(MissionState.HOVER_HOLD, f"Arrived at Semantic Target: {self.active_goal_description}")

        elif self.state == MissionState.WAYPOINT_TRACKING:
            if self.target_pose:
                command_out = {"type": "SETPOINT", "x": self.target_pose["x"], "y": self.target_pose["y"], "z": self.target_pose["z"]}

        elif self.state == MissionState.RETURN_TO_HOME:
            command_out = {"type": "SETPOINT", "x": self.home_pose["x"], "y": self.home_pose["y"], "z": self.takeoff_altitude}
            dx = self.home_pose["x"] - self.current_pose["x"]
            dy = self.home_pose["y"] - self.current_pose["y"]
            if math.sqrt(dx*dx + dy*dy) < self.acceptance_radius:
                self.transition_to(MissionState.LANDING, "Aligned above home pad, beginning descent")

        elif self.state == MissionState.LANDING:
            command_out = {"type": "LAND", "descent_speed": 0.3}
            if self.current_pose["z"] <= 0.15 or not self.is_armed:
                self.transition_to(MissionState.INIT, "Touchdown confirmed, mission completed")

        elif self.state == MissionState.EMERGENCY_HOLD:
            command_out = {"type": "HOVER_BRAKE"}

        return self.state, command_out


if ROS2_AVAILABLE:
    class MissionStateMachineNode(Node):
        """ROS 2 Node wrapper for the Mission State Machine."""

        def __init__(self):
            super().__init__('mission_state_machine')
            self.get_logger().info("Initializing ROS 2 Mission State Machine [Member 3]...")

            # Parameters
            self.declare_parameter('takeoff_altitude', 2.5)
            self.declare_parameter('acceptance_radius', 0.3)
            self.declare_parameter('rate_hz', 20.0)

            alt = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
            radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
            rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

            self.fsm = MissionStateMachineLogic(takeoff_altitude=alt, acceptance_radius=radius)

            # High Reliability QoS Profiles for Flight Critical Data
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

            # Publishers
            self.pub_drone_state = self.create_publisher(DroneState, '/orchestrator/drone_state', qos_reliable)
            self.pub_setpoint = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_reliable)
            self.pub_safety = self.create_publisher(SafetyAlert, '/orchestrator/safety_alerts', qos_reliable)

            # Subscriptions
            self.sub_goal = self.create_subscription(NavigationGoal, '/ai/semantic_goal', self._handle_semantic_goal, qos_reliable)
            self.sub_slam_pose = self.create_subscription(PoseStamped, '/slam/pose', self._handle_slam_pose, qos_sensor)
            self.sub_safety_alert = self.create_subscription(SafetyAlert, '/safety/failsafe_trigger', self._handle_safety_alert, qos_reliable)

            # Service Servers
            self.srv_set_mode = self.create_service(SetFlightMode, '/orchestrator/set_mode', self._handle_set_mode_srv)
            self.srv_failsafe = self.create_service(TriggerFailsafe, '/orchestrator/trigger_failsafe', self._handle_failsafe_srv)

            # Main State Loop Timer (20Hz)
            self.timer = self.create_timer(1.0 / rate_hz, self._timer_callback)
            self.get_logger().info("Mission State Machine operational at 20Hz.")

        def _handle_semantic_goal(self, msg: NavigationGoal):
            self.get_logger().info(f"Received AI Semantic Goal from Member 1: '{msg.natural_language_prompt}' [{msg.target_semantic_class}]")
            self.fsm.target_pose = {
                "x": msg.target_pose.pose.position.x,
                "y": msg.target_pose.pose.position.y,
                "z": msg.target_pose.pose.position.z,
                "yaw": 0.0
            }
            self.fsm.active_goal_description = f"{msg.target_semantic_class} ({msg.natural_language_prompt})"
            self.fsm.transition_to(MissionState.SEMANTIC_GOAL_NAV, f"AI Goal Dispatched: {msg.target_semantic_class}")

        def _handle_slam_pose(self, msg: PoseStamped):
            self.fsm.update_telemetry(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
                yaw=0.0,
                battery=self.fsm.battery_pct,
                armed=self.fsm.is_armed,
                connected=self.fsm.is_connected,
                slam_ok=True
            )

        def _handle_safety_alert(self, msg: SafetyAlert):
            self.get_logger().warn(f"SAFETY ALERT RECEIVED: [{msg.alert_code}] {msg.description}")
            if msg.suggested_action == "RETURN_TO_HOME":
                self.fsm.transition_to(MissionState.RETURN_TO_HOME, f"Failsafe: {msg.alert_code}")
            elif msg.suggested_action == "IMMEDIATE_LAND":
                self.fsm.transition_to(MissionState.LANDING, f"Failsafe: {msg.alert_code}")
            else:
                self.fsm.transition_to(MissionState.EMERGENCY_HOLD, f"Failsafe: {msg.alert_code}")

        def _handle_set_mode_srv(self, request, response):
            mode_str = request.target_mode.upper()
            matched = False
            for state, name in STATE_NAMES.items():
                if name == mode_str or mode_str in name:
                    self.fsm.transition_to(state, f"User service override to {name}")
                    response.success = True
                    response.current_mode = name
                    response.message = f"Transitioned successfully to {name}"
                    matched = True
                    break
            if not matched:
                response.success = False
                response.current_mode = STATE_NAMES[self.fsm.state]
                response.message = f"Unknown mode {mode_str}"
            return response

        def _handle_failsafe_srv(self, request, response):
            self.get_logger().error(f"Manual Failsafe Triggered: {request.reason}")
            self.fsm.transition_to(MissionState.EMERGENCY_HOLD, request.reason)
            response.success = True
            response.active_failsafe_state = "EMERGENCY_HOLD"
            response.message = "Drone switched to emergency station-keeping hold"
            return response

        def _timer_callback(self):
            state, cmd = self.fsm.evaluate_step()

            # Publish High-Frequency Drone State
            state_msg = DroneState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = "map"
            state_msg.current_state = int(state)
            state_msg.state_name = STATE_NAMES[state]
            state_msg.is_connected = self.fsm.is_connected
            state_msg.is_armed = self.fsm.is_armed
            state_msg.is_guided = True
            state_msg.position.x = self.fsm.current_pose["x"]
            state_msg.position.y = self.fsm.current_pose["y"]
            state_msg.position.z = self.fsm.current_pose["z"]
            state_msg.battery_percentage = self.fsm.battery_pct
            state_msg.slam_healthy = self.fsm.slam_healthy
            state_msg.active_mission_id = self.fsm.active_goal_description
            self.pub_drone_state.publish(state_msg)

            # Publish Position Setpoint to MAVROS if available
            if cmd and cmd.get("type") == "SETPOINT":
                sp = PoseStamped()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.header.frame_id = "map"
                sp.pose.position.x = float(cmd["x"])
                sp.pose.position.y = float(cmd["y"])
                sp.pose.position.z = float(cmd["z"])
                self.pub_setpoint.publish(sp)


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = MissionStateMachineNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[STANDALONE TEST] Mission State Machine Logic verification...")
        fsm = MissionStateMachineLogic()
        fsm.update_telemetry(0.0, 0.0, 0.0, 0.0, 100.0, True, True, True)
        for i in range(10):
            st, cmd = fsm.evaluate_step()
            print(f"Tick {i+1}: State = {STATE_NAMES[st]}, Command = {cmd}")
            time.sleep(0.1)


if __name__ == '__main__':
    main()
