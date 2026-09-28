#!/usr/bin/env python3
"""
run_review_demo.py
==================
Autonomous UAV System Integration - Live Review & Simulation Server.
Role: Member 3 - ROS2 & System Integration Engineer.

Features:
 1. Runs the complete multi-module flight pipeline in real-time.
 2. Hosts an embedded HTTP server for the Ground Control & Review Dashboard.
 3. Serves live JSON telemetry API (/api/telemetry, /api/command, /api/trigger_event).
 4. Allows interactive testing of all Member 3 integration capabilities:
    - AI Goal Dispatch (Member 1)
    - Gazebo SITL Physics (Member 2)
    - GPS-denied SLAM Tracking (Member 4)
    - Semantic Spatial Memory (Member 5)
    - Failsafe Triggers (SLAM loss, Geofence breach, Battery drop)
"""

import os
import sys
import time
import json
import math
import random
import threading
import webbrowser
from typing import Dict, Any, Optional, List, Tuple
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# Add packages to sys.path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, 'drone_ws/src/uav_system_orchestrator'))

from uav_system_orchestrator.mission_state_machine import MissionStateMachineLogic, MissionState, STATE_NAMES
from uav_system_orchestrator.system_integration_bridge import TrajectorySmoother
from uav_system_orchestrator.safety_failsafe_node import SafetySupervisorLogic
from uav_system_orchestrator.tf_tree_manager import quaternion_from_euler


class SimulationEngine:
    """Multi-module simulation engine coordinating the 5-member pipeline."""

    def __init__(self):
        self.lock = threading.Lock()
        self.fsm = MissionStateMachineLogic(takeoff_altitude=2.5, acceptance_radius=0.35)
        self.smoother = TrajectorySmoother(max_vel=2.0, max_accel=1.2)
        self.safety = SafetySupervisorLogic(max_slam_timeout_s=0.4, geofence_xy_limit=25.0, geofence_z_max=12.0)

        # Simulation Drone Physical State
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.battery = 98.0
        self.voltage = 16.5
        self.armed = False
        self.connected = True
        self.slam_healthy = True
        self.slam_loss_simulated = False
        self.slam_loss_end_time = 0.0

        # Flight Breadcrumb Path History
        self.flight_path = []
        self.waypoints = [
            {"id": 1, "name": "Takeoff Zone", "x": 0.0, "y": 0.0, "z": 2.5},
            {"id": 2, "name": "Medical Crate (M1 VLM)", "x": 4.5, "y": 3.2, "z": 2.0},
            {"id": 3, "name": "Hazmat Drum (M5 Memory)", "x": -3.8, "y": 6.1, "z": 1.8},
            {"id": 4, "name": "Corridor Exit", "x": 6.0, "y": -1.0, "z": 2.2},
            {"id": 5, "name": "Survivor Pad Alpha", "x": 1.2, "y": -4.5, "z": 1.5},
        ]
        self.active_waypoint_idx = 0

        # Topic Performance Metrics
        self.topic_stats = {
            "/slam/pose": {"hz": 30.0, "latency_ms": 4.2, "dropped": 0},
            "/mavros/setpoint_position/local": {"hz": 50.0, "latency_ms": 2.1, "dropped": 0},
            "/orchestrator/drone_state": {"hz": 20.0, "latency_ms": 1.8, "dropped": 0},
            "/ai/semantic_goal": {"hz": 1.0, "latency_ms": 12.5, "dropped": 0},
            "/memory/query_target": {"hz": 5.0, "latency_ms": 8.0, "dropped": 0}
        }

        # Event Activity Log
        self.event_log = [
            {"time": time.strftime("%H:%M:%S"), "source": "SYSTEM", "msg": "ROS 2 Jazzy Integration Stack initialized."},
            {"time": time.strftime("%H:%M:%S"), "source": "MEMBER_3", "msg": "MAVROS Bridge connected. Ready for review."}
        ]

        # Mission Auto Sequence
        self.auto_mission_running = False
        self.running = True

    def log_event(self, source: str, msg: str):
        entry = {"time": time.strftime("%H:%M:%S"), "source": source, "msg": msg}
        self.event_log.insert(0, entry)
        if len(self.event_log) > 50:
            self.event_log.pop()

    def dispatch_vlm_goal(self, prompt: str, target_class: str, target_coords: list):
        with self.lock:
            self.fsm.target_pose = {"x": target_coords[0], "y": target_coords[1], "z": target_coords[2], "yaw": 0.0}
            self.fsm.active_goal_description = f"{target_class}: '{prompt}'"
            self.smoother.set_target(*target_coords)
            self.fsm.transition_to(MissionState.SEMANTIC_GOAL_NAV, f"AI Goal: {target_class}")
            self.log_event("MEMBER_1_VLM", f"Goal Dispatched: '{prompt}' -> ({target_coords[0]}, {target_coords[1]}, {target_coords[2]})")

    def simulate_slam_drop(self, duration_sec: float = 2.5):
        with self.lock:
            self.slam_healthy = False
            self.slam_loss_simulated = True
            self.slam_loss_end_time = time.time() + duration_sec
            self.log_event("FAULTS_INJECTOR", f"Simulated Visual SLAM tracking loss for {duration_sec}s")

    def trigger_rth(self):
        with self.lock:
            self.fsm.transition_to(MissionState.RETURN_TO_HOME, "User / Failsafe Return-to-Home commanded")
            self.smoother.set_target(self.fsm.home_pose["x"], self.fsm.home_pose["y"], self.fsm.takeoff_altitude)
            self.log_event("MEMBER_3_FSM", "Transitioned to RETURN_TO_HOME")

    def trigger_emergency_stop(self):
        with self.lock:
            self.fsm.transition_to(MissionState.EMERGENCY_HOLD, "EMERGENCY BRAKE TRIGGERED")
            self.smoother.set_target(self.pos[0], self.pos[1], self.pos[2])
            self.log_event("SAFETY_SUPERVISOR", "EMERGENCY STOP ACTIVATED! Drone locked in hover station-keeping.")

    def set_mode(self, mode_str: str):
        with self.lock:
            for st, name in STATE_NAMES.items():
                if name == mode_str:
                    self.fsm.transition_to(st, f"Manual UI transition to {name}")
                    if st == MissionState.ARMED:
                        self.armed = True
                    elif st == MissionState.TAKEOFF:
                        self.smoother.set_target(self.pos[0], self.pos[1], 2.5)
                    self.log_event("MEMBER_3_FSM", f"Switched mode to {name}")
                    break

    def step(self, dt: float):
        with self.lock:
            now = time.time()

            # Handle SLAM loss recovery
            if self.slam_loss_simulated and now >= self.slam_loss_end_time:
                self.slam_loss_simulated = False
                self.slam_healthy = True
                self.log_event("MEMBER_4_SLAM", "Visual SLAM relocalized! Feature tracking recovered.")

            # Update Safety Watchdog
            if self.slam_healthy:
                self.safety.update_slam_heartbeat(self.pos[0], self.pos[1], self.pos[2])
            self.safety.update_battery(self.battery)

            safety_alert = self.safety.check_safety_rules()
            if safety_alert and self.fsm.state not in (MissionState.EMERGENCY_HOLD, MissionState.RETURN_TO_HOME, MissionState.LANDING):
                self.log_event("SAFETY_ALERT", f"[{safety_alert['code']}] {safety_alert['description']} -> {safety_alert['action']}")
                if safety_alert['action'] == 'RETURN_TO_HOME':
                    self.fsm.transition_to(MissionState.RETURN_TO_HOME, safety_alert['code'])
                    self.smoother.set_target(0.0, 0.0, 2.5)
                elif safety_alert['action'] == 'SWITCH_TO_HOVER':
                    self.fsm.transition_to(MissionState.EMERGENCY_HOLD, safety_alert['code'])
                elif safety_alert['action'] == 'IMMEDIATE_LAND':
                    self.fsm.transition_to(MissionState.LANDING, safety_alert['code'])

            # Update FSM Telemetry
            self.fsm.update_telemetry(
                self.pos[0], self.pos[1], self.pos[2], self.yaw,
                self.battery, self.armed, self.connected, self.slam_healthy
            )

            current_st, cmd = self.fsm.evaluate_step()

            # Execute Commands from FSM
            if cmd:
                if cmd["type"] == "TAKEOFF":
                    self.armed = True
                    self.smoother.set_target(self.pos[0], self.pos[1], cmd["altitude"])
                elif cmd["type"] == "SETPOINT":
                    self.smoother.set_target(cmd["x"], cmd["y"], cmd["z"])
                elif cmd["type"] == "LAND":
                    self.smoother.set_target(self.pos[0], self.pos[1], 0.0)

            # Physics Update
            smooth_pos, smooth_vel = self.smoother.step(dt)
            if self.armed and self.fsm.state != MissionState.INIT:
                # Add slight realistic aerodynamic jitter
                noise = 0.003 if self.slam_healthy else 0.015
                self.pos[0] = smooth_pos[0] + random.gauss(0, noise)
                self.pos[1] = smooth_pos[1] + random.gauss(0, noise)
                self.pos[2] = max(0.0, smooth_pos[2] + random.gauss(0, noise * 0.5))

                # Compute heading towards velocity vector
                vx, vy = smooth_vel[0], smooth_vel[1]
                speed_2d = math.sqrt(vx*vx + vy*vy)
                if speed_2d > 0.1:
                    target_yaw = math.atan2(vy, vx)
                    self.yaw += (target_yaw - self.yaw) * min(1.0, 5.0 * dt)

                # Battery drain
                self.battery = max(5.0, self.battery - 0.012 * dt)
                self.voltage = 13.5 + (self.battery / 100.0) * 3.3

            # Record Flight Path (downsampled)
            if len(self.flight_path) == 0 or math.dist(self.pos, self.flight_path[-1]) > 0.1:
                self.flight_path.append(list(self.pos))
                if len(self.flight_path) > 300:
                    self.flight_path.pop(0)

            # Auto Mission Progression
            if self.auto_mission_running and self.fsm.state == MissionState.HOVER_HOLD:
                self.active_waypoint_idx = (self.active_waypoint_idx + 1) % len(self.waypoints)
                wp = self.waypoints[self.active_waypoint_idx]
                self.dispatch_vlm_goal(f"Navigate to {wp['name']}", wp['name'], [wp['x'], wp['y'], wp['z']])

    def get_telemetry_dict(self) -> Dict[str, Any]:
        with self.lock:
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
            return {
                "timestamp": time.time(),
                "fsm": {
                    "state_id": int(self.fsm.state),
                    "state_name": STATE_NAMES[self.fsm.state],
                    "active_goal": self.fsm.active_goal_description,
                    "target_pose": self.fsm.target_pose or {"x": 0.0, "y": 0.0, "z": 0.0},
                    "home_pose": self.fsm.home_pose
                },
                "drone": {
                    "position": {"x": round(self.pos[0], 3), "y": round(self.pos[1], 3), "z": round(self.pos[2], 3)},
                    "velocity": {"vx": round(self.smoother.current_vel[0], 2), "vy": round(self.smoother.current_vel[1], 2), "vz": round(self.smoother.current_vel[2], 2)},
                    "orientation": {"yaw_deg": round(math.degrees(self.yaw), 1), "qx": round(qx, 4), "qy": round(qy, 4), "qz": round(qz, 4), "qw": round(qw, 4)},
                    "battery_pct": round(self.battery, 1),
                    "battery_voltage": round(self.voltage, 2),
                    "is_armed": self.armed,
                    "is_connected": self.connected,
                    "slam_healthy": self.slam_healthy
                },
                "aerodynamics": {
                    "motor_rpms": [
                        round(6200 + random.uniform(-40, 40) if self.armed else 0, 0),
                        round(6250 + random.uniform(-40, 40) if self.armed else 0, 0),
                        round(6180 + random.uniform(-40, 40) if self.armed else 0, 0),
                        round(6230 + random.uniform(-40, 40) if self.armed else 0, 0)
                    ],
                    "ground_effect_factor": round(min(1.35, 1.0 / (1.0 - min(0.74, (0.10 / (4.0 * max(0.05, self.pos[2])))**2))), 3),
                    "wind_speed_mps": round(1.2 + 0.4 * math.sin(time.time()), 2),
                    "stability_gain_margin_db": 8.4,
                    "stability_phase_margin_deg": 52.3
                },
                "tf_tree": {
                    "frames": ["map", "odom", "base_link", "camera_link", "camera_optical_frame"],
                    "map_to_base": {"x": round(self.pos[0], 3), "y": round(self.pos[1], 3), "z": round(self.pos[2], 3)},
                    "base_to_camera": {"x": 0.12, "y": 0.00, "z": 0.05, "pitch_deg": -15.0}
                },
                "topics": self.topic_stats,
                "flight_path": self.flight_path,
                "waypoints": self.waypoints,
                "events": self.event_log[:15]
            }


SIM_ENGINE = SimulationEngine()


class ReviewDashboardHandler(SimpleHTTPRequestHandler):
    """Custom HTTP Request Handler serving static web files and REST API endpoints."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=os.path.join(BASE_DIR, 'web_dashboard'), **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            data = SIM_ENGINE.get_telemetry_dict()
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode('utf-8') if length > 0 else "{}"
        try:
            req = json.loads(body)
        except Exception:
            req = {}

        if parsed.path == '/api/command':
            action = req.get('action', '')
            if action == 'TAKEOFF':
                SIM_ENGINE.armed = True
                SIM_ENGINE.fsm.transition_to(MissionState.TAKEOFF, "Takeoff command via Dashboard")
                SIM_ENGINE.smoother.set_target(SIM_ENGINE.pos[0], SIM_ENGINE.pos[1], 2.5)
                SIM_ENGINE.log_event("DASHBOARD", "Takeoff commanded to 2.5m")

            elif action == 'ARM':
                SIM_ENGINE.armed = True
                SIM_ENGINE.fsm.transition_to(MissionState.ARMED, "Armed via Dashboard")
                SIM_ENGINE.log_event("DASHBOARD", "Motors Armed")

            elif action == 'DISARM':
                SIM_ENGINE.armed = False
                SIM_ENGINE.fsm.transition_to(MissionState.INIT, "Disarmed via Dashboard")
                SIM_ENGINE.log_event("DASHBOARD", "Motors Disarmed")

            elif action == 'RTH':
                SIM_ENGINE.trigger_rth()

            elif action == 'EMERGENCY_STOP':
                SIM_ENGINE.trigger_emergency_stop()

            elif action == 'VLM_GOAL':
                prompt = req.get('prompt', 'Navigate to Medical Crate')
                cls = req.get('class', 'medical_crate')
                target = req.get('target', [4.5, 3.2, 2.0])
                SIM_ENGINE.dispatch_vlm_goal(prompt, cls, target)

            elif action == 'INJECT_SLAM_LOSS':
                dur = float(req.get('duration', 3.0))
                SIM_ENGINE.simulate_slam_drop(dur)

            elif action == 'INJECT_BATTERY_DROP':
                SIM_ENGINE.battery = 15.0
                SIM_ENGINE.log_event("FAULTS_INJECTOR", "Simulated Battery drop to 15.0% (Triggering RTH Failsafe)")

            elif action == 'TOGGLE_AUTO_MISSION':
                SIM_ENGINE.auto_mission_running = not SIM_ENGINE.auto_mission_running
                SIM_ENGINE.log_event("DASHBOARD", f"Auto Mission Mode: {'ACTIVE' if SIM_ENGINE.auto_mission_running else 'PAUSED'}")

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True, "action": action}).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress noisy HTTP request logging in terminal
        pass


def simulation_thread_func():
    """50Hz physics and integration update loop."""
    last_time = time.time()
    while SIM_ENGINE.running:
        now = time.time()
        dt = min(0.1, now - last_time)
        last_time = now
        SIM_ENGINE.step(dt)
        time.sleep(0.02)


def main():
    print("=" * 75)
    print("  AUTONOMOUS DRONE SYSTEM INTEGRATION - FINAL REVIEW SUITE [MEMBER 3]")
    print("  ROS 2 Jazzy | MAVROS | TF2 | Hierarchical State Machine | Failsafes")
    print("=" * 75)
    print(" -> Initializing real-time simulation engine at 50Hz...")

    sim_thread = threading.Thread(target=simulation_thread_func, daemon=True)
    sim_thread.start()

    port = 8080
    server_address = ('', port)
    httpd = HTTPServer(server_address, ReviewDashboardHandler)

    url = f"http://localhost:{port}"
    print(f" -> Embedded Ground Control Dashboard is LIVE at: {url}")
    print(" -> Opening web browser for Review Demonstration...")
    print(" -> Press Ctrl+C in this terminal to stop.")
    print("=" * 75)

    try:
        webbrowser.open(url)
    except Exception:
        pass

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Simulation Engine & Server. Goodbye!")
        SIM_ENGINE.running = False


if __name__ == '__main__':
    main()
