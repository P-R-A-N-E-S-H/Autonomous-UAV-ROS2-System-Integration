#!/usr/bin/env python3
"""Unit tests for Member 2 Flight Stack Dynamics & Mission Execution."""

import unittest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_flight_stack')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_mocks')))

from uav_flight_stack.flight_stack_mission_executor import FlightStackMissionLogic
from uav_mocks.member2_flight_sim_mock import DroneFlightPhysics


class TestFlightStackDynamics(unittest.TestCase):

    def setUp(self):
        self.logic = FlightStackMissionLogic(cruise_speed=1.8)
        self.physics = DroneFlightPhysics()

    def test_mission_loading(self):
        wps = [
            {"x": 1.0, "y": 2.0, "z": 2.5, "dwell_time": 0.1},
            {"x": 3.0, "y": 4.0, "z": 2.0, "dwell_time": 0.1}
        ]
        self.logic.load_waypoints(wps)
        self.assertTrue(self.logic.is_mission_active)
        self.assertEqual(len(self.logic.waypoints), 2)
        self.assertEqual(self.logic.current_wp_idx, 0)

    def test_waypoint_progression_and_completion(self):
        wps = [
            {"x": 0.0, "y": 0.0, "z": 2.5, "dwell_time": 0.05}
        ]
        self.logic.load_waypoints(wps)
        self.logic.current_pose = [0.0, 0.0, 2.5]
        
        # Initial arrival starts dwell
        step1 = self.logic.step()
        self.assertEqual(step1["type"], "NAVIGATE_TO_WAYPOINT")
        self.assertTrue(step1["is_dwelling"])

        time.sleep(0.08) # Wait for dwell time
        step2 = self.logic.step()
        self.assertEqual(step2["type"], "MISSION_COMPLETE")
        self.assertFalse(self.logic.is_mission_active)

    def test_physics_thrust_response(self):
        self.physics.target_sp = [0.0, 0.0, 2.5] # Climb to 2.5m
        for _ in range(30):
            self.physics.step(0.05)
        self.assertGreater(self.physics.pos[2], 2.0)

    def test_physics_battery_depletion(self):
        initial_bat = self.physics.battery_pct
        for _ in range(50):
            self.physics.step(0.1) # 5 seconds of flight
        self.assertLess(self.physics.battery_pct, initial_bat)


if __name__ == '__main__':
    unittest.main()
