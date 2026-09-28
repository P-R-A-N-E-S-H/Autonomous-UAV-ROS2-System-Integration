#!/usr/bin/env python3
"""Unit tests for Safety Supervisor & Fail-Safe Recovery (Member 3)."""

import unittest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_system_orchestrator')))

from uav_system_orchestrator.safety_failsafe_node import SafetySupervisorLogic


class TestSafetyFailsafe(unittest.TestCase):

    def setUp(self):
        self.safety = SafetySupervisorLogic(
            max_slam_timeout_s=0.25,
            geofence_xy_limit=20.0,
            geofence_z_max=10.0
        )

    def test_nominal_conditions(self):
        self.safety.update_slam_heartbeat(5.0, 5.0, 2.5)
        self.safety.update_battery(85.0)
        alert = self.safety.check_safety_rules()
        self.assertIsNone(alert)

    def test_slam_tracking_loss(self):
        self.safety.update_slam_heartbeat(0.0, 0.0, 2.0)
        time.sleep(0.3)
        alert = self.safety.check_safety_rules()
        self.assertIsNotNone(alert)
        self.assertEqual(alert["code"], "SLAM_TRACKING_LOST")
        self.assertEqual(alert["action"], "SWITCH_TO_HOVER")

    def test_geofence_breach(self):
        self.safety.update_slam_heartbeat(25.0, 5.0, 2.0) # > 20.0m limit
        alert = self.safety.check_safety_rules()
        self.assertIsNotNone(alert)
        self.assertEqual(alert["code"], "GEOFENCE_BREACH_XY")
        self.assertEqual(alert["action"], "RETURN_TO_HOME")

    def test_battery_critical_land(self):
        self.safety.update_slam_heartbeat(0.0, 0.0, 2.0)
        self.safety.update_battery(8.0) # < 10%
        alert = self.safety.check_safety_rules()
        self.assertIsNotNone(alert)
        self.assertEqual(alert["code"], "BATTERY_CRITICAL_10")
        self.assertEqual(alert["action"], "IMMEDIATE_LAND")


if __name__ == '__main__':
    unittest.main()
