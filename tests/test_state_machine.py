#!/usr/bin/env python3
"""Unit tests for Mission State Machine Logic (Member 3)."""

import unittest
import sys
import os

# Add orchestrator to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_system_orchestrator')))

from uav_system_orchestrator.mission_state_machine import MissionStateMachineLogic, MissionState


class TestMissionStateMachine(unittest.TestCase):

    def setUp(self):
        self.fsm = MissionStateMachineLogic(takeoff_altitude=2.5, acceptance_radius=0.3)

    def test_initial_state(self):
        self.assertEqual(self.fsm.state, MissionState.INIT)

    def test_prearm_transition(self):
        self.fsm.update_telemetry(0.0, 0.0, 0.0, 0.0, 95.0, False, True, True)
        st, _ = self.fsm.evaluate_step()
        self.assertEqual(st, MissionState.PREARM_CHECKS)

    def test_armed_transition(self):
        self.fsm.update_telemetry(0.0, 0.0, 0.0, 0.0, 95.0, False, True, True)
        self.fsm.evaluate_step() # to PREARM
        st, _ = self.fsm.evaluate_step() # to ARMED
        self.assertEqual(st, MissionState.ARMED)

    def test_takeoff_sequence(self):
        self.fsm.update_telemetry(0.0, 0.0, 0.0, 0.0, 95.0, True, True, True)
        self.fsm.evaluate_step()
        self.fsm.evaluate_step()
        st, cmd = self.fsm.evaluate_step()
        self.assertEqual(st, MissionState.TAKEOFF)
        self.assertEqual(cmd["type"], "TAKEOFF")
        self.assertEqual(cmd["altitude"], 2.5)

    def test_hover_arrival(self):
        self.fsm.state = MissionState.TAKEOFF
        self.fsm.update_telemetry(0.0, 0.0, 2.45, 0.0, 90.0, True, True, True)
        st, _ = self.fsm.evaluate_step()
        self.assertEqual(st, MissionState.HOVER_HOLD)

    def test_semantic_goal_arrival(self):
        self.fsm.state = MissionState.SEMANTIC_GOAL_NAV
        self.fsm.target_pose = {"x": 5.0, "y": 5.0, "z": 2.5}
        self.fsm.update_telemetry(4.95, 5.02, 2.48, 0.0, 85.0, True, True, True)
        st, _ = self.fsm.evaluate_step()
        self.assertEqual(st, MissionState.HOVER_HOLD)


if __name__ == '__main__':
    unittest.main()
