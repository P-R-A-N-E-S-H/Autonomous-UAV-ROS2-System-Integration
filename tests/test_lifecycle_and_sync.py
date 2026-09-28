#!/usr/bin/env python3
"""Unit tests for Member 3 Lifecycle State Orchestrator & Sensor Synchronizer."""

import unittest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_system_orchestrator')))

from uav_system_orchestrator.lifecycle.lifecycle_state_orchestrator import ManagedLifecycleLogic, NodeLifecycleState
from uav_system_orchestrator.sync.multimodal_sensor_synchronizer import ApproximateTimeSyncBuffer


class TestLifecycleAndSync(unittest.TestCase):

    def setUp(self):
        self.lifecycle = ManagedLifecycleLogic("test_node")
        self.sync = ApproximateTimeSyncBuffer(slop_seconds=0.015)

    def test_lifecycle_full_sequence(self):
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.UNCONFIGURED)
        
        # Configure
        self.assertTrue(self.lifecycle.on_configure())
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.INACTIVE)
        
        # Activate
        self.assertTrue(self.lifecycle.on_activate())
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.ACTIVE)
        
        # Deactivate
        self.assertTrue(self.lifecycle.on_deactivate())
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.INACTIVE)
        
        # Cleanup
        self.assertTrue(self.lifecycle.on_cleanup())
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.UNCONFIGURED)
        
        # Shutdown
        self.assertTrue(self.lifecycle.on_shutdown())
        self.assertEqual(self.lifecycle.state, NodeLifecycleState.FINALIZED)

    def test_sync_within_slop(self):
        t0 = time.time()
        self.sync.add_odometry(t0, {"x": 1.0, "y": 2.0, "z": 2.5})
        self.sync.add_detection(t0 + 0.005, {"class": "medical_supply_crate"}) # 5ms delta <= 15ms slop
        
        matched = self.sync.synchronize_next()
        self.assertIsNotNone(matched)
        self.assertLessEqual(matched["time_skew_ms"], 15.0)
        self.assertEqual(matched["detection"]["class"], "medical_supply_crate")

    def test_sync_reject_large_delay(self):
        t0 = time.time()
        self.sync.add_odometry(t0, {"x": 1.0, "y": 2.0, "z": 2.5})
        self.sync.add_detection(t0 + 0.040, {"class": "medical_supply_crate"}) # 40ms delta > 15ms slop
        
        matched = self.sync.synchronize_next()
        self.assertIsNone(matched)


if __name__ == '__main__':
    unittest.main()
