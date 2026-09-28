#!/usr/bin/env python3
"""Unit tests for QoS Reliability & Latency Monitoring (Member 3)."""

import unittest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_system_orchestrator')))

from uav_system_orchestrator.system_health_monitor import TopicStats


class TestQoSReliability(unittest.TestCase):

    def test_topic_frequency_calculation(self):
        stats = TopicStats('/slam/pose', expected_hz=50.0)
        start_time = time.time() - 1.05
        stats.last_time = start_time
        stats.count = 52
        stats.tick(time.time() - 0.01)
        self.assertGreater(stats.current_hz, 45.0)

    def test_latency_calculation(self):
        stats = TopicStats('/slam/pose', expected_hz=50.0)
        now = time.time()
        stats.tick(msg_timestamp=now - 0.015) # 15ms latency
        self.assertAlmostEqual(stats.latency_ms, 15.0, delta=5.0)

    def test_packet_drop_detection(self):
        stats = TopicStats('/slam/pose', expected_hz=50.0)
        stats.last_time = time.time() - 1.1
        stats.count = 20 # Only 20 received when 50 expected
        stats.tick(time.time())
        self.assertGreater(stats.dropped_count, 0)


if __name__ == '__main__':
    unittest.main()
