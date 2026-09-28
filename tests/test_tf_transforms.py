#!/usr/bin/env python3
"""Unit tests for Dynamic TF2 Coordinate Frames & Math (Member 3)."""

import unittest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_system_orchestrator')))

from uav_system_orchestrator.tf_tree_manager import quaternion_from_euler


class TestTFTransforms(unittest.TestCase):

    def test_identity_quaternion(self):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(qz, 0.0)
        self.assertAlmostEqual(qw, 1.0)

    def test_yaw_90_degrees(self):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        self.assertAlmostEqual(norm, 1.0, places=5)
        self.assertAlmostEqual(qz, math.sin(math.pi / 4.0), places=4)
        self.assertAlmostEqual(qw, math.cos(math.pi / 4.0), places=4)

    def test_quaternion_normalization_all_angles(self):
        angles = [(0.1, 0.2, 0.3), (-0.5, 0.8, -1.2), (3.14, 0.0, -1.57)]
        for r, p, y in angles:
            qx, qy, qz, qw = quaternion_from_euler(r, p, y)
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            self.assertAlmostEqual(norm, 1.0, places=5)


if __name__ == '__main__':
    unittest.main()
