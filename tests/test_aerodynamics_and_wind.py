#!/usr/bin/env python3
"""Unit tests for Member 2 Aerodynamics, Ground Effect & System Identification."""

import unittest
import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_gazebo_sim')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../drone_ws/src/uav_flight_stack')))

from uav_gazebo_sim.aerodynamic_disturbance_generator import AerodynamicPhysicsEngine
from uav_flight_stack.sysid.frequency_response_analyzer import SystemIdentificationEngine


class TestAerodynamicsAndSysID(unittest.TestCase):

    def setUp(self):
        self.aero = AerodynamicPhysicsEngine()
        self.sysid = SystemIdentificationEngine()

    def test_ground_effect_amplification(self):
        # Near ground (0.1m) should produce thrust amplification factor > 1.05
        factor_near_ground = self.aero.compute_ground_effect_factor(0.10)
        self.assertGreater(factor_near_ground, 1.05)

        # High altitude (3.0m) should be approximately 1.0 (Out of Ground Effect)
        factor_free_air = self.aero.compute_ground_effect_factor(3.0)
        self.assertAlmostEqual(factor_free_air, 1.0, delta=0.02)

    def test_motor_mixer_quad_x(self):
        hover_thrust = 1.45 * 9.81  # ~14.22 N total (~3.55N per motor)
        thrusts, rpms = self.aero.compute_motor_dynamics(hover_thrust, 0.0, 0.0, 0.0)
        
        self.assertEqual(len(thrusts), 4)
        self.assertEqual(len(rpms), 4)
        # Each motor in hover should run around 6000-7500 RPM
        for rpm in rpms:
            self.assertGreater(rpm, 5000.0)
            self.assertLess(rpm, 8500.0)

    def test_dryden_gust_generation(self):
        self.aero.gust_intensity = 2.0
        gust = self.aero.compute_dryden_wind_gust(0.02)
        self.assertEqual(len(gust), 3)
        self.assertTrue(all(isinstance(v, float) for v in gust))

    def test_sysid_stability_margins(self):
        freqs = [0.1, 1.0, 3.8, 10.0]
        res = self.sysid.evaluate_rate_loop_bode(freqs)
        
        self.assertTrue(res["is_stable"])
        self.assertGreater(res["gain_margin_db"], 6.0)     # Industrial stability threshold > 6dB
        self.assertGreater(res["phase_margin_deg"], 45.0)  # Industrial stability threshold > 45 deg


if __name__ == '__main__':
    unittest.main()
