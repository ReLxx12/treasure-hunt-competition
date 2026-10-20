#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np


NODES_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "nodes"))
if NODES_DIR not in sys.path:
    sys.path.insert(0, NODES_DIR)

from stack_vision import StackVision


class StackVisionTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "min_depth_mm": 150.0,
            "max_depth_mm": 2000.0,
            "min_valid_pixels": 100,
            "foreground_percentile": 5.0,
            "plane_band_mm": 15.0,
            "morphology_kernel": 3,
            "min_component_area": 100,
            "min_stable_frames": 5,
            "center_tolerance_px": 5.0,
            "depth_tolerance_mm": 10.0,
            "max_correction_mm": 40.0,
        }
        self.roi = {
            "x_min": 20,
            "x_max": 100,
            "y_min": 10,
            "y_max": 90,
        }

    def make_depth(self, center=(60, 50), top_depth=600.0):
        image = np.full((100, 120), 900.0, dtype=np.float32)
        u, v = center
        image[v - 12:v + 12, u - 12:u + 12] = top_depth
        return image

    def test_extracts_largest_near_top_plane(self):
        result = StackVision.extract_top_observation(
            self.make_depth(), self.roi, self.config)
        self.assertTrue(result["ok"])
        self.assertAlmostEqual(result["u"], 59.5, delta=1.0)
        self.assertAlmostEqual(result["v"], 49.5, delta=1.0)
        self.assertAlmostEqual(result["depth_mm"], 600.0, delta=1.0)

    def test_rejects_invalid_depth(self):
        image = np.zeros((100, 120), dtype=np.float32)
        result = StackVision.extract_top_observation(
            image, self.roi, self.config)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "insufficient_valid_depth")

    def test_accepts_five_stable_frames_with_outliers(self):
        observations = []
        for u, v, depth in [
                (60, 50, 600),
                (61, 49, 603),
                (59, 51, 598),
                (60, 51, 601),
                (61, 50, 599),
                (90, 80, 750),
                (20, 20, 450)]:
            observations.append({
                "ok": True,
                "u": float(u),
                "v": float(v),
                "depth_mm": float(depth),
                "area": 500,
            })
        result = StackVision.stable_observation(
            observations, self.config)
        self.assertTrue(result["ok"])
        self.assertEqual(result["stable_frames"], 5)
        self.assertAlmostEqual(result["u"], 60.0)
        self.assertAlmostEqual(result["v"], 50.0)

    def test_rejects_unstable_frames(self):
        observations = [
            {
                "ok": True,
                "u": float(20 + index * 12),
                "v": 50.0,
                "depth_mm": float(500 + index * 30),
                "area": 500,
            }
            for index in range(7)
        ]
        result = StackVision.stable_observation(
            observations, self.config)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "unstable_top_plane")

    def test_applies_local_pixel_to_arm_correction(self):
        observation = {
            "ok": True,
            "u": 110.0,
            "v": 90.0,
            "depth_mm": 600.0,
        }
        calibration = {
            "reference_pixel": [100.0, 100.0],
            "pixel_to_arm_mm": [[0.5, 0.0], [0.0, -0.25]],
        }
        result = StackVision.apply_local_calibration(
            observation, calibration, self.config)
        self.assertTrue(result["ok"])
        self.assertAlmostEqual(result["correction_x"], 5.0)
        self.assertAlmostEqual(result["correction_y"], 2.5)

    def test_rejects_excessive_or_zero_calibration(self):
        observation = {
            "ok": True,
            "u": 200.0,
            "v": 100.0,
            "depth_mm": 600.0,
        }
        excessive = StackVision.apply_local_calibration(
            observation,
            {
                "reference_pixel": [100.0, 100.0],
                "pixel_to_arm_mm": [[1.0, 0.0], [0.0, 1.0]],
            },
            self.config)
        self.assertFalse(excessive["ok"])
        self.assertEqual(excessive["reason"], "correction_out_of_range")

        zero = StackVision.apply_local_calibration(
            observation,
            {
                "reference_pixel": [100.0, 100.0],
                "pixel_to_arm_mm": [[0.0, 0.0], [0.0, 0.0]],
            },
            self.config)
        self.assertFalse(zero["ok"])
        self.assertEqual(zero["reason"], "invalid_local_calibration")


if __name__ == "__main__":
    unittest.main()
