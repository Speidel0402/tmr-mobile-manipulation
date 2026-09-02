#!/usr/bin/env python3
"""Offline contracts for the RGB-only plate specialization."""

import importlib.util
from pathlib import Path
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "grasp" / "scripts" / "detect_plate.py"
SPEC = importlib.util.spec_from_file_location("detect_plate", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
detector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(detector)


def pale_green():
    hsv = np.asarray([[[72, 45, 125]]], dtype=np.uint8)
    return tuple(int(value) for value in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def vessel_scene(include_plate=True):
    image = np.full((480, 640, 3), 210, dtype=np.uint8)
    color = pale_green()
    if include_plate:
        cv2.ellipse(image, (145, 72), (82, 95), 38, 0, 360, color, -1)
    cv2.circle(image, (310, 165), 58, color, -1)
    cv2.circle(image, (310, 165), 42, (30, 38, 32), -1)
    cv2.circle(image, (210, 270), 38, color, -1)
    return image


class PlateDetectorContracts(unittest.TestCase):
    def test_selects_large_smooth_pale_green_plate(self):
        best, candidates = detector.detect_plate(vessel_scene())
        self.assertEqual(len(candidates), 1)
        self.assertEqual(best["physical_diameter_mm"], 185.0)
        self.assertGreater(best["equivalent_diameter_px"], 145.0)
        self.assertLess(best["dark_fraction"], 0.10)
        self.assertGreater(best["right_rim_x"], best["center_x"])

    def test_cup_and_food_bowl_cannot_pass_plate_size_and_texture(self):
        with self.assertRaisesRegex(RuntimeError, "plate-sized"):
            detector.detect_plate(vessel_scene(include_plate=False))


if __name__ == "__main__":
    unittest.main()
