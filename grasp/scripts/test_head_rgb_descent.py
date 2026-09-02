#!/usr/bin/env python3

import unittest

from head_rgb_descent import estimate_remaining_down_m


class HeadRgbDescentTests(unittest.TestCase):
    def test_probe_predicts_bounded_remaining_descent(self):
        remaining, scale = estimate_remaining_down_m(212.73, 223.04, 0.04, 228.23)
        self.assertAlmostEqual(scale, 257.75, places=1)
        self.assertAlmostEqual(remaining, 0.0201, places=3)

    def test_unreliable_visual_scale_is_rejected(self):
        with self.assertRaises(RuntimeError):
            estimate_remaining_down_m(220.0, 220.1, 0.02, 228.0)


if __name__ == "__main__":
    unittest.main()
