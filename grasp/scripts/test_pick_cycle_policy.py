#!/usr/bin/env python3
"""Offline regression tests for failures observed on the real robot."""

import unittest

from pick_cycle_policy import classify_close_result, top_pose_policy, visual_tolerances


class PickCyclePolicyTests(unittest.TestCase):
    def test_historical_contact_is_accepted(self):
        result = {
            "status": 6,
            "position": 0.768449,
            "stalled": True,
            "reached_goal": False,
            "feedback_positions": [0.1, 0.4, 0.72, 0.768449],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "object_contact_candidate")
        self.assertTrue(verdict["accepted_as_grasp"])

    def test_real_cup_contact_below_old_absolute_cutoff_is_accepted(self):
        result = {
            "status": 6,
            "position": 0.17814052863436122,
            "stalled": True,
            "reached_goal": False,
            "feedback_positions": [],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "object_contact_candidate")
        self.assertTrue(verdict["accepted_as_grasp"])

    def test_zero_position_stall_is_rejected(self):
        result = {
            "status": 6,
            "position": 0.0,
            "stalled": True,
            "reached_goal": False,
            "feedback_positions": [0.0, 0.0],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "no_motion")
        self.assertFalse(verdict["accepted_as_grasp"])

    def test_fully_closed_is_not_proof_of_grasp(self):
        result = {
            "status": 4,
            "position": 0.8,
            "stalled": False,
            "reached_goal": True,
            "feedback_positions": [0.2, 0.5, 0.8],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "fully_closed_unconfirmed")
        self.assertFalse(verdict["accepted_as_grasp"])

    def test_fresh_state_mismatch_is_rejected(self):
        result = {
            "status": 6,
            "position": 0.70,
            "stalled": True,
            "reached_goal": False,
            "feedback_positions": [0.3, 0.7],
        }
        verdict = classify_close_result(result, post_position=0.0)
        self.assertEqual(verdict["classification"], "state_mismatch")
        self.assertFalse(verdict["accepted_as_grasp"])

    def test_visual_threshold_has_hysteresis(self):
        tolerance = visual_tolerances(2.4)
        self.assertGreaterEqual(tolerance["enter_px"], 5.0)
        self.assertGreater(tolerance["hold_px"], tolerance["enter_px"])
        self.assertLessEqual(tolerance["hold_px"], 8.0)

    def test_small_top_rebound_auto_restores(self):
        self.assertEqual(top_pose_policy(0.0047, 0.91), "auto_restore")
        self.assertEqual(top_pose_policy(0.0004, 0.2), "accept")
        self.assertEqual(top_pose_policy(0.08, 20.0), "hard_fault")


if __name__ == "__main__":
    unittest.main()
