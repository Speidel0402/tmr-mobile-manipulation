#!/usr/bin/env python3
"""Offline regression tests for failures observed on the real robot."""

import unittest
from pathlib import Path

import numpy as np

from cup_rim_detector import nested_in_larger_rim

from pick_cycle_policy import (
    classify_close_result,
    cup_grasp_alignment_accepted,
    grasp_plane_policy,
    top_pose_policy,
    validate_camera_snapshot,
    validate_rgb_freshness,
    visual_tolerances,
)


class PickCyclePolicyTests(unittest.TestCase):
    def test_visual_servo_promotes_sub_deadband_steps_and_keeps_iterating(self):
        source = (Path(__file__).parent / "run_streamed_live_pick_cycle.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("MIN_EXECUTABLE_FINE_STEP_M = 0.0025", source)
        self.assertIn("for iteration in range(1, 17)", source)
        self.assertIn('"fine_step_promoted"', source)
        self.assertIn("DESCENT_M = 0.340", source)
        self.assertIn("HEAD_MIN_DESCENT_M = 0.220", source)
        self.assertIn("HEAD_MAX_DESCENT_M = 0.245", source)
        self.assertIn("cup track jump", source)
        self.assertIn("predicted_point = current_point + jacobian @ actual", source)
        self.assertIn("MAX_RECOVERABLE_VISUAL_ERROR_PX = 11.0", source)
        self.assertIn("z_tolerance=0.010", source)
        self.assertIn('"--force-restore-top"', source)
        self.assertIn("if args.force_restore_top or pose_policy != \"accept\":", source)
        self.assertIn("save_alignment_diagnostic(alignment[\"observation\"])", source)
        self.assertIn("verify_object_removed_after_lift(", source)

    def test_franka_gate_debounces_only_transient_errors(self):
        source = (Path(__file__).parent / "servo_cup_edge_xy.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("deadline = time.monotonic() + 0.35", source)
        self.assertIn("persistent Franka error:", source)
        self.assertIn("if measured_error <= endpoint_tolerance:", source)
        self.assertIn("self.hold_target = list(map(float, self.q))", source)
        self.assertIn("self.publish_hold_target(samples=12)", source)

    def test_place_cycle_preserves_confirmed_order_and_current_pose(self):
        source = (Path(__file__).parent / "run_streamed_live_place_cycle.py").read_text(
            encoding="utf-8"
        )
        ordered = [
            '"TOP_CAPTURED_GRIPPER_UNTOUCHED"',
            '"DESCENDING"',
            '"AT_LOW_GRIPPER_STILL_CLOSED"',
            '"OPENING_AT_LOW"',
            '"RELEASE_VERIFIED"',
            '"LIFTING"',
        ]
        positions = [source.index(item) for item in ordered]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("default=0.255", source)
        self.assertIn('"old_joint_restore_used": False', source)
        self.assertNotIn("REFERENCE_JOINTS", source)
        self.assertNotIn("snapshot", source.lower())

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

    def test_near_closed_success_is_not_contact_evidence(self):
        result = {
            "status": 4,
            "position": 0.7929,
            "stalled": False,
            "reached_goal": True,
            "feedback_positions": [],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "fully_closed_unconfirmed")
        self.assertFalse(verdict["accepted_as_grasp"])

    def test_bowl_inner_circle_is_not_accepted_as_cup(self):
        self.assertTrue(
            nested_in_larger_rim(200.0, 120.0, 34.0, np.asarray([[202.0, 121.0, 45.0]]))
        )
        self.assertFalse(
            nested_in_larger_rim(80.0, 120.0, 34.0, np.asarray([[202.0, 121.0, 56.0]]))
        )

    def test_observed_empty_near_closed_stall_is_rejected(self):
        result = {
            "status": 6,
            "position": 0.7894070484581498,
            "stalled": True,
            "reached_goal": False,
            "feedback_positions": [],
        }
        verdict = classify_close_result(result)
        self.assertEqual(verdict["classification"], "indeterminate_close")
        self.assertFalse(verdict["accepted_as_grasp"])

    def test_empirical_cup_alignment_window_keeps_grasps_and_rejects_miss(self):
        target = (292.5, 167.921509)
        success_a = (290.5455977370352, 166.82725962399616)
        success_b = (294.577448024595, 172.85837834441722)
        missed = (298.67694726153286, 167.60309329252348)

        def error(point):
            return (target[0] - point[0], target[1] - point[1])

        self.assertTrue(cup_grasp_alignment_accepted(error(success_a), 6.0, True))
        self.assertTrue(cup_grasp_alignment_accepted(error(success_b), 6.0, True))
        self.assertFalse(cup_grasp_alignment_accepted(error(missed), 6.0, True))

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

    def test_proven_high_side_grasp_residual_does_not_block_close(self):
        self.assertEqual(grasp_plane_policy(0.0144), "accept_high")
        self.assertEqual(grasp_plane_policy(0.009), "accept")

    def test_same_residual_below_table_side_is_rejected(self):
        self.assertEqual(grasp_plane_policy(-0.0144), "reject")

    def test_left_wrist_camera_contract_is_accepted(self):
        session = validate_camera_snapshot(
            role="left_wrist",
            topic="/wrist_camera_left/color/image_raw",
            frame_id="wrist_camera_left_color_optical_frame",
            session_id="session-a",
            image_shape=(480, 640, 3),
        )
        self.assertEqual(session, "session-a")

    def test_wrong_or_restarted_camera_is_rejected(self):
        common = {
            "topic": "/wrist_camera_left/color/image_raw",
            "frame_id": "wrist_camera_left_color_optical_frame",
            "session_id": "session-b",
            "image_shape": (480, 640, 3),
        }
        with self.assertRaisesRegex(ValueError, "role"):
            validate_camera_snapshot(role="zed_top", **common)
        with self.assertRaisesRegex(ValueError, "restarted"):
            validate_camera_snapshot(
                role="left_wrist",
                expected_session_id="session-a",
                **common,
            )

    def test_fresh_rgb_metadata_is_required(self):
        self.assertEqual(validate_rgb_freshness(0.05, 31), 31)
        with self.assertRaisesRegex(ValueError, "stale RGB"):
            validate_rgb_freshness(0.751, 31)
        with self.assertRaisesRegex(ValueError, "sequence"):
            validate_rgb_freshness(0.05, 0)


if __name__ == "__main__":
    unittest.main()
