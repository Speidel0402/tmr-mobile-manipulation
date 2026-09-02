#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "base" / "scripts"
SEARCH_SOURCE = (SCRIPTS / "14_letter_guided_search.py").read_text(encoding="utf-8")
RUNNER_SOURCE = (SCRIPTS / "14_run_letter_guided_search.sh").read_text(encoding="utf-8")
EXPORT_SOURCE = (SCRIPTS / "zed_frame_export.py").read_text(encoding="utf-8")
sys.path.insert(0, str(SCRIPTS))

import letter_card_vision as vision  # noqa: E402

SEARCH_SPEC = importlib.util.spec_from_file_location(
    "letter_search", SCRIPTS / "14_letter_guided_search.py"
)
assert SEARCH_SPEC is not None and SEARCH_SPEC.loader is not None
search = importlib.util.module_from_spec(SEARCH_SPEC)
sys.modules[SEARCH_SPEC.name] = search
SEARCH_SPEC.loader.exec_module(search)


def detection(letter: str, x: float, row: str, confidence: float = 0.8):
    return vision.LetterDetection(
        letter=letter,
        confidence=confidence,
        center_x_norm=x,
        center_y_norm=0.72 if row == "near" else 0.30,
        area_norm=0.01,
        row=row,
        quad=((0.0, 0.0),) * 4,
    )


class LetterVisionContracts(unittest.TestCase):
    def test_camera_transport_is_isolated_from_control_dds(self) -> None:
        self.assertIn('vision_domain="${TMR_CYCLE_VISION_DOMAIN_ID:-1}"', RUNNER_SOURCE)
        self.assertIn('--camera-file "${frame_file}"', RUNNER_SOURCE)
        self.assertIn("os.replace(self.temporary, self.output)", EXPORT_SOURCE)
        self.assertIn("node.refresh_frame_file()", SEARCH_SOURCE)

    def test_direct_controller_is_explicit_recovery_mode(self) -> None:
        self.assertIn('parser.add_argument(\n        "--direct-controller"', SEARCH_SOURCE)
        self.assertIn("CONTROLLER_COMMAND_TOPIC", SEARCH_SOURCE)

    def test_white_cards_and_synthetic_letters_are_recognized(self) -> None:
        image = np.full((720, 1280, 3), (55, 70, 85), np.uint8)
        for letter, x, y in (("A", 150, 470), ("B", 560, 460), ("E", 930, 170)):
            cv2.rectangle(image, (x, y), (x + 90, y + 130), (245, 245, 245), -1)
            cv2.putText(image, letter, (x + 10, y + 100), cv2.FONT_HERSHEY_SIMPLEX,
                        2.3, (70, 70, 70), 4, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABE", minimum_confidence=0.20).detect(image)
        self.assertEqual({item.letter for item in found}, {"A", "B", "E"})
        self.assertEqual({item.row for item in found if item.letter in {"A", "B"}}, {"near"})
        self.assertEqual({item.row for item in found if item.letter == "E"}, {"far"})

    def test_near_card_connected_to_large_white_region_is_recovered(self) -> None:
        image = np.full((720, 1280, 3), (65, 75, 85), np.uint8)
        cv2.rectangle(image, (0, 0), (390, 540), (245, 245, 245), -1)
        cv2.rectangle(image, (420, 560), (480, 640), (245, 245, 245), -1)
        cv2.rectangle(image, (445, 535), (455, 565), (245, 245, 245), -1)
        cv2.putText(image, "A", (427, 628), cv2.FONT_HERSHEY_SIMPLEX,
                    1.8, (35, 35, 35), 4, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABD", minimum_confidence=0.22).detect(image)
        near_a = [item for item in found if item.letter == "A" and item.row == "near"]
        self.assertTrue(near_a)
        self.assertAlmostEqual(near_a[0].center_x_norm, 0.35, delta=0.04)

    def test_letter_d_is_supported_when_configured(self) -> None:
        image = np.full((720, 1280, 3), (55, 70, 85), np.uint8)
        for letter, x, y in (("A", 150, 470), ("B", 560, 460), ("D", 930, 170)):
            cv2.rectangle(image, (x, y), (x + 90, y + 130), (245, 245, 245), -1)
            cv2.putText(image, letter, (x + 10, y + 100), cv2.FONT_HERSHEY_SIMPLEX,
                        2.3, (70, 70, 70), 4, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABD", minimum_confidence=0.20).detect(image)
        self.assertEqual({item.letter for item in found}, {"A", "B", "D"})

    def test_two_holes_alone_do_not_authorize_a_false_b(self) -> None:
        card = np.full((192, 192, 3), 245, np.uint8)
        cv2.circle(card, (96, 62), 30, (20, 20, 20), 9)
        cv2.circle(card, (96, 132), 30, (20, 20, 20), 9)
        cv2.line(card, (68, 84), (68, 110), (20, 20, 20), 9)
        letter, confidence = vision.LetterCardRecognizer(
            "ABD", minimum_confidence=0.0
        ).classify(card)
        self.assertEqual(letter, "B")
        self.assertLess(confidence, 0.42)

    def test_search_has_spatial_gate_and_authorization_evidence(self) -> None:
        self.assertIn("right_m >= args.min_detection_right_m", SEARCH_SOURCE)
        self.assertIn("evidence_saved", SEARCH_SOURCE)

    def test_same_letter_candidates_are_not_averaged_into_fake_center(self) -> None:
        tracker = search.TargetTracker("E", "far", stable_frames=3)
        observation = tracker.observe(
            [detection("E", 0.35, "far", 0.9), detection("E", 0.65, "far", 0.7)], 0.7
        )
        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation.center_x_norm, 0.35)
        self.assertEqual(observation.members, 1)

    def test_blue_grey_patch_is_not_a_white_card_surface(self) -> None:
        false_patch_hsv = np.full((192, 192, 3), (105, 75, 140), np.uint8)
        false_patch = cv2.cvtColor(false_patch_hsv, cv2.COLOR_HSV2BGR)
        cv2.putText(false_patch, "A", (35, 155), cv2.FONT_HERSHEY_SIMPLEX,
                    3.6, (40, 40, 40), 7, cv2.LINE_AA)
        self.assertFalse(vision.card_surface_is_white(false_patch))
        real_card = np.full((192, 192, 3), 210, np.uint8)
        cv2.putText(real_card, "A", (35, 155), cv2.FONT_HERSHEY_SIMPLEX,
                    3.6, (40, 40, 40), 7, cv2.LINE_AA)
        self.assertTrue(vision.card_surface_is_white(real_card))

    def test_grey_floor_reflection_cannot_authorize_a(self) -> None:
        image = np.full((720, 1280, 3), (125, 135, 145), np.uint8)
        quad = np.array([[570, 520], [665, 455], [720, 535], [625, 600]], np.int32)
        cv2.fillConvexPoly(image, quad, (155, 160, 165))
        cv2.putText(image, "A", (615, 555), cv2.FONT_HERSHEY_SIMPLEX,
                    1.5, (75, 75, 75), 5, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABCDE", minimum_confidence=0.20).detect(image)
        self.assertFalse(any(item.letter == "A" for item in found))

    def test_round_white_plate_cannot_become_a_letter_card(self) -> None:
        image = np.full((720, 1280, 3), (60, 85, 110), np.uint8)
        cv2.circle(image, (640, 450), 90, (230, 230, 230), -1)
        cv2.putText(image, "D", (600, 480), cv2.FONT_HERSHEY_SIMPLEX,
                    2.0, (65, 65, 65), 5, cv2.LINE_AA)
        found = vision.LetterCardRecognizer("ABCDE", minimum_confidence=0.20).detect(image)
        self.assertFalse(any(item.letter == "D" for item in found))

    def test_e_card_is_not_forced_to_a(self) -> None:
        card = np.full((192, 192, 3), 235, np.uint8)
        cv2.putText(card, "E", (35, 155), cv2.FONT_HERSHEY_SIMPLEX,
                    3.6, (35, 35, 35), 7, cv2.LINE_AA)
        letter, confidence = vision.LetterCardRecognizer(
            "ABCDE", minimum_confidence=0.0
        ).classify(card)
        self.assertEqual(letter, "E")
        self.assertGreater(confidence, 0.20)

    def test_weak_false_letter_does_not_lock_row_or_control(self) -> None:
        tracker = search.TargetTracker("A", "auto", minimum_tracking_confidence=0.42)
        for right_m in (0.0, 0.01, 0.02):
            self.assertIsNone(
                tracker.observe([detection("A", 0.18, "far", confidence=0.32)], right_m)
            )
        self.assertIsNone(tracker.locked_row)
        tracker.observe([detection("A", 0.36, "near", confidence=0.62)], 0.03)
        tracker.observe([detection("A", 0.37, "near", confidence=0.64)], 0.04)
        control = tracker.control_observation()
        self.assertIsNotNone(control)
        self.assertEqual(control.row, "near")

    def test_runtime_passes_tracking_threshold_into_tracker(self) -> None:
        runtime = Path(search.__file__).read_text(encoding="utf-8")
        self.assertIn(
            "minimum_tracking_confidence=args.minimum_tracking_confidence",
            runtime,
        )

    def test_center_requires_repeated_low_spread_observations(self) -> None:
        tracker = search.TargetTracker("B", "auto", stable_frames=3)
        for right_m, x in ((0.6, 0.53), (0.62, 0.51), (0.64, 0.52)):
            tracker.observe([detection("B", x, "near")], right_m)
        centered, error, spread = tracker.centered()
        self.assertTrue(centered)
        self.assertLess(abs(error), 0.055)
        self.assertLess(spread, 0.032)

    def test_adaptive_policy_learns_camera_motion_sign(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        first = search.TargetObservation(0.50, 0.70, "near", 0.8, 1)
        second = search.TargetObservation(0.55, 0.64, "near", 0.8, 1)
        self.assertGreater(policy.command(first), 0.0)
        command = policy.command(second)
        self.assertIsNotNone(policy.image_gain_per_m)
        self.assertGreater(command, 0.0)

    def test_initial_probe_moves_left_for_target_left_of_center(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        observation = search.TargetObservation(0.0, 0.18, "far", 0.8, 1)
        self.assertLess(policy.command(observation), 0.0)

    def test_missing_frame_resumes_last_correction_direction(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        policy.command(search.TargetObservation(0.0, 0.18, "far", 0.8, 1))
        self.assertLess(policy.command(None), 0.0)

    def test_gain_anchor_survives_high_frame_rate_small_steps(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        for right_m, x in ((0.50, 0.70), (0.506, 0.696), (0.512, 0.692), (0.519, 0.687)):
            policy.command(search.TargetObservation(right_m, x, "near", 0.86, 1))
        self.assertIsNotNone(policy.image_gain_per_m)
        self.assertLess(policy.image_gain_per_m, 0.0)

    def test_verified_card_center_can_be_projected_through_short_occlusion(self) -> None:
        observation = search.TargetObservation(2.05, 0.70, "near", 0.62, 1)
        projected = search.projected_center_right_m(observation, -0.80)
        self.assertAlmostEqual(projected, 2.30)
        self.assertIsNone(search.projected_center_right_m(observation, -0.30, 0.35))

    def test_low_confidence_detection_cannot_reverse_gain(self) -> None:
        policy = search.AdaptiveCenterPolicy(0.055, 0.04)
        policy.command(search.TargetObservation(1.00, 0.70, "near", 0.86, 1))
        policy.command(search.TargetObservation(1.03, 0.68, "near", 0.86, 1))
        learned = policy.image_gain_per_m
        self.assertEqual(
            policy.command(search.TargetObservation(1.30, 0.90, "near", 0.29, 1)),
            0.0,
        )
        self.assertEqual(policy.image_gain_per_m, learned)

    def test_center_crossing_requires_two_fresh_observations(self) -> None:
        hold = search.CenterHold(0.055, single_frame_hold_s=0.30)
        holding, centered, _, _ = hold.update(
            search.TargetObservation(1.76, 0.483, "near", 0.36, 1), 10.0
        )
        self.assertTrue(holding)
        self.assertFalse(centered)
        holding, centered, error, _ = hold.status(10.31)
        self.assertTrue(holding)
        self.assertFalse(centered)
        holding, centered, error, _ = hold.update(
            search.TargetObservation(1.77, 0.486, "near", 0.40, 1), 10.32
        )
        self.assertTrue(holding)
        self.assertTrue(centered)
        self.assertAlmostEqual(error, -0.0155)

    def test_center_hold_rejects_a_low_confidence_single_frame(self) -> None:
        hold = search.CenterHold(0.055, single_frame_hold_s=0.30)
        hold.update(search.TargetObservation(1.2, 0.51, "near", 0.20, 1), 4.0)
        self.assertFalse(hold.status(4.31)[1])

    def test_acquisition_band_does_not_stop_outside_center_tolerance(self) -> None:
        hold = search.CenterHold(0.055, acquire_tolerance_norm=0.080)
        holding, centered, _, _ = hold.update(
            search.TargetObservation(1.2, 0.57, "near", 0.86, 1), 4.0
        )
        self.assertFalse(holding)
        self.assertFalse(centered)


if __name__ == "__main__":
    unittest.main()
