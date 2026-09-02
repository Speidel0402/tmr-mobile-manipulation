#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "grasp" / "scripts" / "run_streamed_live_place_cycle.py").read_text(
    encoding="utf-8"
)


class LetterPlaceSequenceContracts(unittest.TestCase):
    def test_far_order_is_extend_down_open_up_retract(self) -> None:
        ordered = [
            '"bounded_forward_extension"',
            '"down_before_open"',
            "open_report = command_open(gripper)",
            '"up_after_open"',
            'state["retraction_report"] = move_cartesian_to',
        ]
        locations = [SOURCE.index(item) for item in ordered]
        self.assertEqual(locations, sorted(locations))

    def test_near_keeps_verified_zero_extension(self) -> None:
        self.assertIn('args.placement_row == "near" and args.forward_m > 0.005', SOURCE)
        self.assertIn('parser.add_argument("--forward-m", type=float, default=0.0)', SOURCE)

    def test_far_extension_is_bounded_and_reversible(self) -> None:
        self.assertIn("0.0 <= args.forward_m <= 0.20", SOURCE)
        self.assertIn('"bounded_fault_recovery_retract"', SOURCE)


if __name__ == "__main__":
    unittest.main()
