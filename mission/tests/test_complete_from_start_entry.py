#!/usr/bin/env python3
"""Contracts for the unambiguous full-mission competition entry."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "mission" / "scripts" / "run_complete_from_start.sh").read_text(
    encoding="utf-8"
)


class CompleteFromStartEntryTests(unittest.TestCase):
    def test_always_selects_execute_and_fresh_start(self) -> None:
        self.assertIn("--execute --fresh-start-confirmed", SOURCE)
        self.assertIn("run_three_object_delivery.py", SOURCE)

    def test_rejects_mid_mission_resume_modes(self) -> None:
        self.assertIn("--resume-at-pickup-confirmed", SOURCE)
        self.assertIn("--resume-after-cup-held-confirmed", SOURCE)
        self.assertIn("resume options are not accepted", SOURCE)


if __name__ == "__main__":
    unittest.main()
