#!/usr/bin/env python3
"""Contracts for fast, idempotent base-runtime reuse."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "base" / "scripts" / "19_ensure_navigation_stack.sh").read_text(
    encoding="utf-8"
)


class EnsureNavigationStackTests(unittest.TestCase):
    def test_ready_stack_reuse_requires_one_fresh_odometry_sample(self) -> None:
        body = SOURCE.split("runtime_healthy() {", 1)[1].split("\n}", 1)[0]
        self.assertIn('kill -0 "${pid}"', body)
        self.assertIn('adapters', body)
        self.assertIn("timeout 3 ros2 topic echo --once --no-daemon", body)
        self.assertIn("/swerve_drive_controller/odom", body)

    def test_unhealthy_runtime_still_uses_managed_restart(self) -> None:
        self.assertIn('screen -dmS "${screen_name}"', SOURCE)
        self.assertIn("03_start_navigation.sh", SOURCE)


if __name__ == "__main__":
    unittest.main()
