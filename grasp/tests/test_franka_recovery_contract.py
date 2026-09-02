#!/usr/bin/env python3
"""Offline contracts for the installed Franka FCI recovery sequence."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FrankaRecoveryContracts(unittest.TestCase):
    def test_visual_servo_recovers_error_before_hardware(self) -> None:
        source = (ROOT / "scripts" / "servo_cup_edge_xy.py").read_text(encoding="utf-8")
        self.assertIn("ErrorRecovery", source)
        ensure = source.split("    def ensure_runtime_ready(self):", 1)[1].split(
            "    def publish_hold_target", 1
        )[0]
        self.assertLess(
            ensure.index("self._recover_robot_error()"),
            ensure.index("self._set_hardware_state(State.PRIMARY_STATE_ACTIVE"),
        )
        self.assertLess(
            ensure.index("self._set_hardware_state(State.PRIMARY_STATE_ACTIVE"),
            ensure.index("self.call(self.switch_client"),
        )

    def test_dual_arm_initializer_uses_same_recovery_sequence(self) -> None:
        source = (ROOT / "scripts" / "initialize_dual_arm_pick_pose.py").read_text(
            encoding="utf-8"
        )
        ensure = source.split("    def ensure_runtime_ready(self) -> None:", 1)[1].split(
            "    def publish_hold_target", 1
        )[0]
        self.assertLess(
            ensure.index("self._recover_robot_error()"),
            ensure.index("self._set_hardware_active()"),
        )
        self.assertLess(
            ensure.index("self._set_hardware_active()"),
            ensure.index("self.call(self.switch_client"),
        )


if __name__ == "__main__":
    unittest.main()
