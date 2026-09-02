#!/usr/bin/env python3
"""Keep every dual-arm initialization entrypoint on one joint target set."""

import ast
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


def recorded_pick_joints() -> list[float]:
    tree = ast.parse((ROOT / "scripts" / "run_streamed_live_pick_cycle.py").read_text(encoding="utf-8"))
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "REFERENCE_JOINTS"
            for target in statement.targets
        ):
            call = statement.value
            assert isinstance(call, ast.Call)
            return [float(item) for item in ast.literal_eval(call.args[0])]
    raise AssertionError("REFERENCE_JOINTS not found")


class InitialPoseConsistencyTests(unittest.TestCase):
    def test_left_config_matches_successful_pick_top(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "grasp_initial_state.yaml").read_text(encoding="utf-8"))
        self.assertEqual(config["left"]["positions"], recorded_pick_joints())

    def test_initializer_reads_shared_config_and_resets_both_grippers(self) -> None:
        source = (ROOT / "scripts" / "initialize_dual_arm_pick_pose.py").read_text(encoding="utf-8")
        self.assertIn('"config" / "grasp_initial_state.yaml"', source)
        self.assertIn('"gripper_commanded": True', source)
        self.assertIn("GripperCommand", source)
        self.assertIn('"both_grippers_reset"', source)
        self.assertIn("JOINT_HOLD_TOLERANCE_RAD = 0.006", source)
        self.assertIn('"reset_required"', source)
        self.assertNotIn('"already_at_target"', source)
        self.assertIn("self.publish_hold_target(samples=12)", source)
        config = yaml.safe_load((ROOT / "config" / "grasp_initial_state.yaml").read_text(encoding="utf-8"))
        self.assertEqual(config["grippers"]["reset_order"], ["right", "left"])
        self.assertEqual(config["grippers"]["right"]["position"], 0.8)
        self.assertEqual(config["grippers"]["left"]["position"], 0.0)

    def test_right_arm_parks_before_left_arm_enters_pick_corridor(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "grasp_initial_state.yaml").read_text(encoding="utf-8"))
        self.assertEqual(config["initialization_order"], ["right", "left"])
        source = (ROOT / "scripts" / "start_tmr_system.ps1").read_text(encoding="utf-8")
        self.assertLess(source.index("move_arm right '$right'"), source.index("move_arm left '$left'"))

    def test_system_startup_contains_the_same_left_target(self) -> None:
        startup = (ROOT / "config" / "system_startup.psd1").read_text(encoding="utf-8")
        for value in recorded_pick_joints():
            self.assertIn(str(value), startup)

    def test_system_startup_contains_the_same_right_target(self) -> None:
        config = yaml.safe_load((ROOT / "config" / "grasp_initial_state.yaml").read_text(encoding="utf-8"))
        startup = (ROOT / "config" / "system_startup.psd1").read_text(encoding="utf-8")
        for value in config["right"]["positions"]:
            self.assertIn(str(value), startup)


if __name__ == "__main__":
    unittest.main()
