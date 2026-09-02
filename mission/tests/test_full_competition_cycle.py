#!/usr/bin/env python3
"""Offline compatibility contracts for the complete competition strategy."""

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "mission" / "scripts" / "run_full_competition_cycle.py"
SPEC = importlib.util.spec_from_file_location("full_competition_cycle", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


def config() -> mission.FullConfig:
    return mission.FullConfig(
        base_host="tmr-user@172.16.0.50",
        base_root="/home/tmr-user/tmr_cycle",
        arm_root="/home/aup/tmr-mobile-manipulation",
        arm_env="/home/aup/tmr_env.sh",
        init_timeout_s=180.0,
        outbound_timeout_s=180.0,
        pick_timeout_s=240.0,
        return_timeout_s=180.0,
        place_timeout_s=180.0,
        transition_settle_s=0.5,
    )


class FullCompetitionContracts(unittest.TestCase):
    def test_strategy_order_and_updated_values(self) -> None:
        summary = mission.strategy(config(), Path("state.json"))
        joined = " -> ".join(summary["phases"])
        self.assertLess(joined.index("reset both arm joints"), joined.index("spine to 0.70 m"))
        self.assertLess(joined.index("spine to 0.70 m"), joined.index("outbound"))
        self.assertLess(joined.index("force left top restore"), joined.index("return:"))
        self.assertLess(joined.index("right 0.80 m"), joined.index("right 0.85 m"))
        self.assertEqual(summary["right_total_m"], 1.65)
        self.assertEqual(summary["place_descent_m"], 0.255)

    def test_pick_forces_second_left_restore(self) -> None:
        joined = " ".join(mission.pick_argv(config()))
        self.assertIn("run_streamed_live_pick_cycle.py", joined)
        self.assertIn("--force-restore-top", joined)

    def test_spine_is_explicitly_restored_and_verified(self) -> None:
        joined = " ".join(mission.spine_initialization_argv(config()))
        self.assertIn("initialize_spine_height.py", joined)
        self.assertIn("--target-m 0.700", joined)
        report = {
            "status": "success",
            "moved": True,
            "target_position_m": 0.7,
            "measured_position_m": 0.7,
        }
        self.assertTrue(mission.spine_report_is_stable(report))
        report["measured_position_m"] = 0.69
        self.assertFalse(mission.spine_report_is_stable(report))

    def test_initialization_requires_new_right_parking_hold(self) -> None:
        right_target = list(mission.RIGHT_PARKING_TARGET)
        left_target = list(mission.LEFT_PICK_TOP_TARGET)
        report = {
            "status": "success",
            "order": ["right", "left"],
            "both_stable_hold": True,
            "gripper_commanded": False,
            "reports": [
                {
                    "arm": "right",
                    "moved": True,
                    "stable_hold": True,
                    "target_joint_positions_rad": right_target,
                    "measured_joint_positions_rad": right_target,
                },
                {
                    "arm": "left",
                    "moved": True,
                    "stable_hold": True,
                    "target_joint_positions_rad": left_target,
                    "measured_joint_positions_rad": left_target,
                },
            ],
        }
        self.assertTrue(mission.init_report_is_stable(report))
        report["order"] = ["left", "right"]
        self.assertFalse(mission.init_report_is_stable(report))

    def test_initialization_rejects_left_arm_outside_fixed_pick_top(self) -> None:
        right_target = list(mission.RIGHT_PARKING_TARGET)
        left_target = list(mission.LEFT_PICK_TOP_TARGET)
        report = {
            "status": "success",
            "order": ["right", "left"],
            "both_stable_hold": True,
            "gripper_commanded": False,
            "reports": [
                {
                    "arm": "right",
                    "moved": True,
                    "stable_hold": True,
                    "target_joint_positions_rad": right_target,
                    "measured_joint_positions_rad": right_target,
                },
                {
                    "arm": "left",
                    "moved": True,
                    "stable_hold": True,
                    "target_joint_positions_rad": left_target,
                    "measured_joint_positions_rad": left_target,
                },
            ],
        }
        report["reports"][1]["measured_joint_positions_rad"][2] += 0.025
        self.assertFalse(mission.init_report_is_stable(report))

    def test_pick_and_place_scripts_never_address_right_arm(self) -> None:
        for relative in (
            "grasp/scripts/run_streamed_live_pick_cycle.py",
            "grasp/scripts/run_streamed_live_place_cycle.py",
        ):
            source = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn('"/right/', source)
            self.assertNotIn("'/right/", source)

    def test_return_uses_single_process_updated_route(self) -> None:
        joined = " ".join(mission.return_base_argv(config()))
        self.assertIn("13_run_post_grasp_route.sh", joined)
        self.assertIn("--right-first-m 0.80", joined)
        self.assertIn("--right-second-m 0.85", joined)

    def test_place_uses_updated_relative_descent(self) -> None:
        joined = " ".join(mission.place_argv(config()))
        self.assertIn("run_streamed_live_place_cycle.py", joined)
        self.assertIn("--down-m 0.255", joined)

    def test_structured_phase_reports_are_required(self) -> None:
        self.assertTrue(
            mission.return_report_is_stable(
                {"status": "complete", "next_stage": 5, "zero_command_latched": True, "reports": [{}] * 5}
            )
        )
        self.assertFalse(
            mission.return_report_is_stable(
                {"status": "complete", "next_stage": 4, "zero_command_latched": True, "reports": [{}] * 4}
            )
        )
        self.assertTrue(
            mission.place_report_is_stable(
                {"status": "complete", "phase": "DONE", "released": True, "stable_hold_recovery_attempt": 1}
            )
        )


if __name__ == "__main__":
    unittest.main()
