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
        self.assertLess(joined.index("dual arms"), joined.index("outbound"))
        self.assertLess(joined.index("force left top restore"), joined.index("return:"))
        self.assertLess(joined.index("right 0.80 m"), joined.index("right 0.85 m"))
        self.assertEqual(summary["right_total_m"], 1.65)
        self.assertEqual(summary["place_descent_m"], 0.255)

    def test_pick_forces_second_left_restore(self) -> None:
        joined = " ".join(mission.pick_argv(config()))
        self.assertIn("run_streamed_live_pick_cycle.py", joined)
        self.assertIn("--force-restore-top", joined)

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
