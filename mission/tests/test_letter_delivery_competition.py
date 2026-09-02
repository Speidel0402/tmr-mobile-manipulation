#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "mission" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location(
    "letter_delivery_competition", SCRIPT_DIR / "run_letter_delivery_competition.py"
)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


def full_config() -> mission.FullConfig:
    return mission.FullConfig(
        base_host="tmr-user@172.16.0.50",
        base_root="/home/tmr-user/tmr_cycle",
        arm_root="/home/aup/tmr-mobile-manipulation",
        arm_env="/home/aup/tmr_env.sh",
        init_timeout_s=180.0,
        outbound_timeout_s=180.0,
        pick_timeout_s=240.0,
        return_timeout_s=160.0,
        place_timeout_s=200.0,
        transition_settle_s=0.5,
    )


class LetterDeliveryContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.plan = mission.load_plan(mission.DEFAULT_DELIVERY_CONFIG, None, None)

    def test_parameter_defaults_and_bounds(self) -> None:
        self.assertEqual(self.plan.target_letter, "B")
        self.assertEqual(self.plan.requested_row, "auto")
        self.assertEqual(self.plan.maximum_right_m, 2.4)
        self.assertEqual((self.plan.door_before_m, self.plan.door_forward_m), (0.5, 1.2))

    def test_near_and_far_placement_are_distinct(self) -> None:
        self.assertEqual(mission.placement_values(self.plan, "near"), (0.0, 0.36))
        self.assertEqual(mission.placement_values(self.plan, "far"), (0.16, 0.36))
        far = " ".join(mission.letter_place_argv(full_config(), self.plan, "far", "run"))
        self.assertIn("--placement-row far", far)
        self.assertIn("--forward-m 0.160", far)

    def test_search_command_is_targeted_and_bounded(self) -> None:
        command = " ".join(mission.search_argv(full_config(), self.plan, "run"))
        self.assertIn("--target-letter B", command)
        self.assertIn("--max-right-m 2.400", command)
        self.assertIn("--row auto", command)

    def test_return_uses_measured_search_distance(self) -> None:
        command = " ".join(mission.return_argv(full_config(), self.plan, 1.2345, "run"))
        self.assertIn("--left-m 1.2345", command)
        self.assertIn("--turn-ccw-deg 180.000", command)

    def test_structured_reports_are_required(self) -> None:
        search_report = {
            "status": "success",
            "target_centered": True,
            "target_letter": "B",
            "row": "near",
            "actual_right_m": 1.2,
            "zero_command_latched": True,
        }
        self.assertTrue(mission.search_report_ok(search_report, self.plan))
        search_report["target_centered"] = False
        self.assertFalse(mission.search_report_ok(search_report, self.plan))

    def test_pickup_resume_requires_exact_failed_pick_checkpoint(self) -> None:
        self.assertTrue(
            mission.checkpoint_allows_pickup_resume(
                {"phase": "FAILED", "failed_phase": "PICK_RUNNING"}
            )
        )
        self.assertFalse(
            mission.checkpoint_allows_pickup_resume(
                {"phase": "FAILED", "failed_phase": "OUTBOUND_BASE_RUNNING"}
            )
        )
        self.assertFalse(mission.checkpoint_allows_pickup_resume(None))


if __name__ == "__main__":
    unittest.main()
