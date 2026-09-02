#!/usr/bin/env python3
"""Offline compatibility contracts for the start/cup/bowl mission."""

import importlib.util
import argparse
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "mission" / "scripts" / "run_start_cup_bowl_cycle.py"
SPEC = importlib.util.spec_from_file_location("start_cup_bowl_cycle", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


def config() -> mission.CycleConfig:
    return mission.CycleConfig(
        base_host="tmr-user@172.16.0.50",
        base_root="/home/tmr-user/tmr_cycle",
        arm_root="/home/aup/tmr-mobile-manipulation",
        arm_env="/home/aup/tmr_env.sh",
        init_timeout_s=180.0,
        outbound_timeout_s=180.0,
        pick_timeout_s=300.0,
        place_timeout_s=180.0,
        transition_settle_s=0.5,
    )


class StartCupBowlContracts(unittest.TestCase):
    def test_strategy_has_exact_physical_order_and_depths(self) -> None:
        summary = mission.strategy(config(), Path("state.json"))
        joined = " -> ".join(summary["phases"])
        self.assertLess(joined.index("forward 0.85"), joined.index("cup: open"))
        self.assertLess(joined.index("cup: open"), joined.index("cup: down"))
        self.assertLess(joined.index("cup: down"), joined.index("bowl: open"))
        self.assertLess(joined.index("bowl: open"), joined.rindex("bowl: down"))
        self.assertEqual(summary["cup_descent_m"], 0.340)
        self.assertEqual(summary["bowl_descent_m"], 0.360)

    def test_object_pick_scripts_force_the_same_calibrated_top(self) -> None:
        cup = " ".join(mission.pick_argv(config(), "cup"))
        bowl = " ".join(mission.pick_argv(config(), "bowl"))
        self.assertIn("run_streamed_live_pick_cycle.py", cup)
        self.assertIn("run_streamed_food_bowl_pick_cycle.py", bowl)
        self.assertIn("--force-restore-top", cup)
        self.assertIn("--force-restore-top", bowl)

    def test_same_place_commands_are_down_open_up_with_unique_state(self) -> None:
        cup = " ".join(mission.place_argv(config(), "cup", "run123", 0.340))
        bowl = " ".join(mission.place_argv(config(), "bowl", "run123", 0.360))
        self.assertIn("--placement-row near --forward-m 0 --down-m 0.340", cup)
        self.assertIn("--placement-row near --forward-m 0 --down-m 0.360", bowl)
        self.assertIn("/tmp/tmr_cup_same_place_run123.json", cup)
        self.assertIn("/tmp/tmr_bowl_same_place_run123.json", bowl)

    def test_pick_requires_success_controller_hold_and_done(self) -> None:
        output = "\n".join(
            [
                'PICK={"event":"success"}',
                'PICK={"event":"controller","joint_impedance":"restored_to_hold"}',
                'PICK={"event":"cycle_complete","final_state":"DONE","controller_hold":"stable"}',
            ]
        )
        self.assertTrue(mission.pick_report_is_stable(output, 0)[0])
        self.assertFalse(mission.pick_report_is_stable(output, 1)[0])
        self.assertFalse(mission.pick_report_is_stable(output.replace('"DONE"', '"AT_LOW"'), 0)[0])

    def test_pick_rejects_any_failure_even_if_complete_is_printed(self) -> None:
        output = "\n".join(
            [
                'PICK={"event":"success"}',
                'PICK={"event":"failure","phase":"LIFTING"}',
                'PICK={"event":"controller","joint_impedance":"restored_to_hold"}',
                'PICK={"event":"cycle_complete","final_state":"DONE","controller_hold":"stable"}',
            ]
        )
        self.assertFalse(mission.pick_report_is_stable(output, 0)[0])

    def test_place_requires_verified_open_down_up_and_hold(self) -> None:
        report = {
            "status": "complete",
            "phase": "DONE",
            "released": True,
            "stable_hold_recovery_attempt": 1,
            "open_report": {"verified_open": True},
            "down_report": {"label": "down_before_open"},
            "up_report": {"label": "up_after_open"},
        }
        self.assertTrue(mission.place_report_is_stable(report))
        report["open_report"]["verified_open"] = False
        self.assertFalse(mission.place_report_is_stable(report))

    def test_outbound_is_the_verified_single_process_route(self) -> None:
        joined = " ".join(mission.outbound_argv(config(), "run123"))
        self.assertIn("07_start_to_pickup.py", joined)
        self.assertIn("--disable-collision-guard", joined)

    def test_locked_run_never_reorders_pick_and_place_phases(self) -> None:
        labels = []
        pick_output = "\n".join(
            [
                'PICK={"event":"success"}',
                'PICK={"event":"controller","joint_impedance":"restored_to_hold"}',
                'PICK={"event":"cycle_complete","final_state":"DONE","controller_hold":"stable"}',
            ]
        )
        place_report = {
            "status": "complete",
            "phase": "DONE",
            "released": True,
            "stable_hold_recovery_attempt": 1,
            "open_report": {"verified_open": True},
            "down_report": {"label": "down_before_open"},
            "up_report": {"label": "up_after_open"},
        }

        def fake_command(label, _argv, _timeout_s, _log_path):
            labels.append(label)
            if label in {"cup_pick", "bowl_pick"}:
                output = pick_output
            elif label in {"cup_same_place", "bowl_same_place"}:
                output = json.dumps(place_report)
            else:
                output = "{}"
            return SimpleNamespace(returncode=0, output=output, elapsed_s=0.01)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = argparse.Namespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                resume_after_init_confirmed=False,
            )
            with (
                mock.patch.object(mission, "run_streamed_command", side_effect=fake_command),
                mock.patch.object(mission, "init_report_is_stable", return_value=True),
                mock.patch.object(mission, "spine_report_is_stable", return_value=True),
                mock.patch.object(mission, "base_report_is_stable", return_value=True),
                mock.patch.object(mission.time, "sleep", return_value=None),
            ):
                self.assertEqual(mission.run_locked(args, config()), 0)
            final = json.loads(args.checkpoint.read_text(encoding="utf-8"))

        self.assertEqual(
            labels,
            [
                "dual_init",
                "spine_init",
                "outbound_base",
                "cup_pick",
                "cup_same_place",
                "bowl_pick",
                "bowl_same_place",
            ],
        )
        self.assertEqual(final["phase"], mission.Phase.COMPLETE.value)

    def test_confirmed_resume_skips_only_initialization(self) -> None:
        labels = []
        pick_output = "\n".join(
            [
                'PICK={"event":"success"}',
                'PICK={"event":"controller","joint_impedance":"restored_to_hold"}',
                'PICK={"event":"cycle_complete","final_state":"DONE","controller_hold":"stable"}',
            ]
        )
        place_report = {
            "status": "complete",
            "phase": "DONE",
            "released": True,
            "stable_hold_recovery_attempt": 1,
            "open_report": {"verified_open": True},
            "down_report": {"label": "down_before_open"},
            "up_report": {"label": "up_after_open"},
        }

        def fake_command(label, _argv, _timeout_s, _log_path):
            labels.append(label)
            if label in {"cup_pick", "bowl_pick"}:
                output = pick_output
            elif label in {"cup_same_place", "bowl_same_place"}:
                output = json.dumps(place_report)
            else:
                output = "{}"
            return SimpleNamespace(returncode=0, output=output, elapsed_s=0.01)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = argparse.Namespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                resume_after_init_confirmed=True,
            )
            with (
                mock.patch.object(mission, "run_streamed_command", side_effect=fake_command),
                mock.patch.object(mission, "base_report_is_stable", return_value=True),
                mock.patch.object(mission.time, "sleep", return_value=None),
            ):
                self.assertEqual(mission.run_locked(args, config()), 0)

        self.assertEqual(
            labels,
            [
                "outbound_base",
                "cup_pick",
                "cup_same_place",
                "bowl_pick",
                "bowl_same_place",
            ],
        )


if __name__ == "__main__":
    unittest.main()
