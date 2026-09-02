#!/usr/bin/env python3
"""Offline contracts for held-plate delivery to B."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "mission" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


mission = load("held_plate_letter_mission", SCRIPT_DIR / "run_letter_delivery_competition.py")
wrapper = load("held_plate_to_b", SCRIPT_DIR / "run_held_plate_to_b.py")


def full_config():
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


class HeldPlateToBContracts(unittest.TestCase):
    def test_wrapper_hard_codes_held_plate_b_and_stop_after_place(self):
        args = SimpleNamespace(
            execute=True,
            resume_from_search_start_confirmed=False,
            row="auto",
            base_host="tmr-user@172.16.0.50",
            base_root="/home/tmr-user/tmr_cycle",
            arm_root=ROOT,
            arm_env="/home/aup/tmr_env.sh",
            base_phase_timeout_s=160.0,
            search_timeout_s=100.0,
            place_timeout_s=200.0,
            checkpoint=Path("state.json"),
            log_dir=Path("logs"),
        )
        command = wrapper.build_command(args)
        joined = " ".join(str(value) for value in command)
        self.assertIn("--resume-from-object-held-confirmed", joined)
        self.assertIn("--stop-after-place", joined)
        self.assertIn("--target-letter B", joined)
        self.assertIn("held_plate_to_b.json", joined)

    def test_plate_config_uses_375mm_for_near_and_far(self):
        plan = mission.load_plan(wrapper.PLATE_CONFIG, None, None)
        self.assertEqual(plan.target_letter, "B")
        self.assertEqual(mission.placement_values(plan, "near"), (0.0, 0.375))
        self.assertEqual(mission.placement_values(plan, "far"), (0.16, 0.375))

    def test_stop_after_place_never_dispatches_return(self):
        plan = mission.load_plan(wrapper.PLATE_CONFIG, None, None)
        labels = []
        reports = {
            "prepare-search": {
                "status": "complete",
                "next_stage": 3,
                "zero_command_latched": True,
                "reports": [{}, {}, {}],
            },
            "letter-search": {
                "status": "success",
                "target_centered": True,
                "target_letter": "B",
                "row": "near",
                "actual_right_m": 1.1,
                "zero_command_latched": True,
            },
            "letter-place": {
                "status": "complete",
                "phase": "DONE",
                "released": True,
                "stable_hold_recovery_attempt": 1,
            },
        }

        def fake_run(label, _argv, _timeout, _log):
            labels.append(label)
            return SimpleNamespace(returncode=0, output=json.dumps(reports[label]))

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = SimpleNamespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                base_phase_timeout_s=160.0,
                search_timeout_s=100.0,
                return_timeout_s=220.0,
                stop_after_place=True,
            )
            with mock.patch.object(mission, "run_streamed_command", side_effect=fake_run):
                result = mission.run_locked(
                    args,
                    full_config(),
                    plan,
                    resume_from_object_held=True,
                )
            final = json.loads(args.checkpoint.read_text(encoding="utf-8"))

        self.assertEqual(result, 0)
        self.assertEqual(labels, ["prepare-search", "letter-search", "letter-place"])
        self.assertEqual(final["phase"], "COMPLETE")
        self.assertFalse(final["returned_to_table"])

    def test_search_start_resume_skips_prepare_route(self):
        plan = mission.load_plan(wrapper.PLATE_CONFIG, None, None)
        labels = []
        reports = {
            "letter-search": {
                "status": "success",
                "target_centered": True,
                "target_letter": "B",
                "row": "near",
                "actual_right_m": 0.8,
                "zero_command_latched": True,
            },
            "letter-place": {
                "status": "complete",
                "phase": "DONE",
                "released": True,
                "stable_hold_recovery_attempt": 1,
            },
        }

        def fake_run(label, _argv, _timeout, _log):
            labels.append(label)
            return SimpleNamespace(returncode=0, output=json.dumps(reports[label]))

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = SimpleNamespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                base_phase_timeout_s=160.0,
                search_timeout_s=100.0,
                return_timeout_s=220.0,
                stop_after_place=True,
            )
            with mock.patch.object(mission, "run_streamed_command", side_effect=fake_run):
                result = mission.run_locked(
                    args,
                    full_config(),
                    plan,
                    resume_from_search_start=True,
                )

        self.assertEqual(result, 0)
        self.assertEqual(labels, ["letter-search", "letter-place"])


if __name__ == "__main__":
    unittest.main()
