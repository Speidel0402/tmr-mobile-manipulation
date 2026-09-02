#!/usr/bin/env python3
"""Offline contracts for the parameterized three-object mission."""

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
SPEC = importlib.util.spec_from_file_location(
    "three_object_delivery", SCRIPT_DIR / "run_three_object_delivery.py"
)
assert SPEC is not None and SPEC.loader is not None
mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = mission
SPEC.loader.exec_module(mission)


def config():
    return mission.FullConfig(
        base_host="tmr-user@172.16.0.50",
        base_root="/home/tmr-user/tmr_cycle",
        arm_root="/home/aup/tmr-mobile-manipulation",
        arm_env="/home/aup/tmr_env.sh",
        init_timeout_s=180.0,
        outbound_timeout_s=200.0,
        pick_timeout_s=300.0,
        return_timeout_s=240.0,
        place_timeout_s=220.0,
        transition_settle_s=0.0,
    )


def tasks(cup="B", bowl="A", plate="D"):
    template = mission.load_plan(mission.DEFAULT_DELIVERY_CONFIG, None, None)
    return mission.build_tasks(template, cup, bowl, plate)


class ThreeObjectDeliveryContracts(unittest.TestCase):
    def test_default_assignment_and_object_specific_depths(self):
        values = tasks()
        self.assertEqual(mission.assignment(values), {"cup": "B", "bowl": "A", "plate": "D"})
        self.assertEqual(values[0].plan.alphabet, "ABD")
        self.assertEqual([item.pick_descent_m for item in values], [0.340, 0.360, 0.375])
        self.assertEqual([item.plan.near_down_m for item in values], [0.340, 0.360, 0.375])

    def test_letters_are_real_parameters_and_must_be_distinct(self):
        values = tasks("C", "E", "F")
        self.assertEqual(mission.assignment(values), {"cup": "C", "bowl": "E", "plate": "F"})
        self.assertEqual(values[0].plan.alphabet, "CEF")
        with self.assertRaisesRegex(mission.MissionError, "distinct"):
            tasks("A", "A", "D")

    def test_each_object_uses_its_own_detector_entrypoint(self):
        values = tasks()
        commands = [" ".join(mission.object_pick_argv(config(), item)) for item in values]
        self.assertIn("run_streamed_live_pick_cycle.py", commands[0])
        self.assertIn("run_streamed_food_bowl_pick_cycle.py", commands[1])
        self.assertIn("run_streamed_plate_pick_cycle.py", commands[2])
        self.assertTrue(all("--force-restore-top" in command for command in commands))

    def test_runtime_returns_after_first_two_but_stops_after_plate(self):
        values = tasks()
        labels = []

        def report_for(label):
            if label == "control-mission":
                return {"status": "success", "mode": "mission", "teleop_velocity_enabled": False, "mission_lease": True}
            if label == "control-teleop":
                return {"status": "success", "mode": "teleop", "teleop_velocity_enabled": True, "mission_lease": False}
            if label.endswith("-prepare"):
                return {
                    "status": "complete",
                    "next_stage": 3,
                    "zero_command_latched": True,
                    "reports": [
                        {"stage": "RETREAT_TO_PREDOOR", "actual_forward_m": -1.684},
                        {"stage": "TURN_CCW180", "actual_ccw_deg": 179.34},
                        {"stage": "BACKWARD_AFTER_TURN", "actual_forward_m": -0.2342},
                    ],
                }
            if label.endswith("-search"):
                name = label.split("-")[0]
                letter = {item.name: item.letter for item in values}[name]
                return {"status": "success", "target_centered": True, "target_letter": letter, "row": "near", "actual_right_m": 1.2, "zero_command_latched": True}
            if label.endswith("-place"):
                return {"status": "complete", "phase": "DONE", "released": True, "stable_hold_recovery_attempt": 1}
            if label.endswith("-return"):
                return {
                    "status": "complete",
                    "phase": "COMPLETE",
                    "zero_command_latched": True,
                    "reports": [
                        {"stage": "LEFT_BY_MEASURED_OUTBOUND"},
                        {"stage": "TURN_CW_180"},
                    ],
                    "door_report": {"status": "success", "final_state": "FINAL_STOP"},
                }
            return {}

        def fake_run(label, _argv, _timeout, _log):
            labels.append(label)
            return SimpleNamespace(returncode=0, output=json.dumps(report_for(label)))

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = SimpleNamespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                base_phase_timeout_s=180.0,
                search_timeout_s=120.0,
                return_timeout_s=240.0,
            )
            with (
                mock.patch.object(mission, "run_streamed_command", side_effect=fake_run),
                mock.patch.object(mission, "init_report_is_stable", return_value=True),
                mock.patch.object(mission, "spine_report_is_stable", return_value=True),
                mock.patch.object(mission, "base_report_is_stable", return_value=True),
                mock.patch.object(mission, "pick_report_is_stable", return_value=(True, {})),
            ):
                result = mission.run_locked(args, config(), values)
            final = json.loads(args.checkpoint.read_text(encoding="utf-8"))

        self.assertEqual(result, 0)
        self.assertEqual(
            labels,
            [
                "control-mission", "dual-init", "spine-init", "outbound",
                "cup-pick", "cup-prepare", "cup-search", "cup-place", "cup-return",
                "bowl-pick", "bowl-prepare", "bowl-search", "bowl-place", "bowl-return",
                "plate-pick", "plate-prepare", "plate-search", "plate-place",
                "control-teleop",
            ],
        )
        self.assertEqual(final["phase"], "COMPLETE")
        self.assertEqual(final["completed"], ["cup", "bowl", "plate"])
        self.assertEqual(final["final_location"], "letter_D")
        self.assertFalse(final["returned_after_final"])

    def test_failure_still_restores_teleop_last(self):
        values = tasks()
        labels = []

        def fake_run(label, _argv, _timeout, _log):
            labels.append(label)
            if label == "control-mission":
                report = {"status": "success", "mode": "mission", "teleop_velocity_enabled": False, "mission_lease": True}
                return SimpleNamespace(returncode=0, output=json.dumps(report))
            if label == "control-teleop":
                report = {"status": "success", "mode": "teleop", "teleop_velocity_enabled": True, "mission_lease": False}
                return SimpleNamespace(returncode=0, output=json.dumps(report))
            return SimpleNamespace(returncode=1, output="{}")

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = SimpleNamespace(
                checkpoint=root / "state.json",
                log_dir=root / "logs",
                base_phase_timeout_s=180.0,
                search_timeout_s=120.0,
                return_timeout_s=240.0,
            )
            with mock.patch.object(mission, "run_streamed_command", side_effect=fake_run):
                with self.assertRaises(mission.MissionError):
                    mission.run_locked(args, config(), values)

        self.assertEqual(labels, ["control-mission", "dual-init", "control-teleop"])


if __name__ == "__main__":
    unittest.main()
