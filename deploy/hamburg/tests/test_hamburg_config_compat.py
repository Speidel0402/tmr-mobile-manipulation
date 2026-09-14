from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import hamburg_site_config
from interface_profile import load_interface_profile


class HamburgConfigCompatibilityTests(unittest.TestCase):
    def test_copied_interface_profile_accepts_new_feedback_override(self) -> None:
        profile = json.loads(
            (ROOT / "config" / "interfaces-shanghai.json").read_text(encoding="utf-8")
        )
        override = "TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S"
        profile["arm_motion"].pop("joint_feedback_stale_timeout_s")
        profile["environment_overrides"].pop(override)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old-interface.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            defaulted = load_interface_profile(path, environment={})
            resolved = load_interface_profile(path, environment={override: "1.5"})
        self.assertEqual(defaulted["arm_motion"]["joint_feedback_stale_timeout_s"], 1.0)
        self.assertEqual(resolved["arm_motion"]["joint_feedback_stale_timeout_s"], 1.5)
        self.assertEqual(resolved["applied_environment_overrides"], {override: "1.5"})
        self.assertEqual(resolved["arm_command"], profile["arm_command"])
        self.assertEqual(resolved["spine"], profile["spine"])

    @staticmethod
    def _run_site_config(directory: Path, extra: list[str]) -> tuple[int, str]:
        output = io.StringIO()
        errors = io.StringIO()
        argv = [
            "hamburg_site_config.py",
            "--mission-output", str(directory / "mission.json"),
            "--grasp-output", str(directory / "grasp.json"),
            *extra,
        ]
        with mock.patch.object(sys, "argv", argv), redirect_stdout(output), redirect_stderr(errors):
            try:
                result = hamburg_site_config.main()
            except SystemExit as exc:
                result = int(exc.code)
        return result, errors.getvalue()

    def test_site_config_migrates_saved_grasp_without_changing_reference_geometry(self) -> None:
        grasp = json.loads(
            (ROOT / "config" / "grasp-cycle-shanghai-reference.json").read_text(encoding="utf-8")
        )
        grasp["motion"].pop("joint_feedback_stale_timeout_s")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            template = root / "b215b17-grasp.json"
            template.write_text(json.dumps(grasp), encoding="utf-8")
            result, error = self._run_site_config(root, [
                "--grasp-template", str(template), "--room-length-m", "8.0",
                "--before-door-m", "0.6",
            ])
            self.assertEqual(result, 0, error)
            actual_grasp = json.loads((root / "grasp.json").read_text(encoding="utf-8"))
            mission = json.loads((root / "mission.json").read_text(encoding="utf-8"))
        self.assertEqual(actual_grasp["motion"]["joint_feedback_stale_timeout_s"], 1.0)
        for field in ("posture", "vision", "objects", "spine", "arm_command_interface"):
            self.assertEqual(actual_grasp[field], grasp[field])
        self.assertEqual(mission["hamburg_measurements"]["room_length_m"], 8.0)
        stage = next(item for item in mission["outbound_shanghai_reference"]
                     if item["name"] == "to_before_door")
        self.assertEqual(stage["forward_m"], 0.6)

    def test_site_config_rejects_nonfinite_and_out_of_runtime_range_values_before_writing(self) -> None:
        for option in (
            "--room-length-m=nan", "--room-width-m=inf",
            "--before-door-m=inf", "--initial-forward-m=6.0",
            "--pickup-front-clearance-m=0.1",
        ):
            with self.subTest(option=option), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                result, error = self._run_site_config(root, [option])
                self.assertEqual(result, 2, error)
                self.assertIn("error:", error)
                self.assertFalse((root / "mission.json").exists())
                self.assertFalse((root / "grasp.json").exists())

    def test_site_config_reports_unreadable_template_without_writing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result, error = self._run_site_config(root, [
                "--grasp-template", str(root / "missing.json"),
            ])
            self.assertEqual(result, 2)
            self.assertIn("cannot load site templates", error)
            self.assertFalse((root / "mission.json").exists())
            self.assertFalse((root / "grasp.json").exists())


if __name__ == "__main__":
    unittest.main()
