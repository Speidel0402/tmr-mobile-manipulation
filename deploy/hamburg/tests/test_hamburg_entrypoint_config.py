from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hamburg_preflight import configure_native_publication


class EntrypointConfigTests(unittest.TestCase):
    def test_dry_plans_resolve_same_profile_and_env_as_execution(self):
        profile = json.loads((ROOT / "config/interfaces-shanghai.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "interfaces.json"
            config.write_text(json.dumps(profile))
            environment = dict(os.environ)
            for name in list(environment):
                if name.startswith("TMR_HAMBURG_"):
                    del environment[name]
            environment["TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S"] = "1.5"
            environment["TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S"] = "0.045"
            for entrypoint, extra in (
                ("hamburg_pickup_reset.py", []),
                ("hamburg_grasp_cycle.py", ["--object", "cup"]),
                ("hamburg_mission.py", []),
            ):
                with self.subTest(entrypoint=entrypoint):
                    result = subprocess.run(
                        [sys.executable, str(ROOT / entrypoint), *extra,
                         "--interface-config", str(config)],
                        env=environment, capture_output=True, text=True, timeout=30,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
                    plan = json.loads(result.stdout)
                    self.assertEqual(plan["status"], "plan")
                    self.assertFalse(plan["motion_commanded"])
                    self.assertEqual(plan["arm_motion"]["joint_feedback_stale_timeout_s"], 1.5)
                    self.assertEqual(plan["arm_motion"]["posture_joint_velocity_rad_s"], 0.045)

    def test_mismatched_custom_profile_is_not_silently_ignored(self):
        profile = json.loads((ROOT / "config/interfaces-shanghai.json").read_text())
        profile["arm_command"]["direction"] = [1, 1, 1, 1, 1, 1, 1]
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "interfaces.json"
            config.write_text(json.dumps(profile))
            for entrypoint, extra in (
                ("hamburg_pickup_reset.py", []),
                ("hamburg_grasp_cycle.py", ["--object", "cup"]),
                ("hamburg_mission.py", []),
            ):
                with self.subTest(entrypoint=entrypoint):
                    result = subprocess.run(
                        [sys.executable, str(ROOT / entrypoint), *extra,
                         "--interface-config", str(config),
                         "--output", str(Path(directory) / "report.json")],
                        capture_output=True, text=True, timeout=30,
                    )
                    self.assertEqual(result.returncode, 2, result.stderr + result.stdout)
                    report = json.loads(result.stdout)
                    self.assertFalse(report["motion_commanded"])
                    self.assertIn("direction differs", " ".join(report["errors"]))

    def test_publication_default_and_explicit_venue_settings(self):
        environment = {}
        report = configure_native_publication(environment)
        self.assertEqual(environment["RMW_FASTRTPS_PUBLICATION_MODE"], "ASYNCHRONOUS")
        self.assertEqual(report["warnings"], [])
        environment["RMW_FASTRTPS_PUBLICATION_MODE"] = "SYNCHRONOUS"
        report = configure_native_publication(environment)
        self.assertEqual(report["requested_mode"], "SYNCHRONOUS")
        self.assertTrue(report["warnings"])
        environment["RMW_FASTRTPS_USE_QOS_FROM_XML"] = "1"
        report = configure_native_publication(environment)
        self.assertEqual(report["mode_source"], "venue_xml")
        self.assertTrue(report["warnings"])


if __name__ == "__main__":
    unittest.main()
