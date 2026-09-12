from __future__ import annotations

import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class HamburgPackageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(
            (ROOT / "config" / "venue.json").read_text(encoding="utf-8")
        )

    def test_venue_contract(self) -> None:
        self.assertEqual(self.config["host"]["ros_distro"], "humble")
        self.assertEqual(self.config["host"]["architecture"], "aarch64")
        self.assertEqual(self.config["dds"]["ROS_DOMAIN_ID"], "0")
        self.assertIn(None, self.config["dds"]["ROS_LOCALHOST_ONLY_allowed"])
        self.assertIn("0", self.config["dds"]["ROS_LOCALHOST_ONLY_allowed"])
        self.assertEqual(
            self.config["dds"]["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp"
        )

    def test_camera_topics_and_strategy_are_preserved(self) -> None:
        streams = {item["name"]: item for item in self.config["streams"]}
        self.assertEqual(
            streams["head_camera"]["topic"],
            "/head_camera/zed_node/rgb/color/rect/image",
        )
        self.assertEqual(streams["head_camera"]["qos"], "sensor")
        self.assertEqual(
            streams["left_wrist_camera"]["topic"],
            "/wrist_camera_left/camera/color/image_raw",
        )
        self.assertEqual(
            self.config["strategy"]["object_order"], ["cup", "bowl", "plate"]
        )
        self.assertEqual(
            self.config["strategy"]["destination_mapping"],
            {"cup": "B", "bowl": "A", "plate": "D"},
        )

    def test_hamburg_executables_do_not_use_remote_or_cli_or_override_dds(self) -> None:
        paths = [ROOT / "hamburg_preflight.py", ROOT / "run_hamburg.sh"]
        forbidden = (
            "subprocess",
            "paramiko",
            "ssh ",
            "ros2 topic",
            "ros2 service",
            "ros2 action",
            "os.environ[\"ROS_DOMAIN_ID\"] =",
            "export ROS_DOMAIN_ID=",
            "export RMW_IMPLEMENTATION=",
        )
        for path in paths:
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                self.assertNotIn(token, text, f"{token!r} found in {path.name}")

    def test_audit_finds_submitted_runtime_incompatibilities(self) -> None:
        import importlib.util

        path = ROOT / "audit_submission.py"
        spec = importlib.util.spec_from_file_location("hamburg_audit", path)
        self.assertIsNotNone(spec)
        self.assertIsNotNone(spec.loader)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        root = ROOT.parents[1]
        texts = {
            candidate.relative_to(root).as_posix(): candidate.read_text(
                encoding="utf-8", errors="replace"
            )
            for candidate in module.iter_sources(root)
        }
        self.assertTrue(
            any(module.RULES["runtime_process_spawn"].search(text) for text in texts.values())
        )
        self.assertTrue(
            any(
                module.RULES["ros_participant_created_in_phase_script"].search(text)
                for text in texts.values()
            )
        )

    def test_legacy_interfaces_are_explicitly_not_assumed(self) -> None:
        legacy = set(self.config["legacy_interfaces_not_assumed"])
        self.assertIn("/left_ik/compute_fk", legacy)
        self.assertIn("/left/action_server/ptp_motion", legacy)
        self.assertIn("/franka_spine_node/get_position", legacy)

    def test_unknown_scalar_command_contracts_are_requested_not_guessed(self) -> None:
        module = self._load_module("interface_profile")
        profile = module.load_interface_profile(
            ROOT / "config" / "interfaces-shanghai.json", environment={}
        )
        self.assertEqual(profile["gripper"]["message_type"], "std_msgs/msg/Float32")
        self.assertEqual(profile["gripper"]["open"], 0.8)
        self.assertEqual(profile["gripper"]["closed"], 0.0)
        self.assertEqual(profile["spine"]["message_type"], "std_msgs/msg/Float32")
        self.assertEqual(profile["spine"]["home_m"], 0.7)
        organizer_text = (ROOT / "ORGANIZER_ACTIONS.md").read_text(
            encoding="utf-8"
        )
        self.assertIn("Before either FR3 arm is activated", organizer_text)
        self.assertIn("creates one ROS participant", organizer_text)
        self.assertIn("Fast venue correction", organizer_text)

    @staticmethod
    def _load_module(name: str):
        path = ROOT / f"{name}.py"
        root_text = str(ROOT)
        if root_text not in sys.path:
            sys.path.insert(0, root_text)
        spec = importlib.util.spec_from_file_location(name, path)
        if spec is None or spec.loader is None:
            raise AssertionError(f"cannot load {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_interface_environment_override_and_resolved_commands(self) -> None:
        profile_module = self._load_module("interface_profile")
        profile = profile_module.load_interface_profile(
            ROOT / "config" / "interfaces-shanghai.json",
            environment={
                "TMR_HAMBURG_GRIPPER_OPEN": "1.0",
                "TMR_HAMBURG_SPINE_HOME_M": "0.65",
            },
        )
        resolved = profile_module.apply_interface_profile(self.config, profile)
        commands = {item["name"]: item for item in resolved["command_endpoints"]}
        self.assertEqual(
            commands["left_gripper_target"]["accepted_types"],
            ["std_msgs/msg/Float32"],
        )
        self.assertEqual(commands["left_gripper_target"]["message_field"], "data")
        self.assertEqual(
            commands["left_gripper_target"]["command_values"]["open"], 1.0
        )
        self.assertEqual(
            commands["spine_height_target"]["command_values"]["home"], 0.65
        )
        self.assertEqual(
            profile["applied_environment_overrides"],
            {
                "TMR_HAMBURG_GRIPPER_OPEN": "1.0",
                "TMR_HAMBURG_SPINE_HOME_M": "0.65",
            },
        )

    def test_udp_only_fastdds_profile_validation(self) -> None:
        module = self._load_module("hamburg_preflight")
        with tempfile.TemporaryDirectory() as directory:
            good = Path(directory) / "good.xml"
            good.write_text(
                "<profiles><transport_descriptor><type>UDPv4</type>"
                "</transport_descriptor><useBuiltinTransports>false"
                "</useBuiltinTransports></profiles>",
                encoding="utf-8",
            )
            report, errors = module.fastdds_profile_report(good)
            self.assertEqual(errors, [])
            self.assertTrue(report["udp_only"])

            bad = Path(directory) / "bad.xml"
            bad.write_text(
                "<profiles><transport_descriptor><type>SHM</type>"
                "</transport_descriptor><useBuiltinTransports>true"
                "</useBuiltinTransports></profiles>",
                encoding="utf-8",
            )
            report, errors = module.fastdds_profile_report(bad)
            self.assertTrue(errors)
            self.assertFalse(report["udp_only"])

    def test_arm64_humble_container(self) -> None:
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("--platform=linux/arm64", dockerfile)
        self.assertIn("ros:humble-ros-base-jammy", dockerfile)
        self.assertNotIn("python:3.11", dockerfile)


if __name__ == "__main__":
    unittest.main()
