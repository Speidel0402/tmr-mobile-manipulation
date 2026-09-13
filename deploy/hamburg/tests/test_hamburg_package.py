from __future__ import annotations

import ast
import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock


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
        self.assertEqual(
            set(self.config["temporary_relay_topics_not_allowed"]),
            {"/mobile_base/pose", "/mobile_base/twist", "/spine/joint_states", "/spine/target_height"},
        )

    def test_camera_topics_and_strategy_are_preserved(self) -> None:
        streams = {item["name"]: item for item in self.config["streams"]}
        self.assertEqual(
            streams["head_camera"]["topic"],
            "/head_camera/zed_node/rgb/color/rect/image",
        )
        self.assertEqual(streams["head_camera"]["qos"], "sensor")
        self.assertEqual((streams["head_camera"]["expected_width"],
                          streams["head_camera"]["expected_height"]), (640, 360))
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
        self.assertEqual(
            self.config["strategy"]["table_height_assumption"],
            "Hamburg and Shanghai table heights must be measured separately; no equivalence is assumed",
        )

    def test_hamburg_executables_do_not_use_remote_or_cli_or_override_dds(self) -> None:
        paths = [ROOT / "hamburg_preflight.py", ROOT / "hamburg_grasp_check.py",
                 ROOT / "spine_control.py", ROOT / "hamburg_pickup_reset.py",
                 ROOT / "run_hamburg.sh"]
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
        self.assertNotIn("rclpy.init(", (ROOT / "spine_control.py").read_text(encoding="utf-8"))
        grasp_text = (ROOT / "hamburg_grasp_check.py").read_text(encoding="utf-8")
        self.assertNotIn("create_publisher(", grasp_text)
        self.assertNotIn("send_goal_async(", grasp_text)

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
        self.assertNotIn("/franka_spine_node/get_position", legacy)
        self.assertNotIn("/franka_spine_node/move_absolute", legacy)
        streams = {item["name"] for item in self.config["streams"] if item["required"]}
        self.assertIn("mobile_base_odometry", streams)
        self.assertNotIn("mobile_base_pose", streams)
        self.assertNotIn("mobile_base_twist", streams)
        self.assertNotIn("spine_joint_state", streams)

    def test_native_spine_profile_and_camera_contract(self) -> None:
        module = self._load_module("interface_profile")
        profile = module.load_interface_profile(
            ROOT / "config" / "interfaces-shanghai.json", environment={}
        )
        self.assertEqual(profile["gripper"]["message_type"], "std_msgs/msg/Float32")
        self.assertEqual(profile["gripper"]["open"], 0.8)
        self.assertEqual(profile["gripper"]["closed"], 0.0)
        self.assertEqual(profile["spine"]["interface"], "action")
        self.assertEqual(profile["spine"]["action_type"], "franka_spine_msgs/action/MoveAbsolute")
        self.assertEqual(profile["spine"]["position_service_type"], "franka_spine_msgs/srv/GetPosition")
        self.assertEqual(profile["spine"]["home_m"], 0.7)
        self.assertEqual((profile["head_camera"]["width"], profile["head_camera"]["height"]), (640, 360))
        self.assertEqual(profile["head_camera"]["zed_pub_resolution"], "CUSTOM")
        organizer_text = (ROOT / "ORGANIZER_ACTIONS.md").read_text(
            encoding="utf-8"
        )
        self.assertIn("without", organizer_text)
        self.assertIn("Float32-to-action relay", organizer_text)
        self.assertIn("MoveAbsolute", organizer_text)

    def test_hamburg_native_check_detects_organizer_bridges(self) -> None:
        preflight = self._load_module("hamburg_preflight")
        graph = {
            "/swerve_drive_controller/odom": ["nav_msgs/msg/Odometry"],
            "/mobile_base/pose": ["geometry_msgs/msg/PoseStamped"],
            "/spine/target_height": ["std_msgs/msg/Float32"],
        }
        self.assertEqual(
            preflight.temporary_relay_topics(graph, self.config),
            ["/mobile_base/pose", "/spine/target_height"],
        )
        self.assertEqual(
            preflight.temporary_relay_topics({"/swerve_drive_controller/odom": ["nav_msgs/msg/Odometry"]}, self.config),
            [],
        )

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
                "TMR_HAMBURG_HEAD_CAMERA_WIDTH": "1280",
                "TMR_HAMBURG_HEAD_CAMERA_HEIGHT": "720",
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
        self.assertNotIn("spine_height_target", commands)
        self.assertEqual(resolved["action_endpoints"][0]["action"], "/franka_spine_node/move_absolute")
        self.assertEqual(resolved["service_endpoints"][0]["service"], "/franka_spine_node/get_position")
        head = next(s for s in resolved["streams"] if s["name"] == "head_camera")
        self.assertEqual((head["expected_width"], head["expected_height"]), (1280, 720))
        self.assertEqual(
            profile["applied_environment_overrides"],
            {
                "TMR_HAMBURG_GRIPPER_OPEN": "1.0",
                "TMR_HAMBURG_SPINE_HOME_M": "0.65",
                "TMR_HAMBURG_HEAD_CAMERA_WIDTH": "1280",
                "TMR_HAMBURG_HEAD_CAMERA_HEIGHT": "720",
            },
        )

    def test_spine_topic_override_is_explicit(self) -> None:
        module = self._load_module("interface_profile")
        profile = module.load_interface_profile(
            ROOT / "config" / "interfaces-shanghai.json",
            environment={"TMR_HAMBURG_SPINE_INTERFACE": "topic"},
        )
        resolved = module.apply_interface_profile(self.config, profile)
        self.assertEqual(resolved["action_endpoints"], [])
        self.assertEqual(resolved["service_endpoints"], [])
        spine = next(c for c in resolved["command_endpoints"] if c["name"] == "spine_height_target")
        self.assertEqual(spine["topic"], "/spine/target_height")

    def test_grasp_observation_requires_fresh_stable_detections(self) -> None:
        module = self._load_module("hamburg_grasp_check")
        self.assertIn("left_wrist_camera", module.GRASP_REQUIRED_STREAMS)
        self.assertNotIn("head_camera", module.GRASP_REQUIRED_STREAMS)
        report, errors = module.evaluate_observations([
            (292.0, 168.0), (293.0, 168.5), (292.5, 167.5),
            (292.2, 168.1), (293.1, 167.9),
        ])
        self.assertEqual(errors, [])
        self.assertEqual(report["valid_detections"], 5)
        _, errors = module.evaluate_observations([None, None, (292.0, 168.0), None, None])
        self.assertTrue(errors)

    def test_grasp_wrist_image_rejects_wrong_shape(self) -> None:
        module = self._load_module("hamburg_grasp_check")
        image = types.SimpleNamespace(width=1280, height=720, step=3840,
                                      encoding="rgb8", data=b"")
        with self.assertRaises(ValueError):
            module.decode_wrist_rgb(image)

    def test_grasp_check_saves_reviewable_annotated_wrist_frame(self) -> None:
        import cv2
        import numpy as np

        module = self._load_module("hamburg_grasp_check")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "cup.png"
            module.save_observation_image(np.zeros((480, 640, 3), dtype=np.uint8),
                                          "cup", (292.5, 168.0), output)
            image = cv2.imread(str(output))
            self.assertEqual(image.shape[:2], (480, 640))
            self.assertGreater(int(image[168, 292, 2]), int(image[168, 292, 0]))

    def test_grasp_posture_targets_match_submitted_mission(self) -> None:
        posture = json.loads((ROOT / "config" / "grasp-observation.json").read_text(encoding="utf-8"))
        mission = ast.parse((ROOT.parents[1] / "mission" / "scripts" / "run_full_competition_cycle.py").read_text(encoding="utf-8"))
        constants = {
            target.id: ast.literal_eval(node.value)
            for node in mission.body if isinstance(node, ast.Assign)
            for target in node.targets if isinstance(target, ast.Name)
            and target.id in {"LEFT_PICK_TOP_TARGET", "RIGHT_PARKING_TARGET"}
        }
        self.assertEqual(posture["left_pick_top_rad"], list(constants["LEFT_PICK_TOP_TARGET"]))
        self.assertEqual(posture["right_parking_rad"], list(constants["RIGHT_PARKING_TARGET"]))
        module = self._load_module("hamburg_grasp_check")
        latest = {
            f"{arm}_arm_joint_state": {"names": module.EXPECTED_JOINTS[arm], "position": list(posture[key])}
            for arm, key in (("left", "left_pick_top_rad"), ("right", "right_parking_rad"))
        }
        report, errors = module.posture_report(latest, posture)
        self.assertEqual(errors, [])
        self.assertTrue(report["left"]["within_tolerance"])
        latest["right_arm_joint_state"]["position"][0] += 0.1
        _, errors = module.posture_report(latest, posture)
        self.assertTrue(errors)

    def test_grasp_review_descents_match_shanghai_standalone(self) -> None:
        source = (ROOT.parents[1] / "mission" / "scripts" / "run_standalone_grasp_test.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        legacy = next(ast.literal_eval(node.value) for node in tree.body
                      if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == "OBJECTS"
                              for target in node.targets))
        module = self._load_module("hamburg_grasp_check")
        self.assertEqual(module.OBJECTS, {name: spec[1] for name, spec in legacy.items()})

    def test_geometry_review_keeps_shanghai_values_as_references(self) -> None:
        module = self._load_module("geometry_review")
        template = json.loads((ROOT / "config" / "geometry-observations.json").read_text(encoding="utf-8"))
        missing = module.review(template)
        self.assertEqual(missing["status"], "needs_measurements")
        self.assertFalse(missing["ready_for_motion"])
        self.assertIn("table_top_height_m", missing["missing_hamburg_measurements"])
        template["hamburg"].update({field: 1.0 for field in module.REQUIRED})
        template["hamburg"]["table_top_height_m"] = 0.78
        template["shanghai_reference_table_top_height_m"] = 0.75
        measured = module.review(template)
        self.assertEqual(measured["table_top_delta_hamburg_minus_shanghai_m"], 0.03)
        self.assertFalse(measured["ready_for_motion"])
        template["hamburg"]["room_length_m"] = -1.0
        with self.assertRaises(ValueError):
            module.review(template)

    def test_native_spine_probe_reads_state_without_sending_goal(self) -> None:
        profile_module = self._load_module("interface_profile")
        preflight = self._load_module("hamburg_preflight")
        profile = profile_module.load_interface_profile(environment={})
        config = profile_module.apply_interface_profile(self.config, profile)

        class ServiceType:
            class Request:
                pass

        class ActionType:
            class Goal:
                @staticmethod
                def get_fields_and_field_types():
                    return {name: "float" for name in (
                        "position", "velocity", "acceleration", "deceleration"
                    )}

        class Future:
            def done(self):
                return True

            def result(self):
                return types.SimpleNamespace(success=True, position=0.7)

        service_client = mock.Mock()
        service_client.wait_for_service.return_value = True
        service_client.call_async.return_value = Future()
        action_client = mock.Mock()
        action_client.wait_for_server.return_value = True
        node = mock.Mock()
        node.create_client.return_value = service_client
        node.get_service_names_and_types.return_value = [
            ("/franka_spine_node/get_position", ["franka_spine_msgs/srv/GetPosition"])
        ]
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.spin_until_future_complete = mock.Mock()
        fake_action = types.ModuleType("rclpy.action")
        fake_action.ActionClient = mock.Mock(return_value=action_client)
        fake_utilities = types.ModuleType("rosidl_runtime_py.utilities")
        fake_utilities.get_action = lambda _: ActionType
        fake_utilities.get_service = lambda _: ServiceType
        with mock.patch.dict(sys.modules, {
            "rclpy": fake_rclpy,
            "rclpy.action": fake_action,
            "rosidl_runtime_py.utilities": fake_utilities,
        }):
            report, errors = preflight.probe_native_spine(node, config, 1.0)
            action_client.wait_for_server.return_value = False
            blocked_report, blocked_errors = preflight.probe_native_spine(node, config, 1.0)
        self.assertEqual(errors, [])
        self.assertEqual(report["service"]["position_m"], 0.7)
        self.assertTrue(report["action"]["ready"])
        self.assertFalse(blocked_report["action"]["ready"])
        self.assertTrue(any("native spine action unavailable" in error for error in blocked_errors))
        action_client.send_goal_async.assert_not_called()
        self.assertEqual(node.destroy_client.call_count, 2)
        self.assertEqual(action_client.destroy.call_count, 2)

    def test_native_spine_adapter_uses_existing_node_and_verifies_position(self) -> None:
        module = self._load_module("spine_control")

        class ServiceType:
            class Request:
                pass

        class ActionType:
            class Goal:
                pass

        class Future:
            def __init__(self, value):
                self.value = value

            def done(self):
                return True

            def result(self):
                return self.value

        position_client = mock.Mock()
        position_client.wait_for_service.return_value = True
        position_client.call_async.side_effect = [
            Future(types.SimpleNamespace(success=True, position=0.6)),
            Future(types.SimpleNamespace(success=True, position=0.7)),
        ]
        handle = mock.Mock(accepted=True)
        handle.get_result_async.return_value = Future(types.SimpleNamespace(
            status=4, result=types.SimpleNamespace(success=True, error="")
        ))
        action_client = mock.Mock()
        action_client.wait_for_server.return_value = True
        action_client.send_goal_async.return_value = Future(handle)
        node = mock.Mock()
        node.create_client.return_value = position_client
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.spin_until_future_complete = mock.Mock()
        fake_action = types.ModuleType("rclpy.action")
        fake_action.ActionClient = mock.Mock(return_value=action_client)
        fake_utilities = types.ModuleType("rosidl_runtime_py.utilities")
        fake_utilities.get_action = lambda _: ActionType
        fake_utilities.get_service = lambda _: ServiceType
        fake_status = types.ModuleType("action_msgs.msg")
        fake_status.GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
        with mock.patch.dict(sys.modules, {
            "rclpy": fake_rclpy,
            "rclpy.action": fake_action,
            "rosidl_runtime_py.utilities": fake_utilities,
            "action_msgs.msg": fake_status,
        }):
            controller = module.SpineControl(node, {
                "interface": "action",
                "action_type": "franka_spine_msgs/action/MoveAbsolute",
                "action_name": "/franka_spine_node/move_absolute",
                "position_service_type": "franka_spine_msgs/srv/GetPosition",
                "position_service": "/franka_spine_node/get_position",
                "minimum_m": 0.0,
                "maximum_m": 0.8,
            })
            report = controller.move_absolute(0.7)
            controller.close()
        self.assertTrue(report["moved"])
        self.assertEqual(report["measured_position_m"], 0.7)
        goal = action_client.send_goal_async.call_args.args[0]
        self.assertEqual((goal.position, goal.velocity), (0.7, 0.05))
        self.assertEqual((goal.acceleration, goal.deceleration), (0.1, 0.1))
        node.destroy_client.assert_called_once_with(position_client)

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
        workflow = (ROOT.parents[1] / ".github" / "workflows" / "hamburg-package.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("platforms: linux/arm64", workflow)
        self.assertNotIn("--platform=linux/arm64", dockerfile)
        self.assertIn("ros:humble-ros-base-jammy", dockerfile)
        self.assertIn("python3-opencv", dockerfile)
        self.assertIn("grasp/scripts/cup_rim_detector.py", dockerfile)
        self.assertIn("base/scripts/letter_card_vision.py", dockerfile)
        self.assertIn("hamburg_pickup_reset.py", dockerfile)
        self.assertNotIn("python:3.11", dockerfile)


if __name__ == "__main__":
    unittest.main()
