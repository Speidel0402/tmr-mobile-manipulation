from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hamburg_grasp_cycle import (
    NativeArmGraspCycle,
    alignment_delta_m,
    load_cycle_config,
    plan_report as grasp_plan,
    resolve_cycle_venue_overrides,
    validate_cycle_venue_consistency,
)
from hamburg_pickup_reset import plan_report as pickup_reset_plan, run_reset
from hamburg_mission import (
    HamburgMission,
    NativeBaseControl,
    plan_report as mission_plan,
    resolve_mission_venue_overrides,
    validate_combined_configs,
    validate_mission_config,
    validate_mission_venue_consistency,
)
from hamburg_motion_core import (
    FR3V2_JOINT_LIMITS_RAD,
    base_relative_pose,
    cartesian_waypoints,
    derive_mount_rotation,
    gripper_contact_report,
    smooth_joint_targets,
)
from hamburg_preflight import load_config


class HamburgMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grasp = load_cycle_config(ROOT / "config" / "grasp-cycle-shanghai-reference.json")
        cls.mission = json.loads((ROOT / "config" / "mission-shanghai-reference.json").read_text(encoding="utf-8"))
        cls.q = cls.grasp["posture"]["left_pick_top_rad"]
        cls.mount = derive_mount_rotation(cls.q, cls.grasp["posture"]["left_tool_quaternion_xyzw"])

    def test_all_pick_descents_follow_requested_base_z_and_official_limits(self) -> None:
        start = base_relative_pose(self.q, self.mount)
        for name, item in self.grasp["objects"].items():
            descent = float(item["descent_m"])
            path = cartesian_waypoints(self.q, [0.0, 0.0, -descent], self.mount, 0.010)
            self.assertGreaterEqual(len(path), int(descent / 0.010))
            end = base_relative_pose(path[-1], self.mount)
            np.testing.assert_allclose(end[:3, 3] - start[:3, 3], [0.0, 0.0, -descent], atol=3.0e-4,
                                       err_msg=name)
            self.assertTrue(np.all(np.asarray(path) > FR3V2_JOINT_LIMITS_RAD[:, 0]))
            self.assertTrue(np.all(np.asarray(path) < FR3V2_JOINT_LIMITS_RAD[:, 1]))

    def test_near_and_far_placement_paths_solve_for_each_object(self) -> None:
        for forward in (0.10, 0.26):
            forward_path = cartesian_waypoints(self.q, [forward, 0.0, 0.0], self.mount, 0.010)
            for item in self.grasp["objects"].values():
                down = cartesian_waypoints(forward_path[-1], [0.0, 0.0, -float(item["descent_m"])], self.mount, 0.010)
                self.assertTrue(down)
                self.assertTrue(np.all(np.asarray(down) >= FR3V2_JOINT_LIMITS_RAD[:, 0]))
                self.assertTrue(np.all(np.asarray(down) <= FR3V2_JOINT_LIMITS_RAD[:, 1]))

    def test_joint_stream_respects_velocity_per_sample(self) -> None:
        target = (np.asarray(self.q) + np.asarray([0.02, -0.03, 0.01, 0.0, -0.02, 0.03, 0.01])).tolist()
        stream = smooth_joint_targets(self.q, target, 50.0, 0.06)
        values = np.vstack([self.q, stream])
        self.assertLessEqual(float(np.max(np.abs(np.diff(values, axis=0)))), 0.06 / 50.0 + 1.0e-12)
        np.testing.assert_allclose(stream[-1], target)

    def test_posture_move_uses_venue_posture_velocity(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.commanded = {"left": list(self.q), "right": list(self.q)}
        runner.states = {"left": list(self.q), "right": list(self.q)}
        runner.move_arm_targets = mock.Mock()
        with mock.patch("hamburg_grasp_cycle.smooth_joint_targets", return_value=[]) as smooth:
            runner.move_to_joint_posture("left", list(self.q))
        self.assertEqual(
            smooth.call_args.args[3],
            self.grasp["motion"]["posture_joint_velocity_rad_s"],
        )

    def test_previous_grasp_config_gains_safe_lag_defaults(self) -> None:
        legacy = json.loads(
            (ROOT / "config" / "grasp-cycle-shanghai-reference.json").read_text(
                encoding="utf-8"
            )
        )
        for key in (
            "posture_joint_velocity_rad_s", "following_error_pause_rad",
            "following_error_resume_rad", "following_error_recovery_timeout_s",
        ):
            legacy["motion"].pop(key)
        legacy["motion"]["maximum_following_error_rad"] = 0.16
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy-grasp.json"
            path.write_text(json.dumps(legacy), encoding="utf-8")
            loaded = load_cycle_config(path)
        self.assertEqual(loaded["motion"]["posture_joint_velocity_rad_s"], 0.06)
        self.assertLess(
            loaded["motion"]["following_error_resume_rad"],
            loaded["motion"]["following_error_pause_rad"],
        )
        self.assertLess(
            loaded["motion"]["following_error_pause_rad"],
            loaded["motion"]["maximum_following_error_rad"],
        )

    def test_transient_following_lag_holds_and_recovers(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.states = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.commanded = {"left": [0.0] * 7, "right": [0.21] + [0.0] * 6}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.following_lag_events = deque(maxlen=30)

        def catch_up(_duration):
            runner.states["right"][0] = 0.08
            runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}

        runner.spin_for = catch_up
        runner._check_all_following()
        self.assertEqual(runner.following_lag_events[-1]["status"], "recovered")
        self.assertLessEqual(
            runner.following_lag_events[-1]["recovered_errors_rad"]["right"],
            self.grasp["motion"]["following_error_resume_rad"],
        )

    def test_reported_hamburg_lag_does_not_change_normal_timing(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.states = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.commanded = {"left": [0.0] * 7, "right": [0.1603] + [0.0] * 6}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.following_lag_events = deque(maxlen=30)
        runner._check_all_following()
        self.assertEqual(list(runner.following_lag_events), [])

    def test_hard_or_persistent_following_error_still_aborts(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.states = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.commanded = {"left": [0.0] * 7, "right": [0.25] + [0.0] * 6}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.following_lag_events = deque(maxlen=30)
        with self.assertRaisesRegex(RuntimeError, "hard limit"):
            runner._check_all_following()

        runner.commanded["right"][0] = 0.21
        runner.config["motion"]["following_error_recovery_timeout_s"] = 1.0e-9
        with self.assertRaisesRegex(RuntimeError, "did not recover"):
            runner._check_all_following()
        self.assertEqual(runner.following_lag_events[-1]["status"], "recovery_timeout")

    def test_visual_mapping_points_toward_configured_target(self) -> None:
        point = (340.0, 205.0)
        dx, dy = alignment_delta_m(point, self.grasp)
        jacobian = np.asarray(self.grasp["vision"]["shanghai_base_xy_to_image_uv"])
        target = np.asarray(self.grasp["vision"]["target_right_rim_px"])
        current = np.asarray(point)
        predicted = current + jacobian @ np.asarray([dx, dy])
        self.assertLess(np.linalg.norm(target - predicted), np.linalg.norm(target - current))
        self.assertLessEqual(np.hypot(dx, dy), self.grasp["motion"]["maximum_visual_step_m"] + 1.0e-12)

    def test_gripper_contact_requires_retention_after_lift(self) -> None:
        accepted = gripper_contact_report([0.0, 0.0], [0.008, 0.008], [0.0075, 0.0078], 0.001)
        self.assertTrue(accepted["accepted_as_held"])
        dropped = gripper_contact_report([0.0, 0.0], [0.008, 0.008], [0.0002, 0.0001], 0.001)
        self.assertFalse(dropped["accepted_as_held"])

    def test_entrypoint_plans_are_executable_and_preserve_parameter_scope(self) -> None:
        validate_mission_config(self.mission)
        grasp = grasp_plan(self.grasp, "cup", None)
        reset = pickup_reset_plan(self.grasp)
        mission = mission_plan(self.mission, self.grasp)
        self.assertEqual(grasp["operation"], "Hamburg autonomous observation-grasp-release trial")
        self.assertFalse(reset["base_motion_commanded"])
        self.assertEqual(reset["expected_left_wrist_frame"], {"width": 640, "height": 480})
        self.assertTrue(grasp["single_ros_node_during_motion"])
        self.assertTrue(mission["single_ros_node_during_motion"])
        self.assertEqual(mission["native_interfaces"]["base_state"], "/swerve_drive_controller/odom")
        self.assertIn("room_length_m", mission["hamburg_measurements"])
        self.assertIsNone(mission["hamburg_measurements"]["room_length_m"])

    def test_launcher_routes_physical_modes_instead_of_static_lock(self) -> None:
        launcher = (ROOT / "run_hamburg.sh").read_text(encoding="utf-8")
        self.assertIn('hamburg_pickup_reset.py', launcher)
        self.assertIn('hamburg_grasp_cycle.py', launcher)
        self.assertIn('hamburg_mission.py', launcher)
        self.assertNotIn('physical grasp test is locked', launcher)
        self.assertNotIn('mission is locked', launcher)

    def test_pickup_reset_has_no_base_path_and_orders_the_arm_reset(self) -> None:
        source = (ROOT / "hamburg_pickup_reset.py").read_text(encoding="utf-8")
        self.assertNotIn("NativeBaseControl", source)
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("create_publisher", source)

        events = []

        class Arm:
            def __init__(self):
                self.last_report = {}

            def set_phase(self, _report, phase):
                events.append(("phase", phase))

            def wait_live(self, timeout):
                events.append(("wait_live", timeout))

            def assert_controller_graph(self):
                events.append(("controller_graph",))
                return {"left_joint_target": 1, "right_joint_target": 1,
                        "left_gripper_target": 1}

            def move_to_joint_posture(self, arm, target):
                events.append(("move", arm, list(target)))

            def command_gripper(self, target):
                events.append(("gripper", target))
                return [0.04, 0.04]

            def fresh_wrist_bgr(self):
                events.append(("fresh_wrist",))
                return np.zeros((480, 640, 3), dtype=np.uint8)

            def spin_for(self, duration):
                events.append(("hold", duration))

        class Spine:
            def move_absolute(self, target):
                events.append(("spine", target))
                return {"target_position_m": target, "moved": True}

        with tempfile.TemporaryDirectory() as directory, \
                mock.patch("cv2.imwrite", return_value=True):
            report = run_reset(Arm(), Spine(), self.grasp, Path(directory))

        self.assertEqual(report["status"], "passed")
        self.assertFalse(report["base_motion_commanded"])
        self.assertEqual(report["left_wrist_frame"], {"width": 640, "height": 480})
        ordered_actions = [item[0:2] for item in events if item[0] in
                           {"spine", "move", "gripper", "fresh_wrist"}]
        self.assertEqual(ordered_actions, [
            ("spine", 0.7),
            ("move", "right"),
            ("move", "left"),
            ("gripper", 0.8),
            ("fresh_wrist",),
        ])

    def test_fresh_wrist_frame_skips_bad_post_reset_frames(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.images = deque([(1, object())], maxlen=20)
        runner.image_at = 0.0
        runner.config = self.grasp
        runner.states = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.commanded = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.following_lag_events = deque(maxlen=30)
        calls = 0

        def spin(_duration):
            nonlocal calls
            calls += 1
            runner.images.append((calls + 1, object()))

        runner.spin_for = spin
        expected = np.zeros((480, 640, 3), dtype=np.uint8)
        with mock.patch("hamburg_grasp_cycle.decode_wrist_rgb",
                        side_effect=[ValueError("bad encoding"), expected]):
            actual = runner.fresh_wrist_bgr(timeout_s=0.5)
        self.assertIs(actual, expected)
        self.assertEqual(calls, 2)

    def test_runtime_profiles_must_match_resolved_venue_interfaces(self) -> None:
        venue = load_config(ROOT / "config" / "venue.json")
        validate_cycle_venue_consistency(self.grasp, venue)
        validate_mission_venue_consistency(self.mission, venue)
        changed_grasp = deepcopy(self.grasp)
        changed_grasp["topics"]["left_joint_target"] = "/wrong/arm/target"
        with self.assertRaisesRegex(ValueError, "grasp/venue interface mismatch"):
            validate_cycle_venue_consistency(changed_grasp, venue)
        changed_mission = deepcopy(self.mission)
        changed_mission["topics"]["odometry"] = "/wrong/odom"
        with self.assertRaisesRegex(ValueError, "mission/venue interface mismatch"):
            validate_mission_venue_consistency(changed_mission, venue)

    def test_mission_and_grasp_assignments_cannot_drift(self) -> None:
        validate_combined_configs(self.mission, self.grasp)
        changed = deepcopy(self.mission)
        changed["assignment"]["cup"] = "C"
        with self.assertRaisesRegex(ValueError, "assignments differ"):
            validate_combined_configs(changed, self.grasp)

    def test_explicit_environment_overrides_reach_runtime_configs(self) -> None:
        with mock.patch.dict(os.environ, {
            "TMR_HAMBURG_LEFT_GRIPPER_TOPIC": "/venue/left_gripper",
            "TMR_HAMBURG_GRIPPER_OPEN": "0.9",
            "TMR_HAMBURG_SPINE_HOME_M": "0.65",
            "TMR_HAMBURG_HEAD_CAMERA_WIDTH": "1280",
            "TMR_HAMBURG_HEAD_CAMERA_HEIGHT": "720",
            "TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S": "0.03",
            "TMR_HAMBURG_ARM_MAXIMUM_FOLLOWING_ERROR_RAD": "0.22",
        }):
            venue = load_config(ROOT / "config" / "venue.json")
        grasp = resolve_cycle_venue_overrides(self.grasp, venue)
        mission = resolve_mission_venue_overrides(self.mission, venue)
        self.assertEqual(grasp["topics"]["left_gripper_target"], "/venue/left_gripper")
        self.assertEqual(grasp["gripper"]["open"], 0.9)
        self.assertEqual(grasp["spine"]["initial_target_m"], 0.65)
        self.assertEqual(grasp["motion"]["posture_joint_velocity_rad_s"], 0.03)
        self.assertEqual(grasp["motion"]["maximum_following_error_rad"], 0.22)
        self.assertEqual((mission["head_camera"]["width"], mission["head_camera"]["height"]),
                         (1280, 720))

    def test_full_mission_restores_pickup_view_before_every_object(self) -> None:
        class Arm:
            def __init__(self):
                self.moves = []

            def wait_live(self, _timeout):
                return None

            def assert_controller_graph(self):
                return {}

            def move_to_joint_posture(self, arm, target):
                self.moves.append((arm, list(target)))

        class Base:
            def __init__(self):
                self.active_stage = None
                self.stops = 0

            def wait_ready(self):
                return None

            def run_stages(self, _stages):
                return []

            def search_letter(self, target, _output):
                return {"target": target, "row": "near", "measured_right_m": 0.5}

            def stop(self, _seconds=0.5):
                self.stops += 1

        class Spine:
            def move_absolute(self, target):
                return {"target_position_m": target}

        arm, base = Arm(), Base()
        with tempfile.TemporaryDirectory() as directory:
            runner = HamburgMission(arm, base, Spine(), self.mission, self.grasp, Path(directory))
            runner.pick = mock.Mock(side_effect=lambda name: ({"object": name}, list(self.q)))
            runner.place = mock.Mock(side_effect=lambda _q, row, name: {"object": name, "row": row})
            report = runner.run()
        left_moves = [target for arm_name, target in arm.moves if arm_name == "left"]
        self.assertEqual(len(left_moves), 1 + len(("cup", "bowl", "plate")))
        for target in left_moves:
            self.assertEqual(target, self.grasp["posture"]["left_pick_top_rad"])
        self.assertEqual(report["status"], "complete")

    def test_base_wrappers_publish_stop_when_operation_raises(self) -> None:
        base = NativeBaseControl.__new__(NativeBaseControl)
        base.active_stage = None
        base.stage_history = deque(maxlen=30)
        base.stop = mock.Mock()
        base._search_letter = mock.Mock(side_effect=RuntimeError("camera failed"))
        with self.assertRaisesRegex(RuntimeError, "camera failed"):
            base.search_letter("A", Path("unused.png"))
        base.stop.assert_called_once()

        base.stop.reset_mock()
        base.active_stage = None
        base.stage_history = deque(maxlen=30)
        base.translate = mock.Mock(side_effect=RuntimeError("odometry failed"))
        with self.assertRaisesRegex(RuntimeError, "odometry failed"):
            base.run_stages([{"name": "test", "kind": "translate",
                              "forward_m": 1.0, "left_m": 0.0}])
        base.stop.assert_called_once()
        self.assertEqual(base.stage_history[-1]["status"], "failed")

        base.stop = mock.Mock(side_effect=RuntimeError("zero publish failed"))
        base._search_letter = mock.Mock(side_effect=RuntimeError("original camera failure"))
        with self.assertRaisesRegex(RuntimeError, "original camera failure"):
            base.search_letter("A", Path("unused.png"))
        self.assertEqual(base.stage_history[-1]["status"], "stop_failed")


if __name__ == "__main__":
    unittest.main()
