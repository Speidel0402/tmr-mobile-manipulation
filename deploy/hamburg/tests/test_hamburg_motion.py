from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hamburg_grasp_cycle import alignment_delta_m, load_cycle_config, plan_report as grasp_plan
from hamburg_mission import plan_report as mission_plan, validate_mission_config
from hamburg_motion_core import (
    FR3V2_JOINT_LIMITS_RAD,
    base_relative_pose,
    cartesian_waypoints,
    derive_mount_rotation,
    gripper_contact_report,
    smooth_joint_targets,
)


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
        mission = mission_plan(self.mission, self.grasp)
        self.assertEqual(grasp["operation"], "Hamburg autonomous observation-grasp-release trial")
        self.assertTrue(grasp["single_ros_node_during_motion"])
        self.assertTrue(mission["single_ros_node_during_motion"])
        self.assertEqual(mission["native_interfaces"]["base_state"], "/swerve_drive_controller/odom")
        self.assertIn("room_length_m", mission["hamburg_measurements"])
        self.assertIsNone(mission["hamburg_measurements"]["room_length_m"])

    def test_launcher_routes_physical_modes_instead_of_static_lock(self) -> None:
        launcher = (ROOT / "run_hamburg.sh").read_text(encoding="utf-8")
        self.assertIn('hamburg_grasp_cycle.py', launcher)
        self.assertIn('hamburg_mission.py', launcher)
        self.assertNotIn('physical grasp test is locked', launcher)
        self.assertNotIn('mission is locked', launcher)


if __name__ == "__main__":
    unittest.main()
