from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
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
    encode_arm_command,
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

    def test_absolute_targets_are_encoded_for_hamburg_relative_gello(self) -> None:
        robot_zero = [0.81, -0.20, 0.30, -1.80, 0.40, 1.20, -0.60]
        input_zero = list(robot_zero)
        target = [0.61, -0.35, 0.45, -1.60, 0.25, 1.10, -0.80]
        direction = [-1, -1, 1, 1, 1, 1, -1]
        encoded = encode_arm_command(target, robot_zero, input_zero, direction)
        expected = [
            input_zero[i] + direction[i] * (target[i] - robot_zero[i])
            for i in range(7)
        ]
        np.testing.assert_allclose(encoded, expected)
        decoded = [
            robot_zero[i] + direction[i] * (encoded[i] - input_zero[i])
            for i in range(7)
        ]
        np.testing.assert_allclose(decoded, target)

    def test_arm_publisher_keeps_robot_targets_absolute_internally(self) -> None:
        class JointState:
            def __init__(self):
                self.header = mock.Mock()
                self.name = []
                self.position = []

        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.JointState = JointState
        runner.node = mock.Mock()
        runner.node.get_clock.return_value.now.return_value.to_msg.return_value = object()
        runner.arm_publishers = {"left": mock.Mock(), "right": mock.Mock()}
        runner.arm_publish_lock = threading.RLock()
        runner.robot_activation_reference = {
            "left": [0.0] * 7, "right": [0.1] * 7
        }
        runner.input_activation_reference = {
            "left": [0.0] * 7, "right": [0.1] * 7
        }
        runner.commanded = {
            "left": [0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2],
            "right": [0.2] * 7,
        }
        runner.last_published_arm_input = {"left": None, "right": None}
        runner.last_arm_publish_at = 0.0
        runner.arm_publish_count = 0
        runner.publish_arm_holds()
        left_message = runner.arm_publishers["left"].publish.call_args.args[0]
        np.testing.assert_allclose(
            left_message.position,
            [-0.2, 0.2, 0.2, -0.2, 0.2, -0.2, -0.2],
        )
        self.assertEqual(runner.commanded["left"][0], 0.2)
        self.assertEqual(runner.arm_publish_count, 1)

    def test_dedicated_arm_keepalive_covers_main_thread_gaps(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.config["motion"]["publish_rate_hz"] = 200.0
        runner.arm_keepalive_stop = threading.Event()
        runner.arm_keepalive_error = None
        runner.last_arm_publish_at = 0.0
        runner.publish_arm_holds = mock.Mock()
        runner.arm_keepalive_thread = threading.Thread(
            target=runner._arm_keepalive_loop,
            daemon=True,
        )
        runner.arm_keepalive_thread.start()
        deadline = time.monotonic() + 0.2
        while runner.publish_arm_holds.call_count < 3 and time.monotonic() < deadline:
            time.sleep(0.005)
        runner.stop_arm_keepalive()

        self.assertGreaterEqual(runner.publish_arm_holds.call_count, 3)
        self.assertFalse(runner.arm_keepalive_thread.is_alive())
        self.assertIsNone(runner.arm_keepalive_error)

    def test_input_executor_never_waits_for_arm_publication(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.node = object()
        runner.commanded = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.arm_keepalive_error = None
        runner.publish_arm_holds = mock.Mock(side_effect=AssertionError(
            "DDS arm publication must stay off the input executor thread"
        ))
        fake_rclpy = mock.Mock()
        with mock.patch.dict(sys.modules, {"rclpy": fake_rclpy}), \
                mock.patch("hamburg_grasp_cycle.time.monotonic",
                           side_effect=[0.0, 0.0, 0.0, 0.0, 0.0, 0.2]), \
                mock.patch("hamburg_grasp_cycle.time.sleep"):
            runner.spin_for(0.1)

        self.assertEqual(fake_rclpy.spin_once.call_count, 16)
        runner.publish_arm_holds.assert_not_called()

    def test_keepalive_failure_is_never_reported_as_success(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.arm_keepalive_stop = threading.Event()
        runner.arm_keepalive_error = None
        runner.last_arm_publish_at = 0.0
        runner.publish_arm_holds = mock.Mock(side_effect=RuntimeError("publisher failed"))
        runner.arm_keepalive_thread = threading.Thread(
            target=runner._arm_keepalive_loop,
            daemon=True,
        )
        runner.arm_keepalive_thread.start()
        runner.arm_keepalive_thread.join(timeout=0.2)

        with self.assertRaisesRegex(RuntimeError, "arm keepalive failed.*publisher failed"):
            runner.stop_arm_keepalive()

    def test_wait_live_starts_neutral_stream_before_other_inputs_arrive(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.states = {"left": None, "right": None}
        runner.state_times = {"left": 0.0, "right": 0.0}
        runner.gripper_samples = deque(maxlen=10)
        runner.images = deque(maxlen=10)
        runner.commanded = {"left": None, "right": None}
        runner.robot_activation_reference = {"left": None, "right": None}
        runner.input_activation_reference = {"left": None, "right": None}
        runner.neutral_stream_started_at = 0.0
        runner.neutral_initial_pose = None
        runner.neutral_peak_drift = {"left": 0.0, "right": 0.0}
        runner.publish_arm_holds = mock.Mock()
        spin_calls = 0

        def spin(_duration):
            nonlocal spin_calls
            spin_calls += 1
            now = time.monotonic()
            runner.states = {"left": [0.1] * 7, "right": [0.2] * 7}
            runner.state_times = {"left": now, "right": now}
            if spin_calls == 2:
                runner.gripper_samples.append((now, [0.04, 0.04]))
                runner.images.append((1, object()))

        runner.spin_for = spin
        runner.service_input_callbacks = mock.Mock()
        with mock.patch("hamburg_grasp_cycle.decode_wrist_rgb", return_value=object()):
            runner.wait_live(0.5)

        self.assertEqual(spin_calls, 2)
        self.assertGreater(runner.neutral_stream_started_at, 0.0)
        self.assertEqual(runner.input_activation_reference["right"], [0.2] * 7)
        runner.publish_arm_holds.assert_not_called()

    def test_neutral_activation_sync_precedes_motion_and_rejects_drift(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.config["arm_command_interface"]["activation_sync_duration_s"] = 0.002
        runner.states = {"left": [0.1] * 7, "right": [0.2] * 7}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.last_arm_publish_at = 0.0
        runner.arm_publications = {"left": None, "right": None}
        runner.publish_arm_holds = mock.Mock()

        def neutral_spin(duration):
            runner.publish_arm_holds()
            runner.last_arm_publish_at = time.monotonic()
            for arm in ("left", "right"):
                previous = runner.arm_publications[arm]
                runner.arm_publications[arm] = {
                    "first_completed_at": previous["first_completed_at"] if previous else time.monotonic(),
                    "completed_at": time.monotonic(),
                }
            runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
            time.sleep(duration)

        runner.spin_for = neutral_spin
        report = runner.synchronize_arm_command_interface()
        self.assertEqual(report["status"], "neutral_stream_established")
        self.assertEqual(report["direction"], [-1, -1, 1, 1, 1, 1, -1])
        self.assertEqual(report["input_at_activation"]["right"], [0.2] * 7)
        self.assertGreaterEqual(runner.publish_arm_holds.call_count, 1)

        runner.config["arm_command_interface"]["activation_sync_duration_s"] = 0.01
        runner.config["arm_command_interface"]["activation_sync_maximum_drift_rad"] = 0.01
        runner.states = {"left": [0.1] * 7, "right": [0.2] * 7}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}

        def drifting_spin(_duration):
            runner.states["right"][0] += 0.02
            runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}

        runner.spin_for = drifting_spin
        runner.states["right"][0] += 0.02
        with self.assertRaisesRegex(RuntimeError, "controller inactive"):
            runner.synchronize_arm_command_interface()

    def test_slow_startup_decode_refreshes_feedback_before_guard(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.config["motion"]["joint_feedback_stale_timeout_s"] = 0.5
        runner.states = {"left": list(self.q), "right": list(self.q)}
        runner.state_times = {"left": 0.0, "right": 0.0}
        runner.gripper_samples = deque([(1.0, [0.04, 0.04])])
        runner.images = deque([(1, object())])
        clock = [10.0]

        def refresh(*_args):
            runner.state_times = {"left": clock[0], "right": clock[0]}

        def slow_decode(_image):
            clock[0] += 0.65
            return object()

        runner.spin_for = refresh
        runner.service_input_callbacks = mock.Mock(side_effect=refresh)
        with mock.patch("hamburg_grasp_cycle.time.monotonic", side_effect=lambda: clock[0]), \
                mock.patch("hamburg_grasp_cycle.decode_wrist_rgb", side_effect=slow_decode):
            runner.wait_live(1.0)
        runner.service_input_callbacks.assert_called_once()
        self.assertAlmostEqual(runner.startup_wrist_decode_duration_s, 0.65)
        self.assertEqual(runner.state_times["left"], clock[0])

    def test_delayed_publisher_does_not_skip_joint_ramp_points(self) -> None:
        class JointState:
            def __init__(self):
                self.header = mock.Mock()

        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        q = list(self.grasp["posture"]["right_parking_rad"])
        target = list(q)
        target[0] += 0.012
        runner.commanded = {"left": list(q), "right": list(q)}
        runner.states = {"left": list(q), "right": list(q)}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.robot_activation_reference = {"left": list(q), "right": list(q)}
        runner.input_activation_reference = {"left": list(q), "right": list(q)}
        runner.last_published_arm_input = {"left": None, "right": None}
        now = time.monotonic()
        runner.arm_publications = {
            arm: {"robot_target": list(q), "completed_at": now, "first_completed_at": now,
                  "count": 1, "duration_s": 0.0, "maximum_duration_s": 0.0, "maximum_gap_s": 0.0}
            for arm in ("left", "right")
        }
        runner.arm_publish_lock = threading.RLock()
        runner.arm_keepalive_stop = threading.Event()
        runner.arm_keepalive_error = None
        runner.last_arm_publish_at = 0.0
        runner.arm_publish_count = 0
        runner.following_lag_events = deque(maxlen=30)
        runner.JointState = JointState
        runner.node = mock.Mock()
        published = []
        direction = self.grasp["arm_command_interface"]["direction"]

        def slow_publish(arm, message):
            time.sleep(0.015)
            robot = [q[i] + direction[i] * (message.position[i] - q[i]) for i in range(7)]
            runner.states[arm] = robot
            if arm == "right":
                published.append(robot)

        runner.arm_publishers = {
            arm: mock.Mock(publish=lambda message, arm=arm: slow_publish(arm, message))
            for arm in ("left", "right")
        }

        def callbacks():
            runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}

        runner.service_input_callbacks = callbacks
        runner.move_arm_targets = mock.Mock()
        runner.arm_keepalive_thread = threading.Thread(target=runner._arm_keepalive_loop, daemon=True)
        runner.arm_keepalive_thread.start()
        try:
            runner.move_to_joint_posture("right", target)
        finally:
            runner.stop_arm_keepalive()
        expected = smooth_joint_targets(q, target, 50.0, self.grasp["motion"]["posture_joint_velocity_rad_s"])
        for point in expected:
            self.assertTrue(any(np.allclose(point, sent, atol=1e-12, rtol=0) for sent in published))
        actual_steps = np.diff(np.vstack([q, *published]), axis=0)
        self.assertLessEqual(float(np.max(np.abs(actual_steps))),
                             self.grasp["motion"]["posture_joint_velocity_rad_s"] / 50.0 + 1e-12)
        self.assertGreater(runner.arm_publications["right"]["maximum_duration_s"], 0.01)

    def test_missing_publisher_acknowledgement_times_out(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.config["motion"]["joint_feedback_stale_timeout_s"] = 0.01
        runner.arm_publications = {"left": None, "right": None}
        runner.arm_keepalive_error = None
        runner.service_input_callbacks = mock.Mock()
        runner._check_all_following = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, "target publication timed out"):
            runner._wait_for_arm_publication("right", list(self.q))
        self.assertGreater(runner.service_input_callbacks.call_count, 0)

    def test_healthy_publication_adds_no_extra_control_tick(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.commanded = {"left": list(self.q), "right": list(self.q)}
        runner.states = {"left": list(self.q), "right": list(self.q)}
        runner.arm_publications = {"left": None, "right": None}
        runner.arm_keepalive_error = None
        runner._check_all_following = mock.Mock()
        runner.move_arm_targets = mock.Mock()
        periods = []

        def spin(duration):
            periods.append(duration)
            runner.arm_publications["left"] = {"robot_target": list(runner.commanded["left"])}

        runner.spin_for = spin
        target = list(self.q)
        target[0] += 0.012
        with mock.patch("hamburg_grasp_cycle.time.sleep") as sleep:
            runner.move_to_joint_posture("left", target)
        expected = smooth_joint_targets(self.q, target, 50.0,
                                        self.grasp["motion"]["posture_joint_velocity_rad_s"])
        self.assertEqual(periods, [0.02] * len(expected))
        sleep.assert_not_called()

    def test_neutral_sync_requires_publication_for_both_arms(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.config["motion"]["joint_feedback_stale_timeout_s"] = 0.01
        runner.states = {"left": list(self.q), "right": list(self.q)}
        runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}
        runner.arm_publications = {"left": {"first_completed_at": time.monotonic()}, "right": None}

        def refresh(_duration):
            runner.state_times = {"left": time.monotonic(), "right": time.monotonic()}

        runner.spin_for = refresh
        with self.assertRaisesRegex(RuntimeError, "not published for both arms"):
            runner.synchronize_arm_command_interface()

    def test_fresh_static_feedback_does_not_mask_stalled_arm_publication(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        now = time.monotonic()
        runner.states = {"left": list(self.q), "right": list(self.q)}
        runner.commanded = {"left": list(self.q), "right": list(self.q)}
        runner.state_times = {"left": now, "right": now}
        runner.arm_command_sync = {"status": "neutral_stream_established"}
        runner.arm_publications = {
            "left": {"completed_at": now},
            "right": {"completed_at": now - 1.2},
        }
        with self.assertRaisesRegex(RuntimeError, "right arm command publication became stale"):
            runner._check_all_following()

    def test_previous_grasp_config_gains_safe_lag_defaults(self) -> None:
        legacy = json.loads(
            (ROOT / "config" / "grasp-cycle-shanghai-reference.json").read_text(
                encoding="utf-8"
            )
        )
        for key in (
            "posture_joint_velocity_rad_s", "following_error_pause_rad",
            "following_error_resume_rad", "following_error_recovery_timeout_s",
            "joint_feedback_stale_timeout_s",
        ):
            legacy["motion"].pop(key)
        legacy["arm_command_interface"] = {
            "description": "legacy absolute-looking description",
            "message_type": "sensor_msgs/msg/JointState",
            "units": "radian",
            "control_mode": (
                "continuous joint target consumed by the deployed "
                "Franka joint-impedance controller"
            ),
        }
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
        self.assertEqual(
            loaded["arm_command_interface"]["control_mode"],
            "relative_direction_mapped_gello",
        )
        self.assertEqual(
            loaded["arm_command_interface"]["direction"],
            [-1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0],
        )
        self.assertEqual(loaded["motion"]["joint_feedback_stale_timeout_s"], 1.0)

    def test_feedback_age_guard_uses_bounded_venue_threshold(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = deepcopy(self.grasp)
        runner.states = {"left": [0.0] * 7, "right": [0.0] * 7}
        runner.commanded = {"left": [0.0] * 7, "right": [0.0] * 7}
        now = time.monotonic()
        runner.state_times = {"left": now - 0.65, "right": now - 0.65}
        runner._check_following("left")
        runner.state_times["left"] = now - 1.2
        with self.assertRaisesRegex(RuntimeError, r"limit=1\.000s"):
            runner._check_following("left")

        invalid = deepcopy(self.grasp)
        invalid["motion"]["joint_feedback_stale_timeout_s"] = 5.1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid-grasp.json"
            path.write_text(json.dumps(invalid), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must not exceed 5 s"):
                load_cycle_config(path)

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

            def assert_controller_graph(self, *, require_subscribers=True):
                events.append(("controller_graph", require_subscribers))
                return {"left_joint_target": 1, "right_joint_target": 1,
                        "left_gripper_target": 1}

            def synchronize_arm_command_interface(self):
                events.append(("arm_sync",))
                return {"status": "neutral_stream_established"}

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
                           {"arm_sync", "spine", "move", "gripper", "fresh_wrist"}]
        self.assertEqual(ordered_actions, [
            ("arm_sync",),
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
        runner.service_input_callbacks = mock.Mock()
        expected = np.zeros((480, 640, 3), dtype=np.uint8)
        with mock.patch("hamburg_grasp_cycle.decode_wrist_rgb",
                        side_effect=[ValueError("bad encoding"), expected]):
            actual = runner.fresh_wrist_bgr(timeout_s=0.5)
        self.assertIs(actual, expected)
        self.assertEqual(calls, 2)

    def test_wrist_detection_uses_latest_frame_and_services_after_each_decode(self) -> None:
        runner = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        runner.config = self.grasp
        runner.images = deque([(0, 0)], maxlen=20)
        runner._check_all_following = mock.Mock()
        events = []
        stamp = [0]

        def spin(_duration):
            for _ in range(2):
                stamp[0] += 1
                runner.images.append((stamp[0], stamp[0]))

        def detect(_name, bgr):
            events.append(("detect", bgr))
            return (100.0, 200.0)

        runner.spin_for = spin
        runner.service_input_callbacks = lambda: events.append(("service",))
        with mock.patch("hamburg_grasp_cycle.decode_wrist_rgb", side_effect=lambda frame: frame), \
                mock.patch("hamburg_grasp_cycle.detect_object", side_effect=detect):
            point, image = runner.stable_detection("cup", timeout_s=1.0)
        self.assertEqual(point, (100.0, 200.0))
        self.assertEqual(image, 10)
        self.assertEqual(events, [item for frame in (2, 4, 6, 8, 10)
                                  for item in (("detect", frame), ("service",))])

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
            "TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S": "1.5",
        }):
            venue = load_config(ROOT / "config" / "venue.json")
        grasp = resolve_cycle_venue_overrides(self.grasp, venue)
        mission = resolve_mission_venue_overrides(self.mission, venue)
        self.assertEqual(grasp["topics"]["left_gripper_target"], "/venue/left_gripper")
        self.assertEqual(grasp["gripper"]["open"], 0.9)
        self.assertEqual(grasp["spine"]["initial_target_m"], 0.65)
        self.assertEqual(grasp["motion"]["posture_joint_velocity_rad_s"], 0.03)
        self.assertEqual(grasp["motion"]["maximum_following_error_rad"], 0.22)
        self.assertEqual(grasp["motion"]["joint_feedback_stale_timeout_s"], 1.5)
        self.assertEqual((mission["head_camera"]["width"], mission["head_camera"]["height"]),
                         (1280, 720))

    def test_full_mission_restores_pickup_view_before_every_object(self) -> None:
        class Arm:
            def __init__(self):
                self.moves = []

            def wait_live(self, _timeout):
                return None

            def assert_controller_graph(self, *, require_subscribers=True):
                return {}

            def synchronize_arm_command_interface(self):
                return {"status": "neutral_stream_established"}

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
