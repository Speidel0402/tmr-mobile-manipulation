from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import hamburg_mission as mission
from spine_control import SpineControl


class MissionRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = json.loads(
            (ROOT / "config" / "mission-shanghai-reference.json").read_text(encoding="utf-8")
        )

    def test_base_rejects_arm_failure_before_new_velocity_and_stops(self) -> None:
        for failing_arm in ("left", "right"):
            with self.subTest(arm=failing_arm):
                base = mission.NativeBaseControl.__new__(mission.NativeBaseControl)
                base.config = deepcopy(self.config)
                base.arm = mock.Mock()

                def check(arm):
                    if arm == failing_arm:
                        raise RuntimeError(f"{arm} joint feedback became stale")
                    return 0.0

                base.arm._check_following.side_effect = check
                base.assert_fresh = mock.Mock()
                base.publish = mock.Mock()
                base.stop = mock.Mock()
                base.active_stage = None
                base.stage_history = deque(maxlen=30)
                base.translate = lambda *_: base._tick((0.05, 0.0, 0.0), 0.0)
                with self.assertRaisesRegex(RuntimeError, f"{failing_arm} joint feedback"):
                    base.run_stages([
                        {"name": "route", "kind": "translate", "forward_m": 1.0, "left_m": 0.0}
                    ])
                base.publish.assert_not_called()
                base.stop.assert_called_once()
                base.arm._check_all_following.assert_not_called()
                base.arm.service_input_callbacks.assert_called_once()

    def test_pose_refreshes_pending_inputs_before_staleness_check(self) -> None:
        base = mission.NativeBaseControl.__new__(mission.NativeBaseControl)
        base.config = self.config
        base.pose = (0.0, 0.0, 0.0)
        base.pose_at = 0.0
        base.scans = {}

        def service():
            base.pose = (1.0, 2.0, 0.2)
            base.pose_at = 100.0
            base.scans = {name: (100.0, None) for name in ("front", "rear")}

        base.arm = SimpleNamespace(service_input_callbacks=service)
        with mock.patch.object(mission.time, "monotonic", return_value=100.0):
            self.assertEqual(base._fresh_pose(), (1.0, 2.0, 0.2))

    def test_letter_search_uses_latest_frames_and_refreshes_after_detection(self) -> None:
        base = mission.NativeBaseControl.__new__(mission.NativeBaseControl)
        base.config = deepcopy(self.config)
        search = base.config["letter_search_shanghai_reference"]
        search["minimum_detection_right_m"] = 0.0
        search["post_center_right_m"] = 0.0
        base.head_frames = deque([(stamp, stamp) for stamp in (1, 2, 3)], maxlen=12)
        base.head_at = base.pose_at = 100.0
        base.pose = (0.0, 0.0, 0.0)
        base.scans = {name: (100.0, None) for name in ("front", "rear")}
        events = []
        base.arm = SimpleNamespace(service_input_callbacks=lambda: events.append("service"))
        base.stop = mock.Mock()
        ticks = []

        def tick(_desired, _last):
            ticks.append(True)
            last = base.head_frames[-1][0]
            base.head_frames.extend((stamp, stamp) for stamp in range(last + 1, last + 4))
            return 100.0

        base._tick = tick
        recognized = []

        def detect(frame):
            events.append("detect")
            recognized.append(frame)
            return [SimpleNamespace(letter="A", confidence=0.9, center_x_norm=0.5, row="near")]

        recognizer = SimpleNamespace(detect=detect)
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(mission.time, "monotonic", return_value=100.0), \
                mock.patch.object(mission, "LetterCardRecognizer", return_value=recognizer), \
                mock.patch.object(mission, "decode_raw_bgr", side_effect=lambda frame, *_: frame), \
                mock.patch.object(mission, "annotate", return_value=np.zeros((4, 4, 3), np.uint8)), \
                mock.patch("cv2.imwrite", return_value=True):
            report = base._search_letter("A", Path(directory) / "letter.png")
        self.assertEqual(recognized, [6, 9, 12])
        self.assertEqual(len(ticks), 3)
        self.assertEqual(report["frames_processed"], 3)
        for index, event in enumerate(events):
            if event == "detect":
                self.assertEqual(events[index + 1], "service")

    def test_mission_dry_plan_resolves_interface_and_environment_overrides_without_ros(self) -> None:
        profile = (ROOT / "config" / "interfaces-shanghai.json").read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as directory:
            interface = Path(directory) / "interfaces.json"
            interface.write_text(profile, encoding="utf-8")
            with mock.patch.object(sys, "argv", ["mission", "--interface-config", str(interface)]), \
                    mock.patch.dict(os.environ, {
                        "TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S": "1.25"
                    }, clear=True), \
                    mock.patch.object(mission, "load_config", wraps=mission.load_config) as load, \
                    mock.patch.object(mission, "environment_report") as environment, \
                    mock.patch.object(mission, "write_report") as write:
                self.assertEqual(mission.main(), 0)
        self.assertEqual(load.call_args.args[1], interface)
        environment.assert_not_called()
        self.assertEqual(write.call_args.args[0]["arm_motion"]["joint_feedback_stale_timeout_s"], 1.25)


class SpineCancellationTests(unittest.TestCase):
    def test_missing_acknowledgement_records_unresolved_cancellation_without_resending(self) -> None:
        controller = SpineControl.__new__(SpineControl)
        controller.profile = {"minimum_m": 0.0, "maximum_m": 1.0}
        controller.position = lambda: 0.5
        controller._wait_for_endpoint = lambda *_: True
        controller.action_type = SimpleNamespace(Goal=SimpleNamespace)
        controller.errors = []
        controller.node = mock.Mock()
        controller.action_client = mock.Mock()
        original = RuntimeError("heartbeat failed after send")
        controller._wait = mock.Mock(side_effect=[original, TimeoutError("acknowledgement unavailable")])
        with self.assertRaises(RuntimeError) as raised:
            controller.move_absolute(0.7)
        self.assertIs(raised.exception, original)
        controller.action_client.send_goal_async.assert_called_once()
        self.assertEqual(controller._wait.call_count, 2)
        self.assertEqual(controller._wait.call_args.args[1], 5.0)
        self.assertFalse(controller._wait.call_args.kwargs["service_heartbeat"])
        self.assertIn("acknowledgement unavailable", controller.errors[0])

    def test_acknowledged_goal_is_cancelled_when_heartbeat_or_interrupt_hides_handle(self) -> None:
        for original in (RuntimeError("heartbeat failed after send"), KeyboardInterrupt()):
            with self.subTest(error=type(original).__name__):
                controller = SpineControl.__new__(SpineControl)
                controller.profile = {"minimum_m": 0.0, "maximum_m": 1.0}
                controller.position = lambda: 0.5
                controller._wait_for_endpoint = lambda *_: True
                controller.action_type = SimpleNamespace(Goal=SimpleNamespace)
                controller.errors = []
                controller.node = mock.Mock()

                def completed(value):
                    return SimpleNamespace(done=lambda: True, result=lambda: value)

                handle = mock.Mock(accepted=True)
                handle.cancel_goal_async.return_value = completed(SimpleNamespace(goals_canceling=[1]))
                controller.action_client = SimpleNamespace(
                    wait_for_server=None, send_goal_async=mock.Mock(return_value=completed(handle))
                )
                controller._keep_arm_stream_alive = mock.Mock(side_effect=original)
                with mock.patch.dict(sys.modules, {"rclpy": SimpleNamespace()}):
                    with self.assertRaises(type(original)) as raised:
                        controller.move_absolute(0.7)
                self.assertIs(raised.exception, original)
                controller.action_client.send_goal_async.assert_called_once()
                handle.get_result_async.assert_not_called()
                handle.cancel_goal_async.assert_called_once()
                self.assertEqual(controller.errors, [])

    def test_cancellation_logging_failure_preserves_original_error(self) -> None:
        controller = SpineControl.__new__(SpineControl)
        controller.profile = {"minimum_m": 0.0, "maximum_m": 1.0}
        controller.position = lambda: 0.5
        controller._wait_for_endpoint = lambda *_: True
        controller.action_type = SimpleNamespace(Goal=SimpleNamespace)
        controller.errors = []
        controller.node = mock.Mock()
        controller.node.get_logger().error.side_effect = RuntimeError("logger unavailable")
        handle = mock.Mock(accepted=True)
        controller.action_client = mock.Mock()
        original = RuntimeError("original feedback failure")
        controller._wait = mock.Mock(side_effect=[
            handle, original, RuntimeError("cancel unavailable")
        ])
        with self.assertRaises(RuntimeError) as raised:
            controller.move_absolute(0.7)
        self.assertIs(raised.exception, original)
        self.assertEqual(len(controller.errors), 2)
        self.assertIn("cancel unavailable", controller.errors[0])
        self.assertIn("logger unavailable", controller.errors[1])

    def test_cancellation_refusal_is_reported_without_replacing_original_error(self) -> None:
        controller = SpineControl.__new__(SpineControl)
        controller.profile = {"minimum_m": 0.0, "maximum_m": 1.0}
        controller.position = lambda: 0.5
        controller._wait_for_endpoint = lambda *_: True
        controller.action_type = SimpleNamespace(Goal=SimpleNamespace)
        controller.errors = []
        controller.node = mock.Mock()
        handle = mock.Mock(accepted=True)
        controller.action_client = mock.Mock()
        original = RuntimeError("original feedback failure")
        controller._wait = mock.Mock(side_effect=[
            handle, original, SimpleNamespace(goals_canceling=[])
        ])
        with self.assertRaises(RuntimeError) as raised:
            controller.move_absolute(0.7)
        self.assertIs(raised.exception, original)
        handle.cancel_goal_async.assert_called_once()
        self.assertIn("cancellation was not accepted", controller.errors[0])
        controller.node.get_logger().error.assert_called_once()

    def test_accepted_goal_cancelled_on_all_wait_failures_even_if_heartbeat_failed(self) -> None:
        for original in (RuntimeError("publisher failed"), KeyboardInterrupt(), TimeoutError("deadline")):
            with self.subTest(error=type(original).__name__):
                controller = SpineControl.__new__(SpineControl)
                controller.profile = {"minimum_m": 0.0, "maximum_m": 1.0}
                controller.position = lambda: 0.5
                controller._wait_for_endpoint = lambda *_: True
                controller.action_type = SimpleNamespace(Goal=SimpleNamespace)
                controller.errors = []
                controller.node = mock.Mock()
                result_started = []

                def completed(value):
                    return SimpleNamespace(done=lambda: True, result=lambda: value)

                def result_future():
                    result_started.append(True)
                    return SimpleNamespace(done=lambda: False)

                def heartbeat():
                    if result_started:
                        raise original

                controller._keep_arm_stream_alive = heartbeat
                handle = SimpleNamespace(
                    accepted=True,
                    get_result_async=result_future,
                    cancel_goal_async=mock.Mock(return_value=completed(SimpleNamespace(goals_canceling=[1]))),
                )
                controller.action_client = SimpleNamespace(
                    wait_for_server=None, send_goal_async=lambda _: completed(handle)
                )
                with mock.patch.dict(sys.modules, {"rclpy": SimpleNamespace()}):
                    with self.assertRaises(type(original)) as raised:
                        controller.move_absolute(0.7)
                self.assertIs(raised.exception, original)
                handle.cancel_goal_async.assert_called_once()
                self.assertEqual(controller.errors, [])


if __name__ == "__main__":
    unittest.main()
