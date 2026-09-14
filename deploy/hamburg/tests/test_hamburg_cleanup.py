from __future__ import annotations

from contextlib import ExitStack
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import hamburg_grasp_cycle as grasp
import hamburg_mission as mission


class CleanupTests(unittest.TestCase):
    def test_diagnostic_failures_preserve_primary_error_and_all_cleanup(self):
        for module in (grasp, mission):
            with self.subTest(entrypoint=module.__name__), ExitStack() as stack:
                node = mock.Mock()
                ros = SimpleNamespace(init=mock.Mock(), shutdown=mock.Mock())
                stack.enter_context(mock.patch.dict(sys.modules, {
                    "rclpy": ros, "rclpy.node": SimpleNamespace(Node=lambda _: node)
                }))
                stack.enter_context(mock.patch.dict(os.environ, {}, clear=True))
                args = [module.__name__, "--execute"]
                if module is grasp:
                    args += ["--object", "cup"]
                stack.enter_context(mock.patch.object(sys, "argv", args))
                stack.enter_context(mock.patch.object(module, "environment_report", return_value=({}, [])))
                arm = mock.Mock()
                arm.last_report = {"active_phase": "test_motion", "motion_commanded": True}
                arm.run.side_effect = RuntimeError("original motion failure")
                arm.diagnostic_snapshot.side_effect = RuntimeError("arm diagnostic failed")
                spine = mock.Mock()
                spine.diagnostic_snapshot.side_effect = RuntimeError("spine diagnostic failed")
                stack.enter_context(mock.patch.object(module, "NativeArmGraspCycle", return_value=arm))
                stack.enter_context(mock.patch.object(module, "SpineControl", return_value=spine))
                if module is mission:
                    base = mock.Mock(active_stage=None)
                    base.diagnostic_snapshot.side_effect = RuntimeError("base diagnostic failed")
                    runner = mock.Mock(report=dict(arm.last_report), arm=arm)
                    runner.run.side_effect = RuntimeError("original motion failure")
                    stack.enter_context(mock.patch.object(module, "NativeBaseControl", return_value=base))
                    stack.enter_context(mock.patch.object(module, "HamburgMission", return_value=runner))
                write = stack.enter_context(mock.patch.object(module, "write_report"))
                self.assertEqual(module.main(), 2)
                report = write.call_args.args[0]
                self.assertIn("original motion failure", report["errors"][0])
                self.assertEqual(report["failed_phase"], "test_motion")
                self.assertEqual(len(report["diagnostic_errors"]), 3 if module is mission else 2)
                arm.stop_arm_keepalive.assert_called_once()
                spine.close.assert_called_once()
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()
                if module is mission:
                    base.stop.assert_called_once_with(1.0)
                    runner.persist.assert_called_once()


if __name__ == "__main__":
    unittest.main()
