from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hamburg_grasp_cycle import NativeArmGraspCycle
from hamburg_mission import NativeBaseControl


class CommandOwnerTests(unittest.TestCase):
    def make_runners(self, publishers):
        node = mock.Mock()
        node.get_name.return_value = "runner"
        node.get_namespace.return_value = "/"
        node.get_publishers_info_by_topic.return_value = publishers
        node.get_subscriptions_info_by_topic.return_value = []
        arm = NativeArmGraspCycle.__new__(NativeArmGraspCycle)
        arm.node = node
        arm.config = {"topics": {key: "/" + key for key in (
            "left_joint_target", "right_joint_target", "left_gripper_target",
        )}}
        base = NativeBaseControl.__new__(NativeBaseControl)
        base.node = node
        base.config = {"topics": {"base_command": "/base_command"}}
        return arm, base

    def test_duplicate_names_and_foreign_namespaces_cannot_bypass_owner_guards(self):
        own = SimpleNamespace(node_name="runner", node_namespace="/")
        foreign_namespace = SimpleNamespace(node_name="runner", node_namespace="/other")
        foreign_name = SimpleNamespace(node_name="another_runner", node_namespace="/")
        for publishers in ([own, own], [own, foreign_namespace], [foreign_namespace], [foreign_name]):
            with self.subTest(publishers=publishers):
                arm, base = self.make_runners(publishers)
                with self.assertRaisesRegex(RuntimeError, "competing command publisher"):
                    arm.assert_controller_graph(require_subscribers=False)
                with self.assertRaisesRegex(RuntimeError, "competing base command publisher"):
                    base.wait_ready(timeout_s=0)

    def test_one_local_endpoint_or_pending_discovery_adds_no_new_startup_gate(self):
        own = SimpleNamespace(node_name="runner", node_namespace="/")
        for publishers in ([], [own]):
            with self.subTest(publishers=publishers):
                arm, base = self.make_runners(publishers)
                self.assertEqual(set(arm.assert_controller_graph(require_subscribers=False).values()), {0})
                with self.assertRaisesRegex(RuntimeError, "native odometry, dual LiDAR"):
                    base.wait_ready(timeout_s=0)


if __name__ == "__main__":
    unittest.main()
