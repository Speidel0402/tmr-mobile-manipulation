from __future__ import annotations

import json
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .ik_contract import (
    LEFT_JOINT_NAMES,
    moveit_error_name,
    normalized_quaternion_xyzw,
    ordered_joint_values,
)


class LeftIkClient(Node):
    """Compute left FR3 IK only; this node never commands a controller."""

    def __init__(self) -> None:
        super().__init__("left_ik_client")
        self.declare_parameter("ik_service", "/left_ik/compute_ik")
        self.declare_parameter(
            "joint_state_topic",
            "/left/franka_robot_state_broadcaster/measured_joint_states",
        )
        self.declare_parameter("moveit_joint_state_topic", "/left_ik/joint_states")
        self.declare_parameter("target_pose_topic", "/rim_grasp/left/contact_pose")
        self.declare_parameter("joint_target_topic", "/rim_grasp/left/ik_joint_target")
        self.declare_parameter("result_topic", "/rim_grasp/left/ik_result_json")
        self.declare_parameter("group_name", "left_arm")
        self.declare_parameter("ik_link_name", "left_fr3v2_link8")
        self.declare_parameter("timeout_s", 0.20)
        self.declare_parameter("avoid_collisions", False)

        self.group_name = str(self.get_parameter("group_name").value)
        self.ik_link_name = str(self.get_parameter("ik_link_name").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.avoid_collisions = bool(self.get_parameter("avoid_collisions").value)
        self._seed: Optional[list[float]] = None
        self._pending = None
        self._pending_pose: Optional[PoseStamped] = None

        self.ik_client = self.create_client(
            GetPositionIK, str(self.get_parameter("ik_service").value)
        )
        self.joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_target_topic").value), 10
        )
        self.result_pub = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        # The robot publishes BEST_EFFORT, while MoveIt's CurrentStateMonitor
        # subscribes RELIABLE. Republish read-only state with compatible QoS.
        self.moveit_joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("moveit_joint_state_topic").value),
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            self._on_target_pose,
            10,
        )
        self.get_logger().info(
            "IK-only client ready; output topic is not connected to a controller"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self.moveit_joint_state_pub.publish(msg)
        try:
            self._seed = ordered_joint_values(msg.name, msg.position)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring incomplete joint seed: {exc}")

    def _publish_result(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.result_pub.publish(msg)

    def _on_target_pose(self, pose: PoseStamped) -> None:
        if self._pending is not None:
            self._publish_result({
                "valid": False,
                "invalid_reason": "ik_request_already_in_progress",
                "frame_id": pose.header.frame_id,
            })
            return
        if self._seed is None:
            self._publish_result({
                "valid": False,
                "invalid_reason": "left_joint_seed_unavailable",
                "frame_id": pose.header.frame_id,
            })
            return
        if not pose.header.frame_id:
            self._publish_result({
                "valid": False,
                "invalid_reason": "target_pose_frame_is_empty",
                "frame_id": "",
            })
            return
        try:
            q = normalized_quaternion_xyzw([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ])
        except ValueError as exc:
            self._publish_result({
                "valid": False,
                "invalid_reason": f"invalid_target_orientation:{exc}",
                "frame_id": pose.header.frame_id,
            })
            return
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]

        if not self.ik_client.service_is_ready():
            self._publish_result({
                "valid": False,
                "invalid_reason": "moveit_compute_ik_service_unavailable",
                "frame_id": pose.header.frame_id,
            })
            return

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.group_name
        ik.ik_link_name = self.ik_link_name
        ik.pose_stamped = pose
        ik.robot_state.joint_state.name = list(LEFT_JOINT_NAMES)
        ik.robot_state.joint_state.position = list(self._seed)
        ik.robot_state.is_diff = True
        ik.avoid_collisions = self.avoid_collisions
        seconds = max(0.001, self.timeout_s)
        ik.timeout.sec = int(seconds)
        ik.timeout.nanosec = int((seconds-int(seconds))*1e9)

        self._pending_pose = pose
        self._pending = self.ik_client.call_async(request)
        self._pending.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, future) -> None:
        pose = self._pending_pose
        self._pending = None
        self._pending_pose = None
        try:
            response = future.result()
        except Exception as exc:  # rclpy service exceptions are runtime-specific
            self._publish_result({
                "valid": False,
                "invalid_reason": f"compute_ik_call_failed:{exc}",
                "frame_id": "" if pose is None else pose.header.frame_id,
            })
            return

        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            self._publish_result({
                "valid": False,
                "invalid_reason": moveit_error_name(error_code),
                "moveit_error_code": error_code,
                "frame_id": "" if pose is None else pose.header.frame_id,
            })
            return
        solution = response.solution.joint_state
        try:
            positions = ordered_joint_values(solution.name, solution.position)
        except ValueError as exc:
            self._publish_result({
                "valid": False,
                "invalid_reason": f"invalid_ik_solution:{exc}",
                "moveit_error_code": error_code,
            })
            return

        joint_target = JointState()
        joint_target.header.stamp = self.get_clock().now().to_msg()
        joint_target.name = list(LEFT_JOINT_NAMES)
        joint_target.position = positions
        self.joint_pub.publish(joint_target)
        self._publish_result({
            "valid": True,
            "invalid_reason": "",
            "group_name": self.group_name,
            "ik_link_name": self.ik_link_name,
            "target_frame_id": "" if pose is None else pose.header.frame_id,
            "joint_names": list(LEFT_JOINT_NAMES),
            "joint_positions_rad": positions,
            "seed_joint_positions_rad": self._seed,
            "output_semantics": "IK result only; not sent to any robot controller",
        })


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeftIkClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
