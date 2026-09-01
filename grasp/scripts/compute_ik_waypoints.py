"""Compute a continuous sequence of MoveIt IK waypoints; never commands a robot."""

from __future__ import annotations

import argparse
import json
import math

import rclpy
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node


JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]


def floats(text: str, count: int) -> list[float]:
    values = [float(value) for value in text.split(",")]
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(f"expected {count} finite comma-separated values")
    return values


def normalized(q: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    return [value / norm for value in q]


def slerp(a: list[float], b: list[float], t: float) -> list[float]:
    a, b = normalized(a), normalized(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b, dot = [-value for value in b], -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalized([(1.0 - t) * x + t * y for x, y in zip(a, b)])
    theta = math.acos(dot)
    scale = math.sin(theta)
    return [
        math.sin((1.0 - t) * theta) / scale * x
        + math.sin(t * theta) / scale * y
        for x, y in zip(a, b)
    ]


class Solver(Node):
    def __init__(self, service: str):
        super().__init__("left_ik_waypoint_checker")
        self.client = self.create_client(GetPositionIK, service)

    def solve(self, args, seed, position, quaternion):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = args.group
        ik.ik_link_name = args.link
        ik.pose_stamped.header.frame_id = args.frame
        ik.pose_stamped.pose.position.x = position[0]
        ik.pose_stamped.pose.position.y = position[1]
        ik.pose_stamped.pose.position.z = position[2]
        ik.pose_stamped.pose.orientation.x = quaternion[0]
        ik.pose_stamped.pose.orientation.y = quaternion[1]
        ik.pose_stamped.pose.orientation.z = quaternion[2]
        ik.pose_stamped.pose.orientation.w = quaternion[3]
        ik.robot_state.joint_state.name = JOINT_NAMES
        ik.robot_state.joint_state.position = seed
        ik.robot_state.is_diff = True
        ik.avoid_collisions = False
        ik.timeout.sec = 1
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("compute_ik_timeout")
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError(f"compute_ik_error:{response.error_code.val}")
        names = response.solution.joint_state.name
        values = response.solution.joint_state.position
        mapped = dict(zip(names, values))
        return [float(mapped[name]) for name in JOINT_NAMES]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", default="/left_ik/compute_ik")
    parser.add_argument("--group", default="left_arm")
    parser.add_argument("--link", default="left_fr3v2_link8")
    parser.add_argument("--frame", default="left_fr3v2_link0")
    parser.add_argument("--seed", required=True)
    parser.add_argument("--start-position", required=True)
    parser.add_argument("--start-quaternion", required=True)
    parser.add_argument("--target-position", required=True)
    parser.add_argument("--target-quaternion", required=True)
    parser.add_argument("--steps", type=int, default=12)
    args = parser.parse_args()
    args.seed = floats(args.seed, 7)
    args.start_position = floats(args.start_position, 3)
    args.start_quaternion = floats(args.start_quaternion, 4)
    args.target_position = floats(args.target_position, 3)
    args.target_quaternion = floats(args.target_quaternion, 4)
    if args.steps < 2:
        parser.error("--steps must be at least 2")

    rclpy.init()
    node = Solver(args.service)
    try:
        if not node.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("compute_ik_service_unavailable")
        seed = args.seed
        waypoints = []
        maximum_step = 0.0
        for index in range(1, args.steps + 1):
            t = index / args.steps
            position = [
                (1.0 - t) * a + t * b
                for a, b in zip(args.start_position, args.target_position)
            ]
            quaternion = slerp(args.start_quaternion, args.target_quaternion, t)
            solution = node.solve(args, seed, position, quaternion)
            step_delta = max(abs(a - b) for a, b in zip(solution, seed))
            maximum_step = max(maximum_step, step_delta)
            waypoints.append({
                "index": index,
                "position": position,
                "quaternion_xyzw": quaternion,
                "joint_positions_rad": solution,
                "maximum_joint_step_rad": step_delta,
            })
            seed = solution
        print(json.dumps({
            "valid": True,
            "steps": args.steps,
            "maximum_joint_step_rad": maximum_step,
            "waypoints": waypoints,
            "semantics": "IK continuity check only; no controller command was sent",
        }, separators=(",", ":")))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
