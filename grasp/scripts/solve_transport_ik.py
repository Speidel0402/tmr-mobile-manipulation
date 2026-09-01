#!/usr/bin/env python3
"""Solve a continuous IK path to one arm's raised transport pose; no motion."""

from __future__ import annotations

import argparse
import json
import math

import rclpy
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node


def vector(text: str, size: int) -> list[float]:
    values = [float(item) for item in text.split(",")]
    if len(values) != size or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError(f"expected {size} finite comma-separated values")
    return values


def slerp(first: list[float], second: list[float], fraction: float) -> list[float]:
    def normalize(values):
        norm = math.sqrt(sum(value * value for value in values))
        return [value / norm for value in values]

    first, second = normalize(first), normalize(second)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second, dot = [-value for value in second], -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize([(1.0 - fraction) * a + fraction * b for a, b in zip(first, second)])
    angle = math.acos(dot)
    scale = math.sin(angle)
    return [
        (math.sin((1.0 - fraction) * angle) * a + math.sin(fraction * angle) * b) / scale
        for a, b in zip(first, second)
    ]


class Solver(Node):
    def __init__(self, service: str):
        super().__init__("transport_pose_ik_solver")
        self.client = self.create_client(GetPositionIK, service)

    def fk(self, service: str, frame: str, link: str, names, joints):
        client = self.create_client(GetPositionFK, service)
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("compute_fk_service_unavailable")
        request = GetPositionFK.Request()
        request.header.frame_id = frame
        request.fk_link_names = [link]
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = joints
        request.robot_state.is_diff = True
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result() if future.done() else None
        if response is None or not response.pose_stamped:
            raise RuntimeError("compute_fk_failed")
        return response.pose_stamped[0].pose

    def solve(self, args, names, seed, position, quaternion):
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
        ik.robot_state.joint_state.name = names
        ik.robot_state.joint_state.position = seed
        ik.robot_state.is_diff = True
        ik.avoid_collisions = False
        ik.timeout.sec = 1
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result() if future.done() else None
        if response is None:
            raise RuntimeError("compute_ik_timeout")
        if response.error_code.val != 1:
            raise RuntimeError(f"compute_ik_error:{response.error_code.val}")
        mapped = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        return [float(mapped[name]) for name in names]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--service", default="/left_ik/compute_ik")
    parser.add_argument("--frame")
    parser.add_argument("--seed", type=lambda value: vector(value, 7), required=True)
    parser.add_argument("--start", type=lambda value: vector(value, 3))
    parser.add_argument("--target", type=lambda value: vector(value, 3), required=True)
    parser.add_argument("--quaternion", type=lambda value: vector(value, 4))
    parser.add_argument("--target-quaternion", type=lambda value: vector(value, 4))
    parser.add_argument("--steps", type=int, default=16)
    parser.add_argument("--fk-only", action="store_true")
    args = parser.parse_args()
    args.group = f"{args.arm}_arm"
    args.link = f"{args.arm}_fr3v2_link8"
    args.frame = args.frame or f"{args.arm}_fr3v2_link0"
    names = [f"{args.arm}_fr3v2_joint{index}" for index in range(1, 8)]

    rclpy.init()
    node = Solver(args.service)
    try:
        if not node.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("compute_ik_service_unavailable")
        initial_pose = node.fk(
            args.service.replace("compute_ik", "compute_fk"),
            args.frame,
            args.link,
            names,
            args.seed,
        )
        if args.start is None:
            args.start = [
                initial_pose.position.x,
                initial_pose.position.y,
                initial_pose.position.z,
            ]
        if args.quaternion is None:
            args.quaternion = [
                initial_pose.orientation.x,
                initial_pose.orientation.y,
                initial_pose.orientation.z,
                initial_pose.orientation.w,
            ]
        if args.fk_only:
            print(json.dumps({
                "arm": args.arm,
                "joint_positions_rad": args.seed,
                "position": args.start,
                "quaternion_xyzw": args.quaternion,
            }, separators=(",", ":")))
            return
        seed = list(args.seed)
        target_quaternion = args.target_quaternion or args.quaternion
        waypoints = []
        maximum_step = 0.0
        for index in range(1, args.steps + 1):
            fraction = index / args.steps
            position = [
                start + fraction * (target - start)
                for start, target in zip(args.start, args.target)
            ]
            quaternion = slerp(args.quaternion, target_quaternion, fraction)
            solution = node.solve(args, names, seed, position, quaternion)
            step = max(abs(value - previous) for value, previous in zip(solution, seed))
            maximum_step = max(maximum_step, step)
            waypoints.append({"position": position, "joint_positions_rad": solution, "step_rad": step})
            seed = solution
        print(json.dumps({
            "arm": args.arm,
            "valid": True,
            "maximum_joint_step_rad": maximum_step,
            "start_position": args.start,
            "start_quaternion_xyzw": args.quaternion,
            "target_joint_positions_rad": seed,
            "waypoints": waypoints,
        }, separators=(",", ":")))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
