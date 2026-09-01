#!/usr/bin/env python3
"""Check both sequential transport paths against the MoveIt collision model."""

import argparse
import json

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node


def joints(text):
    values = [float(item) for item in text.split(",")]
    if len(values) != 7:
        raise argparse.ArgumentTypeError("expected seven comma-separated joints")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-plan", required=True)
    parser.add_argument("--right-plan", required=True)
    parser.add_argument("--left-start", type=joints, required=True)
    parser.add_argument("--right-start", type=joints, required=True)
    parser.add_argument("--service", default="/left_ik/check_state_validity")
    args = parser.parse_args()
    left = json.load(open(args.left_plan, encoding="utf-8"))
    right = json.load(open(args.right_plan, encoding="utf-8"))
    left_names = [f"left_fr3v2_joint{index}" for index in range(1, 8)]
    right_names = [f"right_fr3v2_joint{index}" for index in range(1, 8)]

    rclpy.init()
    node = Node("dual_arm_transport_validity")
    client = node.create_client(GetStateValidity, args.service)
    try:
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("state_validity_service_unavailable")

        invalid = []

        def check(label, index, left_joints, right_joints):
            request = GetStateValidity.Request()
            request.group_name = "full_body"
            request.robot_state.joint_state.name = left_names + right_names
            request.robot_state.joint_state.position = left_joints + right_joints
            request.robot_state.is_diff = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            response = future.result() if future.done() else None
            if response is None:
                raise RuntimeError("state_validity_timeout")
            if not response.valid:
                invalid.append({
                    "phase": label,
                    "index": index,
                    "contacts": [
                        [contact.contact_body_1, contact.contact_body_2]
                        for contact in response.contacts
                    ],
                })

        check("start", 0, args.left_start, args.right_start)
        for index, waypoint in enumerate(left["waypoints"], 1):
            check("left_raise", index, waypoint["joint_positions_rad"], args.right_start)
        left_target = left["target_joint_positions_rad"]
        for index, waypoint in enumerate(right["waypoints"], 1):
            check("right_raise", index, left_target, waypoint["joint_positions_rad"])
        right_target = right["target_joint_positions_rad"]
        samples = 80
        for index in range(1, samples + 1):
            fraction = index / samples
            left_joints = [
                start + fraction * (target - start)
                for start, target in zip(args.left_start, left_target)
            ]
            check("left_ptp", index, left_joints, args.right_start)
        for index in range(1, samples + 1):
            fraction = index / samples
            right_joints = [
                start + fraction * (target - start)
                for start, target in zip(args.right_start, right_target)
            ]
            check("right_ptp", index, left_target, right_joints)

        print(json.dumps({
            "valid": not invalid,
            "checked_states": 1 + len(left["waypoints"]) + len(right["waypoints"]) + 2 * samples,
            "invalid_states": invalid,
        }, separators=(",", ":")))
        if invalid:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
