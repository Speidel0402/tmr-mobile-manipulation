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
    parser.add_argument("--order", choices=("right,left", "left,right"), default="right,left")
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
        left_target = left["target_joint_positions_rad"]
        right_target = right["target_joint_positions_rad"]
        order = args.order.split(",")
        stationary = {
            "left": list(args.left_start),
            "right": list(args.right_start),
        }
        plans = {"left": left, "right": right}
        targets = {"left": left_target, "right": right_target}
        for arm in order:
            for index, waypoint in enumerate(plans[arm].get("waypoints", []), 1):
                moving = waypoint["joint_positions_rad"]
                left_joints = moving if arm == "left" else stationary["left"]
                right_joints = moving if arm == "right" else stationary["right"]
                check(f"{arm}_ik", index, left_joints, right_joints)
            stationary[arm] = targets[arm]

        samples = 80
        stationary = {
            "left": list(args.left_start),
            "right": list(args.right_start),
        }
        starts = {"left": args.left_start, "right": args.right_start}
        for arm in order:
            for index in range(1, samples + 1):
                fraction = index / samples
                moving = [
                    start + fraction * (target - start)
                    for start, target in zip(starts[arm], targets[arm])
                ]
                left_joints = moving if arm == "left" else stationary["left"]
                right_joints = moving if arm == "right" else stationary["right"]
                check(f"{arm}_ptp", index, left_joints, right_joints)
            stationary[arm] = targets[arm]

        print(json.dumps({
            "valid": not invalid,
            "order": order,
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
