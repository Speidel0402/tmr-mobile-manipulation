#!/usr/bin/env python3
"""Move the left FR3 vertically in the MoveIt base frame while locking XY/orientation."""

from __future__ import annotations

import argparse
import json

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath

from servo_cup_edge_xy import JOINT_NAMES, ServoNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--delta-z", type=float, required=True)
    parser.add_argument("--max-step-m", type=float, default=0.03)
    parser.add_argument("--joint-velocity", type=float, default=0.10)
    args = parser.parse_args()
    if not -0.35 <= args.delta_z <= 0.35:
        raise RuntimeError("delta-z outside 0.35 m limit")

    rclpy.init()
    node = ServoNode()
    try:
        node.wait_ready()
        node.max_joint_velocity = float(args.joint_velocity)
        q0 = list(node.q)
        start = node.fk(q0)
        target = Pose()
        target.position.x = start.position.x
        target.position.y = start.position.y
        target.position.z = start.position.z + args.delta_z
        target.orientation = start.orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = q0
        request.start_state.is_diff = True
        request.group_name = "left_arm"
        request.link_name = "left_fr3v2_link8"
        request.waypoints = [target]
        request.max_step = float(args.max_step_m)
        request.revolute_jump_threshold = 0.12
        request.avoid_collisions = True
        response = node.call(node.cartesian_client, request)
        path = [list(map(float, p.positions)) for p in response.solution.joint_trajectory.points]
        if response.error_code.val != 1 or response.fraction < 0.999 or not path:
            raise RuntimeError(
                f"vertical path invalid: fraction={response.fraction:.4f}, code={response.error_code.val}"
            )
        for q in path:
            node.move_ptp(q)
        end = node.fk(list(node.q))
        print(json.dumps({
            "start_joint_positions_rad": q0,
            "start_position_m": [start.position.x, start.position.y, start.position.z],
            "start_orientation_xyzw": [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w],
            "requested_delta_z_m": args.delta_z,
            "target_position_m": [target.position.x, target.position.y, target.position.z],
            "cartesian_fraction": float(response.fraction),
            "cartesian_waypoint_count": len(path),
            "end_position_m": [end.position.x, end.position.y, end.position.z],
            "end_orientation_xyzw": [end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w],
            "actual_delta_m": [end.position.x-start.position.x, end.position.y-start.position.y, end.position.z-start.position.z],
            "gripper_commanded": False,
        }, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
