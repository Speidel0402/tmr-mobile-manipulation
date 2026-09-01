#!/usr/bin/env python3
"""Restore the proven left-arm top pose, then descend in base Z."""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from franka_msgs.action import PTPMotion
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath

from servo_cup_edge_xy import JOINT_NAMES, ServoNode


REFERENCE_JOINTS = np.asarray(
    [
        -1.71976900100708,
        -1.6329213380813599,
        1.8240526914596558,
        -2.447446823120117,
        2.177191972732544,
        0.8496646285057068,
        -3.05077862739563,
    ],
    dtype=float,
)


def fresh_state(node: ServoNode, timeout: float = 1.5) -> None:
    node.q = None
    node.robot_state = None
    deadline = time.monotonic() + timeout
    while (node.q is None or node.robot_state is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.q is None or node.robot_state is None:
        raise RuntimeError("left state streams stopped")


def restore_stable_hold(node: ServoNode) -> int:
    """Recover only after the final PTP teardown and prove the streams stay live."""
    time.sleep(2.0)
    last_error = None
    for attempt in range(1, 4):
        try:
            fresh_state(node, timeout=0.8)
        except Exception:
            pass
        try:
            node.ensure_runtime_ready()
            # Re-sample twice.  This catches the late PTP teardown that used to
            # occur after ensure_runtime_ready() had already returned.
            fresh_state(node)
            time.sleep(0.8)
            fresh_state(node)
            return attempt
        except Exception as exc:
            last_error = exc
            time.sleep(1.0)
    raise RuntimeError(f"stable hold recovery failed: {last_error!r}")


def quaternion_error_deg(a, b) -> float:
    qa = np.asarray([a.x, a.y, a.z, a.w], dtype=float)
    qb = np.asarray([b.x, b.y, b.z, b.w], dtype=float)
    qa /= np.linalg.norm(qa)
    qb /= np.linalg.norm(qb)
    return math.degrees(2.0 * math.acos(float(np.clip(abs(np.dot(qa, qb)), -1.0, 1.0))))


def move_ptp_precise(node: ServoNode, q) -> None:
    """Use a tight endpoint only for the final small Cartesian correction."""
    goal = PTPMotion.Goal()
    goal.goal_joint_configuration = list(map(float, q))
    goal.maximum_joint_velocities = [0.04] * 7
    goal.goal_tolerance = 0.0008
    future = node.arm.send_goal_async(goal)
    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("precise PTP goal rejected")
    result_future = handle.get_result_async()
    deadline = time.monotonic() + 30.0
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        node.gate()
        if time.monotonic() >= deadline:
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
            break
    for _ in range(8):
        rclpy.spin_once(node, timeout_sec=0.04)
    measured_error = max(
        abs(float(actual) - float(target))
        for actual, target in zip(node.q, q)
    )
    if measured_error > 0.0018:
        raise RuntimeError(f"precise PTP endpoint error {measured_error:.6f} rad")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--down-m", type=float, default=0.24)
    args = parser.parse_args()
    if not 0.02 <= args.down_m <= 0.35:
        raise RuntimeError("down-m outside 0.02..0.35 m")

    rclpy.init()
    arm = ServoNode()
    impedance_off = False
    hold_restored = False
    try:
        arm.wait_ready()
        arm.set_impedance(False)
        impedance_off = True
        time.sleep(0.35)

        # Restore the complete proven joint posture, not just TCP height.
        arm.max_joint_velocity = 0.06
        arm.move_ptp(REFERENCE_JOINTS.tolist())
        for _ in range(8):
            rclpy.spin_once(arm, timeout_sec=0.04)
        restored_q = np.asarray(arm.q, dtype=float)
        restore_error = float(np.max(np.abs(restored_q - REFERENCE_JOINTS)))
        if restore_error > 0.008:
            raise RuntimeError(f"reference posture error {restore_error:.6f} rad")
        top = arm.fk(restored_q.tolist())

        target = Pose()
        target.position.x = top.position.x
        target.position.y = top.position.y
        target.position.z = top.position.z - float(args.down_m)
        target.orientation = top.orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = restored_q.tolist()
        request.start_state.is_diff = True
        request.group_name = "left_arm"
        request.link_name = "left_fr3v2_link8"
        request.waypoints = [target]
        request.max_step = 0.030
        request.revolute_jump_threshold = 0.12
        request.avoid_collisions = True
        response = arm.call(arm.cartesian_client, request)
        path = [list(map(float, point.positions)) for point in response.solution.joint_trajectory.points]
        if response.error_code.val != 1 or response.fraction < 0.999 or not path:
            raise RuntimeError(
                f"descent path invalid fraction={response.fraction:.4f}, code={response.error_code.val}"
            )

        arm.max_joint_velocity = 0.08
        for q in path:
            arm.move_ptp(q)
        for _ in range(8):
            rclpy.spin_once(arm, timeout_sec=0.04)
        before_hold = arm.fk(list(arm.q))

        correction_history = []
        for correction_attempt in range(1, 3):
            residual = float(before_hold.position.z - target.position.z)
            if abs(residual) <= 0.006:
                break
            if abs(residual) > 0.030:
                raise RuntimeError(f"descent residual outside correction range {residual:+.6f} m")
            correction_request = GetCartesianPath.Request()
            correction_request.header.frame_id = "base"
            correction_request.start_state.joint_state.name = JOINT_NAMES
            correction_request.start_state.joint_state.position = list(arm.q)
            correction_request.start_state.is_diff = True
            correction_request.group_name = "left_arm"
            correction_request.link_name = "left_fr3v2_link8"
            correction_request.waypoints = [target]
            correction_request.max_step = 0.008
            correction_request.revolute_jump_threshold = 0.12
            correction_request.avoid_collisions = True
            correction_response = arm.call(arm.cartesian_client, correction_request)
            correction_path = [
                list(map(float, point.positions))
                for point in correction_response.solution.joint_trajectory.points
            ]
            if (
                correction_response.error_code.val != 1
                or correction_response.fraction < 0.999
                or not correction_path
            ):
                raise RuntimeError(
                    "descent correction path invalid "
                    f"fraction={correction_response.fraction:.4f}, "
                    f"code={correction_response.error_code.val}"
                )
            arm.max_joint_velocity = 0.045
            for q in correction_path[:-1]:
                arm.move_ptp(q)
            move_ptp_precise(arm, correction_path[-1])
            for _ in range(8):
                rclpy.spin_once(arm, timeout_sec=0.04)
            before_hold = arm.fk(list(arm.q))
            correction_history.append(
                {
                    "attempt": correction_attempt,
                    "residual_before_m": residual,
                    "residual_after_m": float(before_hold.position.z - target.position.z),
                }
            )

        recovery_attempt = restore_stable_hold(arm)
        impedance_off = False
        hold_restored = True
        final = arm.fk(list(arm.q))
        actual = [
            final.position.x - top.position.x,
            final.position.y - top.position.y,
            final.position.z - top.position.z,
        ]
        z_error = actual[2] + float(args.down_m)
        orientation_error = quaternion_error_deg(top.orientation, final.orientation)
        if abs(z_error) > 0.012:
            raise RuntimeError(f"descent endpoint error {z_error:+.6f} m")
        print(
            json.dumps(
                {
                    "status": "success",
                    "reference_posture_max_error_rad": restore_error,
                    "restored_top_position_m": [top.position.x, top.position.y, top.position.z],
                    "requested_descent_m": float(args.down_m),
                    "cartesian_fraction": float(response.fraction),
                    "cartesian_waypoint_count": len(path),
                    "correction_history": correction_history,
                    "pre_hold_position_m": [
                        before_hold.position.x,
                        before_hold.position.y,
                        before_hold.position.z,
                    ],
                    "final_position_m": [final.position.x, final.position.y, final.position.z],
                    "actual_delta_m": actual,
                    "orientation_error_deg": orientation_error,
                    "stable_hold_recovery_attempt": recovery_attempt,
                    "gripper_commanded": False,
                },
                indent=2,
            ),
            flush=True,
        )
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        if impedance_off and not hold_restored:
            try:
                restore_stable_hold(arm)
            except Exception as exc:
                print(json.dumps({"event": "hold_restore_failed", "error": repr(exc)}), flush=True)
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
