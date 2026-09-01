#!/usr/bin/env python3
"""Continue immediately from the aligned/open state: down, close, up."""

import json

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node

from pick_cycle_policy import classify_close_result
from servo_cup_edge_xy import JOINT_NAMES, ServoNode


GRIPPER_ACTION = "/left/gripper/robotiq_gripper_controller/gripper_cmd"


def emit(event, **values):
    print("FAST_PICK=" + json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


def close_gripper(node):
    client = ActionClient(node, GripperCommand, GRIPPER_ACTION)
    if not client.wait_for_server(timeout_sec=8.0):
        raise RuntimeError("left gripper action unavailable")
    goal = GripperCommand.Goal()
    goal.command.position = 0.8
    goal.command.max_effort = 1.0
    emit("close_goal_after_descent")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("close rejected")
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=35.0)
    wrapped = result_future.result()
    if wrapped is None:
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        client.destroy()
        raise RuntimeError("close timeout")
    result = wrapped.result
    payload = {
        "status": int(wrapped.status),
        "position": float(result.position),
        "effort": float(result.effort),
        "stalled": bool(result.stalled),
        "reached_goal": bool(result.reached_goal),
    }
    emit("close_result", **payload)
    client.destroy()
    # A stall at position 0.0 is a no-motion actuator failure, not a grasp.
    # The successful real-cup record stalled at 0.768 after substantial travel.
    decision = classify_close_result(payload)
    payload["physical_classification"] = decision
    if not decision["accepted_as_grasp"]:
        raise RuntimeError("unexpected close result " + repr(payload))
    return payload


def move_vertical(arm, delta_z):
    arm.max_joint_velocity = 0.10
    q0 = list(arm.q)
    start = arm.fk(q0)
    target = Pose()
    target.position.x = start.position.x
    target.position.y = start.position.y
    target.position.z = start.position.z + float(delta_z)
    target.orientation = start.orientation
    request = GetCartesianPath.Request()
    request.header.frame_id = "base"
    request.start_state.joint_state.name = JOINT_NAMES
    request.start_state.joint_state.position = q0
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
        raise RuntimeError(f"vertical path invalid fraction={response.fraction:.4f} code={response.error_code.val}")
    for q in path:
        arm.move_ptp(q)
    end = arm.fk(list(arm.q))
    actual = float(end.position.z - start.position.z)
    emit("vertical_done", requested_m=delta_z, actual_m=actual, start_z=float(start.position.z), end_z=float(end.position.z))
    return end, actual


def main():
    rclpy.init()
    arm = ServoNode()
    gripper = Node("fast_left_down_close_up")
    top_z = None
    down_started = False
    lifted = False
    controller_off = False
    failure = None
    operator_stop = False
    close_result = None
    down_actual = None
    try:
        arm.wait_ready()
        top = arm.fk(list(arm.q))
        top_z = float(top.position.z)
        arm.set_impedance(False)
        controller_off = True
        emit("start", top_z=top_z, sequence="down_0.33_then_close_then_up")
        down_started = True
        _, down_actual = move_vertical(arm, -0.33)
        emit("descent_complete_close_now")
        close_result = close_gripper(gripper)
        current = arm.fk(list(arm.q))
        final, up_actual = move_vertical(arm, top_z - float(current.position.z))
        lifted = True
        emit(
            "success",
            down_actual_m=down_actual,
            up_actual_m=up_actual,
            final_z=float(final.position.z),
            top_error_m=float(final.position.z) - top_z,
            close=close_result,
        )
    except KeyboardInterrupt:
        operator_stop = True
        emit("operator_stop", action="hold_without_recovery_motion")
    except Exception as exc:
        failure = repr(exc)
        emit("failure", detail=failure)
        if down_started and top_z is not None and not lifted:
            try:
                current = arm.fk(list(arm.q))
                recovery = top_z - float(current.position.z)
                if 0.00075 < recovery <= 0.35:
                    move_vertical(arm, recovery)
                    emit("internal_fault_recovery_lift_complete")
            except Exception as recovery_exc:
                emit("internal_fault_recovery_lift_failed", detail=repr(recovery_exc))
    finally:
        if controller_off:
            try:
                arm.set_impedance(True)
                emit("controller_restored")
            except Exception as exc:
                emit("controller_restore_failed", detail=repr(exc))
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)
    if operator_stop:
        return


if __name__ == "__main__":
    main()
