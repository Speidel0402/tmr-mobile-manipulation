#!/usr/bin/env python3
"""Place the held object with one ordered down-open-up sequence.

The current measured arm pose is the top reference.  The script never restores
an old joint target, never commands the gripper before the descent, and has no
camera/depth dependency.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node

from servo_cup_edge_xy import JOINT_NAMES, ServoNode


GRIPPER_ACTION = "/left/gripper/robotiq_gripper_controller/gripper_cmd"
MAX_LOWERING_X_M = 1.00


def emit(event: str, **values) -> None:
    print("PLACE=" + json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


def write_state(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)


def command_open(node: Node) -> dict:
    client = ActionClient(node, GripperCommand, GRIPPER_ACTION)
    if not client.wait_for_server(timeout_sec=8.0):
        raise RuntimeError("left gripper action unavailable")
    history = []
    try:
        for attempt in range(1, 3):
            goal = GripperCommand.Goal()
            goal.command.position = 0.0
            goal.command.max_effort = 1.0
            emit("open_goal_after_descent", attempt=attempt)
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            handle = future.result()
            if handle is None or not handle.accepted:
                history.append({"attempt": attempt, "accepted": False})
                continue
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
            wrapped = result_future.result()
            if wrapped is None:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=4.0)
                history.append({"attempt": attempt, "timeout": True})
                continue
            result = wrapped.result
            record = {
                "attempt": attempt,
                "status": int(wrapped.status),
                "position": float(result.position),
                "stalled": bool(result.stalled),
                "reached_goal": bool(result.reached_goal),
            }
            history.append(record)
            emit("open_result", **record)
            if record["position"] <= 0.05 and (
                record["reached_goal"] or record["status"] == GoalStatus.STATUS_SUCCEEDED
            ):
                return {"verified_open": True, "history": history, **record}
            time.sleep(0.15)
    finally:
        client.destroy()
    raise RuntimeError("gripper did not verify open at the placement plane: " + repr(history))


def move_cartesian_to(arm: ServoNode, target: Pose, label: str) -> dict:
    q0 = list(arm.q)
    start = arm.fk(q0)
    request = GetCartesianPath.Request()
    request.header.frame_id = "base"
    request.start_state.joint_state.name = JOINT_NAMES
    request.start_state.joint_state.position = q0
    request.start_state.is_diff = True
    request.group_name = "left_arm"
    request.link_name = "left_fr3v2_link8"
    request.waypoints = [target]
    request.max_step = 0.025
    request.revolute_jump_threshold = 0.12
    request.avoid_collisions = True
    response = arm.call(arm.cartesian_client, request)
    path = [list(map(float, point.positions)) for point in response.solution.joint_trajectory.points]
    if response.error_code.val != 1 or response.fraction < 0.999 or not path:
        raise RuntimeError(
            f"{label} Cartesian path invalid fraction={response.fraction:.4f} code={response.error_code.val}"
        )
    arm.max_joint_velocity = 0.08
    for joints in path:
        arm.move_ptp(joints)
    for _ in range(6):
        rclpy.spin_once(arm, timeout_sec=0.04)
    end = arm.fk(list(arm.q))
    error_xyz = [
        float(end.position.x - target.position.x),
        float(end.position.y - target.position.y),
        float(end.position.z - target.position.z),
    ]
    error = math.sqrt(sum(value * value for value in error_xyz))
    if error > 0.020:
        raise RuntimeError(f"{label} Cartesian endpoint error {error:+.6f} m")
    record = {
        "label": label,
        "start_xyz": [float(start.position.x), float(start.position.y), float(start.position.z)],
        "target_xyz": [float(target.position.x), float(target.position.y), float(target.position.z)],
        "end_xyz": [float(end.position.x), float(end.position.y), float(end.position.z)],
        "error_xyz_m": error_xyz,
        "endpoint_error_m": error,
        "waypoints": len(path),
    }
    emit("vertical_complete", **record)
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fresh-start", action="store_true")
    parser.add_argument("--down-m", type=float, default=0.255)
    parser.add_argument("--forward-m", type=float, default=0.0)
    parser.add_argument("--placement-row", choices=("near", "far"), default="near")
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path("~/tmr-mobile-manipulation/state/place_cycle.json").expanduser(),
    )
    args = parser.parse_args()
    if not 0.18 <= args.down_m <= 0.38:
        raise RuntimeError("down-m outside placement range 0.18..0.38 m")
    if not 0.0 <= args.forward_m <= 0.30:
        raise RuntimeError("forward-m outside bounded placement range 0.00..0.30 m")
    if args.placement_row == "near" and args.forward_m > 0.15:
        raise RuntimeError("near placement extension must remain within 0.15 m")
    if args.placement_row == "far" and args.forward_m < 0.15:
        raise RuntimeError("far placement requires an explicit bounded extension")
    sequence = ["capture_current_top", "move_to_bounded_lowering_staging"]
    sequence.extend(["down", "open", "up"])
    sequence.append("retract_only_if_original_top_was_inside_lowering_envelope")
    sequence.append("stable_hold")
    summary = {
        "sequence": sequence,
        "down_m": float(args.down_m),
        "forward_m": float(args.forward_m),
        "placement_row": args.placement_row,
        "camera_required": False,
        "old_joint_restore_used": False,
    }
    if not args.execute:
        print(json.dumps({"status": "dry_run", **summary}, indent=2), flush=True)
        return 0
    if args.state_file.exists() and not args.fresh_start:
        raise RuntimeError(f"state file exists; refuse accidental place replay: {args.state_file}")

    state = {"status": "running", "phase": "INIT", **summary}
    write_state(args.state_file, state)
    rclpy.init()
    arm = ServoNode()
    gripper = Node("streamed_ordered_left_place")
    impedance_off = False
    released = False
    top_target = None
    staging_target = None
    phase = "INIT"
    operator_stop = False
    failure = None
    try:
        arm.wait_ready()
        top = arm.fk(list(arm.q))
        top_target = Pose()
        top_target.position.x = float(top.position.x)
        top_target.position.y = float(top.position.y)
        top_target.position.z = float(top.position.z)
        top_target.orientation = top.orientation
        state["captured_top_xyz"] = [top.position.x, top.position.y, top.position.z]
        phase = state["phase"] = "TOP_CAPTURED_GRIPPER_UNTOUCHED"
        write_state(args.state_file, state)
        emit("top_captured", xyz=state["captured_top_xyz"], gripper_commanded=False)

        arm.set_impedance(False)
        impedance_off = True
        time.sleep(0.35)
        staging_target = Pose()
        requested_staging_x = top_target.position.x + float(args.forward_m)
        staging_target.position.x = min(requested_staging_x, MAX_LOWERING_X_M)
        staging_target.position.y = top_target.position.y
        staging_target.position.z = top_target.position.z
        staging_target.orientation = top_target.orientation
        effective_forward_m = staging_target.position.x - top_target.position.x
        state["requested_staging_x_m"] = requested_staging_x
        state["bounded_staging_x_m"] = staging_target.position.x
        state["effective_forward_m"] = effective_forward_m
        state["lowering_x_limit_m"] = MAX_LOWERING_X_M
        if abs(effective_forward_m) > 0.005:
            phase = state["phase"] = "MOVING_TO_BOUNDED_LOWERING_STAGING"
            write_state(args.state_file, state)
            state["extension_report"] = move_cartesian_to(
                arm, staging_target, "bounded_forward_extension"
            )
            phase = state["phase"] = "AT_EXTENDED_TOP"
            write_state(args.state_file, state)
        low_target = Pose()
        low_target.position.x = staging_target.position.x
        low_target.position.y = staging_target.position.y
        low_target.position.z = staging_target.position.z - float(args.down_m)
        low_target.orientation = staging_target.orientation
        phase = state["phase"] = "DESCENDING"
        write_state(args.state_file, state)
        down_report = move_cartesian_to(arm, low_target, "down_before_open")
        phase = state["phase"] = "AT_LOW_GRIPPER_STILL_CLOSED"
        state["down_report"] = down_report
        write_state(args.state_file, state)

        phase = state["phase"] = "OPENING_AT_LOW"
        write_state(args.state_file, state)
        open_report = command_open(gripper)
        released = True
        phase = state["phase"] = "RELEASE_VERIFIED"
        state["open_report"] = open_report
        write_state(args.state_file, state)

        phase = state["phase"] = "LIFTING"
        write_state(args.state_file, state)
        up_report = move_cartesian_to(arm, staging_target, "up_after_open")
        state["up_report"] = up_report
        # Restore only a top pose that was already inside the proven lowering
        # envelope.  If this invocation began beyond it (e.g. after a failed
        # over-extension), remain at the recovered staging pose.
        if effective_forward_m > 0.005 and top_target.position.x <= MAX_LOWERING_X_M:
            phase = state["phase"] = "RETRACTING_AFTER_RELEASE"
            write_state(args.state_file, state)
            state["retraction_report"] = move_cartesian_to(
                arm, top_target, "retract_to_captured_top"
            )
        phase = state["phase"] = "DONE"
    except KeyboardInterrupt:
        operator_stop = True
        phase = state["phase"] = "OPERATOR_STOP"
        emit("operator_stop", action="hold_without_unrequested_recovery_motion")
    except Exception as exc:
        failure = repr(exc)
        failed_phase = phase
        state["phase"] = "FAILED"
        state["failed_phase"] = failed_phase
        state["error"] = failure
        emit("failure", phase=failed_phase, detail=failure)
        if top_target is not None and failed_phase in {
            "MOVING_TO_BOUNDED_LOWERING_STAGING",
            "AT_EXTENDED_TOP",
            "DESCENDING",
            "AT_LOW_GRIPPER_STILL_CLOSED",
            "OPENING_AT_LOW",
            "RELEASE_VERIFIED",
            "LIFTING",
        }:
            try:
                current = arm.fk(list(arm.q))
                if staging_target is not None and 0.00075 < staging_target.position.z - current.position.z <= 0.38:
                    move_cartesian_to(arm, staging_target, "bounded_fault_recovery_up")
                current = arm.fk(list(arm.q))
                distance_to_top = math.sqrt(
                    (current.position.x - top_target.position.x) ** 2
                    + (current.position.y - top_target.position.y) ** 2
                    + (current.position.z - top_target.position.z) ** 2
                )
                if 0.00075 < distance_to_top <= 0.22:
                    move_cartesian_to(arm, top_target, "bounded_fault_recovery_retract")
                state["fault_recovery"] = "top_restored"
            except Exception as recovery_exc:
                state["fault_recovery"] = repr(recovery_exc)
    finally:
        if impedance_off:
            try:
                attempt = arm.ensure_stable_runtime_after_ptp()
                state["stable_hold_recovery_attempt"] = attempt
                emit("controller", joint_impedance="restored_to_stable_hold", attempt=attempt)
            except Exception as exc:
                state["controller_restore_error"] = repr(exc)
                if failure is None and not operator_stop:
                    failure = "controller hold recovery failed: " + repr(exc)
        state["released"] = released
        state["status"] = "complete" if phase == "DONE" and failure is None else "failed"
        write_state(args.state_file, state)
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()
    print(json.dumps(state, indent=2), flush=True)
    if operator_stop:
        return 130
    return 0 if state["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
