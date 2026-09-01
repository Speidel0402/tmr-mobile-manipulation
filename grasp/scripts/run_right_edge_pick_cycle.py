#!/usr/bin/env python3
"""Run one ordered right-rim pick cycle on the left FR3/Robotiq pair.

Order is enforced in one process:
open -> visual align -> descend -> close -> return to the recorded top height.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from pick_cycle_policy import classify_close_result
from servo_cup_edge_xy import ServoNode, detect_point


GRIPPER_ACTION = "/left/gripper/robotiq_gripper_controller/gripper_cmd"
REFERENCE_Z_M = 0.596413
REFERENCE_ORIENTATION_XYZW = [
    0.6965795194017754,
    0.7171679017330287,
    0.006319729771827597,
    0.020180061680965224,
]


def emit(event: str, **values):
    print(
        "CYCLE=" + json.dumps({"event": event, **values}, separators=(",", ":")),
        flush=True,
    )


def command_gripper(node: Node, position: float, max_effort: float, label: str):
    client = ActionClient(node, GripperCommand, GRIPPER_ACTION)
    if not client.wait_for_server(timeout_sec=12.0):
        raise RuntimeError("left Robotiq action server unavailable")

    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = float(max_effort)
    emit("gripper_goal", label=label, position=position, max_effort=max_effort)

    feedback_positions = []

    def on_feedback(message):
        feedback_positions.append(float(message.feedback.position))

    future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError(f"{label} gripper goal rejected")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=35.0)
    wrapped = result_future.result()
    if wrapped is None:
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        client.destroy()
        raise RuntimeError(f"{label} gripper result timeout")
    result = wrapped.result
    payload = {
        "label": label,
        "status": int(wrapped.status),
        "position": float(result.position),
        "effort": float(result.effort),
        "stalled": bool(result.stalled),
        "reached_goal": bool(result.reached_goal),
        "feedback_positions": feedback_positions,
    }
    emit("gripper_result", **payload)
    client.destroy()
    return payload


def read_arm_pose():
    arm = ServoNode()
    try:
        arm.wait_ready()
        pose = arm.fk(list(arm.q))
        return {
            "joint_positions_rad": [float(value) for value in arm.q],
            "position_m": [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            "orientation_xyzw": [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ],
        }
    finally:
        arm.destroy_node()


def run_process(command: list[str], label: str, timeout: float):
    env = os.environ.copy()
    env["RCUTILS_LOGGING_MIN_SEVERITY"] = "ERROR"
    emit("process_start", label=label)
    completed = subprocess.run(
        command,
        text=True,
        capture_output=True,
        env=env,
        timeout=timeout,
        check=False,
    )
    if completed.stdout:
        print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n", flush=True)
    if completed.returncode:
        detail = completed.stderr.strip()[-1200:]
        raise RuntimeError(f"{label} failed ({completed.returncode}): {detail}")
    emit("process_done", label=label)
    return completed.stdout


def run_vertical(script: str, delta_z: float, velocity: float):
    stdout = run_process(
        [
            "/usr/bin/python3",
            "-u",
            script,
            "--delta-z",
            f"{delta_z:.9f}",
            "--max-step-m",
            "0.030",
            "--joint-velocity",
            f"{velocity:.3f}",
        ],
        "descend" if delta_z < 0 else "lift",
        timeout=150.0,
    )
    decoder = json.JSONDecoder()
    result = None
    for index, character in enumerate(stdout):
        if character != "{":
            continue
        try:
            candidate, _ = decoder.raw_decode(stdout[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict) and "end_position_m" in candidate:
            result = candidate
    if result is None:
        raise RuntimeError("vertical command completed without a parseable result")
    return result


def observe_right(snapshot_url: str, target: list[float]):
    observation = detect_point(
        snapshot_url,
        edge="right",
        timeout=7.0,
        target_category="cup",
        require_unique_classification=True,
    )
    if not (
        observation.get("classification_valid")
        and observation.get("category") == "cup"
        and observation.get("method") in {
            "unique_target_classification_3frame",
            "three_object_unique_classification_3frame",
        }
    ):
        raise RuntimeError("action denied: cup was not uniquely classified in three consecutive frames")
    point = [float(v) for v in observation["point"]]
    error = [target[0] - point[0], target[1] - point[1]]
    error_norm = math.hypot(*error)
    payload = {
        "point_px": point,
        "target_px": target,
        "error_px": error,
        "error_norm_px": error_norm,
        "confidence": float(observation["confidence"]),
    }
    emit("visual_observation", **payload)
    return payload


def calibrate(args):
    target = [float(args.target_u), float(args.target_v)]
    before = observe_right(args.snapshot_url, target)
    servo_returncode = 0
    if before["error_norm_px"] > args.tolerance_px:
        env = os.environ.copy()
        env["RCUTILS_LOGGING_MIN_SEVERITY"] = "ERROR"
        command = [
            "/usr/bin/python3",
            "-u",
            args.servo_script,
            "--edge",
            "right",
            "--target-u",
            str(args.target_u),
            "--target-v",
            str(args.target_v),
            "--probe-m",
            "0.007",
            "--tolerance-px",
            str(args.tolerance_px),
            "--near-target-px",
            "2.0",
            "--max-iterations",
            "10",
            "--skip-impedance-handoff",
        ]
        emit("process_start", label="visual_calibration")
        completed = subprocess.run(
            command,
            text=True,
            capture_output=True,
            env=env,
            timeout=180.0,
            check=False,
        )
        servo_returncode = int(completed.returncode)
        if completed.stdout:
            print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n", flush=True)
        if completed.stderr and completed.returncode:
            emit("visual_calibration_warning", detail=completed.stderr.strip()[-800:])
        emit("process_done", label="visual_calibration", returncode=servo_returncode)
        if servo_returncode:
            raise RuntimeError(f"visual calibration failed ({servo_returncode})")

    after = observe_right(args.snapshot_url, target)
    if after["error_norm_px"] > args.accept_px:
        raise RuntimeError(
            f"right-rim calibration error {after['error_norm_px']:.3f}px exceeds {args.accept_px:.3f}px"
        )
    return {"before": before, "after": after, "servo_returncode": servo_returncode}


def validate_calibrated_pose(pose, reference_z_m, height_tolerance_m, orientation_tolerance_deg):
    z_error = float(pose["position_m"][2]) - float(reference_z_m)
    q = [float(value) for value in pose["orientation_xyzw"]]
    q_norm = math.sqrt(sum(value * value for value in q))
    ref_norm = math.sqrt(sum(value * value for value in REFERENCE_ORIENTATION_XYZW))
    dot = abs(
        sum(value * reference for value, reference in zip(q, REFERENCE_ORIENTATION_XYZW))
        / max(1e-12, q_norm * ref_norm)
    )
    angle_deg = math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))
    if abs(z_error) > height_tolerance_m:
        raise RuntimeError(f"action denied: calibration height error {z_error:.6f}m")
    if angle_deg > orientation_tolerance_deg:
        raise RuntimeError(f"action denied: tool orientation error {angle_deg:.3f}deg")
    emit("calibrated_pose_valid", z_error_m=z_error, orientation_error_deg=angle_deg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--descent-m", type=float, default=0.33)
    parser.add_argument("--target-u", type=float, default=293.905848)
    parser.add_argument("--target-v", type=float, default=167.921509)
    parser.add_argument("--tolerance-px", type=float, default=5.0)
    parser.add_argument("--accept-px", type=float, default=7.0)
    parser.add_argument("--joint-velocity", type=float, default=0.10)
    # This Robotiq controller reports 0.0 at the open limit and about 0.8 at
    # the closed limit.  Keep these semantics explicit because they are the
    # reverse of the assumption used by the first trial.
    parser.add_argument("--open-position", type=float, default=0.0)
    parser.add_argument("--close-position", type=float, default=0.8)
    parser.add_argument("--max-effort", type=float, default=1.0)
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    parser.add_argument("--servo-script", default="/tmp/servo_cup_edge_xy.py")
    parser.add_argument("--vertical-script", default="/tmp/move_vertical_ground.py")
    parser.add_argument("--record", default="/tmp/right_edge_pick_cycle_33cm.json")
    parser.add_argument("--reference-z-m", type=float, default=REFERENCE_Z_M)
    parser.add_argument("--height-tolerance-m", type=float, default=0.006)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=2.0)
    args = parser.parse_args()
    if not 0.0 < args.descent_m <= 0.35:
        raise RuntimeError("descent must be in (0, 0.35] m")

    record = {
        "started_unix_s": time.time(),
        "ordered_steps": [],
        "requested_descent_m": float(args.descent_m),
    }
    rclpy.init()
    node = Node("ordered_right_edge_pick_cycle")
    descent_started = False
    top_z = None
    failure = None
    operator_stop = False
    returned_to_top = False

    def return_to_top(reason):
        low_pose = read_arm_pose()
        record["pose_before_return"] = low_pose
        current_z = float(low_pose["position_m"][2])
        lift_delta = top_z - current_z
        if abs(lift_delta) > 0.35:
            raise RuntimeError(f"return delta outside 0.35 m limit: {lift_delta:.6f}")
        if abs(lift_delta) > 0.00075:
            up = run_vertical(args.vertical_script, lift_delta, args.joint_velocity)
            record["lift"] = up
            record["ordered_steps"].append("lift_completed")
            final_z = float(up["end_position_m"][2])
        else:
            record["ordered_steps"].append("already_at_top_height")
            final_z = current_z
        record["top_height_error_m"] = final_z - top_z
        emit("returned_to_top", reason=reason, target_z=top_z, final_z=final_z, error_m=final_z-top_z)
    try:
        opened = command_gripper(node, args.open_position, args.max_effort, "open_before_cycle")
        if opened["status"] not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            raise RuntimeError(f"unexpected open action status: {opened['status']}")
        if opened["position"] > 0.05:
            raise RuntimeError(f"gripper did not open sufficiently: {opened['position']:.4f}")
        record["open"] = opened
        record["ordered_steps"].append("open_completed")

        initial_pose = read_arm_pose()
        validate_calibrated_pose(
            initial_pose,
            args.reference_z_m,
            args.height_tolerance_m,
            args.orientation_tolerance_deg,
        )
        record["pose_before_calibration"] = initial_pose

        record["calibration"] = calibrate(args)
        record["ordered_steps"].append("calibration_completed")

        top_pose = read_arm_pose()
        validate_calibrated_pose(
            top_pose,
            args.reference_z_m,
            args.height_tolerance_m,
            args.orientation_tolerance_deg,
        )
        record["top_pose_before_descent"] = top_pose
        top_z = float(top_pose["position_m"][2])
        emit("top_height_recorded", top_z=top_z)
        descent_started = True
        down = run_vertical(args.vertical_script, -args.descent_m, args.joint_velocity)
        record["descent"] = down
        record["ordered_steps"].append("descent_completed")
        emit("descent_confirmed_before_close", top_z=top_z, low_z=float(down["end_position_m"][2]))

        closed = command_gripper(node, args.close_position, args.max_effort, "close_after_descent")
        close_decision = classify_close_result(
            closed,
            open_position=args.open_position,
            close_position=args.close_position,
        )
        close_history = [{"attempt": 1, "result": closed, "decision": close_decision}]
        if not close_decision["accepted_as_grasp"]:
            reset = command_gripper(
                node,
                args.open_position,
                args.max_effort,
                "open_reset_before_close_retry",
            )
            if reset["position"] > args.open_position + 0.05:
                raise RuntimeError(f"gripper reset failed before close retry: {reset}")
            closed = command_gripper(
                node,
                args.close_position,
                args.max_effort,
                "close_after_descent_retry",
            )
            close_decision = classify_close_result(
                closed,
                open_position=args.open_position,
                close_position=args.close_position,
            )
            close_history.append({"attempt": 2, "result": closed, "decision": close_decision})
        closed["physical_classification"] = close_decision
        record["close_history"] = close_history
        close_ok = bool(close_decision["accepted_as_grasp"])
        if not close_ok:
            raise RuntimeError(f"unexpected close action result: {closed}")
        record["close"] = closed
        record["ordered_steps"].append("close_completed")
        return_to_top("normal_after_verified_grasp")
        returned_to_top = True
    except KeyboardInterrupt:
        operator_stop = True
        record["operator_stop"] = True
        emit("operator_stop", action="hold_without_recovery_motion")
    except Exception as exc:
        failure = repr(exc)
        record["failure"] = failure
        emit("failure", detail=failure)
        if descent_started and top_z is not None and not returned_to_top:
            try:
                return_to_top("internal_fault_recovery")
                returned_to_top = True
            except Exception as lift_exc:
                record["lift_failure"] = repr(lift_exc)
                emit("lift_failure", detail=repr(lift_exc))
                failure = failure or repr(lift_exc)
    finally:
        record["finished_unix_s"] = time.time()
        Path(args.record).write_text(json.dumps(record, indent=2), encoding="utf-8")
        emit("record_saved", path=args.record)
        node.destroy_node()
        rclpy.shutdown()

    if failure:
        raise RuntimeError(failure)


if __name__ == "__main__":
    main()
