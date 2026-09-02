#!/usr/bin/env python3
"""One ordered, robot-local live pick cycle for the competition workflow."""

import argparse
import io
import json
import math
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from cup_rim_detector import detect_green_cup_right
from head_rgb_descent import (
    detect_flange_edge_y,
    detect_target_cup,
    estimate_remaining_down_m,
    target_flange_edge_y,
)
from pick_cycle_policy import (
    classify_close_result,
    cup_grasp_alignment_accepted,
    grasp_plane_policy,
    top_pose_policy,
    validate_camera_snapshot,
    validate_rgb_freshness,
    visual_tolerances,
)
from servo_cup_edge_xy import JOINT_NAMES, ServoNode, adaptive_parameters, cap_norm


SNAPSHOT_URL = "http://127.0.0.1:18080/snapshot.npz"
GRIPPER_ACTION = "/left/gripper/robotiq_gripper_controller/gripper_cmd"
# Median horizontal target of the two confirmed real cup grasps.  The old
# symmetric soft window accepted u=298.68, where the fingers missed outside
# the rim and pushed the cup away.
TARGET = np.asarray([292.5, 167.921509], dtype=float)
REFERENCE_Z = 0.596413
DESCENT_M = 0.340
GRASP_Z = REFERENCE_Z - DESCENT_M
HEAD_CAMERA_TOPIC = "/head_camera/zed/rgb/color/rect/image/compressed"
HEAD_COARSE_DESCENT_M = 0.180
HEAD_PROBE_DESCENT_M = 0.020
HEAD_FALLBACK_DESCENT_M = 0.240
HEAD_MIN_DESCENT_M = 0.220
HEAD_MAX_DESCENT_M = 0.245
MAX_RECOVERABLE_VISUAL_ERROR_PX = 11.0
MIN_EXECUTABLE_FINE_STEP_M = 0.0025
MAX_VISUAL_SEARCH_M = 0.38
VISION_SETTLE_BEFORE_SAMPLE_S = 0.55
VISION_STABLE_FRAMES = 5
VISION_STABLE_RADIUS_PX = 8.0
MAX_HIGH_GRASP_PLANE_RESIDUAL_M = 0.020
REFERENCE_Q = np.asarray(
    [0.6965795194017754, 0.7171679017330287, 0.006319729771827597, 0.020180061680965224],
    dtype=float,
)
# Recorded at the top of the successful 33 cm grasp cycle.  Using this
# deterministic joint seed avoids trying to repair a large tool-orientation
# error in place, which previously made the arm swing without reaching the
# calibrated camera/gripper pose.
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
# Empirical pixel-per-metre mapping from a completed bowl alignment at the
# recorded top pose.  A fresh probe may refine it, but a circle-fit jump must
# never be allowed to reverse the known image-x response to arm-x motion.
CALIBRATED_VISUAL_JACOBIAN = np.asarray(
    [[-773.8408883365554, -206.6096777663516],
     [555.7475627583005, 1862.6534184570041]],
    dtype=float,
)


def emit(event, **values):
    print("PICK=" + json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


class HeadRgbObserver(Node):
    def __init__(self):
        super().__init__("pick_head_rgb_descent_observer")
        self.frame = None
        self.stamp_ns = 0
        self.create_subscription(
            CompressedImage,
            HEAD_CAMERA_TOPIC,
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message):
        frame = cv2.imdecode(
            np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR
        )
        if frame is not None:
            self.frame = frame
            self.stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )

    def fresh(self, previous_stamp_ns=0, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.frame is not None and self.stamp_ns > int(previous_stamp_ns):
                return self.frame.copy(), self.stamp_ns
        raise RuntimeError("fresh head ZED RGB frame unavailable")


def snapshot_rgb(expected_session_id=None):
    with urllib.request.urlopen(SNAPSHOT_URL, timeout=2.0) as response:
        payload = response.read()
    with np.load(io.BytesIO(payload), allow_pickle=False) as sample:
        rgb = sample["rgb"].copy()
        session_id = validate_camera_snapshot(
            role=sample["camera_role"].item(),
            topic=sample["rgb_topic"].item(),
            frame_id=sample["rgb_frame_id"].item(),
            session_id=sample["camera_session_id"].item(),
            image_shape=rgb.shape,
            expected_session_id=expected_session_id,
        )
        validate_rgb_freshness(
            age_s=sample["rgb_age_s"].item(),
            sequence=sample["rgb_sequence"].item(),
        )
        return rgb, float(sample["rgb_stamp"]), session_id


def advancing_preflight_snapshot(timeout_s=2.0):
    """Prove the persistent viewer advances before any gripper/arm motion."""
    _first_rgb, first_stamp, session_id = snapshot_rgb()
    deadline = time.monotonic() + float(timeout_s)
    last_error = "RGB timestamp did not advance"
    while time.monotonic() < deadline:
        try:
            rgb, stamp, confirmed_session = snapshot_rgb(session_id)
            if stamp > first_stamp + 1e-6:
                return rgb, stamp, confirmed_session
            last_error = f"RGB timestamp stayed at {stamp:.9f}"
        except Exception as exc:
            last_error = repr(exc)
        time.sleep(0.03)
    raise RuntimeError(f"left-wrist camera preflight failed: {last_error}")


def cup_right_once(bgr):
    return detect_green_cup_right(bgr)


def detect_cup_right(
    previous_stamp=None,
    timeout=15.0,
    expected_session_id=None,
    expected_point=None,
    maximum_tracking_error_px=80.0,
):
    # A completed arm action can leave a short camera/fixture oscillation.
    # Do not let those transient frames enter the unchanged rim detector or
    # its Jacobian estimate; wait, then require consecutive stable results.
    time.sleep(VISION_SETTLE_BEFORE_SAMPLE_S)
    deadline = time.monotonic() + timeout
    last_stamp = -math.inf if previous_stamp is None else float(previous_stamp)
    observations = []
    last_error = "no fresh frame"
    while time.monotonic() < deadline:
        try:
            bgr, stamp, session_id = snapshot_rgb(expected_session_id)
            if expected_session_id is None:
                expected_session_id = session_id
            if stamp <= last_stamp + 1e-6:
                time.sleep(0.025)
                continue
            last_stamp = stamp
            point, detail = cup_right_once(bgr)
            if expected_point is not None:
                tracking_error = float(
                    np.linalg.norm(np.asarray(point, dtype=float) - np.asarray(expected_point, dtype=float))
                )
                if tracking_error > float(maximum_tracking_error_px):
                    last_error = (
                        f"cup track jump {tracking_error:.1f}px exceeds "
                        f"{float(maximum_tracking_error_px):.1f}px"
                    )
                    time.sleep(0.025)
                    continue
            observations.append((point, stamp, detail))
            observations = observations[-VISION_STABLE_FRAMES:]
            if len(observations) >= VISION_STABLE_FRAMES:
                points = np.asarray([item[0] for item in observations], dtype=float)
                center = np.median(points, axis=0)
                deviations = np.linalg.norm(points - center, axis=1)
                if float(np.max(deviations)) <= VISION_STABLE_RADIUS_PX:
                    point = center
                    return {
                        "point": point,
                        "stamp": float(max(item[1] for item in observations)),
                        "spread_px": float(np.max(deviations)),
                        "detail": observations[-1][2],
                        "camera_session_id": expected_session_id,
                    }
                last_error = (
                    f"rim still settling; five-frame spread "
                    f"{float(np.max(deviations)):.1f}px"
                )
            time.sleep(0.025)
        except Exception as exc:
            last_error = repr(exc)
            time.sleep(0.04)
    raise RuntimeError("fresh three-frame cup detection failed: " + last_error)


def command_gripper(node, position, label):
    client = ActionClient(node, GripperCommand, GRIPPER_ACTION)
    if not client.wait_for_server(timeout_sec=8.0):
        raise RuntimeError("left gripper action unavailable")
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = 1.0
    started = time.monotonic()
    emit("gripper_goal", label=label, position=position)
    feedback_positions = []

    def on_feedback(message):
        feedback_positions.append(float(message.feedback.position))

    future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError(label + " gripper goal rejected")
    emit("gripper_goal_accepted", label=label)
    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=35.0)
    except KeyboardInterrupt:
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        client.destroy()
        raise
    wrapped = result_future.result()
    if wrapped is None:
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        client.destroy()
        raise RuntimeError(label + " gripper timeout")
    result = wrapped.result
    payload = {
        "label": label,
        "status": int(wrapped.status),
        "position": float(result.position),
        "effort": float(result.effort),
        "stalled": bool(result.stalled),
        "reached_goal": bool(result.reached_goal),
        "elapsed_s": float(time.monotonic() - started),
        "feedback_positions": feedback_positions,
    }
    emit("gripper_result", **payload)
    client.destroy()
    return payload


def close_result_is_real(result, open_position=0.0, close_position=0.8):
    decision = classify_close_result(
        result,
        open_position=open_position,
        close_position=close_position,
    )
    return (
        bool(decision["accepted_as_grasp"]),
        float(decision["progress"]),
        str(decision["classification"]),
    )


def close_gripper_once(node):
    """Close once after descent; never reopen and disturb the target."""
    result = command_gripper(node, 0.8, "close_after_fixed_descent")
    valid, progress, verdict = close_result_is_real(result)
    result["close_progress"] = progress
    result["verdict"] = verdict
    history = [{"attempt": 1, "close": result}]
    emit("close_verdict", attempt=1, valid=valid, progress=progress, verdict=verdict)
    if not valid:
        raise RuntimeError("single close did not confirm object contact: " + repr(history))
    return result, history


def top_errors(pose):
    z_error = float(pose.position.z) - REFERENCE_Z
    q = np.asarray([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    dot = abs(float(np.dot(q / np.linalg.norm(q), REFERENCE_Q / np.linalg.norm(REFERENCE_Q))))
    angle = math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))
    return z_error, angle


def read_settled_pose(arm, samples=5, interval_s=0.06):
    poses = []
    for _ in range(samples):
        poses.append(arm.fk(list(arm.q)))
        time.sleep(interval_s)
    median_z = float(np.median([pose.position.z for pose in poses]))
    pose = min(poses, key=lambda item: abs(float(item.position.z) - median_z))
    emit(
        "pose_settled",
        z=float(pose.position.z),
        z_span_m=float(max(item.position.z for item in poses) - min(item.position.z for item in poses)),
        samples=samples,
    )
    return pose


def validate_top(pose, z_tolerance=0.010, orientation_tolerance_deg=3.0):
    z_error, angle = top_errors(pose)
    if abs(z_error) > z_tolerance or angle > orientation_tolerance_deg:
        raise RuntimeError(f"calibration pose mismatch z={z_error:.6f}m orientation={angle:.3f}deg")
    emit("top_pose", z=float(pose.position.z), z_error_m=z_error, orientation_error_deg=angle)


def restore_reference_pose(arm):
    arm.max_joint_velocity = 0.07
    end = None
    for attempt in range(1, 3):
        q0 = list(arm.q)
        start = arm.fk(q0)
        target = Pose()
        target.position.x = start.position.x
        target.position.y = start.position.y
        target.position.z = REFERENCE_Z
        target.orientation.x = float(REFERENCE_Q[0])
        target.orientation.y = float(REFERENCE_Q[1])
        target.orientation.z = float(REFERENCE_Q[2])
        target.orientation.w = float(REFERENCE_Q[3])
        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = q0
        request.start_state.is_diff = True
        request.group_name = "left_arm"
        request.link_name = "left_fr3v2_link8"
        request.waypoints = [target]
        request.max_step = 0.010
        request.revolute_jump_threshold = 0.12
        request.avoid_collisions = True
        response = arm.call(arm.cartesian_client, request)
        path = [list(map(float, p.positions)) for p in response.solution.joint_trajectory.points]
        if response.error_code.val != 1 or response.fraction < 0.999 or not path:
            raise RuntimeError(f"reference-pose path invalid fraction={response.fraction:.4f} code={response.error_code.val}")
        for q in path:
            arm.move_ptp(q)
        end = arm.fk(list(arm.q))
        z_error, angle_error = top_errors(end)
        emit("reference_pose_restore_attempt", attempt=attempt, z=float(end.position.z), z_error_m=z_error, orientation_error_deg=angle_error, waypoints=len(path))
        if abs(z_error) <= 0.002 and angle_error <= 1.0:
            break
    validate_top(end)
    arm.lock_z = float(end.position.z)
    arm.lock_orientation = end.orientation
    emit("reference_pose_restored", z=float(end.position.z))
    return end


def restore_recorded_top_joint_pose(arm):
    """Restore the proven top pose and verify the measured joint endpoint."""
    arm.max_joint_velocity = 0.08
    target = REFERENCE_JOINTS.tolist()
    for attempt in range(1, 3):
        arm.move_ptp(target)
        for _ in range(5):
            rclpy.spin_once(arm, timeout_sec=0.04)
        error = np.asarray(arm.q, dtype=float) - REFERENCE_JOINTS
        max_error = float(np.max(np.abs(error)))
        emit(
            "recorded_top_restore_attempt",
            attempt=attempt,
            max_joint_error_rad=max_error,
            joint_error_rad=error.tolist(),
        )
        if max_error <= 0.008:
            break
    if max_error > 0.008:
        raise RuntimeError(
            f"recorded top pose not reached; max joint error {max_error:.6f}rad"
        )
    pose = arm.fk(list(arm.q))
    validate_top(pose, z_tolerance=0.008, orientation_tolerance_deg=2.0)
    arm.lock_z = float(pose.position.z)
    arm.lock_orientation = pose.orientation
    emit("recorded_top_restored", z=float(pose.position.z))
    return pose


def visual_tolerance(observation):
    return visual_tolerances(observation.get("spread_px", 0.0))["enter_px"]


def recover_best_visual_pose(arm, best, stamp, camera_session_id, iterations, reason):
    """Return the best measured pose when a finer Cartesian step is unreachable."""
    if float(np.linalg.norm(np.asarray(arm.q) - np.asarray(best["q"]))) > 1e-4:
        arm.move_ptp(best["q"])
    try:
        confirmation = detect_cup_right(
            previous_stamp=stamp,
            expected_session_id=camera_session_id,
            expected_point=best["point"],
            maximum_tracking_error_px=35.0,
        )
        best["point"] = confirmation["point"]
        best["error"] = TARGET - confirmation["point"]
        best["observation"] = confirmation
    except RuntimeError as exc:
        # The cached observation was captured at this exact joint pose.  A
        # transient missed frame must not discard an already usable alignment.
        emit("best_pose_confirmation_unavailable", reason=reason, error=str(exc))
    if cup_grasp_alignment_accepted(
        best["error"], MAX_RECOVERABLE_VISUAL_ERROR_PX, confirmation=True
    ):
        emit(
            "aligned_best_recovered",
            point_px=best["point"].tolist(),
            error_px=best["error"].tolist(),
            tolerance_px=MAX_RECOVERABLE_VISUAL_ERROR_PX,
            reason=reason,
        )
        return {
            "point": best["point"],
            "error": best["error"],
            "iterations": iterations,
            "mode": "best_pose_at_kinematic_boundary",
        }
    return None


def calibrate(arm, camera_session_id, preflight_stamp):
    initial_pose = arm.fk(list(arm.q))
    base_origin = np.asarray([initial_pose.position.x, initial_pose.position.y], dtype=float)
    first = detect_cup_right(
        previous_stamp=preflight_stamp,
        expected_session_id=camera_session_id,
    )
    p0, stamp = first["point"], first["stamp"]
    emit("observe", stage="initial", point_px=p0.tolist(), target_px=TARGET.tolist(), spread_px=first["spread_px"])
    initial_error = TARGET - p0
    initial_tolerance = visual_tolerance(first)
    if cup_grasp_alignment_accepted(initial_error, initial_tolerance):
        emit("aligned_near", point_px=p0.tolist(), error_px=initial_error.tolist(), tolerance_px=initial_tolerance, reason="initial_within_detector_resolution")
        return {"point": p0, "error": initial_error, "iterations": 0, "mode": "initial_soft_accept"}

    # When the target begins at the extreme right, probe inward so the first
    # calibration motion cannot push its rim out of the image.
    probe_x_sign = -1.0 if float(p0[0]) > 600.0 else 1.0
    actual_x, base_x, _ = arm.move_xy([0.007 * probe_x_sign, 0.0])
    obs_x = detect_cup_right(previous_stamp=stamp, expected_session_id=camera_session_id)
    px, stamp = obs_x["point"], obs_x["stamp"]
    emit("probe", axis="x", actual_m=actual_x.tolist(), point_px=px.tolist())

    actual_y, base_y, _ = arm.move_xy([-0.007 * probe_x_sign, 0.007])
    obs_y = detect_cup_right(previous_stamp=stamp, expected_session_id=camera_session_id)
    py, stamp = obs_y["point"], obs_y["stamp"]
    emit("probe", axis="y", actual_m=actual_y.tolist(), point_px=py.tolist())

    displacement = np.column_stack((base_x - base_origin, base_y - base_origin))
    pixels = np.column_stack((px - p0, py - p0))
    if abs(float(np.linalg.det(displacement))) < 2.5e-5:
        raise RuntimeError("visual probe displacement singular")
    jacobian = pixels @ np.linalg.inv(displacement)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-9))
    if jacobian[0, 0] >= -100.0:
        emit(
            "jacobian_fallback",
            reason="probe_direction_inconsistent",
            measured=jacobian.tolist(),
            replacement=CALIBRATED_VISUAL_JACOBIAN.tolist(),
        )
        jacobian = CALIBRATED_VISUAL_JACOBIAN.copy()
        singular = np.linalg.svd(jacobian, compute_uv=False)
        condition = float(singular[0] / singular[-1])
    elif singular[-1] < 60.0 or condition > 30.0:
        raise RuntimeError(f"visual Jacobian unreliable {singular.tolist()} condition={condition:.3f}")
    emit("jacobian", matrix=jacobian.tolist(), condition=condition)
    probe_jacobian = jacobian.copy()

    current_point = py
    current_base = base_y
    current_observation = obs_y
    trust = 0.70
    stagnation = 0
    best = {
        "point": current_point.copy(),
        "error": TARGET - current_point,
        "q": list(arm.q),
        "base": current_base.copy(),
        "observation": current_observation,
        "norm": float(np.linalg.norm(TARGET - current_point)),
    }
    for iteration in range(1, 17):
        error = TARGET - current_point
        tolerance = visual_tolerance(current_observation)
        if cup_grasp_alignment_accepted(error, tolerance):
            confirm = detect_cup_right(
                previous_stamp=stamp,
                expected_session_id=camera_session_id,
                expected_point=current_point,
                maximum_tracking_error_px=30.0,
            )
            stamp = confirm["stamp"]
            final_error = TARGET - confirm["point"]
            hold_tolerance = min(8.0, max(tolerance, visual_tolerance(confirm)) + 1.0)
            if cup_grasp_alignment_accepted(
                final_error, hold_tolerance, confirmation=True
            ):
                emit("aligned", iteration=iteration - 1, point_px=confirm["point"].tolist(), error_px=final_error.tolist(), tolerance_px=hold_tolerance, reason="confirmed_with_hysteresis")
                return {"point": confirm["point"], "error": final_error, "iterations": iteration - 1, "mode": "confirmed_soft_accept"}
            current_point = confirm["point"]
            current_observation = confirm
            error = final_error
        gain, limit = adaptive_parameters(float(np.linalg.norm(error)), trust)
        step = cap_norm(np.linalg.pinv(jacobian, rcond=0.06) @ error * gain, limit)
        if not np.all(np.isfinite(step)):
            jacobian = probe_jacobian.copy()
            step = cap_norm(np.linalg.pinv(jacobian, rcond=0.06) @ error * gain, limit)
        step_norm = float(np.linalg.norm(step))
        if step_norm < MIN_EXECUTABLE_FINE_STEP_M:
            # The PTP action's practical joint tolerance maps to roughly a
            # 2 mm Cartesian deadband.  Smaller commands soft-complete without
            # moving, so keep iterating with the smallest empirically
            # executable step and let the next image reverse/refine it.
            if step_norm <= 1e-9:
                jacobian = probe_jacobian.copy()
                step = cap_norm(np.linalg.pinv(jacobian, rcond=0.06) @ error, limit)
                step_norm = float(np.linalg.norm(step))
            if step_norm <= 1e-9:
                raise RuntimeError("visual correction unavailable after Jacobian reset")
            step = step * (MIN_EXECUTABLE_FINE_STEP_M / step_norm)
            emit(
                "fine_step_promoted",
                iteration=iteration,
                minimum_executable_m=MIN_EXECUTABLE_FINE_STEP_M,
            )
        if float(np.linalg.norm(current_base + step - base_origin)) > MAX_VISUAL_SEARCH_M:
            recovered = recover_best_visual_pose(
                arm, best, stamp, camera_session_id, iteration - 1, "visual_search_limit"
            )
            if recovered is not None:
                return recovered
            restore_recorded_top_joint_pose(arm)
            raise RuntimeError(f"visual search exceeded {MAX_VISUAL_SEARCH_M:.2f}m")
        old_norm = float(np.linalg.norm(error))
        try:
            actual, new_base, end_pose = arm.move_xy(step)
        except RuntimeError as exc:
            if "horizontal path invalid" not in str(exc):
                raise
            recovered = recover_best_visual_pose(
                arm, best, stamp, camera_session_id, iteration - 1, "cartesian_kinematic_boundary"
            )
            if recovered is not None:
                return recovered
            restore_recorded_top_joint_pose(arm)
            raise RuntimeError(
                "visual correction reached a kinematic boundary before a usable alignment"
            ) from exc
        predicted_point = current_point + jacobian @ actual
        observation = detect_cup_right(
            previous_stamp=stamp,
            expected_session_id=camera_session_id,
            expected_point=predicted_point,
            maximum_tracking_error_px=70.0,
        )
        stamp = observation["stamp"]
        new_point = observation["point"]
        observed = new_point - current_point
        denominator = float(actual @ actual)
        new_norm = float(np.linalg.norm(TARGET - new_point))
        improved = new_norm < old_norm - 0.5
        trust = min(1.0, trust * 1.12) if improved else max(0.25, trust * 0.55)
        stagnation = 0 if improved else stagnation + 1
        # Hough coordinates are quantized.  Update J only for motions/pixel
        # changes large enough to exceed that measurement noise, and blend the
        # update so one quantized sample cannot corrupt the local model.
        if denominator > 2.25e-6 and float(np.linalg.norm(observed)) > max(2.5, 1.5 * observation["spread_px"]):
            candidate = jacobian + np.outer(observed - jacobian @ actual, actual) / denominator
            candidate = 0.75 * jacobian + 0.25 * candidate
            candidate_singular = np.linalg.svd(candidate, compute_uv=False)
            candidate_condition = float(candidate_singular[0] / max(candidate_singular[-1], 1e-9))
            if candidate_singular[-1] >= 60.0 and candidate_condition <= 20.0:
                jacobian = candidate
            else:
                emit("jacobian_update_ignored", iteration=iteration, condition=candidate_condition)
        if new_norm < best["norm"]:
            best = {
                "point": new_point.copy(),
                "error": TARGET - new_point,
                "q": list(arm.q),
                "base": new_base.copy(),
                "observation": observation,
                "norm": new_norm,
            }
        emit("correct", iteration=iteration, command_m=step.tolist(), actual_m=actual.tolist(), point_px=new_point.tolist(), error_px=(TARGET-new_point).tolist(), error_norm_px=new_norm, z=float(end_pose.position.z))
        current_point, current_base, current_observation = new_point, new_base, observation
        if stagnation >= 3:
            jacobian = probe_jacobian.copy()
            trust = 0.35
            stagnation = 0
            emit("jacobian_reset", iteration=iteration, reason="three_non_improving_updates")

    recovered = recover_best_visual_pose(
        arm, best, stamp, camera_session_id, 16, "iteration_limit"
    )
    if recovered is not None:
        return recovered
    restore_recorded_top_joint_pose(arm)
    raise RuntimeError("visual alignment remained outside recoverable range " + repr(best["error"].tolist()))


def move_vertical(arm, delta_z):
    if not -0.38 <= delta_z <= 0.38:
        raise RuntimeError("vertical delta outside limit")
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
    path = [list(map(float, p.positions)) for p in response.solution.joint_trajectory.points]
    if response.error_code.val != 1 or response.fraction < 0.999 or not path:
        raise RuntimeError(f"vertical path invalid fraction={response.fraction:.4f} code={response.error_code.val}")
    for q in path:
        arm.move_ptp(q)
    end = arm.fk(list(arm.q))
    actual = float(end.position.z - start.position.z)
    emit("vertical_done", requested_m=delta_z, actual_m=actual, start_z=float(start.position.z), end_z=float(end.position.z), waypoints=len(path))
    return end, actual


def descend_with_head_rgb(arm, head_camera):
    """Coarse descend, estimate Z from the visible flange, then refine."""
    total_actual = 0.0
    stamp_ns = 0
    try:
        top_frame, stamp_ns = head_camera.fresh()
        cup = detect_target_cup(top_frame)
        desired_edge_y = target_flange_edge_y(cup)
        emit(
            "head_z_cup",
            cup=cup,
            desired_flange_edge_y_px=desired_edge_y,
        )
    except Exception as exc:
        emit(
            "head_z_fallback",
            stage="top_detection",
            detail=repr(exc),
            fallback_down_m=HEAD_FALLBACK_DESCENT_M,
        )
        return move_vertical(arm, -HEAD_FALLBACK_DESCENT_M)

    low_pose, actual = move_vertical(arm, -HEAD_COARSE_DESCENT_M)
    total_actual += actual
    try:
        coarse_frame, stamp_ns = head_camera.fresh(stamp_ns)
        first_edge_y = detect_flange_edge_y(coarse_frame, cup)

        low_pose, actual = move_vertical(arm, -HEAD_PROBE_DESCENT_M)
        total_actual += actual
        probe_frame, stamp_ns = head_camera.fresh(stamp_ns)
        second_edge_y = detect_flange_edge_y(probe_frame, cup)
        remaining_down_m, scale_px_per_m = estimate_remaining_down_m(
            first_edge_y,
            second_edge_y,
            HEAD_PROBE_DESCENT_M,
            desired_edge_y,
        )
        measured_total_m = -total_actual
        desired_total_m = float(
            np.clip(
                measured_total_m + remaining_down_m,
                HEAD_MIN_DESCENT_M,
                HEAD_MAX_DESCENT_M,
            )
        )
        correction_down_m = desired_total_m - measured_total_m
        emit(
            "head_z_solution",
            first_edge_y_px=first_edge_y,
            second_edge_y_px=second_edge_y,
            desired_edge_y_px=desired_edge_y,
            scale_px_per_m=scale_px_per_m,
            measured_total_m=measured_total_m,
            correction_down_m=correction_down_m,
            desired_total_m=desired_total_m,
        )
        if abs(correction_down_m) >= 0.0015:
            low_pose, actual = move_vertical(arm, -correction_down_m)
            total_actual += actual
        try:
            final_frame, _ = head_camera.fresh(stamp_ns)
            final_edge_y = detect_flange_edge_y(final_frame, cup)
            emit(
                "head_z_verified",
                final_edge_y_px=final_edge_y,
                desired_edge_y_px=desired_edge_y,
                residual_px=final_edge_y - desired_edge_y,
                total_down_m=-total_actual,
            )
        except Exception as verify_exc:
            emit("head_z_verify_soft_failure", detail=repr(verify_exc))
        return low_pose, total_actual
    except Exception as exc:
        measured_total_m = -total_actual
        fallback_correction_m = HEAD_FALLBACK_DESCENT_M - measured_total_m
        emit(
            "head_z_fallback",
            stage="low_probe",
            detail=repr(exc),
            measured_total_m=measured_total_m,
            fallback_correction_m=fallback_correction_m,
        )
        if abs(fallback_correction_m) >= 0.0015:
            low_pose, actual = move_vertical(arm, -fallback_correction_m)
            total_actual += actual
        return low_pose, total_actual


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--force-restore-top",
        action="store_true",
        help="restore the recorded successful top joints even when the measured pose is already acceptable",
    )
    args = parser.parse_args()
    rclpy.init()
    arm = ServoNode()
    gripper = Node("streamed_ordered_left_pick")
    head_camera = HeadRgbObserver()
    impedance_off = False
    top_z = None
    failure = None
    operator_stop = False
    cycle_success = False
    phase = "INIT"
    try:
        arm.wait_ready()
        phase = "CAMERA_PREFLIGHT"
        _preflight_rgb, camera_preflight_stamp, camera_session_id = advancing_preflight_snapshot()
        emit(
            "camera_verified",
            role="left_wrist",
            session_id=camera_session_id,
            rgb_stamp=camera_preflight_stamp,
        )
        phase = "OPENING"
        opened = command_gripper(gripper, 0.0, "open_before_motion")
        if not (
            opened["status"] == GoalStatus.STATUS_SUCCEEDED
            and opened["reached_goal"]
            and opened["position"] <= 0.05
        ):
            opened = command_gripper(gripper, 0.0, "open_before_motion_retry")
        if opened["position"] > 0.05 or not opened["reached_goal"]:
            raise RuntimeError("gripper open state was not verified")
        phase = "OPEN_VERIFIED"
        emit("order_confirmed", completed="open", next="visual_alignment")

        starting_pose = arm.fk(list(arm.q))
        starting_z_error, starting_angle_error = top_errors(starting_pose)
        emit("starting_pose_before_handoff", z=float(starting_pose.position.z), z_error_m=starting_z_error, orientation_error_deg=starting_angle_error)
        arm.set_impedance(False)
        impedance_off = True
        emit("controller", joint_impedance="inactive_for_motion")

        starting_pose = read_settled_pose(arm)
        starting_z_error, starting_angle_error = top_errors(starting_pose)
        pose_policy = top_pose_policy(starting_z_error, starting_angle_error)
        emit("starting_pose_after_handoff", z=float(starting_pose.position.z), z_error_m=starting_z_error, orientation_error_deg=starting_angle_error, policy=pose_policy)
        if args.force_restore_top or pose_policy != "accept":
            phase = "RESTORING_REFERENCE_POSE"
            emit(
                "top_restore_requested",
                forced=bool(args.force_restore_top),
                measured_policy=pose_policy,
            )
            restore_recorded_top_joint_pose(arm)
        else:
            arm.lock_z = float(starting_pose.position.z)
            arm.lock_orientation = starting_pose.orientation
        validate_top(arm.fk(list(arm.q)))

        phase = "VISUAL_ALIGNING"
        alignment = calibrate(arm, camera_session_id, camera_preflight_stamp)
        top_pose = arm.fk(list(arm.q))
        validate_top(top_pose)
        top_z = float(top_pose.position.z)
        phase = "ALIGNED"
        emit("order_confirmed", completed="visual_alignment", next="descend", final_error_px=alignment["error"].tolist())

        phase = "DESCENDING"
        low_pose, down_actual = move_vertical(arm, -DESCENT_M)
        measured_down_m = -float(down_actual)
        if abs(measured_down_m - DESCENT_M) > 0.008:
            raise RuntimeError(
                f"fixed descent did not reach {DESCENT_M:.3f}m: {measured_down_m:.6f}m"
            )
        phase = "AT_LOW"
        emit(
            "order_confirmed",
            completed=f"fixed_{DESCENT_M:.3f}m_descent",
            next="close",
            down_actual_m=down_actual,
        )

        phase = "CLOSING"
        closed, close_history = close_gripper_once(gripper)
        phase = "GRASP_VERIFIED"
        emit("order_confirmed", completed="close", next="lift")

        current = arm.fk(list(arm.q))
        phase = "LIFTING"
        final_pose, up_actual = move_vertical(arm, top_z - float(current.position.z))
        phase = "DONE"
        cycle_success = True
        emit("success", alignment_error_px=alignment["error"].tolist(), alignment_mode=alignment.get("mode"), down_actual_m=down_actual, up_actual_m=up_actual, final_z=float(final_pose.position.z), top_height_error_m=float(final_pose.position.z)-top_z, close=closed, close_history=close_history)
    except KeyboardInterrupt:
        operator_stop = True
        emit("operator_stop", stopped_at_phase=phase, action="hold_without_recovery_motion")
    except Exception as exc:
        failure = repr(exc)
        failed_phase = phase
        emit("failure", phase=failed_phase, detail=failure)
        # Internal faults at the grasp plane use an explicit bounded recovery.
        # This is deliberately outside finally: an operator STOP must not
        # silently trigger an extra lift command.
        if top_z is not None and phase in {"DESCENDING", "AT_LOW", "CLOSING", "GRASP_VERIFIED", "LIFTING"}:
            try:
                phase = "RECOVER_LIFT"
                current = arm.fk(list(arm.q))
                recovery = top_z - float(current.position.z)
                if 0.00075 < recovery <= 0.38:
                    move_vertical(arm, recovery)
                phase = "RECOVERED_AT_TOP"
                emit("internal_fault_recovery_complete", failed_phase=failed_phase)
            except Exception as recovery_exc:
                emit("internal_fault_recovery_failed", failed_phase=failed_phase, detail=repr(recovery_exc))
    finally:
        if impedance_off:
            try:
                # PTP reports its result before its controller teardown has
                # fully settled.  Restoring impedance immediately races that
                # teardown and can leave the hardware UNCONFIGURED.  Let the
                # action server finish, then recover the complete left runtime
                # (hardware, state broadcasters, and impedance hold) once.
                recovery_attempt = arm.ensure_stable_runtime_after_ptp()
                emit("controller", joint_impedance="restored_to_hold", operator_stop=operator_stop, recovery_attempt=recovery_attempt)
                settled_hold = read_settled_pose(arm)
                hold_z_error, hold_angle_error = top_errors(settled_hold)
                emit("hold_pose_after_handoff", z=float(settled_hold.position.z), z_error_m=hold_z_error, orientation_error_deg=hold_angle_error)
            except Exception as exc:
                emit("controller_restore_failed", detail=repr(exc))
                if failure is None and not operator_stop:
                    failure = "controller hold recovery failed: " + repr(exc)
        arm.destroy_node()
        gripper.destroy_node()
        head_camera.destroy_node()
        rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)
    if operator_stop:
        return
    if not cycle_success:
        raise RuntimeError("pick cycle exited without success")
    emit("cycle_complete", final_state=phase, controller_hold="stable")


if __name__ == "__main__":
    main()
