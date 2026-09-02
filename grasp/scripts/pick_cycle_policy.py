#!/usr/bin/env python3
"""Pure decision policy for the pick cycle; deliberately ROS-independent."""

import math


STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6


def validate_camera_snapshot(
    *,
    role,
    topic,
    frame_id,
    session_id,
    image_shape,
    expected_session_id=None,
    expected_shape=(480, 640, 3),
):
    """Validate the calibrated left-wrist RGB stream without ROS imports."""
    if str(role) != "left_wrist":
        raise ValueError(f"wrong camera role: {role!r}")
    expected_topic = "/wrist_camera_left/color/image_raw"
    if str(topic) != expected_topic:
        raise ValueError(f"wrong RGB topic: {topic!r}")
    normalized_frame = str(frame_id).lower()
    # Some RealSense launch files omit the namespace from frame_id, so the
    # exact left-wrist topic/role is authoritative.  Only reject empty or
    # explicitly incompatible frames here to avoid a brittle naming gate.
    if not normalized_frame or "right" in normalized_frame or "zed" in normalized_frame:
        raise ValueError(f"wrong RGB frame: {frame_id!r}")
    session = str(session_id)
    if not session:
        raise ValueError("camera session id is empty")
    if expected_session_id is not None and session != str(expected_session_id):
        raise ValueError("camera viewer restarted during visual alignment")
    if tuple(int(value) for value in image_shape) != tuple(expected_shape):
        raise ValueError(
            f"camera resolution/profile changed: {tuple(image_shape)!r}, expected {tuple(expected_shape)!r}"
        )
    return session


def classify_close_result(
    result,
    open_position=0.0,
    close_position=0.8,
    post_position=None,
):
    """Separate action completion from physical object-contact evidence."""
    final = float(result["position"] if post_position is None else post_position)
    reported = float(result["position"])
    feedback = [float(value) for value in result.get("feedback_positions", [])]
    direction = 1.0 if close_position >= open_position else -1.0
    observed = [float(open_position), reported, final, *feedback]
    progress = max(direction * (value - open_position) for value in observed)
    required_progress = max(0.08, 0.15 * abs(close_position - open_position))
    state_consistent = abs(final - reported) <= 0.03
    moved = progress >= required_progress
    gap = abs(close_position - final)

    if not state_consistent:
        classification = "state_mismatch"
    elif not moved:
        classification = "no_motion"
    elif (
        int(result["status"]) == STATUS_ABORTED
        and bool(result["stalled"])
        # A Robotiq goal aborted because of a stall is the expected signature
        # of object contact.  Use the already scale-aware `moved` evidence;
        # the old absolute 0.20 cutoff rejected real cups at 0.178--0.196.
        and moved
        and gap >= 0.012
    ):
        classification = "object_contact_candidate"
    elif (
        int(result["status"]) == STATUS_SUCCEEDED
        and bool(result["reached_goal"])
        and 0.005 <= gap <= 0.03
    ):
        # The Robotiq controller can report reached_goal when contact occurs
        # inside its goal tolerance.  The real cup grasp observed on hardware
        # ended at 0.7929 for a 0.8000 command (7.1 mm command-space gap) and
        # visibly held the cup.  Preserve that evidence while still rejecting
        # an actually empty close at the 0.8000 mechanical endpoint.
        classification = "object_contact_within_goal_tolerance"
    elif (
        int(result["status"]) == STATUS_SUCCEEDED
        and bool(result["reached_goal"])
        and gap < 0.005
    ):
        classification = "fully_closed_unconfirmed"
    else:
        classification = "indeterminate_close"

    return {
        "accepted_as_grasp": classification in {
            "object_contact_candidate",
            "object_contact_within_goal_tolerance",
        },
        "classification": classification,
        "progress": float(progress),
        "required_progress": float(required_progress),
        "final_position": final,
        "reported_position": reported,
        "state_consistent": bool(state_consistent),
    }


def visual_tolerances(spread_px):
    """Hysteretic tolerance derived from observed frame-to-frame quantization."""
    spread = max(0.0, float(spread_px))
    enter = min(7.0, max(5.0, 2.0 * spread + 1.0))
    hold = min(8.0, enter + 2.0)
    return {"enter_px": float(enter), "hold_px": float(hold)}


def top_pose_policy(z_error_m, orientation_error_deg):
    """Small controller handoff rebound is repairable, not a hard failure."""
    z_error = abs(float(z_error_m))
    angle = abs(float(orientation_error_deg))
    if z_error <= 0.001 and angle <= 0.5:
        return "accept"
    if z_error <= 0.05 and angle <= 15.0:
        return "auto_restore"
    return "hard_fault"


def grasp_plane_policy(residual_m, nominal_tolerance_m=0.012, high_limit_m=0.020):
    """Classify the measured TCP height relative to the calibrated grasp plane.

    A positive residual is above the table and may use the empirically proven
    high-side allowance.  The same allowance is deliberately never mirrored
    below the target because that would reduce table clearance.
    """
    residual = float(residual_m)
    if abs(residual) <= float(nominal_tolerance_m):
        return "accept"
    if float(nominal_tolerance_m) < residual <= float(high_limit_m):
        return "accept_high"
    return "reject"


def close_to(value, target, tolerance):
    return math.isfinite(float(value)) and abs(float(value) - float(target)) <= float(tolerance)
