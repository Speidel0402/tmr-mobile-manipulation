#!/usr/bin/env python3
"""Pure decision policy for the pick cycle; deliberately ROS-independent."""

import math


STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6


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
        and gap <= 0.03
    ):
        classification = "fully_closed_unconfirmed"
    else:
        classification = "indeterminate_close"

    return {
        "accepted_as_grasp": classification == "object_contact_candidate",
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


def close_to(value, target, tolerance):
    return math.isfinite(float(value)) and abs(float(value) - float(target)) <= float(tolerance)
