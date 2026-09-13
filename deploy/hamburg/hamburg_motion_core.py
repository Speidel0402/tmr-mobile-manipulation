#!/usr/bin/env python3
"""Pure math for Hamburg's native Franka joint-target command path.

The controller topic happens to contain ``gello`` in its deployed name.  This
module does not use a GELLO leader or teleoperation; it computes autonomous
FR3v2 targets and validates them against the official robot model.  The runtime
publisher converts these robot-space targets to Hamburg's activation-referenced,
direction-mapped input convention.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


# Franka Arm3Rv2 values from the official franka_description fr3v2 model.
FR3V2_JOINT_LIMITS_RAD = np.asarray([
    [-2.9007400167, 2.9007400167],
    [-1.8360900167, 1.8360900167],
    [-2.9007400167, 2.9007400167],
    [-3.0770200167, -0.1169370833],
    [-2.8763033500, 2.8763033500],
    [0.4398226500, 4.6216333500],
    [-3.0508333500, 3.0508333500],
], dtype=float)

# URDF joint-origin transforms, followed by the fixed link7 -> link8 offset.
FR3V2_ORIGINS = (
    ((0.0, 0.0, 0.333), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, -0.316, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.0825, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((-0.0825, 0.384, 0.0), (-math.pi / 2.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
    ((0.088, 0.0, 0.0), (math.pi / 2.0, 0.0, 0.0)),
)


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion_matrix_xyzw(value: list[float] | tuple[float, ...]) -> np.ndarray:
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite values")
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def link0_to_link8(joints: list[float] | np.ndarray) -> np.ndarray:
    q = np.asarray(joints, dtype=float)
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError("expected seven finite FR3v2 joints")
    transform = np.eye(4)
    for angle, (xyz, rpy) in zip(q, FR3V2_ORIGINS):
        origin = np.eye(4)
        origin[:3, :3] = _rz(rpy[2]) @ _ry(rpy[1]) @ _rx(rpy[0])
        origin[:3, 3] = xyz
        rotation = np.eye(4)
        rotation[:3, :3] = _rz(float(angle))
        transform = transform @ origin @ rotation
    link8 = np.eye(4)
    link8[:3, 3] = (0.0, 0.0, 0.107)
    return transform @ link8


def derive_mount_rotation(reference_joints: list[float],
                          reference_quaternion_xyzw: list[float]) -> np.ndarray:
    """Recover the robot-base to arm-link0 rotation from the Shanghai record."""
    link_rotation = link0_to_link8(reference_joints)[:3, :3]
    base_rotation = quaternion_matrix_xyzw(reference_quaternion_xyzw)
    return base_rotation @ link_rotation.T


def base_relative_pose(joints: list[float] | np.ndarray,
                       mount_rotation: np.ndarray) -> np.ndarray:
    pose = link0_to_link8(joints)
    pose[:3, :3] = mount_rotation @ pose[:3, :3]
    pose[:3, 3] = mount_rotation @ pose[:3, 3]
    return pose


def _rotation_vector(matrix: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    skew = np.asarray([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ])
    if angle < 1.0e-8:
        return 0.5 * skew
    return angle * skew / (2.0 * math.sin(angle))


def _pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    return np.concatenate((
        target[:3, 3] - current[:3, 3],
        _rotation_vector(target[:3, :3] @ current[:3, :3].T),
    ))


def numerical_jacobian(joints: np.ndarray, mount_rotation: np.ndarray,
                       step_rad: float = 1.0e-5) -> np.ndarray:
    current = base_relative_pose(joints, mount_rotation)
    result = np.zeros((6, 7), dtype=float)
    for index in range(7):
        changed = joints.copy()
        changed[index] += step_rad
        pose = base_relative_pose(changed, mount_rotation)
        result[:3, index] = (pose[:3, 3] - current[:3, 3]) / step_rad
        result[3:, index] = _rotation_vector(
            pose[:3, :3] @ current[:3, :3].T
        ) / step_rad
    return result


def solve_pose(seed: list[float] | np.ndarray, target: np.ndarray,
               mount_rotation: np.ndarray, *, maximum_iterations: int = 160) -> np.ndarray:
    q = np.asarray(seed, dtype=float).copy()
    lower, upper = FR3V2_JOINT_LIMITS_RAD[:, 0], FR3V2_JOINT_LIMITS_RAD[:, 1]
    if np.any(q < lower - 1.0e-6) or np.any(q > upper + 1.0e-6):
        raise ValueError("seed is outside the official FR3v2 joint limits")
    for _ in range(maximum_iterations):
        pose = base_relative_pose(q, mount_rotation)
        error = _pose_error(pose, target)
        if np.linalg.norm(error[:3]) <= 3.0e-4 and np.linalg.norm(error[3:]) <= 2.0e-3:
            return q
        jacobian = numerical_jacobian(q, mount_rotation)
        damping = 0.012
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping * damping * np.eye(6), error
        )
        maximum = float(np.max(np.abs(delta)))
        if maximum > 0.035:
            delta *= 0.035 / maximum
        q = np.clip(q + delta, lower + 1.0e-6, upper - 1.0e-6)
    error = _pose_error(base_relative_pose(q, mount_rotation), target)
    raise ValueError(
        "FR3v2 IK did not converge: position={:.6f}m orientation={:.6f}rad".format(
            float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        )
    )


def cartesian_waypoints(seed: list[float], delta_xyz_m: list[float],
                        mount_rotation: np.ndarray, step_m: float = 0.010) -> list[list[float]]:
    delta = np.asarray(delta_xyz_m, dtype=float)
    if delta.shape != (3,) or not np.all(np.isfinite(delta)):
        raise ValueError("Cartesian delta must contain three finite metres")
    distance = float(np.linalg.norm(delta))
    if distance > 0.40:
        raise ValueError("Cartesian move exceeds the 0.40 m trial bound")
    if distance < 1.0e-9:
        return []
    count = max(1, int(math.ceil(distance / step_m)))
    start_q = np.asarray(seed, dtype=float)
    start_pose = base_relative_pose(start_q, mount_rotation)
    q = start_q.copy()
    result: list[list[float]] = []
    for index in range(1, count + 1):
        target = start_pose.copy()
        target[:3, 3] = start_pose[:3, 3] + delta * (index / count)
        q = solve_pose(q, target, mount_rotation)
        result.append(q.tolist())
    return result


def smooth_joint_targets(start: list[float], target: list[float], rate_hz: float,
                         maximum_velocity_rad_s: float) -> list[list[float]]:
    first, last = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
    if first.shape != (7,) or last.shape != (7,) or not np.all(np.isfinite(first + last)):
        raise ValueError("joint interpolation needs two finite 7D vectors")
    if rate_hz <= 0.0 or maximum_velocity_rad_s <= 0.0:
        raise ValueError("rate and joint velocity must be positive")
    # smoothstep has a peak derivative of 1.5 at its midpoint.  Account for
    # that peak so the per-sample command never exceeds the configured speed.
    duration = 1.5 * float(np.max(np.abs(last - first))) / maximum_velocity_rad_s
    count = max(1, int(math.ceil(duration * rate_hz)))
    result = []
    for index in range(1, count + 1):
        u = index / count
        blend = u * u * (3.0 - 2.0 * u)
        result.append((first + blend * (last - first)).tolist())
    return result


def gripper_contact_report(empty_closed: list[float], object_closed: list[float],
                           retained_closed: list[float], minimum_delta: float) -> dict[str, Any]:
    empty = np.asarray(empty_closed, dtype=float)
    contact = np.asarray(object_closed, dtype=float)
    retained = np.asarray(retained_closed, dtype=float)
    if empty.size == 0 or empty.shape != contact.shape or contact.shape != retained.shape:
        raise ValueError("gripper feedback vectors must be non-empty and have equal shape")
    contact_delta = float(np.max(np.abs(contact - empty)))
    retention_delta = float(np.max(np.abs(retained - empty)))
    drift = float(np.max(np.abs(retained - contact)))
    accepted = contact_delta >= minimum_delta and retention_delta >= minimum_delta and drift <= max(
        minimum_delta, contact_delta * 0.45
    )
    return {
        "empty_closed": empty.tolist(),
        "object_closed": contact.tolist(),
        "after_lift": retained.tolist(),
        "contact_delta": contact_delta,
        "retention_delta": retention_delta,
        "post_lift_drift": drift,
        "minimum_contact_delta": minimum_delta,
        "accepted_as_held": accepted,
    }
