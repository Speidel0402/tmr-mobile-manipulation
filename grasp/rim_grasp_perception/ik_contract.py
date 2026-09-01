from __future__ import annotations

from typing import Iterable, Mapping, Sequence

import numpy as np


LEFT_JOINT_NAMES = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))


def ordered_joint_values(
    names: Sequence[str], positions: Sequence[float],
    required: Iterable[str] = LEFT_JOINT_NAMES,
) -> list[float]:
    """Return the required joints in a deterministic order or raise ValueError."""
    if len(names) != len(positions):
        raise ValueError("joint names and positions have different lengths")
    values: Mapping[str, float] = dict(zip(names, positions))
    missing = [name for name in required if name not in values]
    if missing:
        raise ValueError("missing joints: " + ", ".join(missing))
    result = [float(values[name]) for name in required]
    if not np.all(np.isfinite(result)):
        raise ValueError("joint seed contains non-finite values")
    return result


def normalized_quaternion_xyzw(values: Sequence[float]) -> list[float]:
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError("zero quaternion is invalid")
    return (q / norm).tolist()


def moveit_error_name(code: int) -> str:
    names = {
        1: "SUCCESS",
        -1: "FAILURE",
        -2: "PLANNING_FAILED",
        -10: "START_STATE_IN_COLLISION",
        -12: "GOAL_IN_COLLISION",
        -14: "GOAL_CONSTRAINTS_VIOLATED",
        -21: "FRAME_TRANSFORM_FAILURE",
        -31: "NO_IK_SOLUTION",
    }
    return names.get(int(code), f"MOVEIT_ERROR_{int(code)}")
