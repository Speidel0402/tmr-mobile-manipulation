from __future__ import annotations

import numpy as np


def normalize(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError("cannot normalize near-zero vector")
    return v / n


def pose_matrix(position, quaternion_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError("invalid zero quaternion")
    s = 2.0 / n
    r = np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ])
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = r, np.asarray(position, dtype=np.float64)
    return out


def matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=np.float64)[:3, :3]
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s, 0.25*s])
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = np.sqrt(1+r[0,0]-r[1,1]-r[2,2])*2
            q = np.array([0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s, (r[2,1]-r[1,2])/s])
        elif i == 1:
            s = np.sqrt(1+r[1,1]-r[0,0]-r[2,2])*2
            q = np.array([(r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s, (r[0,2]-r[2,0])/s])
        else:
            s = np.sqrt(1+r[2,2]-r[0,0]-r[1,1])*2
            q = np.array([(r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s, (r[1,0]-r[0,1])/s])
    return q / np.linalg.norm(q)


def contact_orientation(closing_direction: np.ndarray, approach_direction: np.ndarray) -> np.ndarray:
    """Pose convention: +X closes inward, -Z is the approach/travel direction."""
    approach = normalize(approach_direction)
    z_axis = -approach
    x_raw = np.asarray(closing_direction) - z_axis * np.dot(closing_direction, z_axis)
    x_axis = normalize(x_raw)
    y_axis = normalize(np.cross(z_axis, x_axis))
    x_axis = normalize(np.cross(y_axis, z_axis))
    return matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))
