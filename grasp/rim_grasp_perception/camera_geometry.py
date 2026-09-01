from __future__ import annotations

from typing import Tuple

import numpy as np
import cv2

from .types import CameraIntrinsics


def depth_to_meters(depth: np.ndarray, encoding: str, scale_m: float) -> np.ndarray:
    if encoding.upper() in ("16UC1", "MONO16"):
        out = depth.astype(np.float32) * float(scale_m)
    elif encoding.upper() == "32FC1":
        out = depth.astype(np.float32)
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    out[~np.isfinite(out)] = 0.0
    out[out < 0.0] = 0.0
    return out


def deproject_depth(depth_m: np.ndarray, intr: CameraIntrinsics) -> Tuple[np.ndarray, np.ndarray]:
    v, u = np.indices(depth_m.shape, dtype=np.float32)
    z = depth_m
    valid = np.isfinite(z) & (z > 0.0)
    x = (u - intr.cx) * z / intr.fx
    y = (v - intr.cy) * z / intr.fy
    if intr.distortion and np.any(np.abs(intr.distortion) > 1e-12):
        uv = np.column_stack((u[valid], v[valid])).reshape(-1, 1, 2)
        k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], np.float64)
        und = cv2.undistortPoints(uv.astype(np.float64), k, np.asarray(intr.distortion, np.float64)).reshape(-1, 2)
        x[valid], y[valid] = und[:, 0] * z[valid], und[:, 1] * z[valid]
    return np.stack((x, y, z), axis=-1), valid


def align_depth_to_color(
    depth_m: np.ndarray,
    depth_intr: CameraIntrinsics,
    color_intr: CameraIntrinsics,
    rotation_depth_to_color: np.ndarray,
    translation_depth_to_color_m: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project depth into color pixels using a nearest-surface z-buffer; no hole filling."""
    points, valid = deproject_depth(depth_m, depth_intr)
    src = points[valid]
    if src.size == 0:
        shape = (color_intr.height, color_intr.width)
        return np.zeros(shape, np.float32), np.zeros(shape, bool)
    dst = src @ np.asarray(rotation_depth_to_color, dtype=np.float64).reshape(3, 3).T
    dst += np.asarray(translation_depth_to_color_m, dtype=np.float64).reshape(1, 3)
    z = dst[:, 2]
    good = np.isfinite(dst).all(axis=1) & (z > 0.0)
    dst, z = dst[good], z[good]
    if color_intr.distortion and np.any(np.abs(color_intr.distortion) > 1e-12):
        k = np.array([[color_intr.fx, 0, color_intr.cx], [0, color_intr.fy, color_intr.cy], [0, 0, 1]], np.float64)
        uv, _ = cv2.projectPoints(dst, np.zeros(3), np.zeros(3), k, np.asarray(color_intr.distortion, np.float64))
        uv = uv.reshape(-1, 2)
        u, v = np.rint(uv[:, 0]).astype(np.int32), np.rint(uv[:, 1]).astype(np.int32)
    else:
        u = np.rint(color_intr.fx * dst[:, 0] / z + color_intr.cx).astype(np.int32)
        v = np.rint(color_intr.fy * dst[:, 1] / z + color_intr.cy).astype(np.int32)
    good = (u >= 0) & (u < color_intr.width) & (v >= 0) & (v < color_intr.height)
    u, v, z = u[good], v[good], z[good].astype(np.float32)
    flat = v * color_intr.width + u
    order = np.lexsort((z, flat))
    flat_s = flat[order]
    first = np.ones(flat_s.size, dtype=bool)
    if flat_s.size > 1:
        first[1:] = flat_s[1:] != flat_s[:-1]
    chosen = order[first]
    aligned = np.zeros(color_intr.width * color_intr.height, np.float32)
    aligned[flat[chosen]] = z[chosen]
    aligned = aligned.reshape(color_intr.height, color_intr.width)
    return aligned, aligned > 0.0


def pixels_to_points(uv: np.ndarray, depth_m: np.ndarray, intr: CameraIntrinsics) -> Tuple[np.ndarray, np.ndarray]:
    uv = np.asarray(uv, dtype=np.float64)
    u = np.rint(uv[:, 0]).astype(int)
    v = np.rint(uv[:, 1]).astype(int)
    in_bounds = (u >= 0) & (u < depth_m.shape[1]) & (v >= 0) & (v < depth_m.shape[0])
    z = np.zeros(len(uv), dtype=np.float64)
    z[in_bounds] = depth_m[v[in_bounds], u[in_bounds]]
    valid = in_bounds & np.isfinite(z) & (z > 0.0)
    p = np.full((len(uv), 3), np.nan, dtype=np.float64)
    p[valid, 2] = z[valid]
    if intr.distortion and np.any(np.abs(intr.distortion) > 1e-12):
        k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], np.float64)
        und = cv2.undistortPoints(uv[valid].reshape(-1, 1, 2), k,
                                  np.asarray(intr.distortion, np.float64)).reshape(-1, 2)
        p[valid, 0], p[valid, 1] = und[:, 0] * z[valid], und[:, 1] * z[valid]
    else:
        p[valid, 0] = (uv[valid, 0] - intr.cx) * z[valid] / intr.fx
        p[valid, 1] = (uv[valid, 1] - intr.cy) * z[valid] / intr.fy
    return p, valid


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def project_points(points: np.ndarray, intr: CameraIntrinsics) -> Tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points, dtype=np.float64)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
    uv = np.full((len(points), 2), np.nan, dtype=np.float64)
    if not valid.any():
        return uv, valid
    k = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], np.float64)
    projected, _ = cv2.projectPoints(points[valid], np.zeros(3), np.zeros(3), k,
                                     np.asarray(intr.distortion, np.float64))
    uv[valid] = projected.reshape(-1, 2)
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < intr.width) & (uv[:, 1] >= 0) & (uv[:, 1] < intr.height)
    return uv, valid


def sample_observed_depth(uv: np.ndarray, depth_m: np.ndarray, radius_px: int = 2, min_samples: int = 2):
    values = np.zeros(len(uv), dtype=np.float64)
    valid = np.zeros(len(uv), dtype=bool)
    h, w = depth_m.shape
    for i, (uf, vf) in enumerate(np.asarray(uv)):
        u, v = int(round(uf)), int(round(vf))
        x0, x1, y0, y1 = max(0, u-radius_px), min(w, u+radius_px+1), max(0, v-radius_px), min(h, v+radius_px+1)
        patch = depth_m[y0:y1, x0:x1]
        observed = patch[np.isfinite(patch) & (patch > 0)]
        if len(observed) >= min_samples:
            values[i], valid[i] = float(np.median(observed)), True
    return values, valid
