from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .camera_geometry import pixels_to_points, sample_observed_depth
from .math3d import normalize
from .types import CameraIntrinsics


@dataclass
class Plane:
    normal: np.ndarray
    offset: float
    inliers: np.ndarray

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        return np.asarray(points) @ self.normal + self.offset


@dataclass
class RimEstimate:
    center_uv: np.ndarray
    ellipse: Tuple[Tuple[float, float], Tuple[float, float], float]
    candidate_uv: np.ndarray
    candidate_xyz: np.ndarray
    candidate_valid: np.ndarray
    depth_support: float
    ellipse_residual: float
    height_std_m: float
    edge_support: float
    debug_edges: np.ndarray


def fit_table_plane(points: np.ndarray, distance_threshold: float, min_points: int, seed: int = 7) -> Optional[Plane]:
    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < min_points:
        return None
    try:
        import open3d as o3d
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        model, ids = cloud.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=500)
        n = normalize(np.asarray(model[:3]))
        d = float(model[3]) / np.linalg.norm(model[:3])
        ids = np.asarray(ids, dtype=int)
    except ImportError:
        rng, best = np.random.default_rng(seed), np.array([], dtype=int)
        best_n, best_d = None, 0.0
        for _ in range(350):
            s = points[rng.choice(len(points), 3, replace=False)]
            n = np.cross(s[1] - s[0], s[2] - s[0])
            if np.linalg.norm(n) < 1e-8:
                continue
            n = normalize(n)
            d = -float(np.dot(n, s[0]))
            ids = np.flatnonzero(np.abs(points @ n + d) < distance_threshold)
            if len(ids) > len(best):
                best, best_n, best_d = ids, n, d
        if best_n is None:
            return None
        ids, n, d = best, best_n, best_d
    if len(ids) < min_points:
        return None
    # The camera usually observes the tabletop from above: orient normal toward camera origin.
    centroid = points[ids].mean(axis=0)
    if np.dot(n, -centroid) < 0:
        n, d = -n, -d
    return Plane(n, d, ids)


def extract_table_points(depth_m: np.ndarray, intr: CameraIntrinsics, object_masks: List[np.ndarray], stride: int = 4) -> np.ndarray:
    background = np.ones(depth_m.shape, dtype=bool)
    for mask in object_masks:
        background &= ~mask.astype(bool)
    v, u = np.nonzero(background[::stride, ::stride] & (depth_m[::stride, ::stride] > 0))
    uv = np.column_stack((u * stride, v * stride))
    pts, valid = pixels_to_points(uv, depth_m, intr)
    return pts[valid]


def _ellipse_residual(ellipse, points_uv: np.ndarray) -> float:
    (cx, cy), (major, minor), angle = ellipse
    if min(major, minor) <= 2:
        return float("inf")
    theta = np.deg2rad(angle)
    c, s = np.cos(theta), np.sin(theta)
    d = points_uv - np.array([cx, cy])
    x = c * d[:, 0] + s * d[:, 1]
    y = -s * d[:, 0] + c * d[:, 1]
    rho = np.sqrt((x/(major/2))**2 + (y/(minor/2))**2)
    return float(np.median(np.abs(rho - 1.0)))


def _sample_ellipse(ellipse, count: int) -> np.ndarray:
    (cx, cy), (a, b), angle = ellipse
    t = np.linspace(0, 2*np.pi, count, endpoint=False)
    local = np.column_stack((0.5*a*np.cos(t), 0.5*b*np.sin(t)))
    th = np.deg2rad(angle)
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return local @ rot.T + np.array([cx, cy])


def extract_rim(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    mask: np.ndarray,
    intr: CameraIntrinsics,
    plane: Plane,
    rim_band_px: int,
    candidate_count: int,
) -> Tuple[Optional[RimEstimate], str]:
    mask8 = (mask.astype(bool) * 255).astype(np.uint8)
    if cv2.countNonZero(mask8) < 100:
        return None, "mask_too_small"
    k = max(3, 2 * (rim_band_px // 2) + 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    cleaned = cv2.morphologyEx(mask8, cv2.MORPH_CLOSE, kernel)
    eroded = cv2.erode(cleaned, kernel)
    # Candidate edges must lie inside the detection, not merely on its outer silhouette.
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, 50, 140)
    # Exclude the entire outer-boundary band. Remaining edges may be a rim, but still
    # need ellipse residual, local depth, table height and height-consistency checks.
    candidates = cv2.bitwise_and(edges, eroded)
    contours, _ = cv2.findContours(candidates, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours = [c for c in contours if len(c) >= 20]
    if not contours:
        return None, "no_internal_rim_edge"
    best = None
    for contour in contours:
        pts = contour[:, 0, :].astype(np.float64)
        ellipse = cv2.fitEllipse(contour)
        residual = _ellipse_residual(ellipse, pts)
        perimeter = np.pi * (3*(sum(ellipse[1])/2) - np.sqrt((3*ellipse[1][0]/2+ellipse[1][1]/2)*(ellipse[1][0]/2+3*ellipse[1][1]/2)))
        support = min(1.0, len(pts) / max(perimeter, 1.0))
        value = residual + 0.20 * (1.0 - support)
        if best is None or value < best[0]:
            best = (value, ellipse, residual, support)
    _, ellipse, residual, edge_support = best
    uv = _sample_ellipse(ellipse, candidate_count)
    local_depth, local_valid = sample_observed_depth(uv, depth_m, radius_px=2, min_samples=2)
    sampling_depth = np.zeros_like(depth_m)
    rounded = np.rint(uv).astype(int)
    in_bounds = (rounded[:, 0] >= 0) & (rounded[:, 0] < depth_m.shape[1]) & (rounded[:, 1] >= 0) & (rounded[:, 1] < depth_m.shape[0])
    ids = np.flatnonzero(local_valid & in_bounds)
    sampling_depth[rounded[ids, 1], rounded[ids, 0]] = local_depth[ids]
    xyz, valid = pixels_to_points(uv, sampling_depth, intr)
    support = float(valid.mean())
    if not valid.any():
        return None, "rim_has_no_reliable_depth"
    heights = plane.signed_distance(xyz[valid])
    height_std = float(np.std(heights))
    return RimEstimate(
        center_uv=np.asarray(ellipse[0]), ellipse=ellipse, candidate_uv=uv,
        candidate_xyz=xyz, candidate_valid=valid, depth_support=support,
        ellipse_residual=residual, height_std_m=height_std,
        edge_support=edge_support, debug_edges=candidates,
    ), ""


def project_to_plane(v: np.ndarray, normal: np.ndarray) -> np.ndarray:
    return np.asarray(v) - normal * np.dot(v, normal)
