from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .camera_geometry import project_points, sample_observed_depth
from .config import GeometryConfig, GripperConfig
from .geometry import Plane, RimEstimate, project_to_plane
from .math3d import contact_orientation, normalize
from .types import CameraIntrinsics


@dataclass
class Candidate:
    rim_position: np.ndarray
    contact_position: np.ndarray
    quaternion_xyzw: np.ndarray
    approach: np.ndarray
    closing: np.ndarray
    opening_m: float
    insertion_m: float
    score: float
    index: int


def choose_grasp_candidate(
    rim: RimEstimate,
    plane: Plane,
    all_points: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    gripper: GripperConfig,
    geometry: GeometryConfig,
) -> Tuple[Optional[Candidate], str]:
    valid_xyz = rim.candidate_xyz[rim.candidate_valid]
    if len(valid_xyz) < 5:
        return None, "too_few_3d_rim_points"
    center = np.nanmean(valid_xyz, axis=0)
    up = normalize(plane.normal)
    approach = -up
    best, rejected_unknown, rejected_collision = None, 0, 0
    cloud = np.asarray(all_points, dtype=np.float64)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    for i, p in enumerate(rim.candidate_xyz):
        if not rim.candidate_valid[i]:
            rejected_unknown += 1
            continue
        inward = project_to_plane(center - p, up)
        if np.linalg.norm(inward) < 1e-6:
            continue
        closing = normalize(inward)
        opening = max(gripper.min_opening_m, 2*gripper.finger_thickness_m + gripper.opening_margin_m)
        if opening > gripper.max_opening_m:
            continue
        rim_height = float(plane.signed_distance(p[None])[0])
        available = rim_height - gripper.min_contact_height_above_table_m - gripper.table_clearance_m
        if available <= 0:
            continue
        insertion = min(gripper.max_insertion_depth_m, available)
        inner = p + closing * (0.5*opening)
        outer = p - closing * (0.5*opening)
        tangent = normalize(np.cross(up, closing))
        if insertion > gripper.finger_length_m:
            continue
        # Sweep both finite-size finger volumes from a pre-contact clearance down to
        # the final insertion depth. Ray visibility distinguishes observed free space
        # from missing depth; missing pixels are never assumed collision-free.
        samples = []
        intended_contact = []
        travel = np.linspace(-gripper.path_clearance_m, insertion, 7)
        for side, q in ((1.0, inner), (-1.0, outer)):
            for lateral in (-0.5*gripper.finger_width_m, 0.0, 0.5*gripper.finger_width_m):
                for radial in (-0.5*gripper.finger_thickness_m, 0.0, 0.5*gripper.finger_thickness_m):
                    for z in travel:
                        samples.append(q + tangent*lateral + closing*radial + approach*z)
                        intended_contact.append(abs(z) < 0.004 and abs(lateral) < 1e-9 and radial*side < 0)
        samples = np.asarray(samples)
        intended_contact = np.asarray(intended_contact, dtype=bool)
        uv, projected = project_points(samples, intrinsics)
        observed_z, observed = sample_observed_depth(uv, depth_m, radius_px=2, min_samples=2)
        known = projected & observed
        unknown_fraction = float(1.0 - known.mean())
        if unknown_fraction > geometry.max_unknown_fraction:
            rejected_unknown += 1
            continue
        occupied = known & (observed_z < samples[:, 2] - geometry.obstacle_clearance_m)
        occupied[intended_contact] = False
        if np.any(occupied):
            rejected_collision += 1
            continue
        visible_clearance = observed_z[known] - samples[known, 2]
        clearance_score = float(np.clip(np.median(visible_clearance) / (3*geometry.obstacle_clearance_m), 0, 1))
        score = 0.35*rim.depth_support + 0.25*(1-min(1, rim.ellipse_residual/geometry.max_ellipse_residual)) + 0.15*rim.edge_support + 0.25*clearance_score
        q = contact_orientation(closing, approach)
        cand = Candidate(p, p.copy(), q, approach, closing, opening, insertion, score, i)
        if best is None or cand.score > best.score:
            best = cand
    if best is None:
        if rejected_unknown:
            return None, "candidate_path_contains_unobserved_space"
        if rejected_collision:
            return None, "candidate_finger_collision"
        return None, "no_candidate_satisfies_gripper_geometry"
    return best, ""
