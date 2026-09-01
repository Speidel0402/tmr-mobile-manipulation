from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from .camera_geometry import align_depth_to_color, deproject_depth, transform_points
from .config import AppConfig
from .geometry import extract_rim, extract_table_points, fit_table_plane
from .gripper import choose_grasp_candidate
from .math3d import matrix_to_quaternion, pose_matrix
from .types import CameraIntrinsics, Detection, GraspResult, PoseValue


class PerceptionPipeline:
    def __init__(self, config: AppConfig, detector=None):
        self.cfg = config
        self.detector = detector

    def process(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        rgb_intr: CameraIntrinsics,
        depth_intr: CameraIntrinsics,
        stamp_s: float,
        depth_to_color: Optional[np.ndarray],
        camera_to_output: Optional[np.ndarray] = None,
        output_frame: Optional[str] = None,
        detections: Optional[List[Detection]] = None,
    ) -> Tuple[List[GraspResult], Dict]:
        if rgb.shape[:2] != (rgb_intr.height, rgb_intr.width):
            return [self._invalid(stamp_s, "", "", "rgb_intrinsics_resolution_mismatch")], {}
        if depth_m.shape != (depth_intr.height, depth_intr.width):
            return [self._invalid(stamp_s, "", "", "depth_intrinsics_resolution_mismatch")], {}
        if self.cfg.depth_is_aligned_to_rgb:
            aligned = depth_m.astype(np.float32, copy=True)
            aligned_valid = aligned > 0
        else:
            if depth_to_color is None:
                return [self._invalid(stamp_s, "", "", "depth_to_color_extrinsics_missing")], {}
            aligned, aligned_valid = align_depth_to_color(
                depth_m, depth_intr, rgb_intr, depth_to_color[:3, :3], depth_to_color[:3, 3])
        valid_range = (aligned >= self.cfg.geometry.min_depth_m) & (aligned <= self.cfg.geometry.max_depth_m)
        aligned[~valid_range] = 0.0
        aligned_valid &= valid_range
        if float(aligned_valid.mean()) < 0.05:
            return [self._invalid(stamp_s, "", "", "insufficient_aligned_depth")], {"aligned_depth": aligned}
        if detections is None:
            if self.detector is None:
                return [self._invalid(stamp_s, "", "", "detector_not_loaded")], {"aligned_depth": aligned}
            detections = self.detector.infer(rgb)
        if not detections:
            return [self._invalid(stamp_s, "", "", "no_target_detection")], {"aligned_depth": aligned}
        table_pts = extract_table_points(aligned, rgb_intr, [d.mask for d in detections])
        plane = fit_table_plane(table_pts, self.cfg.geometry.table_distance_threshold_m,
                                self.cfg.geometry.table_min_points)
        if plane is None:
            return [self._invalid(stamp_s, "", "", "table_plane_unreliable")], {"aligned_depth": aligned}
        all_grid, all_valid = deproject_depth(aligned, rgb_intr)
        all_points = all_grid[all_valid]
        results, debug = [], {"aligned_depth": aligned, "detections": detections, "rims": []}
        for idx, det in enumerate(detections):
            object_id = f"frame-{int(stamp_s*1e9)}-{idx}"
            rim, reason = extract_rim(rgb, aligned, det.mask, rgb_intr, plane,
                                      self.cfg.geometry.rim_band_px, self.cfg.geometry.candidate_count)
            if rim is None:
                results.append(self._invalid(stamp_s, object_id, det.category, reason, det.score))
                continue
            debug["rims"].append(rim)
            if rim.depth_support < self.cfg.geometry.min_rim_depth_support:
                results.append(self._invalid(stamp_s, object_id, det.category, "rim_depth_support_too_low", det.score,
                                             {"rim_depth_support": rim.depth_support}))
                continue
            if rim.ellipse_residual > self.cfg.geometry.max_ellipse_residual:
                results.append(self._invalid(stamp_s, object_id, det.category, "rim_ellipse_fit_unreliable", det.score,
                                             {"ellipse_residual": rim.ellipse_residual}))
                continue
            rim_heights = plane.signed_distance(rim.candidate_xyz[rim.candidate_valid])
            if float(np.median(rim_heights)) < self.cfg.geometry.min_rim_height_m:
                results.append(self._invalid(stamp_s, object_id, det.category, "candidate_is_table_or_bottom_edge", det.score))
                continue
            if rim.height_std_m > self.cfg.geometry.max_rim_height_std_m:
                results.append(self._invalid(stamp_s, object_id, det.category, "rim_3d_height_inconsistent", det.score))
                continue
            candidate, reason = choose_grasp_candidate(rim, plane, all_points, aligned, rgb_intr,
                                                        self.cfg.gripper, self.cfg.geometry)
            if candidate is None:
                results.append(self._invalid(stamp_s, object_id, det.category, reason, det.score))
                continue
            out_frame = output_frame or rgb_intr.frame_id
            p, closing, approach, q = candidate.contact_position, candidate.closing, candidate.approach, candidate.quaternion_xyzw
            rim_p = candidate.rim_position.copy()
            contact_tf = pose_matrix(p, q)
            if camera_to_output is not None:
                rim_p = transform_points(rim_p[None], camera_to_output)[0]
                contact_tf = camera_to_output @ contact_tf
                p, q = contact_tf[:3, 3], matrix_to_quaternion(contact_tf[:3, :3])
                closing = camera_to_output[:3, :3] @ closing
                approach = camera_to_output[:3, :3] @ approach
            tcp_pose = None
            if self.cfg.contact_to_tcp is not None:
                ctt = pose_matrix(self.cfg.contact_to_tcp["translation_m"], self.cfg.contact_to_tcp["quaternion_xyzw"])
                tcp = contact_tf @ ctt
                tcp_pose = PoseValue(tcp[:3, 3].tolist(), matrix_to_quaternion(tcp[:3, :3]).tolist())
            results.append(GraspResult(
                timestamp=stamp_s, frame_id=out_frame, camera_id=self.cfg.camera_id,
                object_id=object_id, category=det.category, valid=True, invalid_reason="",
                rim_position=rim_p.tolist(), contact_pose=PoseValue(p.tolist(), q.tolist()),
                approach_direction=approach.tolist(), closing_direction=closing.tolist(), tcp_pose=tcp_pose,
                pregrasp_opening_width_m=float(candidate.opening_m), insertion_depth_m=float(candidate.insertion_m),
                detection_score=float(det.score), geometry_score=float(candidate.score),
                diagnostics={"rim_depth_support": rim.depth_support, "ellipse_residual": rim.ellipse_residual,
                             "rim_height_std_m": rim.height_std_m, "edge_support": rim.edge_support,
                             "local_collision_check_only": True, "arm_reachability_checked": False},
            ))
        return results, debug

    def _invalid(self, stamp, object_id, category, reason, score=0.0, diagnostics=None):
        frame = self.cfg.common_frame or self.cfg.camera_optical_frame
        return GraspResult(timestamp=stamp, frame_id=frame, camera_id=self.cfg.camera_id,
                           object_id=object_id, category=category, valid=False,
                           invalid_reason=reason, detection_score=float(score),
                           diagnostics=diagnostics or {})
