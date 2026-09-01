import cv2
import numpy as np

from rim_grasp_perception.geometry import Plane, extract_rim, fit_table_plane
from rim_grasp_perception.types import CameraIntrinsics


def test_table_plane_fit_and_normal_points_to_camera():
    rng = np.random.default_rng(2)
    xy = rng.uniform(-0.4, 0.4, (2000, 2))
    z = 0.6 + rng.normal(0, 0.001, 2000)
    plane = fit_table_plane(np.column_stack((xy, z)), 0.004, 1000)
    assert plane is not None
    assert plane.normal[2] < -0.98
    assert abs(np.median(plane.signed_distance(np.column_stack((xy, z))))) < 0.003


def test_internal_ellipse_generates_rim_not_mask_outer_boundary():
    h, w = 240, 320
    rgb = np.full((h, w, 3), 90, np.uint8)
    mask = np.zeros((h, w), np.uint8)
    cv2.circle(mask, (160, 120), 105, 1, -1)
    cv2.ellipse(rgb, (160, 120), (70, 42), 8, 0, 360, (245, 245, 245), 3)
    depth = np.full((h, w), 0.60, np.float32)
    band = np.zeros_like(mask)
    cv2.ellipse(band, (160, 120), (70, 42), 8, 0, 360, 1, 7)
    depth[band > 0] = 0.50
    intr = CameraIntrinsics(w, h, 300, 300, 160, 120, [])
    plane = Plane(np.array([0.0, 0.0, -1.0]), 0.60, np.arange(1000))
    rim, reason = extract_rim(rgb, depth, mask, intr, plane, 8, 32)
    assert reason == ""
    assert rim is not None
    assert np.linalg.norm(rim.center_uv - [160, 120]) < 3
    assert rim.depth_support > 0.9
    assert np.median(plane.signed_distance(rim.candidate_xyz[rim.candidate_valid])) > 0.08
