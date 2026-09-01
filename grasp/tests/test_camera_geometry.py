import numpy as np

from rim_grasp_perception.camera_geometry import (
    align_depth_to_color,
    depth_to_meters,
    pixels_to_points,
    transform_points,
)
from rim_grasp_perception.types import CameraIntrinsics


def test_depth_scale_16uc1_and_invalid_zero():
    raw = np.array([[0, 1000, 2500]], dtype=np.uint16)
    out = depth_to_meters(raw, "16UC1", 0.001)
    np.testing.assert_allclose(out, [[0.0, 1.0, 2.5]])


def test_identity_alignment_preserves_valid_depth():
    intr = CameraIntrinsics(3, 2, 100.0, 100.0, 1.0, 0.5, [])
    depth = np.array([[1.0, 1.1, 1.2], [1.3, 0.0, 1.5]], np.float32)
    aligned, valid = align_depth_to_color(depth, intr, intr, np.eye(3), np.zeros(3))
    np.testing.assert_allclose(aligned, depth)
    np.testing.assert_array_equal(valid, depth > 0)


def test_pixel_deprojection_and_rigid_transform():
    intr = CameraIntrinsics(4, 4, 2.0, 2.0, 1.0, 1.0, [])
    depth = np.zeros((4, 4), np.float32); depth[1, 3] = 2.0
    p, valid = pixels_to_points(np.array([[3.0, 1.0]]), depth, intr)
    assert valid[0]
    np.testing.assert_allclose(p[0], [2.0, 0.0, 2.0])
    t = np.eye(4); t[:3, 3] = [1, 2, 3]
    np.testing.assert_allclose(transform_points(p, t)[0], [3, 2, 5])
