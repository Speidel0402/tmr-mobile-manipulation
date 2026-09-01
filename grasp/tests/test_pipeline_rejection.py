import numpy as np
import cv2

from rim_grasp_perception.config import AppConfig, GeometryConfig, GripperConfig, ModelConfig, TopicsConfig
from rim_grasp_perception.pipeline import PerceptionPipeline
from rim_grasp_perception.types import CameraIntrinsics, Detection


def _config():
    topics = TopicsConfig(*(["unused"] * 9))
    return AppConfig("left", topics, ModelConfig(), GeometryConfig(), GripperConfig(),
                     camera_optical_frame="color", depth_is_aligned_to_rgb=False)


def test_unaligned_depth_without_extrinsics_is_invalid():
    cfg = _config()
    intr = CameraIntrinsics(8, 6, 10, 10, 4, 3, [], "color")
    rgb = np.zeros((6, 8, 3), np.uint8)
    depth = np.ones((6, 8), np.float32)
    results, _ = PerceptionPipeline(cfg).process(rgb, depth, intr, intr, 1.0, None, detections=[])
    assert not results[0].valid
    assert results[0].invalid_reason == "depth_to_color_extrinsics_missing"


def test_insufficient_depth_is_invalid_without_model_inference():
    cfg = _config(); cfg.depth_is_aligned_to_rgb = True
    intr = CameraIntrinsics(20, 20, 10, 10, 10, 10, [], "color")
    rgb = np.zeros((20, 20, 3), np.uint8)
    depth = np.zeros((20, 20), np.float32)
    results, _ = PerceptionPipeline(cfg).process(rgb, depth, intr, intr, 1.0, np.eye(4), detections=[])
    assert results[0].invalid_reason == "insufficient_aligned_depth"


def _synthetic_scene(rim_depth=0.50):
    h, w = 240, 320
    rgb = np.full((h, w, 3), 90, np.uint8)
    mask = np.zeros((h, w), np.uint8); cv2.circle(mask, (160, 120), 105, 1, -1)
    cv2.ellipse(rgb, (160, 120), (70, 42), 8, 0, 360, (245, 245, 245), 3)
    depth = np.full((h, w), 0.60, np.float32)
    inside = np.zeros_like(mask); cv2.ellipse(inside, (160, 120), (66, 38), 8, 0, 360, 1, -1)
    depth[inside > 0] = 0.56
    band = np.zeros_like(mask); cv2.ellipse(band, (160, 120), (70, 42), 8, 0, 360, 1, 7)
    depth[band > 0] = rim_depth
    intr = CameraIntrinsics(w, h, 300, 300, 160, 120, [], "synthetic_color_optical")
    det = Detection("cup", .9, np.array([55, 15, 265, 225]), mask.astype(bool))
    return rgb, depth, intr, det


def test_complete_synthetic_pipeline_can_produce_valid_contact():
    cfg = _config(); cfg.depth_is_aligned_to_rgb = True; cfg.geometry.table_min_points = 300
    cfg.gripper.finger_width_m = .004; cfg.gripper.finger_thickness_m = .004; cfg.gripper.opening_margin_m = .040
    rgb, depth, intr, det = _synthetic_scene()
    results, _ = PerceptionPipeline(cfg).process(rgb, depth, intr, intr, 1.0, np.eye(4), detections=[det])
    assert results[0].valid
    assert results[0].tcp_pose is None
    assert results[0].frame_id == "synthetic_color_optical"


def test_internal_edge_at_table_height_is_rejected_as_bottom():
    cfg = _config(); cfg.depth_is_aligned_to_rgb = True; cfg.geometry.table_min_points = 300
    rgb, depth, intr, det = _synthetic_scene(rim_depth=0.60)
    depth[det.mask] = 0.60
    results, _ = PerceptionPipeline(cfg).process(rgb, depth, intr, intr, 1.0, np.eye(4), detections=[det])
    assert not results[0].valid
    assert results[0].invalid_reason == "candidate_is_table_or_bottom_edge"
