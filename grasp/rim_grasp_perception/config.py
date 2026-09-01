from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class TopicsConfig:
    rgb: str
    depth: str
    rgb_info: str
    depth_info: str
    depth_to_color_extrinsics: str
    result_json: str
    contact_pose: str
    overlay: str
    markers: str


@dataclass
class ModelConfig:
    grounding_model: str = "IDEA-Research/grounding-dino-tiny"
    sam2_model: str = "facebook/sam2.1-hiera-small"
    model_cache: str = "models"
    local_files_only: bool = True
    categories: List[str] = field(default_factory=lambda: ["cup", "bowl", "plate"])
    box_threshold: float = 0.30
    text_threshold: float = 0.25
    min_mask_pixels: int = 500
    device: str = "cuda"


@dataclass
class GeometryConfig:
    depth_scale_m: float = 0.001
    min_depth_m: float = 0.10
    max_depth_m: float = 1.50
    sync_slop_s: float = 0.035
    target_timeout_s: float = 0.75
    tf_timeout_s: float = 0.08
    tf_max_age_s: float = 0.12
    table_distance_threshold_m: float = 0.006
    table_min_points: int = 800
    table_voxel_m: float = 0.008
    min_rim_depth_support: float = 0.55
    max_ellipse_residual: float = 0.12
    min_rim_height_m: float = 0.008
    max_rim_height_std_m: float = 0.015
    rim_band_px: int = 8
    candidate_count: int = 32
    max_unknown_fraction: float = 0.25
    obstacle_clearance_m: float = 0.010


@dataclass
class GripperConfig:
    finger_thickness_m: float = 0.010
    finger_width_m: float = 0.016
    finger_length_m: float = 0.055
    min_opening_m: float = 0.015
    max_opening_m: float = 0.090
    opening_margin_m: float = 0.006
    min_contact_height_above_table_m: float = 0.008
    max_insertion_depth_m: float = 0.035
    table_clearance_m: float = 0.010
    path_clearance_m: float = 0.012


@dataclass
class AppConfig:
    camera_id: str
    topics: TopicsConfig
    model: ModelConfig
    geometry: GeometryConfig
    gripper: GripperConfig
    common_frame: str = ""
    camera_optical_frame: str = ""
    end_effector_frame: str = ""
    require_common_frame_tf: bool = False
    depth_is_aligned_to_rgb: bool = False
    publish_markers: bool = True
    depth_to_color: Optional[Dict[str, List[float]]] = None
    contact_to_tcp: Optional[Dict[str, List[float]]] = None


def load_config(path: str) -> AppConfig:
    p = Path(path).expanduser().resolve()
    with p.open("r", encoding="utf-8") as f:
        raw: Dict[str, Any] = yaml.safe_load(f)
    raw["model"] = ModelConfig(**raw.get("model", {}))
    cache = Path(raw["model"].model_cache).expanduser()
    if not cache.is_absolute():
        raw["model"].model_cache = str((p.parent / cache).resolve())
    raw["geometry"] = GeometryConfig(**raw.get("geometry", {}))
    raw["gripper"] = GripperConfig(**raw.get("gripper", {}))
    raw["topics"] = TopicsConfig(**raw["topics"])
    return AppConfig(**raw)
