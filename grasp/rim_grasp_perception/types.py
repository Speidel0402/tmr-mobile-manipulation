from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Sequence[float] = field(default_factory=list)
    frame_id: str = ""


@dataclass
class Detection:
    category: str
    score: float
    box_xyxy: np.ndarray
    mask: np.ndarray


@dataclass
class PoseValue:
    position: List[float]
    orientation_xyzw: List[float]


@dataclass
class GraspResult:
    timestamp: float
    frame_id: str
    camera_id: str
    object_id: str
    category: str
    valid: bool
    invalid_reason: str
    rim_position: Optional[List[float]] = None
    contact_pose: Optional[PoseValue] = None
    approach_direction: Optional[List[float]] = None
    closing_direction: Optional[List[float]] = None
    tcp_pose: Optional[PoseValue] = None
    pregrasp_opening_width_m: Optional[float] = None
    insertion_depth_m: Optional[float] = None
    detection_score: float = 0.0
    geometry_score: float = 0.0
    diagnostics: Dict[str, Any] = field(default_factory=dict)
    id_semantics: str = "frame-local detection id; not a persistent tracking id"
    coordinate_semantics: Dict[str, str] = field(default_factory=lambda: {
        "rim_position": "observed 3D point on the vessel rim surface",
        "contact_pose": "planned center between inner and outer finger contacts",
        "tcp_pose": "robot TCP target only when a calibrated contact-to-TCP transform is configured",
    })

    def to_dict(self) -> Dict[str, Any]:
        return _native(asdict(self))


def _native(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {k: _native(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_native(v) for v in value]
    return value
