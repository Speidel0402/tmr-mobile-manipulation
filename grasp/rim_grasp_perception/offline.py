from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .camera_geometry import depth_to_meters
from .config import load_config
from .model import GroundedSAM2
from .pipeline import PerceptionPipeline
from .types import CameraIntrinsics, Detection
from .visualization import draw_overlay


def _intr(raw, default_frame="camera_color_optical_frame"):
    return CameraIntrinsics(int(raw["width"]), int(raw["height"]), float(raw["fx"]), float(raw["fy"]),
                            float(raw["cx"]), float(raw["cy"]), raw.get("distortion", []), raw.get("frame_id", default_frame))


def _matrix(raw):
    if raw is None:
        return None
    out = np.eye(4)
    out[:3, :3] = np.asarray(raw["rotation_row_major"], float).reshape(3, 3)
    out[:3, 3] = np.asarray(raw["translation_m"], float)
    return out


def _load_detections(path, shape):
    if not path:
        return None
    root = Path(path).resolve().parent
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    out = []
    for item in data:
        mask = cv2.imread(str(root / item["mask"]), cv2.IMREAD_GRAYSCALE)
        if mask is None or mask.shape != shape:
            raise ValueError(f"invalid mask: {item['mask']}")
        out.append(Detection(item["category"], float(item.get("score", 1.0)),
                             np.asarray(item["box_xyxy"], float), mask > 0))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run the exact RGB-D rim pipeline on saved files")
    ap.add_argument("--config", required=True)
    ap.add_argument("--rgb", required=True)
    ap.add_argument("--depth", required=True, help="16-bit PNG/TIFF or floating-point .npy")
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--detections-json", help="Optional masks for geometry-only validation")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    calibration = yaml.safe_load(Path(args.calibration).read_text(encoding="utf-8"))
    bgr = cv2.imread(args.rgb, cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(args.rgb)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if Path(args.depth).suffix.lower() == ".npy":
        raw_depth = np.load(args.depth)
    else:
        raw_depth = cv2.imread(args.depth, cv2.IMREAD_UNCHANGED)
    if raw_depth is None:
        raise FileNotFoundError(args.depth)
    depth_encoding = calibration.get("depth_encoding", "32FC1" if raw_depth.dtype.kind == "f" else "16UC1")
    depth = depth_to_meters(raw_depth, depth_encoding, cfg.geometry.depth_scale_m)
    supplied = _load_detections(args.detections_json, rgb.shape[:2])
    detector = None if supplied is not None else GroundedSAM2(cfg.model)
    pipeline = PerceptionPipeline(cfg, detector)
    d2c = _matrix(calibration.get("depth_to_color"))
    if d2c is None and cfg.depth_to_color:
        d2c = _matrix(cfg.depth_to_color)
    camera_to_output = _matrix(calibration.get("camera_to_output"))
    output_frame = calibration.get("output_frame", calibration["rgb_intrinsics"].get("frame_id", "camera_color_optical_frame"))
    stamp = float(calibration.get("timestamp", 0.0))
    results, debug = pipeline.process(rgb, depth, _intr(calibration["rgb_intrinsics"]),
                                      _intr(calibration["depth_intrinsics"], "camera_depth_optical_frame"),
                                      stamp, d2c, camera_to_output, output_frame, supplied)
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    (out / "result.json").write_text(json.dumps([r.to_dict() for r in results], indent=2, ensure_ascii=False), encoding="utf-8")
    overlay = draw_overlay(rgb, results, debug)
    cv2.imwrite(str(out / "overlay.png"), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))
    if "aligned_depth" in debug:
        np.save(out / "aligned_depth_m.npy", debug["aligned_depth"])
    print(json.dumps([r.to_dict() for r in results], ensure_ascii=False))


if __name__ == "__main__":
    main()
