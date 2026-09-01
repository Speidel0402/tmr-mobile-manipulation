#!/usr/bin/env python3
"""Detect cup, dark-granule bowl, and plate with strict RGB multi-frame gates."""

from __future__ import annotations

import argparse
from io import BytesIO
import json
import math
from pathlib import Path
import time
from urllib.request import urlopen

import cv2
import numpy as np

from three_object_detector import (
    CLASS_LABELS,
    aggregate_detections,
    detect_three_objects,
    draw_detection_overlay,
    serializable_result,
)


def load_snapshot(path: str, url: str):
    if path:
        data = np.load(path)
    else:
        with urlopen(url, timeout=3.0) as response:
            data = np.load(BytesIO(response.read()))
    return {
        "rgb": data["rgb"],
        "depth": data["depth"],
        "camera_k": data["camera_k"],
        "depth_scale_m": float(data["depth_scale_m"]),
        "rgb_stamp": float(data["rgb_stamp"]),
        "depth_stamp": float(data["depth_stamp"]),
    }


def depth_preview(depth_m: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 2.0)
    preview = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        lower, upper = np.percentile(depth_m[valid], [2.0, 98.0])
        if upper > lower:
            preview[valid] = np.clip((depth_m[valid] - lower) * 255.0 / (upper - lower), 0, 255).astype(np.uint8)
    return cv2.applyColorMap(255 - preview, cv2.COLORMAP_TURBO)


def capture_and_detect(args):
    requested_frames = 1 if args.snapshot else int(args.frames)
    deadline = time.monotonic() + float(args.capture_timeout_s)
    samples = []
    detections = []
    processing_ms = []
    seen_stamps = set()
    capture_rejections = []
    detector_warmed = False

    while len(samples) < requested_frames and time.monotonic() < deadline:
        try:
            sample = load_snapshot(args.snapshot, args.snapshot_url)
        except Exception as exc:
            capture_rejections.append(f"snapshot_retry_{type(exc).__name__}")
            if args.snapshot:
                break
            time.sleep(0.03)
            continue
        stamp_key = round(sample["rgb_stamp"], 6)
        if not args.snapshot and stamp_key in seen_stamps:
            time.sleep(0.02)
            continue
        seen_stamps.add(stamp_key)
        sync_offset = abs(sample["rgb_stamp"] - sample["depth_stamp"])
        if not args.snapshot:
            frame_age = time.time() - sample["rgb_stamp"]
            if frame_age < -0.05 or frame_age > args.max_frame_age_s:
                capture_rejections.append(f"stale_frame_age_{frame_age:.3f}s")
                time.sleep(0.02)
                continue

        # One-time OpenCV/Hough initialization is not part of the steady-state
        # onboard frame budget.  The deployed process stays resident, so warm it
        # once before recording latency or temporal evidence.
        if not detector_warmed:
            detect_three_objects(sample["rgb"])
            detector_warmed = True
        started = time.perf_counter()
        result = detect_three_objects(sample["rgb"])
        elapsed_ms = 1000.0 * (time.perf_counter() - started)
        result["rgb_stamp"] = sample["rgb_stamp"]
        result["depth_stamp"] = sample["depth_stamp"]
        result["rgb_depth_time_offset_s"] = sync_offset
        result["processing_time_ms"] = elapsed_ms
        samples.append(sample)
        detections.append(result)
        processing_ms.append(elapsed_ms)
        if args.snapshot:
            break
        time.sleep(float(args.frame_interval_s))

    return samples, detections, processing_ms, capture_rejections


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", default="", help="Saved RGB-D snapshot.npz")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    parser.add_argument("--output-dir", default="outputs/three_table_objects")
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--frame-interval-s", type=float, default=0.035)
    parser.add_argument("--capture-timeout-s", type=float, default=1.5)
    parser.add_argument("--max-sync-offset-s", type=float, default=0.035)
    parser.add_argument("--max-frame-age-s", type=float, default=0.25)
    parser.add_argument("--max-p95-processing-ms", type=float, default=100.0)
    args = parser.parse_args()
    if not 1 <= args.frames <= 9:
        raise RuntimeError("frames must be in [1, 9]")

    samples, detections, processing_ms, capture_rejections = capture_and_detect(args)
    requested_frames = 1 if args.snapshot else args.frames
    minimum_valid = 1 if requested_frames == 1 else max(4, int(math.ceil(0.8 * requested_frames)))
    if not detections:
        aggregate = {
            "valid": False,
            "invalid_reason": "no_fresh_synchronized_frames",
            "objects": [],
            "frame_count": 0,
            "valid_frame_count": 0,
        }
    else:
        aggregate = aggregate_detections(detections, minimum_valid_frames=minimum_valid)

    p95_ms = float(np.percentile(processing_ms, 95)) if processing_ms else math.inf
    mean_ms = float(np.mean(processing_ms)) if processing_ms else math.inf
    invalid_reasons = []
    if len(detections) != requested_frames:
        invalid_reasons.append(f"captured_{len(detections)}_of_{requested_frames}_requested_frames")
    if not aggregate.get("valid"):
        invalid_reasons.append(aggregate.get("invalid_reason", "temporal_validation_failed"))
    if p95_ms > args.max_p95_processing_ms:
        invalid_reasons.append(f"processing_p95_{p95_ms:.1f}ms_exceeds_budget")
    categories = sorted(item["category"] for item in aggregate.get("objects", []))
    if categories and categories != sorted(CLASS_LABELS):
        invalid_reasons.append("class_set_not_unique_and_complete")

    payload = {
        "valid": not invalid_reasons,
        "invalid_reason": ";".join(reason for reason in invalid_reasons if reason),
        "camera_id": "left_wrist",
        "frame_id": "wrist_camera_left_color_optical_frame",
        "perception_mode": "rgb_only",
        "classification_height_requirement_m": {
            "reference": 0.59641,
            "tolerance": 0.001,
            "verified_by_this_script": False,
        },
        "objects": aggregate.get("objects", []) if not invalid_reasons else [],
        "diagnostic_last_frame_objects": serializable_result(detections[-1]).get("objects", []) if detections else [],
        "frame_count": len(detections),
        "valid_frame_count": aggregate.get("valid_frame_count", 0),
        "minimum_valid_frames": minimum_valid,
        "temporal_validated": requested_frames >= 3 and aggregate.get("valid", False),
        "processing_time_ms": {
            "mean": mean_ms,
            "p95": p95_ms,
            "per_frame": processing_ms,
        },
        "capture_rejections": capture_rejections,
        "rgb_depth_time_offset_s_max": max(
            (item.get("rgb_depth_time_offset_s", 0.0) for item in detections), default=None
        ),
        "motion_profiles": {
            "cup": {
                "profile": "cup_right_edge_v1",
                "motion_validated": True,
                "descent_m": 0.33,
                "right_edge_target_px": [293.905848, 167.921509],
            },
            "bean_bowl": {"profile": None, "motion_validated": False},
            "plate": {"profile": None, "motion_validated": False},
        },
        "action_ready": False,
        "fail_closed": True,
        "note": "Classification is not permission to move. Bowl/plate motion profiles remain uncalibrated.",
    }

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if samples:
        sample = samples[-1]
        last = detections[-1]
        display = dict(last)
        if invalid_reasons:
            display["valid"] = False
            display["invalid_reason"] = payload["invalid_reason"]
        overlay = draw_detection_overlay(sample["rgb"], display)
        depth_m = sample["depth"].astype(np.float32) * sample["depth_scale_m"]
        cv2.imwrite(str(output_dir / "rgb.jpg"), sample["rgb"])
        cv2.imwrite(str(output_dir / "depth_preview.jpg"), depth_preview(depth_m))
        cv2.imwrite(str(output_dir / "overlay.jpg"), overlay)
        debug = last.get("debug", {})
        if debug.get("tray_mask") is not None:
            cv2.imwrite(str(output_dir / "tray_mask.png"), debug["tray_mask"])
        if debug.get("edges") is not None:
            cv2.imwrite(str(output_dir / "edges.png"), debug["edges"])
    (output_dir / "result.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"Artifacts: {output_dir}")
    raise SystemExit(0 if payload["valid"] else 2)


if __name__ == "__main__":
    main()
