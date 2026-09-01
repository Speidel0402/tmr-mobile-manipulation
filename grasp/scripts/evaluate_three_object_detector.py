#!/usr/bin/env python3
"""Offline perturbation and fail-closed evaluation for three_object_detector."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from three_object_detector import CLASS_LABELS, detect_three_objects


def photometric_variants(bgr: np.ndarray):
    variants = [("original", bgr.copy())]
    for factor in (0.72, 0.84, 1.16, 1.28):
        variants.append((f"brightness_{factor:.2f}", np.clip(bgr.astype(np.float32) * factor, 0, 255).astype(np.uint8)))
    for gamma in (0.72, 1.35):
        lut = np.asarray([np.clip((value / 255.0) ** gamma * 255.0, 0, 255) for value in range(256)], dtype=np.uint8)
        variants.append((f"gamma_{gamma:.2f}", cv2.LUT(bgr, lut)))
    for kernel in (3, 5, 7):
        variants.append((f"blur_{kernel}", cv2.GaussianBlur(bgr, (kernel, kernel), 0)))
    rng = np.random.default_rng(20260831)
    for sigma in (3.0, 6.0, 9.0):
        noise = rng.normal(0.0, sigma, bgr.shape)
        variants.append((f"noise_{sigma:.0f}", np.clip(bgr.astype(np.float32) + noise, 0, 255).astype(np.uint8)))
    for quality in (75, 55):
        ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            variants.append((f"jpeg_{quality}", cv2.imdecode(encoded, cv2.IMREAD_COLOR)))
    channel_scales = (
        ("warm", (0.88, 1.00, 1.14)),
        ("cool", (1.15, 1.00, 0.88)),
    )
    for name, scales in channel_scales:
        adjusted = bgr.astype(np.float32) * np.asarray(scales, dtype=np.float32)[None, None, :]
        variants.append((f"white_balance_{name}", np.clip(adjusted, 0, 255).astype(np.uint8)))
    height, width = bgr.shape[:2]
    gradient = np.linspace(0.74, 1.12, width, dtype=np.float32)[None, :, None]
    variants.append(("illumination_gradient", np.clip(bgr.astype(np.float32) * gradient, 0, 255).astype(np.uint8)))
    return variants


def negative_variants(bgr: np.ndarray, depth_m: np.ndarray, baseline: dict):
    objects = {item["category"]: item for item in baseline["objects"]}
    negatives = []
    tray_color = tuple(int(value) for value in np.median(bgr[80:110, 330:380], axis=(0, 1)))
    tray_depth = float(np.median(depth_m[80:110, 330:380]))
    for label in CLASS_LABELS:
        image = bgr.copy()
        depth = depth_m.copy()
        center = tuple(int(round(value)) for value in objects[label]["center_px"])
        radius = int(round(1.28 * objects[label]["rim_radius_px"]))
        cv2.circle(image, center, radius, tray_color, -1, cv2.LINE_AA)
        cv2.circle(depth, center, radius, tray_depth, -1, cv2.LINE_AA)
        negatives.append((f"missing_{label}", image, depth))

    image = bgr.copy()
    depth = depth_m.copy()
    cup = objects["cup"]
    cx, cy = (int(round(value)) for value in cup["center_px"])
    radius = int(round(cup["rim_radius_px"]))
    cv2.rectangle(image, (cx - radius - 4, cy - 5), (cx + radius + 4, cy + radius + 8), tray_color, -1)
    cv2.rectangle(depth, (cx - radius - 4, cy - 5), (cx + radius + 4, cy + radius + 8), tray_depth, -1)
    negatives.append(("cup_half_occluded", image, depth))

    image = bgr.copy()
    depth = depth_m.copy()
    cv2.rectangle(image, (200, 80), (500, 300), (230, 230, 230), -1)
    cv2.rectangle(depth, (200, 80), (500, 300), tray_depth, -1)
    negatives.append(("tray_heavily_occluded", image, depth))
    return negatives


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--output", default="outputs/three_object_detector_robustness.json")
    args = parser.parse_args()
    data = np.load(args.snapshot)
    bgr = data["rgb"]
    depth_m = data["depth"].astype(np.float32) * float(data["depth_scale_m"])
    camera_k = data["camera_k"]

    # Warm OpenCV before recording steady-state runtime.
    detect_three_objects(bgr)
    baseline = detect_three_objects(bgr)
    if not baseline["valid"]:
        raise RuntimeError(f"baseline invalid: {baseline['invalid_reason']}")

    accepted_results = []
    accepted_times = []
    for name, image in photometric_variants(bgr):
        started = time.perf_counter()
        result = detect_three_objects(image)
        accepted_times.append(1000.0 * (time.perf_counter() - started))
        labels = sorted(item["category"] for item in result.get("objects", []))
        accepted_results.append(
            {
                "name": name,
                "valid": bool(result.get("valid")),
                "labels": labels,
                "correct_unique_labels": labels == sorted(CLASS_LABELS),
                "invalid_reason": result.get("invalid_reason", ""),
            }
        )

    negative_results = []
    for name, image, depth in negative_variants(bgr, depth_m, baseline):
        result = detect_three_objects(image)
        negative_results.append(
            {
                "name": name,
                "valid": bool(result.get("valid")),
                "correctly_rejected": not bool(result.get("valid")),
                "invalid_reason": result.get("invalid_reason", ""),
            }
        )

    accepted_ok = sum(item["valid"] and item["correct_unique_labels"] for item in accepted_results)
    rejected_ok = sum(item["correctly_rejected"] for item in negative_results)
    payload = {
        "valid": accepted_ok == len(accepted_results) and rejected_ok == len(negative_results),
        "accepted_perturbations": {
            "passed": accepted_ok,
            "total": len(accepted_results),
            "results": accepted_results,
        },
        "fail_closed_cases": {
            "passed": rejected_ok,
            "total": len(negative_results),
            "results": negative_results,
        },
        "processing_time_ms": {
            "mean": float(np.mean(accepted_times)),
            "p95": float(np.percentile(accepted_times, 95)),
            "max": float(np.max(accepted_times)),
        },
        "baseline_centers_px": {
            item["category"]: item["center_px"] for item in baseline["objects"]
        },
    }
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    raise SystemExit(0 if payload["valid"] else 2)


if __name__ == "__main__":
    main()
