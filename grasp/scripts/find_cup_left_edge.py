#!/usr/bin/env python3
"""Lightweight RGB-D cup-rim detector for the left wrist camera.

The detector intentionally avoids neural-network dependencies.  It uses a Hough
proposal, RGB/depth support scoring, and a local ellipse refinement to report the
leftmost visible point of the cup's outer rim.
"""

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


def load_snapshot(path: str, url: str):
    if path:
        data = np.load(path)
    else:
        with urlopen(url, timeout=8) as response:
            data = np.load(BytesIO(response.read()))
    return {
        "rgb": data["rgb"],
        "depth": data["depth"],
        "camera_k": data["camera_k"],
        "depth_scale_m": float(data["depth_scale_m"]),
        "rgb_stamp": float(data["rgb_stamp"]),
        "depth_stamp": float(data["depth_stamp"]),
    }


def ring_edge_support(edges, x, y, radius, tolerance=3, samples=180):
    height, width = edges.shape
    supported = 0
    for angle in np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False):
        u = int(round(x + radius * math.cos(angle)))
        v = int(round(y + radius * math.sin(angle)))
        x0, x1 = max(0, u - tolerance), min(width, u + tolerance + 1)
        y0, y1 = max(0, v - tolerance), min(height, v + tolerance + 1)
        supported += int(x0 < x1 and y0 < y1 and np.any(edges[y0:y1, x0:x1]))
    return supported / samples


def valid_depth(depth_m, mask):
    values = depth_m[mask]
    return values[np.isfinite(values) & (values > 0.05) & (values < 2.0)]


def propose_rims(bgr, depth_m):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2.0)
    edges = cv2.Canny(blurred, 35, 110)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = gray.shape
    short_side = min(height, width)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(30, int(round(0.08 * short_side))),
        param1=100,
        param2=24,
        minRadius=max(14, int(round(0.035 * short_side))),
        maxRadius=int(round(0.18 * short_side)),
    )
    if circles is None:
        return [], edges

    yy, xx = np.indices(gray.shape)
    proposals = []
    for x, y, radius in circles[0]:
        if x - 1.45 * radius < 0 or x + 1.45 * radius >= width:
            continue
        if y - 1.45 * radius < 0 or y + 1.45 * radius >= height:
            continue
        radial = np.hypot(xx - x, yy - y)
        inner = radial < 0.68 * radius
        outside = (radial > 1.08 * radius) & (radial < 1.34 * radius)
        inner_depth = valid_depth(depth_m, inner)
        outside_depth = valid_depth(depth_m, outside)
        if inner_depth.size < 100 or outside_depth.size < 100:
            continue

        edge_support = ring_edge_support(edges, x, y, radius)
        saturation_delta = abs(
            float(np.median(hsv[:, :, 1][inner]))
            - float(np.median(hsv[:, :, 1][outside]))
        )
        depth_delta = abs(float(np.median(inner_depth)) - float(np.median(outside_depth)))
        color_score = min(1.0, saturation_delta / 45.0)
        depth_score = min(1.0, depth_delta / 0.030)
        score = 0.68 * edge_support + 0.17 * color_score + 0.15 * depth_score
        proposals.append(
            {
                "center": (float(x), float(y)),
                "radius": float(radius),
                "edge_support": float(edge_support),
                "saturation_delta": float(saturation_delta),
                "depth_delta_m": float(depth_delta),
                "score": float(score),
            }
        )

    # A reliable vessel rim needs substantially more closed-edge support than the
    # tray corners and printed/structural circles seen in the current workspace.
    # The lower edge-support bound still rejects the current tray/robot false
    # circles (<0.46), while accepting a partially occluded cup rim seen in the
    # higher wrist-camera pose (about 0.54 support).
    reliable = [item for item in proposals if item["edge_support"] >= 0.52 and item["score"] >= 0.60]
    if not reliable:
        return [], edges
    reliable.sort(key=lambda item: (item["radius"], -item["score"]))
    return reliable, edges


def ellipse_points(ellipse, samples=1440):
    (cx, cy), (axis_1, axis_2), angle_deg = ellipse
    theta = math.radians(angle_deg)
    cos_theta, sin_theta = math.cos(theta), math.sin(theta)
    parameter = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    a, b = axis_1 / 2.0, axis_2 / 2.0
    cos_t, sin_t = np.cos(parameter), np.sin(parameter)
    x = cx + a * cos_t * cos_theta - b * sin_t * sin_theta
    y = cy + a * cos_t * sin_theta + b * sin_t * cos_theta
    return np.column_stack((x, y))


def ellipse_edge_support(edges, ellipse, tolerance=2):
    height, width = edges.shape
    supported = 0
    points = ellipse_points(ellipse, samples=240)
    for x, y in points:
        u, v = int(round(x)), int(round(y))
        x0, x1 = max(0, u - tolerance), min(width, u + tolerance + 1)
        y0, y1 = max(0, v - tolerance), min(height, v + tolerance + 1)
        supported += int(x0 < x1 and y0 < y1 and np.any(edges[y0:y1, x0:x1]))
    return supported / len(points)


def refine_outer_rim(bgr, edges, proposal):
    x0, y0 = proposal["center"]
    radius = proposal["radius"]
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.circle(mask, (int(round(x0)), int(round(y0))), int(round(1.58 * radius)), 255, -1)
    local_edges = cv2.bitwise_and(edges, mask)
    contours, _ = cv2.findContours(local_edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    choices = []
    for contour in contours:
        if len(contour) < max(40, int(round(1.3 * radius))):
            continue
        points = contour[:, 0, :].astype(np.float64)
        radial = np.hypot(points[:, 0] - x0, points[:, 1] - y0)
        angles = (np.arctan2(points[:, 1] - y0, points[:, 0] - x0) + 2.0 * np.pi) % (2.0 * np.pi)
        angular_coverage = len(np.unique((angles / (2.0 * np.pi) * 72).astype(int))) / 72.0
        if angular_coverage < 0.55:
            continue
        median_radius = float(np.median(radial))
        if not 0.85 * radius <= median_radius <= 1.45 * radius:
            continue
        try:
            ellipse = cv2.fitEllipseAMS(contour)
        except cv2.error:
            continue
        (cx, cy), (axis_1, axis_2), _ = ellipse
        if min(axis_1, axis_2) <= 0:
            continue
        center_error = math.hypot(cx - x0, cy - y0) / radius
        axis_ratio = max(axis_1, axis_2) / min(axis_1, axis_2)
        mean_diameter = 0.5 * (axis_1 + axis_2)
        if center_error > 0.38 or axis_ratio > 1.55:
            continue
        if not 1.55 * radius <= mean_diameter <= 2.75 * radius:
            continue
        support = ellipse_edge_support(edges, ellipse)
        circularity_score = max(0.0, 1.0 - (axis_ratio - 1.0) / 0.55)
        outer_score = min(1.0, median_radius / (1.12 * radius))
        score = (
            0.35 * angular_coverage
            + 0.35 * support
            + 0.18 * circularity_score
            + 0.12 * outer_score
            - 0.10 * center_error
        )
        choices.append((score, ellipse, contour, support, angular_coverage))

    if not choices:
        # The Hough proposal is still a useful, bounded fallback.
        ellipse = ((x0, y0), (2.0 * radius, 2.0 * radius), 0.0)
        return ellipse, local_edges, 0.65 * proposal["score"], "hough_fallback"
    score, ellipse, _, support, angular_coverage = max(choices, key=lambda item: item[0])
    confidence = min(1.0, 0.55 * proposal["score"] + 0.45 * score)
    return ellipse, local_edges, confidence, "ellipse_refined"


def depth_and_xyz(depth_m, camera_k, point):
    u, v = np.rint(point).astype(int)
    height, width = depth_m.shape
    x0, x1 = max(0, u - 3), min(width, u + 4)
    y0, y1 = max(0, v - 3), min(height, v + 4)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 2.0)]
    if valid.size < 6:
        return None, None, float(valid.size / max(1, patch.size))
    z = float(np.median(valid))
    fx, fy = float(camera_k[0, 0]), float(camera_k[1, 1])
    cx, cy = float(camera_k[0, 2]), float(camera_k[1, 2])
    xyz = [(float(point[0]) - cx) * z / fx, (float(point[1]) - cy) * z / fy, z]
    return z, xyz, float(valid.size / patch.size)


def draw_overlay(bgr, ellipse, left_point, confidence):
    overlay = bgr.copy()
    center = tuple(int(round(value)) for value in ellipse[0])
    axes = tuple(int(round(value / 2.0)) for value in ellipse[1])
    cv2.ellipse(overlay, center, axes, float(ellipse[2]), 0, 360, (0, 255, 0), 2, cv2.LINE_AA)
    u, v = (int(round(value)) for value in left_point)
    cv2.circle(overlay, (u, v), 8, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.drawMarker(overlay, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 3, cv2.LINE_AA)
    label = f"cup left rim ({u}, {v})  conf={confidence:.2f}"
    origin = (max(8, min(bgr.shape[1] - 390, u + 14)), max(26, v - 18))
    cv2.rectangle(overlay, (origin[0] - 5, origin[1] - 20), (origin[0] + 380, origin[1] + 7), (20, 20, 20), -1)
    cv2.putText(overlay, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)
    return overlay


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", default="", help="Local snapshot.npz; omit to use the live URL")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    parser.add_argument("--output-dir", default="outputs/cup_left_edge")
    parser.add_argument("--max-sync-offset", type=float, default=0.035)
    args = parser.parse_args()

    sample = load_snapshot(args.snapshot, args.snapshot_url)
    bgr = sample["rgb"]
    depth_m = sample["depth"].astype(np.float32) * sample["depth_scale_m"]
    sync_offset = abs(sample["rgb_stamp"] - sample["depth_stamp"])
    if bgr.shape[:2] != depth_m.shape:
        raise RuntimeError("RGB and depth are not pixel-aligned")
    if sync_offset > args.max_sync_offset:
        raise RuntimeError(f"RGB-D time offset {sync_offset:.6f}s exceeds {args.max_sync_offset:.6f}s")

    processing_start = time.perf_counter()
    proposals, edges = propose_rims(bgr, depth_m)
    if not proposals:
        raise RuntimeError("no reliable cup-rim proposal")
    # In this project the cup is the smallest reliable vessel rim in the frame.
    proposal = proposals[0]
    ellipse, local_edges, confidence, method = refine_outer_rim(bgr, edges, proposal)
    rim_points = ellipse_points(ellipse)
    left_point = rim_points[int(np.argmin(rim_points[:, 0]))]
    depth_value, xyz, depth_support = depth_and_xyz(depth_m, sample["camera_k"], left_point)
    overlay = draw_overlay(bgr, ellipse, left_point, confidence)
    processing_time_ms = 1000.0 * (time.perf_counter() - processing_start)

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "rgb.jpg"), bgr)
    cv2.imwrite(str(output_dir / "edges.png"), local_edges)
    cv2.imwrite(str(output_dir / "overlay.jpg"), overlay)

    result = {
        "valid": bool(confidence >= 0.65),
        "camera_id": "left_wrist",
        "frame_id": "wrist_camera_left_color_optical_frame",
        "image_size": [int(bgr.shape[1]), int(bgr.shape[0])],
        "pixel_coordinate_semantics": "zero-based (u right, v down), origin at top-left",
        "cup_left_rim_px": [float(left_point[0]), float(left_point[1])],
        "cup_left_rim_px_rounded": [int(round(left_point[0])), int(round(left_point[1]))],
        "rim_ellipse": {
            "center_px": [float(ellipse[0][0]), float(ellipse[0][1])],
            "diameters_px": [float(ellipse[1][0]), float(ellipse[1][1])],
            "angle_deg": float(ellipse[2]),
        },
        "depth_m": depth_value,
        "point_camera_m": xyz,
        "depth_patch_support": depth_support,
        "confidence": float(confidence),
        "processing_time_ms": float(processing_time_ms),
        "method": method,
        "proposal": proposal,
        "rgb_depth_time_offset_s": float(sync_offset),
        "note": "2D/optical-camera result only; do not treat it as a robot-base grasp pose.",
    }
    (output_dir / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"Artifacts: {output_dir}")
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
