#!/usr/bin/env python3
"""Run a safe 2D rim demo on a real left-wrist RGB frame or MJPEG stream."""

import argparse
from io import BytesIO
import json
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

from three_object_detector import detect_three_objects

PROJECT = Path(__file__).resolve().parents[1]


def read_frame(input_path: str, stream_url: str) -> np.ndarray:
    if input_path:
        frame = cv2.imread(input_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise FileNotFoundError(input_path)
        return frame
    cap = cv2.VideoCapture(stream_url)
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"cannot read left wrist stream: {stream_url}")
    return frame


def read_snapshot(snapshot_url: str):
    with urlopen(snapshot_url, timeout=8) as response:
        data = np.load(BytesIO(response.read()))
        return (
            data["rgb"], data["depth"], data["camera_k"],
            float(data["depth_scale_m"]),
            float(data["rgb_stamp"]), float(data["depth_stamp"]),
        )


def table_region(bgr: np.ndarray):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # The current scene has a bright, low-saturation tabletop; the higher value
    # threshold avoids merging the gray floor and robot covers into the table hull.
    mask = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([179, 85, 255]))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.zeros(mask.shape, np.uint8), None
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    result = np.zeros(mask.shape, np.uint8)
    cv2.fillConvexPoly(result, hull, 255)
    return result, hull


def circle_edge_support(edges: np.ndarray, x: float, y: float, radius: float, tolerance_px: int = 6) -> float:
    angles = np.linspace(0, 2*np.pi, 180, endpoint=False)
    supported = 0
    for angle in angles:
        u = int(round(x + radius*np.cos(angle)))
        v = int(round(y + radius*np.sin(angle)))
        y0, y1 = max(0, v-tolerance_px), min(edges.shape[0], v+tolerance_px+1)
        x0, x1 = max(0, u-tolerance_px), min(edges.shape[1], u+tolerance_px+1)
        supported += int(y0 < y1 and x0 < x1 and np.any(edges[y0:y1, x0:x1]))
    return supported / len(angles)


def _legacy_detect_vessels(bgr: np.ndarray, table_mask: np.ndarray):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, 50, 130)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
        param1=100, param2=32, minRadius=18, maxRadius=110)
    if circles is None:
        return [], edges
    yy, xx = np.indices(gray.shape)
    candidates = []
    for x, y, radius in circles[0]:
        if not (0 <= int(y) < gray.shape[0] and 0 <= int(x) < gray.shape[1]):
            continue
        distance = np.sqrt((xx-x)**2 + (yy-y)**2)
        inner = distance < 0.72*radius
        ring = (distance > 1.05*radius) & (distance < 1.28*radius)
        if inner.sum() < 100 or ring.sum() < 100:
            continue
        inner_sat = float(np.mean(hsv[:, :, 1][inner]))
        ring_sat = float(np.mean(hsv[:, :, 1][ring]))
        color_contrast = abs(inner_sat - ring_sat) / 255.0
        edge_support = circle_edge_support(edges, x, y, radius)
        # The vessels can sit on a dark tray rather than directly on the bright
        # table. Require both a closed edge and some inside/outside appearance
        # change instead of rejecting candidates by tabletop color.
        if not (edge_support >= 0.65 or (edge_support >= 0.55 and color_contrast >= 0.10)):
            continue
        score = 0.58*edge_support + 0.42*min(1.0, color_contrast/0.35)
        candidates.append({
            "center_px": [float(x), float(y)],
            "rim_radius_px": float(radius),
            "edge_support": edge_support,
            "inner_outer_color_contrast": color_contrast,
            "selection_score": score,
        })
    # Suppress multiple Hough circles belonging to the same physical rim.
    kept = []
    for item in sorted(candidates, key=lambda value: value["selection_score"], reverse=True):
        center = np.asarray(item["center_px"])
        if any(np.linalg.norm(center-np.asarray(old["center_px"])) <
               0.60*max(item["rim_radius_px"], old["rim_radius_px"]) for old in kept):
            continue
        kept.append(item)

    # This demo scene is explicitly defined to contain three vessel-like objects.
    # Keep the three strongest distinct rims, then assign semantic labels by their
    # relative observed radius rather than fixed pixel thresholds.
    kept = sorted(kept[:3], key=lambda item: item["rim_radius_px"])
    relative_labels = ("cup", "bowl", "plate")
    for index, item in enumerate(kept):
        item["category"] = relative_labels[index] if len(kept) == 3 else "unknown_vessel"
        item["object_id"] = f"left-frame-object-{index}"
        item["grasp_order"] = index + 1
        item["selected"] = index == 0 and len(kept) == 3
    return kept, edges


def detect_vessels(bgr: np.ndarray, table_mask: np.ndarray, depth_m=None):
    """Compatibility wrapper around the strict three-class RGB-D detector.

    The former implementation ignored ``table_mask`` and assigned labels by
    radius order; keep it above only for comparison, never for action output.
    """
    result = detect_three_objects(bgr)
    edges = result.get("debug", {}).get("edges")
    if edges is None:
        edges = np.zeros(bgr.shape[:2], dtype=np.uint8)
    if not result.get("valid"):
        return [], edges
    order = {"cup": 1, "bean_bowl": 2, "plate": 3}
    candidates = []
    for obj in result["objects"]:
        features = obj["features"]
        candidates.append({
            "center_px": list(obj["center_px"]),
            "rim_radius_px": float(obj["rim_radius_px"]),
            "edge_support": float(features["edge_support"]),
            "inner_outer_color_contrast": float(features["color_contrast"]),
            "selection_score": float(obj["confidence"]),
            "category": obj["category"],
            "object_id": f"left-frame-{obj['category']}",
            "grasp_order": order[obj["category"]],
            "selected": obj["category"] == "cup",
            "class_margin": float(obj["class_margin"]),
        })
    candidates.sort(key=lambda item: item["grasp_order"])
    return candidates, edges


def add_metric_grasp_points(candidates, depth, camera_k, depth_scale_m):
    """Choose an observed rim point with depth support and back-project it."""
    if depth is None or camera_k is None:
        return
    depth_m = depth.astype(np.float32) * depth_scale_m
    fx, fy = float(camera_k[0, 0]), float(camera_k[1, 1])
    cx, cy = float(camera_k[0, 2]), float(camera_k[1, 2])
    height, width = depth_m.shape
    for item in candidates:
        center = np.asarray(item["center_px"], dtype=float)
        radius = float(item["rim_radius_px"])
        options = []
        for angle in np.linspace(0, 2*np.pi, 96, endpoint=False):
            uv = center + radius*np.asarray([np.cos(angle), np.sin(angle)])
            u, v = np.rint(uv).astype(int)
            if not (3 <= u < width-3 and 3 <= v < height-3):
                continue
            patch = depth_m[v-3:v+4, u-3:u+4]
            valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 2.0)]
            if valid.size < 6:
                continue
            other_clearance = min((
                np.linalg.norm(uv-np.asarray(other["center_px"])) - other["rim_radius_px"]
                for other in candidates if other is not item
            ), default=radius)
            options.append((valid.size + 0.05*other_clearance, uv, float(np.median(valid)), valid.size/patch.size))
        if not options:
            item["metric_valid"] = False
            item["metric_invalid_reason"] = "no_reliable_aligned_depth_on_rim"
            continue
        _, uv, z, support = max(options, key=lambda value: value[0])
        u, v = uv
        xyz = np.asarray([(u-cx)*z/fx, (v-cy)*z/fy, z])
        closing_px = center - uv
        closing_px /= max(np.linalg.norm(closing_px), 1e-9)
        item["grasp_rim_px"] = [float(u), float(v)]
        item["rim_position_camera_m"] = xyz.tolist()
        item["contact_position_camera_m"] = xyz.tolist()
        item["closing_direction_image"] = closing_px.tolist()
        item["rim_depth_support"] = float(support)
        item["metric_valid"] = True
        item["metric_invalid_reason"] = ""


def draw_result(bgr, table_hull, candidates):
    out = bgr.copy()
    if table_hull is not None:
        cv2.polylines(out, [table_hull], True, (255, 180, 0), 2)
        cv2.putText(out, "TABLE", tuple(table_hull[:, 0, :].min(axis=0) + [4, 18]),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 180, 0), 2)
    for item in candidates:
        x, y = np.rint(item["center_px"]).astype(int)
        radius = int(round(item["rim_radius_px"]))
        color = (0, 255, 0) if item["selected"] else (0, 180, 255)
        cv2.circle(out, (x, y), radius, color, 3)
        cv2.circle(out, (x, y), 4, color, -1)
        label = f"#{item['grasp_order']} {item['category']} r={radius}px"
        label_origin = (390, 28 + 27*(item["grasp_order"]-1))
        cv2.rectangle(out, (label_origin[0]-5, label_origin[1]-19),
                      (635, label_origin[1]+5), (25, 25, 25), -1)
        cv2.putText(out, label, label_origin, cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
        cv2.putText(out, f"#{item['grasp_order']}", (x-12, y+7),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        if item.get("metric_valid"):
            start = np.rint(item["grasp_rim_px"]).astype(int)
            direction = np.asarray(item["closing_direction_image"])
            end = np.rint(start + max(18, radius//2)*direction).astype(int)
            cv2.circle(out, tuple(start), 6, (255, 0, 255), -1)
            cv2.arrowedLine(out, tuple(start), tuple(end), (255, 0, 255), 3, tipLength=.25)
    status = "3 VESSELS / GRASP QUEUE READY" if len(candidates) == 3 else f"INVALID: expected 3 rims, found {len(candidates)}"
    cv2.putText(out, status, (12, out.shape[0]-16), cv2.FONT_HERSHEY_SIMPLEX, .65,
                (0, 255, 0) if candidates else (0, 0, 255), 2)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="", help="Saved real RGB image; omit to capture MJPEG")
    parser.add_argument("--stream-url", default="http://127.0.0.1:18080/rgb.mjpg")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    parser.add_argument("--output-dir", default=str(PROJECT / "outputs" / "camera_image_demo_left"))
    args = parser.parse_args()
    depth = camera_k = None
    depth_scale_m = 1.0
    rgb_stamp = depth_stamp = time.time()
    if args.input:
        bgr = read_frame(args.input, args.stream_url)
    else:
        bgr, depth, camera_k, depth_scale_m, rgb_stamp, depth_stamp = read_snapshot(args.snapshot_url)
    table_mask, hull = table_region(bgr)
    depth_m = depth.astype(np.float32) * depth_scale_m if depth is not None else None
    candidates, _ = detect_vessels(bgr, table_mask, depth_m=depth_m)
    add_metric_grasp_points(candidates, depth, camera_k, depth_scale_m)
    overlay = draw_result(bgr, hull, candidates)
    output = Path(args.output_dir).resolve(); output.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output / "rgb.jpg"), bgr)
    cv2.imwrite(str(output / "table_mask.png"), table_mask)
    cv2.imwrite(str(output / "overlay.jpg"), overlay)
    metric_valid = len(candidates) == 3 and all(item.get("metric_valid", False) for item in candidates)
    if len(candidates) != 3:
        invalid_reason = f"expected_three_reliable_rims_found_{len(candidates)}"
    elif not metric_valid:
        invalid_reason = "one_or_more_rims_lack_reliable_aligned_depth"
    else:
        invalid_reason = ""
    payload = {
        "timestamp": rgb_stamp,
        "camera_id": "left",
        "frame_id": "wrist_camera_left_color_optical_frame",
        "valid": metric_valid,
        "selected_object": candidates[0] if metric_valid else None,
        "grasp_queue": candidates if metric_valid else [],
        "all_objects": candidates,
        "rgb_depth_time_offset_s": abs(rgb_stamp-depth_stamp),
        "coordinate_semantics": "camera optical frame: +X right, +Y down, +Z forward; contact center is approximated by the observed rim point",
        "invalid_reason": invalid_reason,
        "note": "Self-check only. Camera-frame coordinates are not robot-base or executable TCP poses.",
    }
    (output / "result.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    print(f"Artifacts: {output}")
    raise SystemExit(0 if payload["valid"] else 2)


if __name__ == "__main__":
    main()
