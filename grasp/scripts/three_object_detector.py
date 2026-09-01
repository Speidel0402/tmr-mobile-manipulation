#!/usr/bin/env python3
"""Fast RGB detector for the calibrated three-object tabletop scene.

The scene contains one cup, one bowl filled with dark granular material, and
one large shallow plate/bowl.  Detection is deliberately local and
deterministic: tray ROI -> rim proposals -> RGB/texture features -> one
global one-to-one class assignment.  Ambiguous scenes are rejected instead of
silently relabeling candidates by radius.

Depth may be supplied for diagnostics, but it is deliberately not used to
classify or authorize an action.  The wrist D405 depth on the pale plate is
noisier than the sub-pixel-stable RGB rim.
"""

from __future__ import annotations

import copy
import itertools
import math
from typing import Iterable

import cv2
import numpy as np


CLASS_LABELS = ("cup", "bean_bowl", "plate")


def _clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    return float(min(upper, max(lower, value)))


def _gaussian(value: float, mean: float, sigma: float) -> float:
    return float(math.exp(-0.5 * ((value - mean) / max(sigma, 1e-9)) ** 2))


def _valid_depth(depth_m: np.ndarray | None, mask: np.ndarray) -> np.ndarray:
    if depth_m is None:
        return np.empty(0, dtype=np.float32)
    values = depth_m[mask]
    return values[np.isfinite(values) & (values > 0.05) & (values < 2.0)]


def _ring_edge_support(
    edges: np.ndarray,
    center_x: float,
    center_y: float,
    radius: float,
    tolerance_px: int = 4,
    samples: int = 180,
) -> float:
    height, width = edges.shape
    supported = 0
    for angle in np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False):
        u = int(round(center_x + radius * math.cos(angle)))
        v = int(round(center_y + radius * math.sin(angle)))
        x0, x1 = max(0, u - tolerance_px), min(width, u + tolerance_px + 1)
        y0, y1 = max(0, v - tolerance_px), min(height, v + tolerance_px + 1)
        supported += int(x0 < x1 and y0 < y1 and np.any(edges[y0:y1, x0:x1]))
    return float(supported / samples)


def _ellipse_points(ellipse, samples: int = 1440) -> np.ndarray:
    (center_x, center_y), (diameter_a, diameter_b), angle_deg = ellipse
    angle = math.radians(float(angle_deg))
    parameter = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=False)
    cos_t, sin_t = np.cos(parameter), np.sin(parameter)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    radius_a, radius_b = diameter_a / 2.0, diameter_b / 2.0
    x = center_x + radius_a * cos_t * cos_a - radius_b * sin_t * sin_a
    y = center_y + radius_a * cos_t * sin_a + radius_b * sin_t * cos_a
    return np.column_stack((x, y))


def _ellipse_edge_support(edges: np.ndarray, ellipse, tolerance_px: int = 3) -> float:
    height, width = edges.shape
    points = _ellipse_points(ellipse, samples=180)
    supported = 0
    for x, y in points:
        u, v = int(round(x)), int(round(y))
        x0, x1 = max(0, u - tolerance_px), min(width, u + tolerance_px + 1)
        y0, y1 = max(0, v - tolerance_px), min(height, v + tolerance_px + 1)
        supported += int(x0 < x1 and y0 < y1 and np.any(edges[y0:y1, x0:x1]))
    return float(supported / len(points))


def _refine_rim_ellipse(edges: np.ndarray, x: float, y: float, radius: float):
    mask = np.zeros(edges.shape, dtype=np.uint8)
    cv2.circle(mask, (int(round(x)), int(round(y))), int(round(1.48 * radius)), 255, -1)
    local_edges = cv2.bitwise_and(edges, mask)
    contours, _ = cv2.findContours(local_edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    choices = []
    for contour in contours:
        if len(contour) < max(36, int(round(radius))):
            continue
        points = contour[:, 0, :].astype(np.float64)
        radial = np.hypot(points[:, 0] - x, points[:, 1] - y)
        angles = (np.arctan2(points[:, 1] - y, points[:, 0] - x) + 2.0 * np.pi) % (
            2.0 * np.pi
        )
        coverage = len(np.unique((angles / (2.0 * np.pi) * 72).astype(int))) / 72.0
        median_radius = float(np.median(radial))
        if coverage < 0.48 or not 0.78 * radius <= median_radius <= 1.42 * radius:
            continue
        try:
            ellipse = cv2.fitEllipseAMS(contour)
        except cv2.error:
            continue
        (cx, cy), (axis_a, axis_b), _ = ellipse
        if min(axis_a, axis_b) <= 0:
            continue
        axis_ratio = max(axis_a, axis_b) / min(axis_a, axis_b)
        center_error = math.hypot(cx - x, cy - y) / max(radius, 1e-6)
        mean_diameter = 0.5 * (axis_a + axis_b)
        if axis_ratio > 1.62 or center_error > 0.42:
            continue
        if not 1.45 * radius <= mean_diameter <= 2.70 * radius:
            continue
        support = _ellipse_edge_support(edges, ellipse)
        size_agreement = _gaussian(mean_diameter / (2.0 * radius), 1.0, 0.22)
        score = 0.42 * support + 0.28 * coverage + 0.20 * size_agreement + 0.10 * (
            1.0 - min(1.0, center_error)
        )
        choices.append((score, support, ellipse))
    if not choices:
        ellipse = ((float(x), float(y)), (2.0 * float(radius), 2.0 * float(radius)), 0.0)
        return ellipse, _ellipse_edge_support(edges, ellipse), "hough_circle"
    _, support, ellipse = max(choices, key=lambda item: item[0])
    return ellipse, float(support), "ellipse_refined"


def detect_tray_region(bgr: np.ndarray):
    """Return the dark tray mask/hull while excluding the robot at image bottom."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = bgr.shape[:2]
    cutoff_row = int(round(0.79 * height))
    value = hsv[:, :, 2]
    # A single global Otsu threshold can merge the tray with the dark floor and
    # image borders.  Search several exposure-relative dark thresholds and
    # score rectangular, central components; this retains adaptation without
    # letting one merged full-frame component invalidate the whole scene.
    otsu_threshold, _ = cv2.threshold(
        value[:cutoff_row], 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    upper = value[:cutoff_row]
    primary_threshold = int(round(float(otsu_threshold)))
    thresholds = {
        int(round(float(np.percentile(upper, percentile))))
        for percentile in (10, 15, 20, 25, 30)
    }
    thresholds.update(
        int(round(float(otsu_threshold) * factor)) for factor in (0.45, 0.55, 0.65, 0.75)
    )
    thresholds.add(primary_threshold)
    thresholds = sorted(max(25, min(170, threshold)) for threshold in thresholds)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19, 19))
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    choices = []
    image_area = float(height * width)
    for threshold in thresholds:
        is_primary = threshold == primary_threshold
        mask = (value <= threshold).astype(np.uint8) * 255
        mask[cutoff_row:, :] = 0
        if is_primary:
            # Preserve the proven legacy path whenever global Otsu gives a
            # geometrically valid tray.  Fallback logic must not perturb it.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        else:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, box_width, box_height = cv2.boundingRect(contour)
            width_fraction = box_width / float(width)
            height_fraction = box_height / float(height)
            aspect = box_width / max(box_height, 1)
            if is_primary:
                if not 0.28 <= width_fraction <= 0.86:
                    continue
                if not 0.24 <= height_fraction <= 0.76:
                    continue
                if not 0.85 <= aspect <= 2.05:
                    continue
                if not 0.07 * image_area <= area <= 0.60 * image_area:
                    continue
            else:
                if not 0.25 <= width_fraction <= 0.88:
                    continue
                if not 0.20 <= height_fraction <= 0.76:
                    continue
                if not 0.80 <= aspect <= 2.20:
                    continue
                if not 0.05 * image_area <= area <= 0.60 * image_area:
                    continue
            touches = sum(
                (
                    x <= 1,
                    y <= 1,
                    x + box_width >= width - 1,
                    y + box_height >= cutoff_row - 1,
                )
            )
            if touches >= 2:
                continue
            fill_ratio = area / max(1.0, float(box_width * box_height))
            center_x, center_y = x + box_width / 2.0, y + box_height / 2.0
            center_distance = math.hypot(
                (center_x - 0.55 * width) / width,
                (center_y - 0.42 * height) / height,
            )
            size_prior = math.exp(
                -0.5
                * (
                    ((width_fraction - 0.48) / 0.17) ** 2
                    + ((height_fraction - 0.43) / 0.17) ** 2
                )
            )
            aspect_prior = math.exp(-0.5 * ((aspect - 1.42) / 0.55) ** 2)
            center_prior = max(0.0, 1.0 - 1.8 * center_distance)
            border_penalty = 0.18 * touches
            if is_primary:
                score = area * fill_ratio * max(0.35, 1.0 - center_distance)
            else:
                score = (
                    0.34 * fill_ratio
                    + 0.28 * size_prior
                    + 0.20 * aspect_prior
                    + 0.18 * center_prior
                    - border_penalty
                )
            choices.append(
                (
                    score,
                    contour,
                    (x, y, box_width, box_height),
                    fill_ratio,
                    area,
                    mask,
                    threshold,
                )
            )
    if not choices:
        return {
            "valid": False,
            "invalid_reason": "dark_tray_not_found",
            "mask": np.zeros((height, width), dtype=np.uint8),
            "hull": None,
            "bbox_px": None,
            "confidence": 0.0,
        }

    primary_choices = [item for item in choices if item[6] == primary_threshold]
    _, contour, bbox, fill_ratio, area, selected_mask, selected_threshold = max(
        primary_choices if primary_choices else choices,
        key=lambda item: item[0],
    )
    hull = cv2.convexHull(contour)
    tray_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(tray_mask, hull, 255)
    area_scale = min(1.0, area / (0.16 * image_area))
    confidence = _clamp(0.48 + 0.34 * fill_ratio + 0.18 * area_scale)
    return {
        "valid": True,
        "invalid_reason": "",
        "mask": tray_mask,
        "hull": hull,
        "bbox_px": [int(value) for value in bbox],
        "confidence": confidence,
        "threshold_v": int(selected_threshold),
    }


def _candidate_features(
    gray: np.ndarray,
    hsv: np.ndarray,
    yy: np.ndarray,
    xx: np.ndarray,
    depth_m: np.ndarray | None,
    tray_mask: np.ndarray,
    tray_short_side: float,
    edges: np.ndarray,
    laplacian: np.ndarray,
    circle,
):
    x, y, radius = (float(value) for value in circle)
    radial = np.hypot(xx - x, yy - y)
    disk = radial < 0.90 * radius
    inner = radial < 0.66 * radius
    rim_band = (radial > 0.82 * radius) & (radial < 1.14 * radius)
    outside = (radial > 1.08 * radius) & (radial < 1.33 * radius)
    if int(inner.sum()) < 180 or int(outside.sum()) < 180:
        return None

    tray_coverage = float(np.mean(tray_mask[disk] > 0))
    if tray_coverage < 0.58:
        return None
    edge_support = _ring_edge_support(edges, x, y, radius)
    inner_gray = gray[inner]
    outside_gray = gray[outside]
    inner_sat = hsv[:, :, 1][inner]
    outside_sat = hsv[:, :, 1][outside]
    inner_depth = _valid_depth(depth_m, inner)
    rim_depth = _valid_depth(depth_m, rim_band)
    outside_depth = _valid_depth(depth_m, outside)
    depth_support = float(inner_depth.size / max(1, int(inner.sum()))) if depth_m is not None else 0.0
    inner_depth_m = float(np.median(inner_depth)) if inner_depth.size else None
    rim_depth_m = float(np.median(rim_depth)) if rim_depth.size else None
    outside_depth_m = float(np.median(outside_depth)) if outside_depth.size else None
    # Vessel cavity depth is the stable discriminant here: cup > filled bowl >
    # shallow plate.  Compare the interior to its own rim before considering
    # the surrounding tray, whose depth changes with object placement.
    if inner_depth_m is not None and rim_depth_m is not None:
        depth_contrast_m = abs(inner_depth_m - rim_depth_m)
    elif inner_depth_m is not None and outside_depth_m is not None:
        depth_contrast_m = abs(inner_depth_m - outside_depth_m)
    else:
        depth_contrast_m = None

    median_gray = float(np.median(inner_gray))
    outside_median_gray = float(np.median(outside_gray))
    brightness_ratio = median_gray / max(1.0, outside_median_gray)
    laplacian_variance = float(np.var(laplacian[inner]))
    normalized_laplacian = laplacian_variance / max(64.0, median_gray * median_gray)
    edge_density = float(np.mean(edges[inner] > 0))
    texture_score = _clamp(
        0.55 * (normalized_laplacian / 0.080) + 0.45 * (edge_density / 0.035)
    )
    darkness_score = _clamp((95.0 - median_gray) / 75.0)
    brightness_score = _clamp((median_gray - 85.0) / 90.0)
    saturation_score = _clamp((float(np.median(inner_sat)) - 35.0) / 90.0)
    color_contrast = abs(float(np.median(inner_sat)) - float(np.median(outside_sat))) / 255.0
    ellipse, ellipse_support, method = _refine_rim_ellipse(edges, x, y, radius)
    ellipse_radius = math.sqrt(float(ellipse[1][0]) * float(ellipse[1][1])) / 2.0
    proposal_score = _clamp(
        0.64 * edge_support
        + 0.17 * tray_coverage
        + 0.11 * min(1.0, color_contrast / 0.22)
        + 0.08 * ellipse_support
    )
    return {
        "hough_center_px": [x, y],
        "hough_radius_px": radius,
        "ellipse": ellipse,
        "ellipse_radius_px": float(ellipse_radius),
        "rim_fit_method": method,
        "edge_support": edge_support,
        "ellipse_edge_support": ellipse_support,
        "tray_coverage": tray_coverage,
        "radius_to_tray_short": radius / max(1.0, tray_short_side),
        "median_gray": median_gray,
        "outside_median_gray": outside_median_gray,
        "inner_outer_brightness_ratio": float(brightness_ratio),
        "median_saturation": float(np.median(inner_sat)),
        "color_contrast": color_contrast,
        "darkness_score": darkness_score,
        "brightness_score": brightness_score,
        "texture_score": texture_score,
        "normalized_laplacian": normalized_laplacian,
        "edge_density_inner": edge_density,
        "depth_support": depth_support,
        "inner_depth_m": inner_depth_m,
        "rim_depth_m": rim_depth_m,
        "outside_depth_m": outside_depth_m,
        "depth_contrast_m": depth_contrast_m,
        "proposal_score": proposal_score,
    }


def _class_scores(candidate) -> dict[str, float]:
    size = float(candidate["radius_to_tray_short"])
    edge = float(candidate["edge_support"])
    texture = float(candidate["texture_score"])
    darkness = float(candidate["darkness_score"])
    brightness = float(candidate["brightness_score"])
    saturation = float(candidate["median_saturation"])
    saturation_score = _clamp((saturation - 35.0) / 90.0)
    brightness_ratio = float(candidate["inner_outer_brightness_ratio"])
    cup_ratio = _gaussian(brightness_ratio, 1.30, 0.46)
    bowl_ratio = _gaussian(brightness_ratio, 0.52, 0.30)
    plate_ratio = _gaussian(brightness_ratio, 2.45, 0.90)
    # RGB-only action scores.  Keeping the weights normalized makes behavior
    # identical whether a depth frame is present, missing, or noisy.
    scores = {
        "cup": (
            0.35 * _gaussian(size, 0.165, 0.045)
            + 0.20 * cup_ratio
            + 0.20 * saturation_score
            + 0.1875 * edge
            + 0.0625 * (1.0 - texture)
        ),
        "bean_bowl": (
            0.2222 * _gaussian(size, 0.198, 0.050)
            + 0.20 * texture
            + 0.3333 * bowl_ratio
            + 0.1556 * edge
            + 0.0556 * saturation_score
            + 0.0333 * darkness
        ),
        "plate": (
            0.4598 * _gaussian(size, 0.335, 0.072)
            + 0.2299 * plate_ratio
            + 0.1724 * (1.0 - texture)
            + 0.0920 * edge
            + 0.0460 * (1.0 - saturation_score)
        ),
    }
    return {label: _clamp(score) for label, score in scores.items()}


def _suppress_overlapping(candidates: list[dict]) -> list[dict]:
    priority = sorted(
        candidates,
        key=lambda item: item["proposal_score"] + 0.002 * item["hough_radius_px"],
        reverse=True,
    )
    kept = []
    for candidate in priority:
        center = np.asarray(candidate["hough_center_px"], dtype=float)
        radius = float(candidate["hough_radius_px"])
        duplicate = False
        for old in kept:
            old_center = np.asarray(old["hough_center_px"], dtype=float)
            old_radius = float(old["hough_radius_px"])
            if np.linalg.norm(center - old_center) < 0.70 * (radius + old_radius):
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept[:8]


def _assign_unique_classes(candidates: list[dict], tray_confidence: float):
    if len(candidates) < 3:
        return None, "fewer_than_three_distinct_rims"
    for candidate in candidates:
        candidate["class_scores"] = _class_scores(candidate)

    assignments = []
    for indices in itertools.permutations(range(len(candidates)), len(CLASS_LABELS)):
        chosen = {label: candidates[index] for label, index in zip(CLASS_LABELS, indices)}
        cup_radius = chosen["cup"]["hough_radius_px"]
        bowl_radius = chosen["bean_bowl"]["hough_radius_px"]
        plate_radius = chosen["plate"]["hough_radius_px"]
        relational = 0.0
        relational += 0.12 if bowl_radius > 1.035 * cup_radius else -0.28
        relational += 0.18 if plate_radius > 1.32 * max(cup_radius, bowl_radius) else -0.34
        total = relational + sum(
            chosen[label]["class_scores"][label] for label in CLASS_LABELS
        )
        assignments.append((float(total), indices, chosen))
    assignments.sort(key=lambda item: item[0], reverse=True)
    best_total, _, best = assignments[0]
    second_total = assignments[1][0] if len(assignments) > 1 else -math.inf
    global_margin = float(best_total - second_total)

    objects = []
    invalid_reasons = []
    for label in CLASS_LABELS:
        candidate = best[label]
        scores = candidate["class_scores"]
        assigned_score = float(scores[label])
        other_score = max(float(score) for name, score in scores.items() if name != label)
        class_margin = assigned_score - other_score
        geometry_confidence = _clamp(
            0.40 * candidate["edge_support"]
            + 0.18 * candidate["ellipse_edge_support"]
            + 0.18 * candidate["tray_coverage"]
            + 0.14 * candidate["proposal_score"]
            + 0.10 * tray_confidence
        )
        confidence = _clamp(0.80 * assigned_score + 0.20 * geometry_confidence)
        if assigned_score < 0.64:
            invalid_reasons.append(f"{label}_class_score_low")
        if confidence < 0.68:
            invalid_reasons.append(f"{label}_confidence_low")
        if class_margin < 0.09:
            invalid_reasons.append(f"{label}_class_ambiguous")

        ellipse = candidate["ellipse"]
        rim = _ellipse_points(ellipse)
        left = rim[int(np.argmin(rim[:, 0]))]
        right = rim[int(np.argmax(rim[:, 0]))]
        top = rim[int(np.argmin(rim[:, 1]))]
        bottom = rim[int(np.argmax(rim[:, 1]))]
        center = [float(ellipse[0][0]), float(ellipse[0][1])]
        radius = float(candidate["ellipse_radius_px"])
        objects.append(
            {
                "category": label,
                "center_px": center,
                "rim_radius_px": radius,
                "rim_ellipse": {
                    "center_px": center,
                    "diameters_px": [float(ellipse[1][0]), float(ellipse[1][1])],
                    "angle_deg": float(ellipse[2]),
                },
                "bbox_px": [
                    float(np.min(rim[:, 0])),
                    float(np.min(rim[:, 1])),
                    float(np.max(rim[:, 0])),
                    float(np.max(rim[:, 1])),
                ],
                "rim_extrema_px": {
                    "left": left.tolist(),
                    "right": right.tolist(),
                    "top": top.tolist(),
                    "bottom": bottom.tolist(),
                },
                "depth_m": candidate["inner_depth_m"],
                "rim_depth_m": candidate["rim_depth_m"],
                "confidence": confidence,
                "class_score": assigned_score,
                "class_margin": float(class_margin),
                "class_scores": {name: float(score) for name, score in scores.items()},
                "features": {
                    key: copy.deepcopy(value)
                    for key, value in candidate.items()
                    if key
                    not in {
                        "ellipse",
                        "class_scores",
                    }
                },
            }
        )
    if global_margin < 0.09:
        invalid_reasons.append("global_assignment_ambiguous")
    return {
        "objects": objects,
        "assignment_total": float(best_total),
        "assignment_margin": global_margin,
        "invalid_reasons": invalid_reasons,
    }, "" if not invalid_reasons else ";".join(invalid_reasons)


def detect_three_objects(
    bgr: np.ndarray,
    depth_m: np.ndarray | None = None,
    camera_k: np.ndarray | None = None,
):
    """Detect and uniquely classify cup, filled bowl, and plate in one frame."""
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 image")
    if depth_m is not None and depth_m.shape != bgr.shape[:2]:
        raise ValueError("depth must be pixel-aligned with bgr")
    tray = detect_tray_region(bgr)
    if not tray["valid"]:
        return {
            "valid": False,
            "invalid_reason": tray["invalid_reason"],
            "objects": [],
            "tray": {key: value for key, value in tray.items() if key not in {"mask", "hull"}},
            "debug": {"tray_mask": tray["mask"], "tray_hull": tray["hull"], "edges": None},
        }

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if float(np.percentile(gray, 80)) < 135.0 or float(np.median(gray)) > 132.0:
        detection_gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    else:
        detection_gray = gray
    blurred = cv2.GaussianBlur(detection_gray, (9, 9), 2.0)
    edges = cv2.Canny(blurred, 35, 110)
    laplacian = cv2.Laplacian(gray, cv2.CV_32F)
    yy, xx = np.indices(gray.shape)
    x, y, tray_width, tray_height = tray["bbox_px"]
    tray_short = float(min(tray_width, tray_height))
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(30, int(round(0.14 * tray_short))),
        param1=90,
        param2=26,
        minRadius=max(18, int(round(0.075 * tray_short))),
        maxRadius=int(round(0.46 * tray_short)),
    )
    if circles is None:
        return {
            "valid": False,
            "invalid_reason": "no_circular_rim_proposals",
            "objects": [],
            "tray": {key: value for key, value in tray.items() if key not in {"mask", "hull"}},
            "debug": {"tray_mask": tray["mask"], "tray_hull": tray["hull"], "edges": edges},
        }

    candidates = []
    for circle in circles[0]:
        center_x, center_y, radius = (float(value) for value in circle)
        u, v = int(round(center_x)), int(round(center_y))
        if not (0 <= u < bgr.shape[1] and 0 <= v < bgr.shape[0]):
            continue
        if tray["mask"][v, u] == 0:
            continue
        if center_x - 0.72 * radius < 0 or center_x + 0.72 * radius >= bgr.shape[1]:
            continue
        if center_y - 0.72 * radius < 0 or center_y + 0.72 * radius >= bgr.shape[0]:
            continue
        candidate = _candidate_features(
            gray,
            hsv,
            yy,
            xx,
            depth_m,
            tray["mask"],
            tray_short,
            edges,
            laplacian,
            circle,
        )
        if candidate is None:
            continue
        if candidate["edge_support"] < 0.46 or candidate["proposal_score"] < 0.50:
            continue
        candidates.append(candidate)
    candidates = _suppress_overlapping(candidates)
    assignment, reason = _assign_unique_classes(candidates, tray["confidence"])
    if assignment is None:
        return {
            "valid": False,
            "invalid_reason": reason,
            "objects": [],
            "tray": {key: value for key, value in tray.items() if key not in {"mask", "hull"}},
            "candidate_count": len(candidates),
            "debug": {"tray_mask": tray["mask"], "tray_hull": tray["hull"], "edges": edges},
        }

    if camera_k is not None:
        fx, fy = float(camera_k[0, 0]), float(camera_k[1, 1])
        cx, cy = float(camera_k[0, 2]), float(camera_k[1, 2])
        for item in assignment["objects"]:
            if item["depth_m"] is None:
                item["center_camera_m"] = None
                continue
            u, v = item["center_px"]
            z = float(item["depth_m"])
            item["center_camera_m"] = [(u - cx) * z / fx, (v - cy) * z / fy, z]

    invalid_reason = ";".join(assignment["invalid_reasons"])
    return {
        "valid": not assignment["invalid_reasons"],
        "invalid_reason": invalid_reason,
        "objects": assignment["objects"],
        "assignment_total": assignment["assignment_total"],
        "assignment_margin": assignment["assignment_margin"],
        "candidate_count": len(candidates),
        "tray": {key: value for key, value in tray.items() if key not in {"mask", "hull"}},
        "debug": {
            "tray_mask": tray["mask"],
            "tray_hull": tray["hull"],
            "edges": edges,
            "candidates": candidates,
        },
    }


def aggregate_detections(results: Iterable[dict], minimum_valid_frames: int = 3):
    """Robustly combine live frames and reject unstable classifications."""
    results = list(results)
    valid_results = [item for item in results if item.get("valid")]
    required = min(len(results), max(1, int(minimum_valid_frames)))
    if len(valid_results) < required:
        return {
            "valid": False,
            "invalid_reason": f"only_{len(valid_results)}_of_{len(results)}_frames_valid",
            "objects": [],
            "frame_count": len(results),
            "valid_frame_count": len(valid_results),
        }
    if len(results) >= 2 and not all(item.get("valid") for item in results[-2:]):
        return {
            "valid": False,
            "invalid_reason": "last_two_frames_not_both_valid",
            "objects": [],
            "frame_count": len(results),
            "valid_frame_count": len(valid_results),
        }

    output_objects = []
    stability_reasons = []
    for label in CLASS_LABELS:
        observations = [
            next(obj for obj in frame["objects"] if obj["category"] == label)
            for frame in valid_results
        ]
        centers = np.asarray([item["center_px"] for item in observations], dtype=float)
        radii = np.asarray([item["rim_radius_px"] for item in observations], dtype=float)
        median_center = np.median(centers, axis=0)
        median_radius = float(np.median(radii))
        center_errors = np.linalg.norm(centers - median_center, axis=1)
        radius_errors = np.abs(radii - median_radius)
        inliers = (center_errors <= 4.0) & (radius_errors <= 3.5)
        if int(inliers.sum()) < required:
            stability_reasons.append(f"{label}_temporal_cluster_insufficient")
            continue
        inlier_indices = np.flatnonzero(inliers)
        representative_index = int(inlier_indices[np.argmin(center_errors[inliers])])
        representative = copy.deepcopy(observations[representative_index])
        inlier_confidence = np.asarray(
            [observations[index]["confidence"] for index in inlier_indices], dtype=float
        )
        max_center_error = float(np.max(center_errors[inliers]))
        max_radius_error = float(np.max(radius_errors[inliers]))
        center_mad = float(np.median(center_errors[inliers]))
        diameter_cv = float(np.std(2.0 * radii[inliers]) / max(1e-6, np.mean(2.0 * radii[inliers])))
        depths = np.asarray(
            [observations[index].get("rim_depth_m", np.nan) for index in inlier_indices],
            dtype=float,
        )
        finite_depths = depths[np.isfinite(depths)]
        depth_std_m = float(np.std(finite_depths)) if finite_depths.size >= 2 else 0.0
        stability_factor = _clamp(1.0 - 0.08 * max_center_error - 0.06 * max_radius_error, 0.65, 1.0)
        aggregate_center = np.median(centers[inliers], axis=0)
        ellipse_diameters = np.asarray(
            [observations[index]["rim_ellipse"]["diameters_px"] for index in inlier_indices],
            dtype=float,
        )
        aggregate_diameters = np.median(ellipse_diameters, axis=0)
        aggregate_angle = float(representative["rim_ellipse"]["angle_deg"])
        aggregate_ellipse = (
            tuple(aggregate_center.tolist()),
            tuple(aggregate_diameters.tolist()),
            aggregate_angle,
        )
        aggregate_rim = _ellipse_points(aggregate_ellipse)
        left = aggregate_rim[int(np.argmin(aggregate_rim[:, 0]))]
        right = aggregate_rim[int(np.argmax(aggregate_rim[:, 0]))]
        top = aggregate_rim[int(np.argmin(aggregate_rim[:, 1]))]
        bottom = aggregate_rim[int(np.argmax(aggregate_rim[:, 1]))]
        representative["center_px"] = aggregate_center.tolist()
        representative["rim_radius_px"] = float(math.sqrt(np.prod(aggregate_diameters)) / 2.0)
        representative["rim_ellipse"] = {
            "center_px": aggregate_center.tolist(),
            "diameters_px": aggregate_diameters.tolist(),
            "angle_deg": aggregate_angle,
        }
        representative["rim_extrema_px"] = {
            "left": left.tolist(),
            "right": right.tolist(),
            "top": top.tolist(),
            "bottom": bottom.tolist(),
        }
        representative["bbox_px"] = [
            float(np.min(aggregate_rim[:, 0])),
            float(np.min(aggregate_rim[:, 1])),
            float(np.max(aggregate_rim[:, 0])),
            float(np.max(aggregate_rim[:, 1])),
        ]
        representative["confidence"] = float(np.median(inlier_confidence) * stability_factor)
        representative["temporal"] = {
            "inlier_frames": int(inliers.sum()),
            "total_valid_frames": len(valid_results),
            "max_center_error_px": max_center_error,
            "max_radius_error_px": max_radius_error,
            "center_mad_px": center_mad,
            "diameter_cv": diameter_cv,
            "rim_depth_std_m": depth_std_m,
            "stability_factor": stability_factor,
        }
        if center_mad > 1.5:
            stability_reasons.append(f"{label}_center_mad_high")
        if diameter_cv > 0.03:
            stability_reasons.append(f"{label}_diameter_cv_high")
        # Depth jitter is reported for diagnostics only.  Classification and
        # action gating are RGB-only; the RGB ellipse stability gates above are
        # both faster and measurably more repeatable on this scene.
        if representative["confidence"] < 0.68:
            stability_reasons.append(f"{label}_temporal_confidence_low")
        output_objects.append(representative)

    if len(output_objects) != len(CLASS_LABELS):
        stability_reasons.append("one_or_more_classes_missing_after_temporal_filter")
    return {
        "valid": not stability_reasons,
        "invalid_reason": ";".join(stability_reasons),
        "objects": output_objects if not stability_reasons else [],
        "frame_count": len(results),
        "valid_frame_count": len(valid_results),
    }


def draw_detection_overlay(bgr: np.ndarray, result: dict) -> np.ndarray:
    overlay = bgr.copy()
    hull = result.get("debug", {}).get("tray_hull")
    if hull is not None:
        cv2.polylines(overlay, [hull], True, (255, 170, 0), 2, cv2.LINE_AA)
    colors = {"cup": (0, 255, 0), "bean_bowl": (0, 180, 255), "plate": (255, 120, 0)}
    objects = result.get("objects", [])
    for index, item in enumerate(objects):
        ellipse = item["rim_ellipse"]
        center = tuple(int(round(value)) for value in ellipse["center_px"])
        axes = tuple(int(round(value / 2.0)) for value in ellipse["diameters_px"])
        color = colors.get(item["category"], (255, 255, 255))
        cv2.ellipse(
            overlay,
            center,
            axes,
            float(ellipse["angle_deg"]),
            0,
            360,
            color,
            3,
            cv2.LINE_AA,
        )
        cv2.circle(overlay, center, 4, color, -1, cv2.LINE_AA)
        short_label = item["category"].replace("bean_bowl", "BOWL").upper()
        cv2.putText(
            overlay,
            short_label,
            (center[0] - 28, center[1] + 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )
        legend = f"{item['category']}  ({center[0]},{center[1]})  conf={item['confidence']:.2f}"
        legend_origin = (13, 27 + 27 * index)
        cv2.rectangle(
            overlay,
            (8, legend_origin[1] - 19),
            (330, legend_origin[1] + 6),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            overlay,
            legend,
            legend_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.53,
            color,
            2,
            cv2.LINE_AA,
        )
    status = "VALID: 3 UNIQUE CLASSES" if result.get("valid") else "INVALID: " + result.get("invalid_reason", "unknown")
    cv2.rectangle(overlay, (7, overlay.shape[0] - 38), (min(overlay.shape[1] - 7, 620), overlay.shape[0] - 7), (20, 20, 20), -1)
    cv2.putText(
        overlay,
        status[:74],
        (13, overlay.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        (0, 255, 0) if result.get("valid") else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay


def serializable_result(result: dict) -> dict:
    return {key: value for key, value in result.items() if key != "debug"}
