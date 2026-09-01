#!/usr/bin/env python3
"""RGB-only detector for the calibrated green cup rim."""

import cv2
import numpy as np


def _refine_right_edge(edges, center_x, center_y, radius):
    yy, xx = np.indices(edges.shape)
    radial = np.hypot(xx - center_x, yy - center_y)
    rim_points = np.column_stack(
        np.nonzero((edges > 0) & (radial >= 0.78 * radius) & (radial <= 1.22 * radius))[::-1]
    ).astype(np.float32)
    if len(rim_points) < 35:
        return np.asarray([center_x + radius, center_y], dtype=float), None
    ellipse = cv2.fitEllipse(rim_points.reshape(-1, 1, 2))
    (fit_x, fit_y), (diameter_a, diameter_b), angle_deg = ellipse
    if np.hypot(fit_x - center_x, fit_y - center_y) > 0.28 * radius:
        return np.asarray([center_x + radius, center_y], dtype=float), None
    if not (
        1.35 * radius <= diameter_a <= 2.65 * radius
        and 1.35 * radius <= diameter_b <= 2.65 * radius
        and max(diameter_a, diameter_b) / max(1.0, min(diameter_a, diameter_b)) <= 1.45
    ):
        return np.asarray([center_x + radius, center_y], dtype=float), None
    angles = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    rotation = np.deg2rad(angle_deg)
    ellipse_x = (
        fit_x
        + 0.5 * diameter_a * np.cos(angles) * np.cos(rotation)
        - 0.5 * diameter_b * np.sin(angles) * np.sin(rotation)
    )
    ellipse_y = (
        fit_y
        + 0.5 * diameter_a * np.cos(angles) * np.sin(rotation)
        + 0.5 * diameter_b * np.sin(angles) * np.cos(rotation)
    )
    index = int(np.argmax(ellipse_x))
    return np.asarray([ellipse_x[index], ellipse_y[index]], dtype=float), {
        "center_px": [float(fit_x), float(fit_y)],
        "diameters_px": [float(diameter_a), float(diameter_b)],
        "angle_deg": float(angle_deg),
        "edge_point_count": int(len(rim_points)),
    }


def detect_green_cup_right(bgr):
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("bgr must be an HxWx3 image")
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2.0)
    circles = None
    for accumulator_threshold in (26, 24, 22):
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=90,
            param2=accumulator_threshold,
            minRadius=28,
            maxRadius=48,
        )
        if circles is not None:
            break
    if circles is None:
        raise RuntimeError("no cup-sized rim")

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(blurred, 35, 110)
    yy, xx = np.ogrid[: bgr.shape[0], : bgr.shape[1]]
    candidates = []
    for center_x, center_y, radius in circles[0]:
        if not (35 < center_x < bgr.shape[1] - 35 and 45 < center_y < 0.78 * bgr.shape[0]):
            continue
        inner = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= (0.58 * radius) ** 2
        hue = float(np.median(hsv[:, :, 0][inner]))
        saturation = float(np.median(hsv[:, :, 1][inner]))
        value = float(np.median(hsv[:, :, 2][inner]))
        texture = float(np.std(gray[inner]))
        if not (45.0 <= hue <= 92.0 and saturation >= 52.0 and value >= 45.0):
            continue
        # Beans are dark and highly textured; the plate is excluded by radius.
        if value < 52.0 and texture > 24.0:
            continue
        score = (
            saturation
            + 0.18 * value
            - 1.2 * abs(float(radius) - 37.0)
            - 0.12 * max(0.0, texture - 18.0)
        )
        candidates.append(
            (score, float(center_x), float(center_y), float(radius), hue, saturation, value, texture)
        )
    if not candidates:
        raise RuntimeError("green cup candidate absent")

    _, center_x, center_y, radius, hue, saturation, value, texture = max(candidates)
    point, ellipse = _refine_right_edge(edges, center_x, center_y, radius)
    return point, {
        "center_px": [center_x, center_y],
        "radius_px": radius,
        "hsv_median": [hue, saturation, value],
        "inner_texture": texture,
        "ellipse": ellipse,
        "method": "ellipse_refined" if ellipse is not None else "hough_circle",
    }
