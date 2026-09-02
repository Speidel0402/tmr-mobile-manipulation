#!/usr/bin/env python3
"""RGB-only detector for the calibrated cup rim.

Colour is deliberately only a soft cue: the wrist D405's white balance can
move the same pale-green cup across a large part of HSV space.
"""

import cv2
import numpy as np


def nested_in_larger_rim(center_x, center_y, radius, larger_circles):
    """Identify a bowl/plate inner circle masquerading as the 76 mm cup."""
    if larger_circles is None:
        return False
    for larger_x, larger_y, larger_radius in larger_circles:
        # A 120 mm bowl is only about 1.58 times the cup diameter and
        # perspective commonly reduces its observed outer/inner ratio to
        # 1.3--1.5.  The old 1.50 cutoff therefore let the food-bowl inner rim
        # masquerade as a 76 mm cup.
        if larger_radius < 1.28 * radius:
            continue
        if np.hypot(larger_x - center_x, larger_y - center_y) <= 0.34 * larger_radius:
            return True
    return False


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

    larger = None
    for accumulator_threshold in (26, 23, 20):
        larger = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=55,
            param1=90,
            param2=accumulator_threshold,
            minRadius=44,
            maxRadius=115,
        )
        if larger is not None:
            break
    larger_circles = None if larger is None else larger[0]

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(blurred, 35, 110)
    yy, xx = np.ogrid[: bgr.shape[0], : bgr.shape[1]]
    candidates = []
    for center_x, center_y, radius in circles[0]:
        if not (35 < center_x < bgr.shape[1] - 35 and 45 < center_y < 0.78 * bgr.shape[0]):
            continue
        if nested_in_larger_rim(center_x, center_y, radius, larger_circles):
            continue
        inner = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= (0.58 * radius) ** 2
        hue = float(np.median(hsv[:, :, 0][inner]))
        saturation = float(np.median(hsv[:, :, 1][inner]))
        value = float(np.median(hsv[:, :, 2][inner]))
        texture = float(np.std(gray[inner]))
        radial = np.hypot(xx - center_x, yy - center_y)
        rim = (radial >= 0.82 * radius) & (radial <= 1.18 * radius)
        rim_support = float(np.mean(edges[rim] > 0))
        rim_hsv = hsv[rim]
        green_rim_fraction = float(
            np.mean(
                (rim_hsv[:, 0] >= 28)
                & (rim_hsv[:, 0] <= 110)
                & (rim_hsv[:, 1] >= 8)
                & (rim_hsv[:, 1] <= 190)
                & (rim_hsv[:, 2] >= 50)
            )
        )

        # 76 mm cup calibration at the fixed pickup height gives roughly
        # r=36 px.  This size and a supported circular rim are much more stable
        # than hue.  They also reject the 120 mm bowl and 185 mm plate.
        if not (
            30.0 <= radius <= 43.5
            and rim_support >= 0.052
            and green_rim_fraction >= 0.28
            and value >= 42.0
        ):
            continue
        # Reject dark, highly textured food while tolerating a spoon/reflection.
        if texture > 29.0 or (value < 52.0 and texture > 22.0):
            continue
        pale_green_bonus = 5.0 if 8.0 <= saturation <= 150.0 else 0.0
        score = (
            300.0 * rim_support
            + pale_green_bonus
            + 8.0 * green_rim_fraction
            + 0.04 * value
            - 2.0 * abs(float(radius) - 37.0)
            - 0.12 * max(0.0, texture - 18.0)
        )
        candidates.append(
            (
                score,
                float(center_x),
                float(center_y),
                float(radius),
                hue,
                saturation,
                value,
                texture,
                rim_support,
                green_rim_fraction,
            )
        )
    if not candidates:
        raise RuntimeError("green cup candidate absent")

    (
        _, center_x, center_y, radius, hue, saturation, value, texture,
        rim_support, green_rim_fraction,
    ) = max(candidates)
    point, ellipse = _refine_right_edge(edges, center_x, center_y, radius)
    return point, {
        "center_px": [center_x, center_y],
        "radius_px": radius,
        "hsv_median": [hue, saturation, value],
        "inner_texture": texture,
        "rim_support": rim_support,
        "green_rim_fraction": green_rim_fraction,
        "physical_diameter_mm": 76.0,
        "ellipse": ellipse,
        "method": "ellipse_refined" if ellipse is not None else "hough_circle",
    }
