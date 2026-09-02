#!/usr/bin/env python3
"""Lightweight monocular Z calibration from the fixed head ZED RGB image."""

from __future__ import annotations

import cv2
import numpy as np


GRIPPER_COLUMN_NORM = 294.0 / 640.0
TARGET_FLANGE_RADIUS_RATIO = 0.35


def detect_target_cup(image: np.ndarray) -> dict:
    """Select the cup whose right rim lies below the calibrated gripper column."""
    if image is None or image.ndim != 3:
        raise RuntimeError("head RGB image is invalid")
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 1.5)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(20, int(round(width * 0.038))),
        param1=80,
        param2=24,
        minRadius=max(8, int(round(width * 0.017))),
        maxRadius=max(30, int(round(width * 0.070))),
    )
    if circles is None:
        raise RuntimeError("head RGB cup circle unavailable")
    reference_x = width * GRIPPER_COLUMN_NORM
    candidates = []
    for cx, cy, radius in circles[0]:
        if not (0.30 * width <= cx <= 0.55 * width):
            continue
        if not (0.48 * height <= cy <= 0.86 * height):
            continue
        right_x = float(cx + radius)
        expected_radius = width * (18.0 / 640.0)
        score = (
            abs(right_x - reference_x)
            + 1.5 * abs(float(radius) - expected_radius)
            + 0.04 * abs(float(cy) - 0.65 * height)
        )
        candidates.append((score, float(cx), float(cy), float(radius), right_x))
    if not candidates:
        raise RuntimeError("no head RGB cup candidate in pickup ROI")
    score, cx, cy, radius, right_x = min(candidates)
    if abs(right_x - reference_x) > 0.075 * width:
        raise RuntimeError("head RGB cup is not under the gripper descent column")
    return {
        "center_x_px": cx,
        "center_y_px": cy,
        "radius_px": radius,
        "right_rim_x_px": right_x,
        "score": float(score),
    }


def target_flange_edge_y(cup: dict) -> float:
    return float(cup["center_y_px"]) - TARGET_FLANGE_RADIUS_RATIO * float(
        cup["radius_px"]
    )


def detect_flange_edge_y(image: np.ndarray, cup: dict) -> float:
    """Track the visible lower edge of the silver gripper flange.

    The black fingertips merge with the black tray and cup interior at grasp
    height.  The silver flange remains high contrast, and its rigid offset to
    the fingertips is fixed by the tool geometry.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width = image.shape[:2]
    cx = float(cup["center_x_px"])
    cy = float(cup["center_y_px"])
    radius = float(cup["radius_px"])
    x0 = max(0, int(round(cx + 2)))
    x1 = min(width, int(round(cx + max(40.0, 2.5 * radius))))
    y0 = max(0, int(round(cy - 1.5 * radius)))
    y1 = min(height, int(round(cy - 0.25 * radius)))
    if x1 - x0 < 20 or y1 - y0 < 12:
        raise RuntimeError("head RGB flange ROI is invalid")
    profile = np.mean(gray[y0:y1, x0:x1], axis=1)
    smooth = np.convolve(profile, np.ones(5, dtype=float) / 5.0, mode="same")
    derivative = np.diff(smooth)
    active = derivative < -3.5
    groups = []
    start = None
    for index, value in enumerate(active.tolist() + [False]):
        if value and start is None:
            start = index
        elif not value and start is not None:
            indices = np.arange(start, index)
            weights = -derivative[indices]
            strength = float(np.sum(weights))
            centre = float(np.sum(indices * weights) / max(strength, 1e-9))
            groups.append((strength, centre))
            start = None
    if not groups:
        raise RuntimeError("head RGB silver flange edge unavailable")
    strength, centre = max(groups)
    if strength < 14.0:
        raise RuntimeError("head RGB silver flange edge is too weak")
    return float(y0 + centre)


def estimate_remaining_down_m(
    first_edge_y: float,
    second_edge_y: float,
    probe_down_m: float,
    desired_edge_y: float,
) -> tuple[float, float]:
    slope = (float(second_edge_y) - float(first_edge_y)) / float(probe_down_m)
    if not 120.0 <= slope <= 600.0:
        raise RuntimeError(f"head RGB vertical scale is unreliable: {slope:.1f}px/m")
    remaining = (float(desired_edge_y) - float(second_edge_y)) / slope
    return float(np.clip(remaining, -0.012, 0.050)), float(slope)


__all__ = [
    "detect_target_cup",
    "target_flange_edge_y",
    "detect_flange_edge_y",
    "estimate_remaining_down_m",
]
