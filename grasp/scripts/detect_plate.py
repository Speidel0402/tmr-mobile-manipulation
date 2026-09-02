#!/usr/bin/env python3
"""Lightweight RGB-only detector for the pale-green 185 mm plate."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


VESSEL_DIAMETERS_MM = {"cup": 76.0, "food_bowl": 120.0, "plate": 185.0}

# At the fixed calibrated wrist height the plate's fitted equivalent diameter
# is about 176 px.  The bowl and cup are about 122 px and 76 px respectively,
# so this gate keeps object identity independent of the other two vessels.
PLATE_EQUIVALENT_DIAMETER_PX = (145.0, 220.0)
PLATE_EXPECTED_DIAMETER_PX = 176.0


def _ellipse_coordinates(shape, ellipse):
    (center_x, center_y), (diameter_a, diameter_b), angle_deg = ellipse
    yy, xx = np.indices(shape)
    angle = np.deg2rad(angle_deg)
    dx = xx - center_x
    dy = yy - center_y
    along_a = dx * np.cos(angle) + dy * np.sin(angle)
    along_b = -dx * np.sin(angle) + dy * np.cos(angle)
    normalized_radius = np.sqrt(
        (along_a / max(1.0, 0.5 * diameter_a)) ** 2
        + (along_b / max(1.0, 0.5 * diameter_b)) ** 2
    )
    return normalized_radius


def _rightmost_ellipse_point(ellipse):
    (center_x, center_y), (diameter_a, diameter_b), angle_deg = ellipse
    angles = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    rotation = np.deg2rad(angle_deg)
    ellipse_x = (
        center_x
        + 0.5 * diameter_a * np.cos(angles) * np.cos(rotation)
        - 0.5 * diameter_b * np.sin(angles) * np.sin(rotation)
    )
    ellipse_y = (
        center_y
        + 0.5 * diameter_a * np.cos(angles) * np.sin(rotation)
        + 0.5 * diameter_b * np.sin(angles) * np.cos(rotation)
    )
    index = int(np.argmax(ellipse_x))
    return float(ellipse_x[index]), float(ellipse_y[index])


def detect_plate(image):
    """Return the plate and all valid plate-sized candidates.

    The three vessels share the same pale-green color.  Plate identity is
    therefore determined by color, the calibrated 185 mm size class, and a
    smooth bright interior.  The latter rejects the food-filled bowl even if
    a partial contour is over-fitted.
    """
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be an HxWx3 BGR image")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 1.8)
    edges = cv2.Canny(blurred, 35, 110)

    # Saturation >= 24 separates the pale-green vessels from the white table.
    # Value >= 80 keeps a brightened dark-green tray from bridging the three
    # vessel components, while retaining the plate at 70% nominal exposure.
    green = cv2.inRange(
        hsv,
        np.asarray([35, 24, 80], dtype=np.uint8),
        np.asarray([105, 170, 230], dtype=np.uint8),
    )
    green = cv2.morphologyEx(
        green, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    green = cv2.morphologyEx(
        green, cv2.MORPH_CLOSE, np.ones((7, 7), dtype=np.uint8)
    )

    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(green)
    height, width = gray.shape
    candidates = []
    for label in range(1, count):
        component_area = int(stats[label, cv2.CC_STAT_AREA])
        if component_area < 9000:
            continue
        component = np.uint8(labels == label) * 255
        contours, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if len(contour) < 40:
            continue
        ellipse = cv2.fitEllipse(contour)
        (center_x, center_y), (diameter_a, diameter_b), angle_deg = ellipse
        minor_diameter = float(min(diameter_a, diameter_b))
        major_diameter = float(max(diameter_a, diameter_b))
        equivalent_diameter = float(np.sqrt(diameter_a * diameter_b))
        axis_ratio = major_diameter / max(1.0, minor_diameter)

        if not (20.0 <= center_x <= width - 20.0 and -20.0 <= center_y <= 0.82 * height):
            continue
        if not (
            PLATE_EQUIVALENT_DIAMETER_PX[0]
            <= equivalent_diameter
            <= PLATE_EQUIVALENT_DIAMETER_PX[1]
        ):
            continue
        if minor_diameter < 125.0 or major_diameter > 255.0 or axis_ratio > 1.42:
            continue

        normalized_radius = _ellipse_coordinates(gray.shape, ellipse)
        disk = normalized_radius <= 1.0
        inner = normalized_radius <= 0.68
        rim = (normalized_radius >= 0.78) & (normalized_radius <= 1.08)
        if int(np.count_nonzero(inner)) < 2500:
            continue

        inner_hsv = hsv[inner]
        rim_hsv = hsv[rim]
        inner_gray = gray[inner]
        pale_inner_fraction = float(
            np.mean(
                (inner_hsv[:, 0] >= 35)
                & (inner_hsv[:, 0] <= 105)
                & (inner_hsv[:, 1] >= 20)
                & (inner_hsv[:, 1] <= 170)
                & (inner_hsv[:, 2] >= 60)
            )
        )
        green_rim_fraction = float(
            np.mean(
                (rim_hsv[:, 0] >= 35)
                & (rim_hsv[:, 0] <= 105)
                & (rim_hsv[:, 1] >= 20)
                & (rim_hsv[:, 1] <= 170)
                & (rim_hsv[:, 2] >= 55)
            )
        )
        dark_fraction = float(np.mean(inner_gray < 75))
        texture_std = float(np.std(inner_gray))
        edge_fraction = float(np.mean(edges[inner] > 0))
        visible_ellipse_pixels = max(1, int(np.count_nonzero(disk)))
        component_fill_fraction = min(1.0, component_area / visible_ellipse_pixels)

        if pale_inner_fraction < 0.62 or green_rim_fraction < 0.42:
            continue
        if dark_fraction > 0.38 or texture_std > 25.0 or edge_fraction > 0.030:
            continue
        if component_fill_fraction < 0.62:
            continue

        right_rim_x, right_rim_y = _rightmost_ellipse_point(ellipse)
        if not (0.0 <= right_rim_x < width and 0.0 <= right_rim_y < height):
            continue
        score = (
            3.0 * pale_inner_fraction
            + 2.0 * green_rim_fraction
            + 2.0 * component_fill_fraction
            - 2.5 * dark_fraction
            - 0.025 * texture_std
            - 0.012 * abs(equivalent_diameter - PLATE_EXPECTED_DIAMETER_PX)
            - 0.8 * max(0.0, axis_ratio - 1.15)
        )
        candidates.append(
            {
                "center_x": float(center_x),
                "center_y": float(center_y),
                "diameter_a_px": float(diameter_a),
                "diameter_b_px": float(diameter_b),
                "equivalent_diameter_px": equivalent_diameter,
                "axis_ratio": axis_ratio,
                "angle_deg": float(angle_deg),
                "component_area_px": component_area,
                "component_fill_fraction": component_fill_fraction,
                "pale_inner_fraction": pale_inner_fraction,
                "green_rim_fraction": green_rim_fraction,
                "dark_fraction": dark_fraction,
                "texture_std": texture_std,
                "edge_fraction": edge_fraction,
                "right_rim_x": right_rim_x,
                "right_rim_y": right_rim_y,
                "physical_diameter_mm": VESSEL_DIAMETERS_MM["plate"],
                "score": float(score),
            }
        )

    if not candidates:
        raise RuntimeError("no pale-green plate-sized smooth candidate")
    best = max(candidates, key=lambda item: item["score"])
    return best, candidates


def draw_detection(image, best):
    shown = image.copy()
    ellipse = (
        (best["center_x"], best["center_y"]),
        (best["diameter_a_px"], best["diameter_b_px"]),
        best["angle_deg"],
    )
    cv2.ellipse(shown, ellipse, (0, 255, 0), 3)
    rim = (round(best["right_rim_x"]), round(best["right_rim_y"]))
    cv2.drawMarker(shown, rim, (0, 0, 255), cv2.MARKER_CROSS, 24, 3)
    cv2.putText(
        shown,
        "PLATE RIGHT RIM",
        (max(5, rim[0] - 150), max(28, rim[1] - 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return shown


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"cannot read {args.image}")
    best, candidates = detect_plate(image)
    if args.output:
        if not cv2.imwrite(str(args.output), draw_detection(image, best)):
            raise RuntimeError(f"cannot write {args.output}")
    print(json.dumps({"best": best, "candidates": candidates}, indent=2))


if __name__ == "__main__":
    main()
