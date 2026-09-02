#!/usr/bin/env python3
"""Lightweight wrist-RGB food-bowl detector for the competition table."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


VESSEL_DIAMETERS_MM = {"cup": 76.0, "food_bowl": 120.0, "plate": 185.0}
# At the fixed calibrated wrist-camera height, successful bowl observations
# span roughly 48--59 px in radius.  This tolerance covers perspective and
# Hough quantization while excluding the 30--42 px cup/food inner circles and
# the substantially larger plate.
FOOD_BOWL_RADIUS_PX = (45.0, 65.0)


def detect_food_bowl(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(gray, (9, 9), 1.5)
    edges = cv2.Canny(gray, 45, 110)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.1,
        minDist=48,
        param1=80,
        param2=25,
        minRadius=30,
        maxRadius=75,
    )
    if circles is None:
        raise RuntimeError("no circular vessel candidates")
    height, width = gray.shape
    yy, xx = np.ogrid[:height, :width]
    candidates = []
    for x, y, radius in circles[0]:
        # The three task objects stay on the central tray.  Keeping this broad
        # ROI excludes the dark cable/equipment at the far left while still
        # allowing the bowl to move all the way to the calibrated grasp pixel.
        if not (180 <= x <= 610 and 40 <= y <= 340):
            continue
        # Keep a circle whose visible rim is still inside the frame.  The old
        # extra four-pixel margin discarded the real bowl near the right edge
        # and made the detector jump to unrelated circular objects.
        if x - radius < 0 or x + radius >= width:
            continue
        if not (FOOD_BOWL_RADIUS_PX[0] <= radius <= FOOD_BOWL_RADIUS_PX[1]):
            continue
        interior = (xx - x) ** 2 + (yy - y) ** 2 <= (0.68 * radius) ** 2
        rim = (
            ((xx - x) ** 2 + (yy - y) ** 2 >= (0.78 * radius) ** 2)
            & ((xx - x) ** 2 + (yy - y) ** 2 <= (1.04 * radius) ** 2)
        )
        values = gray[interior]
        hsv_values = hsv[interior]
        rim_hsv = hsv[rim]
        green_rim_fraction = float(
            np.mean(
                (rim_hsv[:, 0] >= 32)
                & (rim_hsv[:, 0] <= 100)
                & (rim_hsv[:, 1] >= 8)
                & (rim_hsv[:, 1] <= 150)
                & (rim_hsv[:, 2] >= 65)
            )
        )
        # All three task vessels have a pale-green outer wall.  Dark food and
        # the spoon can form strong inner Hough circles, but they do not have
        # this green annulus and must never become vessel candidates.
        if green_rim_fraction < 0.35:
            continue
        dark_fraction = float(np.mean(values < 82))
        very_dark_fraction = float(np.mean(values < 58))
        mean_gray = float(np.mean(values))
        texture_std = float(np.std(values))
        edge_fraction = float(np.mean(edges[interior] > 0))
        brown_fraction = float(
            np.mean(
                (hsv_values[:, 0] >= 3)
                & (hsv_values[:, 0] <= 30)
                & (hsv_values[:, 1] >= 35)
                & (hsv_values[:, 2] >= 35)
                & (hsv_values[:, 2] <= 190)
            )
        )
        # Pellets create many internal edges and mixed dark/brown pixels.  The
        # empty cup and plate have smooth interiors, so texture is weighted
        # more heavily than raw darkness (which also occurs in the tray).
        score = (
            3.0 * dark_fraction
            + 1.5 * very_dark_fraction
            + 16.0 * edge_fraction
            + 5.0 * brown_fraction
            + 2.0 * green_rim_fraction
            + 0.025 * texture_std
            - 0.010 * abs(float(radius) - 55.0)
        )
        candidates.append(
            {
                "center_x": float(x),
                "center_y": float(y),
                "radius": float(radius),
                "dark_fraction": dark_fraction,
                "very_dark_fraction": very_dark_fraction,
                "mean_gray": mean_gray,
                "texture_std": texture_std,
                "edge_fraction": edge_fraction,
                "brown_fraction": brown_fraction,
                "green_rim_fraction": green_rim_fraction,
                "physical_diameter_mm": VESSEL_DIAMETERS_MM["food_bowl"],
                "score": score,
            }
        )
    if not candidates:
        raise RuntimeError("all circular candidates rejected")
    best = max(candidates, key=lambda item: item["score"])
    if best["dark_fraction"] < 0.30 or best["edge_fraction"] < 0.020:
        raise RuntimeError("best circle lacks the dark textured food interior")
    best["right_rim_x"] = best["center_x"] + best["radius"]
    best["right_rim_y"] = best["center_y"]
    return best, candidates


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"cannot read {args.image}")
    best, candidates = detect_food_bowl(image)
    if args.output:
        shown = image.copy()
        center = (round(best["center_x"]), round(best["center_y"]))
        radius = round(best["radius"])
        rim = (round(best["right_rim_x"]), round(best["right_rim_y"]))
        cv2.circle(shown, center, radius, (0, 255, 0), 3)
        cv2.drawMarker(shown, rim, (0, 0, 255), cv2.MARKER_CROSS, 24, 3)
        cv2.putText(
            shown,
            "FOOD BOWL RIGHT RIM",
            (max(5, center[0] - 100), max(28, center[1] - radius - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if not cv2.imwrite(str(args.output), shown):
            raise RuntimeError(f"cannot write {args.output}")
    print(json.dumps({"best": best, "candidates": candidates}, indent=2))


if __name__ == "__main__":
    main()
