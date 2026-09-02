#!/usr/bin/env python3
"""Lightweight wrist-RGB food-bowl detector for the competition table."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def detect_food_bowl(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 1.5)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.1,
        minDist=48,
        param1=80,
        param2=25,
        minRadius=30,
        maxRadius=70,
    )
    if circles is None:
        raise RuntimeError("no circular vessel candidates")
    height, width = gray.shape
    yy, xx = np.ogrid[:height, :width]
    candidates = []
    for x, y, radius in circles[0]:
        # Ignore the gripper/body region and objects clipped by the image edge.
        if y > 280 or x < radius + 4 or x > width - radius - 4:
            continue
        interior = (xx - x) ** 2 + (yy - y) ** 2 <= (0.68 * radius) ** 2
        values = gray[interior]
        dark_fraction = float(np.mean(values < 82))
        very_dark_fraction = float(np.mean(values < 58))
        mean_gray = float(np.mean(values))
        # Food pellets make this vessel uniquely dark inside.  Radius and a
        # weak right-half prior separate it from the empty cup and plate.
        score = (
            5.0 * dark_fraction
            + 3.0 * very_dark_fraction
            - 0.012 * abs(float(radius) - 48.0)
            + 0.0008 * float(x)
        )
        candidates.append(
            {
                "center_x": float(x),
                "center_y": float(y),
                "radius": float(radius),
                "dark_fraction": dark_fraction,
                "very_dark_fraction": very_dark_fraction,
                "mean_gray": mean_gray,
                "score": score,
            }
        )
    if not candidates:
        raise RuntimeError("all circular candidates rejected")
    best = max(candidates, key=lambda item: item["score"])
    if best["dark_fraction"] < 0.22:
        raise RuntimeError("best circle lacks the dark food interior")
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
