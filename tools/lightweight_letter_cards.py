#!/usr/bin/env python3
"""Extract and enhance small letter cards from a ZED RGB frame.

The detector is deliberately lightweight: HSV thresholding, connected contours,
quadrilateral perspective correction, and adaptive thresholding.  It has no
depth, ROS, neural-network, or OCR runtime dependency.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def order_quad(points: np.ndarray) -> np.ndarray:
    points = points.reshape(4, 2).astype(np.float32)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).ravel()
    return np.array(
        [points[np.argmin(sums)], points[np.argmin(diffs)],
         points[np.argmax(sums)], points[np.argmax(diffs)]],
        dtype=np.float32,
    )


def warp_card(image: np.ndarray, quad: np.ndarray, size: int = 180) -> np.ndarray:
    src = order_quad(quad)
    dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]], np.float32)
    return cv2.warpPerspective(image, cv2.getPerspectiveTransform(src, dst), (size, size))


def detect_cards(image: np.ndarray) -> list[tuple[int, int, int, int, np.ndarray]]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = image.shape[:2]
    scale = max(1.0, width / 640.0)
    # Only propose low-saturation white paper. Closing reconnects a card across
    # its printed glyph or a narrow cable occlusion; saturated cup pixels never
    # enter the proposal mask.
    mask = cv2.inRange(hsv, (0, 0, 100), (179, 60, 255))
    kernel_size = max(3, int(round(3 * scale)) | 1)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((kernel_size, kernel_size), np.uint8), iterations=2
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    found = []
    min_area = 55 * scale * scale
    max_area = 5000 * scale * scale
    for label in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[label])
        if not min_area <= area <= max_area:
            continue
        if y < int(0.14 * height) or min(w, h) < 8 * scale or max(w, h) > 115 * scale:
            continue
        ratio = max(w, h) / max(1.0, min(w, h))
        if ratio > 2.7:
            continue

        # A real card is brighter than the immediately surrounding tabletop.
        pad = max(6, int(round(7 * scale)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(width, x + w + pad), min(height, y + h + pad)
        outer = hsv[y0:y1, x0:x1]
        ring = np.ones(outer.shape[:2], dtype=bool)
        ring[y - y0:y - y0 + h, x - x0:x - x0 + w] = False
        inner = hsv[y:y + h, x:x + w]
        if float(np.mean(inner[:, :, 1])) > 63.0:
            continue
        if float(np.mean(inner[:, :, 2])) - float(np.mean(outer[:, :, 2][ring])) < 24.0:
            continue

        component = np.zeros(mask.shape, np.uint8)
        component[labels == label] = 255
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        if min(rw, rh) < 7 * scale:
            continue
        quad = cv2.boxPoints(rect)
        qx, qy, qw, qh = cv2.boundingRect(quad.astype(np.int32))
        if qx < 4 or qx + qw >= width - 4 or qy + qh >= height - 4:
            continue
        found.append((qx, qy, qw, qh, quad))

    # Suppress nested/duplicate contours by center distance.
    found.sort(key=lambda item: item[2] * item[3], reverse=True)
    unique = []
    for candidate in found:
        cx = candidate[0] + candidate[2] / 2
        cy = candidate[1] + candidate[3] / 2
        if all((cx - (u[0] + u[2] / 2)) ** 2 + (cy - (u[1] + u[3] / 2)) ** 2 > 18 ** 2 for u in unique):
            unique.append(candidate)
    return sorted(unique, key=lambda item: (item[1], item[0]))


def enhance(card: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(card, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(2.0, (8, 8)).apply(gray)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, 31, 8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--annotated", type=Path, required=True)
    parser.add_argument("--montage", type=Path, required=True)
    args = parser.parse_args()

    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"cannot read {args.image}")
    cards = detect_cards(image)
    annotated = image.copy()
    tiles = []
    for index, (x, y, w, h, quad) in enumerate(cards, 1):
        card = warp_card(image, quad)
        binary = enhance(card)
        tile = np.hstack((card, cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)))
        cv2.putText(tile, f"card {index}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 255), 2, cv2.LINE_AA)
        tiles.append(tile)
        cv2.polylines(annotated, [quad.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(annotated, str(index), (x, max(18, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    if tiles:
        montage = np.vstack(tiles)
    else:
        montage = np.full((180, 360, 3), 255, np.uint8)
        cv2.putText(montage, "No cards detected", (38, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    args.annotated.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.annotated), annotated)
    cv2.imwrite(str(args.montage), montage)
    print(f"cards={len(cards)}")
    for index, item in enumerate(cards, 1):
        print(f"card={index} bbox={item[:4]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
