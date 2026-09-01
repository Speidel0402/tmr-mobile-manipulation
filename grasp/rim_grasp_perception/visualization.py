from __future__ import annotations

import cv2
import numpy as np


COLORS = [(80, 220, 80), (255, 160, 40), (80, 160, 255), (220, 80, 180)]


def draw_overlay(rgb: np.ndarray, results, debug) -> np.ndarray:
    out = rgb.copy()
    detections = debug.get("detections", [])
    rims = iter(debug.get("rims", []))
    for i, det in enumerate(detections):
        color = COLORS[i % len(COLORS)]
        layer = np.zeros_like(out)
        layer[det.mask] = color
        out = cv2.addWeighted(out, 1.0, layer, 0.30, 0)
        x1, y1, x2, y2 = np.asarray(det.box_xyxy, int)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        cv2.putText(out, f"{det.category} {det.score:.2f}", (x1, max(18, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
    for rim in debug.get("rims", []):
        cv2.ellipse(out, rim.ellipse, (255, 255, 0), 2)
        for uv, ok in zip(rim.candidate_uv, rim.candidate_valid):
            cv2.circle(out, tuple(np.rint(uv).astype(int)), 2, (0, 255, 0) if ok else (255, 0, 0), -1)
    for r in results:
        label = "VALID" if r.valid else f"INVALID: {r.invalid_reason}"
        cv2.putText(out, label, (12, out.shape[0] - 16 - 22*results.index(r)), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (0, 255, 0) if r.valid else (255, 70, 70), 2)
    return out
