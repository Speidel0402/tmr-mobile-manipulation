#!/usr/bin/env python3
"""Summarize Hamburg room/table measurements without deriving motion commands."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
DEFAULT_INPUT = HERE / "config" / "geometry-observations.json"
REQUIRED = (
    "room_length_m", "room_width_m", "door_clear_width_m",
    "table_top_height_m", "table_length_m", "table_width_m",
    "pickup_base_to_table_edge_m",
)
SHANGHAI_REFERENCES = {
    "spine_home_m": 0.7,
    "object_descent_m": {"cup": 0.340, "bowl": 0.360, "plate": 0.375},
    "initial_forward_m": 0.85,
    "before_door_m": 0.50,
    "forward_from_before_door_m": 1.20,
}


def review(data: dict[str, Any]) -> dict[str, Any]:
    if data.get("schema_version") != 1 or not isinstance(data.get("hamburg"), dict):
        raise ValueError("expected schema_version 1 with a hamburg measurement object")
    measured = data["hamburg"]
    missing: list[str] = []
    for field in REQUIRED:
        value = measured.get(field)
        if value is None:
            missing.append(field)
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{field} must be a positive finite measurement in metres")
    shanghai_height = data.get("shanghai_reference_table_top_height_m")
    if shanghai_height is not None and (
        isinstance(shanghai_height, bool)
        or not isinstance(shanghai_height, (int, float))
        or not math.isfinite(shanghai_height)
        or shanghai_height <= 0
    ):
        raise ValueError("shanghai_reference_table_top_height_m must be positive metres or null")
    delta = None
    if shanghai_height is not None and measured.get("table_top_height_m") is not None:
        delta = round(float(measured["table_top_height_m"]) - float(shanghai_height), 4)
    return {
        "status": "needs_measurements" if missing else "geometry_review_required",
        "motion_commanded": False,
        "ready_for_motion": False,
        "missing_hamburg_measurements": missing,
        "hamburg": measured,
        "table_top_delta_hamburg_minus_shanghai_m": delta,
        "shanghai_reference_values_not_motion_targets": SHANGHAI_REFERENCES,
        "review": [
            "Confirm clearance and base pose before attempting Shanghai pick-top posture or 0.7 m spine height.",
            "Do not scale Shanghai routes by a room-size ratio; validate the actual map, doorway and table pose.",
            "Any revised spine height, descent or route requires measured arm/base clearance and an object-specific trial.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        data = json.loads(args.input.read_text(encoding="utf-8"))
        report = review(data)
    except (OSError, ValueError, TypeError) as exc:
        report = {"status": "invalid", "motion_commanded": False,
                  "ready_for_motion": False, "error": str(exc)}
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(rendered + "\n", encoding="utf-8")
        temporary.replace(args.output)
    return 2 if report["status"] == "invalid" else 0


if __name__ == "__main__":
    raise SystemExit(main())
