#!/usr/bin/env python3
"""Create explicit Hamburg mission/grasp overrides from measured values."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from hamburg_grasp_cycle import validate_motion_config


HERE = Path(__file__).resolve().parent
MISSION_DEFAULT = HERE / "config" / "mission-shanghai-reference.json"
GRASP_DEFAULT = HERE / "config" / "grasp-cycle-shanghai-reference.json"


def set_if(value: Any, target: dict[str, Any], key: str) -> None:
    if value is not None:
        target[key] = value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission-template", type=Path, default=MISSION_DEFAULT)
    parser.add_argument("--grasp-template", type=Path, default=GRASP_DEFAULT)
    parser.add_argument("--mission-output", type=Path, required=True)
    parser.add_argument("--grasp-output", type=Path, required=True)
    for name in ("room-length", "room-width", "door-clear-width", "tabletop-height",
                 "tabletop-length", "tabletop-width", "pickup-standoff"):
        parser.add_argument(f"--{name}-m", type=float)
    parser.add_argument("--initial-forward-m", type=float)
    parser.add_argument("--before-door-m", type=float)
    parser.add_argument("--through-door-m", type=float)
    parser.add_argument("--pickup-approach-maximum-m", type=float)
    parser.add_argument("--pickup-front-clearance-m", type=float)
    parser.add_argument("--cup-descent-m", type=float)
    parser.add_argument("--bowl-descent-m", type=float)
    parser.add_argument("--plate-descent-m", type=float)
    parser.add_argument("--spine-m", type=float)
    parser.add_argument("--posture-velocity-rad-s", type=float)
    parser.add_argument("--maximum-following-error-rad", type=float)
    parser.add_argument("--following-error-pause-rad", type=float)
    parser.add_argument("--following-error-resume-rad", type=float)
    parser.add_argument("--following-error-recovery-timeout-s", type=float)
    parser.add_argument("--joint-feedback-stale-timeout-s", type=float)
    args = parser.parse_args()

    mission = json.loads(args.mission_template.read_text(encoding="utf-8"))
    grasp = json.loads(args.grasp_template.read_text(encoding="utf-8"))
    measurements = mission["hamburg_measurements"]
    for argument, key in (
        (args.room_length_m, "room_length_m"), (args.room_width_m, "room_width_m"),
        (args.door_clear_width_m, "door_clear_width_m"),
        (args.tabletop_height_m, "tabletop_height_m"),
        (args.tabletop_length_m, "tabletop_length_m"),
        (args.tabletop_width_m, "tabletop_width_m"),
        (args.pickup_standoff_m, "pickup_standoff_m"),
    ):
        set_if(argument, measurements, key)
    stages = {item["name"]: item for item in mission["outbound_shanghai_reference"]}
    set_if(args.initial_forward_m, stages["initial_forward"], "forward_m")
    set_if(args.before_door_m, stages["to_before_door"], "forward_m")
    set_if(args.through_door_m, stages["through_door"], "forward_m")
    set_if(args.pickup_approach_maximum_m, stages["approach_pickup_table"], "maximum_forward_m")
    set_if(args.pickup_front_clearance_m, stages["approach_pickup_table"], "front_clearance_m")
    for name in ("cup", "bowl", "plate"):
        set_if(getattr(args, f"{name}_descent_m"), grasp["objects"][name], "descent_m")
    set_if(args.spine_m, grasp["spine"], "initial_target_m")
    for argument, key in (
        (args.posture_velocity_rad_s, "posture_joint_velocity_rad_s"),
        (args.maximum_following_error_rad, "maximum_following_error_rad"),
        (args.following_error_pause_rad, "following_error_pause_rad"),
        (args.following_error_resume_rad, "following_error_resume_rad"),
        (args.following_error_recovery_timeout_s, "following_error_recovery_timeout_s"),
        (args.joint_feedback_stale_timeout_s, "joint_feedback_stale_timeout_s"),
    ):
        set_if(argument, grasp["motion"], key)
    for key, value in measurements.items():
        if key != "source" and value is not None and float(value) <= 0.0:
            parser.error(f"--{key.replace('_', '-')} must be positive")
    for key, value in (
        ("initial-forward-m", stages["initial_forward"]["forward_m"]),
        ("before-door-m", stages["to_before_door"]["forward_m"]),
        ("through-door-m", stages["through_door"]["forward_m"]),
        ("pickup-approach-maximum-m", stages["approach_pickup_table"]["maximum_forward_m"]),
        ("pickup-front-clearance-m", stages["approach_pickup_table"]["front_clearance_m"]),
    ):
        if float(value) <= 0.0:
            parser.error(f"--{key} must be positive")
    for name in ("cup", "bowl", "plate"):
        descent = float(grasp["objects"][name]["descent_m"])
        if not 0.05 <= descent <= 0.40:
            parser.error(f"--{name}-descent-m must be between 0.05 and 0.40")
    if not 0.0 <= float(grasp["spine"]["initial_target_m"]) <= 0.8:
        parser.error("--spine-m must be between 0.0 and 0.8")
    try:
        validate_motion_config(grasp["motion"])
    except (KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))
    mission["profile_name"] = "hamburg_site_override"
    mission["profile_warning"] = (
        "This file contains explicit site values supplied for Hamburg. Any unchanged route field "
        "still retains its individually named Shanghai reference and must be interpreted that way."
    )
    for path, value in ((args.mission_output, mission), (args.grasp_output, grasp)):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"mission_config": str(args.mission_output),
                      "grasp_config": str(args.grasp_output),
                      "remaining_unmeasured_hamburg_fields": [
                          key for key, value in measurements.items()
                          if key != "source" and value is None
                      ]}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
