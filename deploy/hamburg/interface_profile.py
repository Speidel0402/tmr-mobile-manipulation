#!/usr/bin/env python3
"""Load and validate the Hamburg command-interface compatibility profile."""

from __future__ import annotations

from copy import deepcopy
import json
import math
import os
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
DEFAULT_PROFILE = HERE / "config" / "interfaces-shanghai.json"


def _set_path(value: dict[str, Any], dotted_path: str, replacement: Any) -> None:
    target: dict[str, Any] = value
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        target = target[part]
    current = target[parts[-1]]
    if isinstance(current, float):
        replacement = float(replacement)
    elif isinstance(current, int):
        replacement = int(replacement)
    target[parts[-1]] = replacement


def load_interface_profile(
    path: Path = DEFAULT_PROFILE,
    environment: dict[str, str] | None = None,
) -> dict[str, Any]:
    profile = json.loads(path.read_text(encoding="utf-8"))
    environ = os.environ if environment is None else environment
    applied = {}
    for name, dotted_path in profile.get("environment_overrides", {}).items():
        if name in environ and str(environ[name]).strip() != "":
            _set_path(profile, dotted_path, environ[name])
            applied[name] = str(environ[name])
    profile["applied_environment_overrides"] = applied
    validate_interface_profile(profile)
    return profile


def validate_interface_profile(profile: dict[str, Any]) -> None:
    gripper = profile["gripper"]
    spine = profile["spine"]
    for label, section in (("gripper", gripper),):
        if "/msg/" not in str(section["message_type"]):
            raise ValueError(f"{label}.message_type must be a ROS message type")
        if not str(section["field"]).strip():
            raise ValueError(f"{label}.field must not be empty")
    low = float(gripper["minimum"])
    high = float(gripper["maximum"])
    opened = float(gripper["open"])
    closed = float(gripper["closed"])
    if not low <= closed < opened <= high:
        raise ValueError("gripper values must satisfy minimum <= closed < open <= maximum")
    home = float(spine["home_m"])
    if not math.isfinite(home) or not float(spine["minimum_m"]) <= home <= float(
        spine["maximum_m"]
    ):
        raise ValueError("spine.home_m is outside the configured range")
    interface = spine["interface"]
    if interface == "action":
        if "/action/" not in spine["action_type"] or "/srv/" not in spine["position_service_type"]:
            raise ValueError("spine action and position service must have ROS interface types")
        if not spine["action_name"].startswith("/") or not spine["position_service"].startswith("/"):
            raise ValueError("spine action and position service require absolute names")
    elif interface == "topic":
        if "/msg/" not in spine["message_type"] or not spine["field"]:
            raise ValueError("spine topic requires a ROS message type and field")
        if not spine["topic"].startswith("/"):
            raise ValueError("spine topic requires an absolute name")
    else:
        raise ValueError(f"unsupported spine interface {interface!r}")
    camera = profile["head_camera"]
    if int(camera["width"]) <= 0 or int(camera["height"]) <= 0:
        raise ValueError("head camera dimensions must be positive")
    arm_motion = profile["arm_motion"]
    for field in (
        "posture_joint_velocity_rad_s", "maximum_following_error_rad",
        "following_error_pause_rad", "following_error_resume_rad",
        "following_error_recovery_timeout_s",
    ):
        value = float(arm_motion[field])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"arm_motion.{field} must be positive and finite")
    if not (
        float(arm_motion["following_error_resume_rad"])
        < float(arm_motion["following_error_pause_rad"])
        < float(arm_motion["maximum_following_error_rad"])
        <= 0.35
    ):
        raise ValueError(
            "arm_motion following-error limits must satisfy resume < pause < maximum <= 0.35"
        )


def apply_interface_profile(
    venue: dict[str, Any], profile: dict[str, Any]
) -> dict[str, Any]:
    resolved = deepcopy(venue)
    commands = {item["name"]: item for item in resolved["command_endpoints"]}
    gripper = profile["gripper"]
    spine = profile["spine"]
    for name, topic in (
        ("left_gripper_target", gripper["left_topic"]),
        ("right_gripper_target", gripper["right_topic"]),
    ):
        commands[name].update(
            {
                "topic": topic,
                "accepted_types": [gripper["message_type"]],
                "message_field": gripper["field"],
                "command_values": {
                    "open": gripper["open"],
                    "closed": gripper["closed"],
                    "unit": gripper["unit"],
                },
            }
        )
    commands.pop("spine_height_target", None)
    resolved["command_endpoints"] = list(commands.values())
    resolved["service_endpoints"] = []
    resolved["action_endpoints"] = []
    if spine["interface"] == "topic":
        resolved["command_endpoints"].append({
            "name": "spine_height_target",
            "topic": spine["topic"],
            "accepted_types": [spine["message_type"]],
            "message_field": spine["field"],
            "command_values": {
                "home": spine["home_m"],
                "unit": spine["unit"],
                "absolute": bool(spine["absolute"]),
            },
            "required": True,
        })
    else:
        resolved["service_endpoints"].append({
            "name": "spine_position",
            "service": spine["position_service"],
            "accepted_types": [spine["position_service_type"]],
            "required": True,
        })
        resolved["action_endpoints"].append({
            "name": "spine_height_target",
            "action": spine["action_name"],
            "accepted_types": [spine["action_type"]],
            "required": True,
        })
    for stream in resolved["streams"]:
        if stream["name"] == "head_camera":
            stream["expected_width"] = int(profile["head_camera"]["width"])
            stream["expected_height"] = int(profile["head_camera"]["height"])
    resolved["interface_profile"] = profile
    return resolved
