#!/usr/bin/env python3
"""Run one autonomous Hamburg observation-to-grasp-to-release trial.

The deployed Franka controller consumes JointState targets on topics whose
names contain ``gello``.  No GELLO leader is used here: this single ROS node
computes and streams autonomous targets directly to that controller input.
"""

from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
from typing import Any

import numpy as np

from hamburg_grasp_check import decode_wrist_rgb, detect_object, save_observation_image
from hamburg_motion_core import (
    FR3V2_JOINT_LIMITS_RAD,
    cartesian_waypoints,
    derive_mount_rotation,
    gripper_contact_report,
    smooth_joint_targets,
)
from hamburg_preflight import environment_report, load_config
from spine_control import SpineControl


HERE = Path(__file__).resolve().parent
DEFAULT_CYCLE_CONFIG = HERE / "config" / "grasp-cycle-shanghai-reference.json"
DEFAULT_VENUE_CONFIG = HERE / "config" / "venue.json"
OBJECTS = ("cup", "bowl", "plate")


def validate_motion_config(motion: dict[str, Any]) -> None:
    positive_motion = (
        "publish_rate_hz", "maximum_joint_velocity_rad_s",
        "posture_joint_velocity_rad_s", "maximum_following_error_rad",
        "following_error_pause_rad", "following_error_resume_rad",
        "following_error_recovery_timeout_s", "endpoint_tolerance_rad",
        "endpoint_timeout_s", "cartesian_waypoint_step_m",
        "maximum_visual_xy_travel_m", "maximum_visual_step_m",
    )
    for field in positive_motion:
        value = float(motion[field])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"motion.{field} must be positive and finite")
    if float(motion["posture_joint_velocity_rad_s"]) > float(
        motion["maximum_joint_velocity_rad_s"]
    ):
        raise ValueError(
            "motion.posture_joint_velocity_rad_s must not exceed "
            "motion.maximum_joint_velocity_rad_s"
        )
    resume = float(motion["following_error_resume_rad"])
    pause = float(motion["following_error_pause_rad"])
    maximum = float(motion["maximum_following_error_rad"])
    if not resume < pause < maximum:
        raise ValueError(
            "motion following-error limits must satisfy resume < pause < maximum"
        )
    if maximum > 0.35:
        raise ValueError("motion.maximum_following_error_rad must not exceed 0.35 rad")


def load_cycle_config(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    required = {
        "profile_name", "parameter_scope", "arm_command_interface", "topics",
        "joint_names", "posture", "motion", "vision", "spine", "gripper", "objects",
    }
    missing = sorted(required - data.keys())
    if missing:
        raise ValueError(f"grasp-cycle config is missing: {', '.join(missing)}")
    required_topics = {
        "left_joint_state", "right_joint_state", "left_joint_target", "right_joint_target",
        "left_gripper_state", "left_gripper_target", "left_wrist_image",
    }
    missing_topics = sorted(required_topics - data["topics"].keys())
    if missing_topics:
        raise ValueError("grasp-cycle topics are missing: " + ", ".join(missing_topics))
    for name, topic in data["topics"].items():
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError(f"topics.{name} must be an absolute ROS topic")
    for arm in ("left", "right"):
        names = data["joint_names"][arm]
        if len(names) != 7 or len(set(names)) != 7 or not all(isinstance(name, str) and name for name in names):
            raise ValueError(f"joint_names.{arm} must contain seven unique names")
    for label, arm in (("left_pick_top_rad", "left"), ("right_parking_rad", "right")):
        values = np.asarray(data["posture"][label], dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError(f"posture.{label} must contain seven finite joints")
        if np.any(values <= FR3V2_JOINT_LIMITS_RAD[:, 0]) or np.any(values >= FR3V2_JOINT_LIMITS_RAD[:, 1]):
            raise ValueError(f"posture.{label} violates official FR3v2 joint limits")
    quaternion = np.asarray(data["posture"]["left_tool_quaternion_xyzw"], dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1.0e-6:
        raise ValueError("posture.left_tool_quaternion_xyzw must be a non-zero finite quaternion")
    # Config copies made from the previous Hamburg commit remain runnable.  The
    # new defaults add a separate posture-speed override and a bounded
    # hold-and-catch-up window without changing an older file's normal ramp.
    motion = data["motion"]
    maximum_velocity = float(motion["maximum_joint_velocity_rad_s"])
    maximum_error = float(motion["maximum_following_error_rad"])
    motion.setdefault("posture_joint_velocity_rad_s", maximum_velocity)
    motion.setdefault("following_error_pause_rad", min(0.20, maximum_error * 0.80))
    motion.setdefault(
        "following_error_resume_rad", float(motion["following_error_pause_rad"]) * 0.70
    )
    motion.setdefault("following_error_recovery_timeout_s", 5.0)
    validate_motion_config(motion)
    vision = data["vision"]
    width, height = int(vision["width"]), int(vision["height"])
    if (width, height) != (640, 480):
        raise ValueError("vision dimensions must match the calibrated 640x480 wrist stream")
    encodings = vision.get("encodings")
    if not isinstance(encodings, list) or not encodings or any(
        encoding not in {"rgb8", "bgr8"} for encoding in encodings
    ):
        raise ValueError("vision.encodings must contain only rgb8 and/or bgr8")
    target = np.asarray(vision["target_right_rim_px"], dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)) or not (0 <= target[0] < width and 0 <= target[1] < height):
        raise ValueError("vision.target_right_rim_px must lie inside the wrist image")
    jacobian = np.asarray(vision["shanghai_base_xy_to_image_uv"], dtype=float)
    if jacobian.shape != (2, 2) or not np.all(np.isfinite(jacobian)) or abs(float(np.linalg.det(jacobian))) < 1.0e-6:
        raise ValueError("vision.shanghai_base_xy_to_image_uv must be a finite invertible 2x2 matrix")
    if int(vision["maximum_iterations"]) <= 0:
        raise ValueError("vision.maximum_iterations must be positive")
    for field in ("maximum_stable_spread_px", "alignment_tolerance_px"):
        if not math.isfinite(float(vision[field])) or float(vision[field]) <= 0.0:
            raise ValueError(f"vision.{field} must be positive and finite")
    gripper = data["gripper"]
    for field in ("command_duration_s", "settle_timeout_s", "settled_spread",
                  "minimum_contact_delta_from_empty_closed"):
        if not math.isfinite(float(gripper[field])) or float(gripper[field]) <= 0.0:
            raise ValueError(f"gripper.{field} must be positive and finite")
    if not math.isfinite(float(gripper["open"])) or not math.isfinite(float(gripper["closed"])):
        raise ValueError("gripper open/closed values must be finite")
    if not 0.0 <= float(gripper["closed"]) < float(gripper["open"]) <= 1.0:
        raise ValueError("gripper values must satisfy 0 <= closed < open <= 1")
    spine_target = float(data["spine"]["initial_target_m"])
    if not math.isfinite(spine_target) or not 0.0 <= spine_target <= 0.8:
        raise ValueError("spine.initial_target_m must be within the official 0.0-0.8 m range")
    if data["arm_command_interface"].get("message_type") != "sensor_msgs/msg/JointState":
        raise ValueError("arm command interface must be sensor_msgs/msg/JointState")
    if set(data["objects"]) != set(OBJECTS):
        raise ValueError("objects must contain exactly cup, bowl, and plate")
    for object_name in OBJECTS:
        descent = float(data["objects"][object_name]["descent_m"])
        destination = data["objects"][object_name]["destination"]
        if not math.isfinite(descent) or not 0.05 <= descent <= 0.40:
            raise ValueError(f"{object_name} descent is outside the 0.05-0.40 m trial bound")
        if not isinstance(destination, str) or len(destination) != 1 or not destination.isupper():
            raise ValueError(f"{object_name} destination must be one uppercase letter")
    return data


def validate_cycle_venue_consistency(config: dict[str, Any], venue: dict[str, Any]) -> None:
    """Reject duplicated interface values that drifted apart before motion."""
    streams = {item["name"]: item["topic"] for item in venue["streams"]}
    commands = {item["name"]: item["topic"] for item in venue["command_endpoints"]}
    expected = {
        "left_joint_state": streams["left_arm_joint_state"],
        "right_joint_state": streams["right_arm_joint_state"],
        "left_gripper_state": streams["left_gripper_joint_state"],
        "left_wrist_image": streams["left_wrist_camera"],
        "left_joint_target": commands["left_arm_joint_target"],
        "right_joint_target": commands["right_arm_joint_target"],
        "left_gripper_target": commands["left_gripper_target"],
    }
    mismatches = [
        f"topics.{name}={config['topics'].get(name)!r}, venue={topic!r}"
        for name, topic in expected.items() if config["topics"].get(name) != topic
    ]
    profile_gripper = venue["interface_profile"]["gripper"]
    if profile_gripper["message_type"] != "std_msgs/msg/Float32" or profile_gripper["field"] != "data":
        mismatches.append(
            "native grasp execution currently requires std_msgs/msg/Float32 field data for the gripper"
        )
    if venue["interface_profile"]["spine"]["interface"] != "action":
        mismatches.append("native Hamburg execution requires the organizer-confirmed spine action interface")
    expected_types = {
        "left_arm_joint_state": "sensor_msgs/msg/JointState",
        "right_arm_joint_state": "sensor_msgs/msg/JointState",
        "left_gripper_joint_state": "sensor_msgs/msg/JointState",
        "left_wrist_camera": "sensor_msgs/msg/Image",
    }
    stream_entries = {item["name"]: item for item in venue["streams"]}
    command_entries = {item["name"]: item for item in venue["command_endpoints"]}
    for name, expected_type in expected_types.items():
        if expected_type not in stream_entries[name]["accepted_types"]:
            mismatches.append(f"{name} must provide {expected_type}")
    for name in ("left_arm_joint_target", "right_arm_joint_target"):
        if "sensor_msgs/msg/JointState" not in command_entries[name]["accepted_types"]:
            mismatches.append(f"{name} must accept sensor_msgs/msg/JointState")
    for field in ("open", "closed"):
        if not math.isclose(float(config["gripper"][field]), float(profile_gripper[field]), abs_tol=1.0e-9):
            mismatches.append(
                f"gripper.{field}={config['gripper'][field]!r}, interface_profile={profile_gripper[field]!r}"
            )
    if mismatches:
        raise ValueError("grasp/venue interface mismatch: " + "; ".join(mismatches))


def resolve_cycle_venue_overrides(config: dict[str, Any], venue: dict[str, Any]) -> dict[str, Any]:
    """Apply only explicitly requested venue-profile overrides to execution."""
    resolved = deepcopy(config)
    profile = venue["interface_profile"]
    applied = profile.get("applied_environment_overrides", {})
    changes: dict[str, Any] = {}
    mappings = {
        "TMR_HAMBURG_LEFT_GRIPPER_TOPIC": (
            ("topics", "left_gripper_target"), profile["gripper"]["left_topic"]
        ),
        "TMR_HAMBURG_GRIPPER_OPEN": (("gripper", "open"), profile["gripper"]["open"]),
        "TMR_HAMBURG_GRIPPER_CLOSED": (("gripper", "closed"), profile["gripper"]["closed"]),
        "TMR_HAMBURG_SPINE_HOME_M": (("spine", "initial_target_m"), profile["spine"]["home_m"]),
        "TMR_HAMBURG_ARM_POSTURE_VELOCITY_RAD_S": (
            ("motion", "posture_joint_velocity_rad_s"),
            profile["arm_motion"]["posture_joint_velocity_rad_s"],
        ),
        "TMR_HAMBURG_ARM_MAXIMUM_FOLLOWING_ERROR_RAD": (
            ("motion", "maximum_following_error_rad"),
            profile["arm_motion"]["maximum_following_error_rad"],
        ),
        "TMR_HAMBURG_ARM_FOLLOWING_ERROR_PAUSE_RAD": (
            ("motion", "following_error_pause_rad"),
            profile["arm_motion"]["following_error_pause_rad"],
        ),
        "TMR_HAMBURG_ARM_FOLLOWING_ERROR_RESUME_RAD": (
            ("motion", "following_error_resume_rad"),
            profile["arm_motion"]["following_error_resume_rad"],
        ),
        "TMR_HAMBURG_ARM_FOLLOWING_ERROR_RECOVERY_TIMEOUT_S": (
            ("motion", "following_error_recovery_timeout_s"),
            profile["arm_motion"]["following_error_recovery_timeout_s"],
        ),
    }
    for environment_name, (path, value) in mappings.items():
        if environment_name not in applied:
            continue
        resolved[path[0]][path[1]] = value
        changes[".".join(path)] = value
    if changes:
        resolved["applied_venue_overrides"] = changes
    validate_motion_config(resolved["motion"])
    validate_cycle_venue_consistency(resolved, venue)
    return resolved


def alignment_delta_m(point_px: tuple[float, float], config: dict[str, Any]) -> tuple[float, float]:
    """Map a wrist-image error to base-frame XY using the Shanghai calibration."""
    vision = config["vision"]
    matrix = np.asarray(vision["shanghai_base_xy_to_image_uv"], dtype=float)
    if matrix.shape != (2, 2) or abs(float(np.linalg.det(matrix))) < 1.0e-6:
        raise ValueError("visual Jacobian is singular")
    target = np.asarray(vision["target_right_rim_px"], dtype=float)
    delta = np.linalg.solve(matrix, target - np.asarray(point_px, dtype=float))
    maximum = float(config["motion"]["maximum_visual_step_m"])
    norm = float(np.linalg.norm(delta))
    if norm > maximum:
        delta *= maximum / norm
    return float(delta[0]), float(delta[1])


def plan_report(config: dict[str, Any], object_name: str, descent_override: float | None) -> dict[str, Any]:
    descent = float(descent_override if descent_override is not None else config["objects"][object_name]["descent_m"])
    return {
        "schema_version": 1,
        "operation": "Hamburg autonomous observation-grasp-release trial",
        "object": object_name,
        "command_interface": config["arm_command_interface"],
        "phases": [
            "native spine MoveAbsolute to pickup height",
            "right arm to parking and left arm to Shanghai pickup-top reference",
            "empty-close gripper feedback baseline, then reopen",
            "fresh wrist observations and iterative visual XY correction",
            f"Cartesian descent {descent:.3f} m using official FR3v2 kinematics",
            "close, lift, and compare gripper feedback with empty-close baseline",
            "return to release height, open, lift, and finish at pickup top",
        ],
        "single_ros_node_during_motion": True,
        "motion_commanded": False,
        "parameter_scope": config["parameter_scope"],
        "selected_descent_m": descent,
        "arm_motion": {
            key: config["motion"][key] for key in (
                "posture_joint_velocity_rad_s", "maximum_joint_velocity_rad_s",
                "following_error_resume_rad", "following_error_pause_rad",
                "maximum_following_error_rad", "following_error_recovery_timeout_s",
            )
        },
    }


class NativeArmGraspCycle:
    def __init__(self, node: Any, venue: dict[str, Any], config: dict[str, Any], output_dir: Path) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float32

        self.node = node
        self.venue = venue
        self.config = config
        self.output_dir = output_dir
        self.JointState = JointState
        self.Float32 = Float32
        self.states: dict[str, list[float] | None] = {"left": None, "right": None}
        self.state_times = {"left": 0.0, "right": 0.0}
        self.gripper_samples: deque[tuple[float, list[float]]] = deque(maxlen=300)
        self.images: deque[tuple[int, Any]] = deque(maxlen=20)
        self.image_at = 0.0
        self.commanded: dict[str, list[float] | None] = {"left": None, "right": None}
        self.following_lag_events: deque[dict[str, Any]] = deque(maxlen=30)
        self.last_report: dict[str, Any] = {}
        topics = config["topics"]

        def arm_callback(arm: str):
            expected = config["joint_names"][arm]

            def callback(message: Any) -> None:
                values = dict(zip(message.name, message.position))
                if all(name in values for name in expected):
                    sample = [float(values[name]) for name in expected]
                elif len(message.position) == 7 and not message.name:
                    sample = [float(value) for value in message.position]
                else:
                    return
                if all(math.isfinite(value) for value in sample):
                    self.states[arm] = sample
                    self.state_times[arm] = time.monotonic()

            return callback

        def gripper_callback(message: Any) -> None:
            sample = [float(value) for value in message.position]
            if sample and all(math.isfinite(value) for value in sample):
                self.gripper_samples.append((time.monotonic(), sample))

        def image_callback(message: Any) -> None:
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if stamp <= 0:
                stamp = time.monotonic_ns()
            if not self.images or stamp > self.images[-1][0]:
                self.images.append((stamp, message))
                self.image_at = time.monotonic()

        node.create_subscription(JointState, topics["left_joint_state"], arm_callback("left"), qos_profile_sensor_data)
        node.create_subscription(JointState, topics["right_joint_state"], arm_callback("right"), qos_profile_sensor_data)
        node.create_subscription(JointState, topics["left_gripper_state"], gripper_callback, qos_profile_sensor_data)
        node.create_subscription(Image, topics["left_wrist_image"], image_callback, qos_profile_sensor_data)
        self.arm_publishers = {
            "left": node.create_publisher(JointState, topics["left_joint_target"], 10),
            "right": node.create_publisher(JointState, topics["right_joint_target"], 10),
        }
        self.gripper_publisher = node.create_publisher(Float32, topics["left_gripper_target"], 10)

    def spin_for(self, seconds: float, *, publish_hold: bool = True) -> None:
        import rclpy

        deadline = time.monotonic() + seconds
        period = 1.0 / float(self.config["motion"]["publish_rate_hz"])
        while time.monotonic() < deadline:
            started = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if publish_hold and all(self.commanded.values()):
                self.publish_arm_holds()
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)

    def wait_live(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05, publish_hold=False)
            if all(self.states.values()) and self.gripper_samples and self.images:
                break
        missing = []
        if self.states["left"] is None:
            missing.append("left measured joints")
        if self.states["right"] is None:
            missing.append("right measured joints")
        if not self.gripper_samples:
            missing.append("left gripper feedback")
        if not self.images:
            missing.append("left wrist image")
        if missing:
            raise RuntimeError("missing live inputs: " + ", ".join(missing))
        try:
            decode_wrist_rgb(self.images[-1][1])
        except Exception as exc:
            raise RuntimeError(
                f"left wrist image is unusable before motion: {type(exc).__name__}: {exc}"
            ) from exc
        self.commanded = {arm: list(self.states[arm] or []) for arm in ("left", "right")}

    def diagnostic_snapshot(self) -> dict[str, Any]:
        """Return bounded live-state context suitable for a failure report."""
        now = time.monotonic()
        return {
            "joint_feedback_age_s": {
                arm: (round(now - stamp, 3) if stamp > 0.0 else None)
                for arm, stamp in self.state_times.items()
            },
            "measured_joints": self.states,
            "commanded_joints": self.commanded,
            "gripper_sample_count": len(self.gripper_samples),
            "latest_gripper_feedback": (
                self.gripper_samples[-1][1] if self.gripper_samples else None
            ),
            "latest_gripper_feedback_age_s": (
                round(now - self.gripper_samples[-1][0], 3)
                if self.gripper_samples else None
            ),
            "buffered_wrist_frames": len(self.images),
            "latest_wrist_frame_age_s": (
                round(now - self.image_at, 3) if self.image_at > 0.0 else None
            ),
            "following_lag_events": list(getattr(self, "following_lag_events", [])),
        }

    def fresh_wrist_bgr(self, timeout_s: float = 3.0) -> Any:
        """Wait for and decode a wrist frame received after this call starts."""
        last_stamp = self.images[-1][0] if self.images else 0
        decode_errors: deque[str] = deque(maxlen=3)
        fresh_frames = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            self._check_all_following()
            fresh = [(stamp, image) for stamp, image in self.images if stamp > last_stamp]
            for stamp, image in fresh:
                last_stamp = stamp
                fresh_frames += 1
                try:
                    return decode_wrist_rgb(image)
                except Exception as exc:
                    decode_errors.append(f"{type(exc).__name__}: {exc}")
        age = time.monotonic() - self.image_at if self.image_at > 0.0 else math.inf
        detail = "; ".join(decode_errors) if decode_errors else "no post-reset frame received"
        raise RuntimeError(
            "no fresh usable left wrist frame after reset "
            f"(fresh_frames={fresh_frames}, latest_age_s={age:.3f}, detail={detail})"
        )

    def set_phase(self, report: dict[str, Any], phase: str) -> None:
        report["active_phase"] = phase
        print(json.dumps({
            "event": "phase",
            "operation": report.get("entrypoint", "grasp-test"),
            "object": report.get("object"),
            "phase": phase,
        }), file=sys.stderr, flush=True)
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / f"{report.get('object', 'grasp')}-progress.json"
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            temporary.replace(path)
        except Exception as exc:
            warning = f"progress_persist: {type(exc).__name__}: {exc}"
            if warning not in report.setdefault("report_warnings", []):
                report["report_warnings"].append(warning)
            print(warning, file=sys.stderr, flush=True)

    def assert_controller_graph(self) -> dict[str, int]:
        topics = self.config["topics"]
        counts: dict[str, int] = {}
        for key in ("left_joint_target", "right_joint_target", "left_gripper_target"):
            count = len(self.node.get_subscriptions_info_by_topic(topics[key]))
            counts[key] = count
            if count < 1:
                raise RuntimeError(f"no controller subscribes to {topics[key]}")
            external = [
                info for info in self.node.get_publishers_info_by_topic(topics[key])
                if info.node_name != self.node.get_name()
            ]
            if external:
                owners = sorted({f"{info.node_namespace}/{info.node_name}" for info in external})
                raise RuntimeError(f"competing command publisher on {topics[key]}: {owners}")
        return counts

    def publish_arm_holds(self) -> None:
        for arm in ("left", "right"):
            values = self.commanded[arm]
            if values is None:
                continue
            message = self.JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = "tmr_hamburg_autonomous_joint_target"
            message.name = list(self.config["joint_names"][arm])
            message.position = list(values)
            self.arm_publishers[arm].publish(message)

    def _check_following(self, arm: str) -> float:
        measured = self.states[arm]
        commanded = self.commanded[arm]
        if measured is None or commanded is None:
            raise RuntimeError(f"{arm} joint feedback disappeared")
        age = time.monotonic() - self.state_times[arm]
        if age > 0.5:
            raise RuntimeError(f"{arm} joint feedback became stale (age={age:.3f}s)")
        joint_errors = [abs(a - b) for a, b in zip(measured, commanded)]
        joint_index = max(range(len(joint_errors)), key=joint_errors.__getitem__)
        error = joint_errors[joint_index]
        if error > float(self.config["motion"]["maximum_following_error_rad"]):
            limit = float(self.config["motion"]["maximum_following_error_rad"])
            joint_name = self.config["joint_names"][arm][joint_index]
            raise RuntimeError(
                f"{arm} following error {error:.4f} rad at {joint_name} exceeds hard "
                f"limit {limit:.4f} rad (commanded={commanded[joint_index]:.4f}, "
                f"measured={measured[joint_index]:.4f})"
            )
        return error

    def _check_all_following(self) -> None:
        errors = {arm: self._check_following(arm) for arm in ("left", "right")}
        pause = float(self.config["motion"]["following_error_pause_rad"])
        if max(errors.values()) <= pause:
            return

        resume = float(self.config["motion"]["following_error_resume_rad"])
        timeout = float(self.config["motion"]["following_error_recovery_timeout_s"])
        started = time.monotonic()
        event: dict[str, Any] = {
            "status": "holding_for_controller",
            "phase": getattr(self, "last_report", {}).get("active_phase", "unknown"),
            "trigger_errors_rad": {arm: round(value, 5) for arm, value in errors.items()},
            "trigger_measured_joints": {
                arm: list(self.states[arm] or []) for arm in ("left", "right")
            },
            "trigger_commanded_joints": {
                arm: list(self.commanded[arm] or []) for arm in ("left", "right")
            },
            "peak_errors_rad": {arm: round(value, 5) for arm, value in errors.items()},
            "pause_rad": pause,
            "resume_rad": resume,
            "hard_limit_rad": float(self.config["motion"]["maximum_following_error_rad"]),
        }
        self.following_lag_events.append(event)
        print(json.dumps({"event": "arm_following_lag", **event}), file=sys.stderr, flush=True)
        deadline = started + timeout
        while time.monotonic() < deadline:
            # Keep publishing the current target without advancing the ramp.
            self.spin_for(min(0.05, max(0.0, deadline - time.monotonic())))
            errors = {arm: self._check_following(arm) for arm in ("left", "right")}
            for arm, value in errors.items():
                event["peak_errors_rad"][arm] = round(
                    max(float(event["peak_errors_rad"][arm]), value), 5
                )
            if max(errors.values()) <= resume:
                event.update({
                    "status": "recovered",
                    "duration_s": round(time.monotonic() - started, 3),
                    "recovered_errors_rad": {
                        arm: round(value, 5) for arm, value in errors.items()
                    },
                })
                print(
                    json.dumps({"event": "arm_following_recovered", **event}),
                    file=sys.stderr, flush=True,
                )
                return
        event.update({
            "status": "recovery_timeout",
            "duration_s": round(time.monotonic() - started, 3),
            "final_errors_rad": {arm: round(value, 5) for arm, value in errors.items()},
        })
        print(json.dumps({"event": "arm_following_timeout", **event}), file=sys.stderr, flush=True)
        worst_arm = max(errors, key=errors.get)
        raise RuntimeError(
            f"{worst_arm} following lag did not recover below {resume:.4f} rad "
            f"within {timeout:.2f}s (final={errors[worst_arm]:.4f} rad)"
        )

    def move_arm_targets(self, arm: str, targets: list[list[float]]) -> None:
        rate = float(self.config["motion"]["publish_rate_hz"])
        velocity = float(self.config["motion"]["maximum_joint_velocity_rad_s"])
        for target in targets:
            lower, upper = FR3V2_JOINT_LIMITS_RAD[:, 0], FR3V2_JOINT_LIMITS_RAD[:, 1]
            q = np.asarray(target, dtype=float)
            if q.shape != (7,) or np.any(q <= lower) or np.any(q >= upper):
                raise RuntimeError(f"{arm} target violates official FR3v2 joint limits")
            start = list(self.commanded[arm] or self.states[arm] or [])
            for point in smooth_joint_targets(start, target, rate, velocity):
                self.commanded[arm] = point
                self.spin_for(1.0 / rate)
                self._check_all_following()
        tolerance = float(self.config["motion"]["endpoint_tolerance_rad"])
        timeout = float(self.config["motion"]["endpoint_timeout_s"])
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            self._check_all_following()
            measured = self.states[arm]
            if measured is not None and max(abs(a - b) for a, b in zip(measured, targets[-1])) <= tolerance:
                return
        measured = self.states[arm]
        endpoint_error = (
            max(abs(a - b) for a, b in zip(measured, targets[-1]))
            if measured is not None else math.inf
        )
        raise RuntimeError(
            f"{arm} failed to settle at commanded target within {timeout:.2f}s "
            f"(endpoint_error={endpoint_error:.4f} rad, tolerance={tolerance:.4f} rad)"
        )

    def move_to_joint_posture(self, arm: str, target: list[float]) -> None:
        start = list(self.commanded[arm] or self.states[arm] or [])
        targets = smooth_joint_targets(
            start, target,
            float(self.config["motion"]["publish_rate_hz"]),
            float(self.config["motion"]["posture_joint_velocity_rad_s"]),
        )
        # The interpolation above is already rate-limited; stream it directly.
        rate = float(self.config["motion"]["publish_rate_hz"])
        for point in targets:
            self.commanded[arm] = point
            self.spin_for(1.0 / rate)
            self._check_all_following()
        self.move_arm_targets(arm, [list(target)])

    def command_gripper(self, value: float) -> list[float]:
        message = self.Float32()
        message.data = float(value)
        duration = float(self.config["gripper"]["command_duration_s"])
        deadline = time.monotonic() + duration
        period = 1.0 / float(self.config["motion"]["publish_rate_hz"])
        while time.monotonic() < deadline:
            self.gripper_publisher.publish(message)
            self.spin_for(period)
            self._check_all_following()
        return self.settled_gripper_feedback(float(self.config["gripper"]["settle_timeout_s"]))

    def settled_gripper_feedback(self, timeout_s: float) -> list[float]:
        spread_limit = float(self.config["gripper"]["settled_spread"])
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            self._check_all_following()
            recent = [values for stamp, values in self.gripper_samples if time.monotonic() - stamp <= 0.5]
            if len(recent) >= 4 and len({len(item) for item in recent}) == 1:
                array = np.asarray(recent, dtype=float)
                if float(np.max(np.ptp(array, axis=0))) <= spread_limit:
                    return np.median(array, axis=0).tolist()
        now = time.monotonic()
        recent_count = sum(1 for stamp, _values in self.gripper_samples if now - stamp <= 0.5)
        latest_age = now - self.gripper_samples[-1][0] if self.gripper_samples else math.inf
        raise RuntimeError(
            "left gripper feedback did not settle "
            f"(recent_samples={recent_count}, latest_age_s={latest_age:.3f})"
        )

    def stable_detection(self, object_name: str, timeout_s: float = 8.0) -> tuple[tuple[float, float], Any]:
        points: deque[tuple[float, float] | None] = deque(maxlen=7)
        last_stamp = self.images[-1][0] if self.images else 0
        last_bgr = None
        fresh_frames = 0
        detection_errors: deque[str] = deque(maxlen=3)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            self._check_all_following()
            fresh = [(stamp, image) for stamp, image in self.images if stamp > last_stamp]
            for stamp, image in fresh:
                last_stamp = stamp
                fresh_frames += 1
                try:
                    last_bgr = decode_wrist_rgb(image)
                    points.append(detect_object(object_name, last_bgr))
                except Exception as exc:
                    detection_errors.append(f"{type(exc).__name__}: {exc}")
                    points.append(None)
                valid = [point for point in points if point is not None]
                if len(valid) >= 5 and len(points) >= 5 and all(point is not None for point in list(points)[-2:]):
                    array = np.asarray(valid, dtype=float)
                    median = np.median(array, axis=0)
                    spread = float(np.max(np.linalg.norm(array - median, axis=1)))
                    if spread <= float(self.config["vision"]["maximum_stable_spread_px"]):
                        return (float(median[0]), float(median[1])), last_bgr
            self.images = deque([(stamp, image) for stamp, image in self.images if stamp >= last_stamp], maxlen=20)
        valid = sum(point is not None for point in points)
        detail = "; ".join(detection_errors) if detection_errors else "detector returned no rim"
        raise RuntimeError(
            f"no stable fresh {object_name} rim detection "
            f"(fresh_frames={fresh_frames}, recent_valid={valid}/{len(points)}, detail={detail})"
        )

    def visual_align(self, object_name: str, mount_rotation: np.ndarray,
                     records: list[dict[str, Any]]) -> tuple[list[float], Any]:
        total = np.zeros(2, dtype=float)
        last_bgr = None
        target_px = np.asarray(self.config["vision"]["target_right_rim_px"], dtype=float)
        for iteration in range(int(self.config["vision"]["maximum_iterations"])):
            point, last_bgr = self.stable_detection(object_name)
            error = target_px - np.asarray(point)
            record = {"iteration": iteration + 1, "rim_point_px": list(point),
                      "target_px": target_px.tolist(), "error_px": error.tolist()}
            if float(np.linalg.norm(error)) <= float(self.config["vision"]["alignment_tolerance_px"]):
                record["status"] = "aligned"
                records.append(record)
                return list(self.commanded["left"] or []), last_bgr
            dx, dy = alignment_delta_m(point, self.config)
            total += (dx, dy)
            if float(np.linalg.norm(total)) > float(self.config["motion"]["maximum_visual_xy_travel_m"]):
                raise RuntimeError("visual correction exceeded the Shanghai-reference travel bound")
            seed = list(self.commanded["left"] or [])
            waypoints = cartesian_waypoints(
                seed, [dx, dy, 0.0], mount_rotation,
                float(self.config["motion"]["cartesian_waypoint_step_m"]),
            )
            record.update({"status": "corrected", "base_xy_delta_m": [dx, dy],
                           "accumulated_xy_m": total.tolist()})
            records.append(record)
            self.move_arm_targets("left", waypoints)
        raise RuntimeError("visual alignment did not converge within the configured iterations")

    def run(self, object_name: str, descent_m: float, spine: SpineControl) -> dict[str, Any]:
        started = time.monotonic()
        report: dict[str, Any] = {
            "schema_version": 1,
            "operation": "Hamburg autonomous observation-grasp-release trial",
            "object": object_name,
            "status": "running",
            "motion_commanded": False,
            "single_ros_node_during_motion": True,
            "parameter_profile": self.config["profile_name"],
            "parameter_scope": self.config["parameter_scope"],
            "applied_venue_overrides": self.config.get("applied_venue_overrides", {}),
            "arm_motion": {
                key: self.config["motion"][key] for key in (
                    "posture_joint_velocity_rad_s", "maximum_joint_velocity_rad_s",
                    "following_error_resume_rad", "following_error_pause_rad",
                    "maximum_following_error_rad", "following_error_recovery_timeout_s",
                )
            },
            "selected_descent_m": descent_m,
            "phases_completed": [],
            "visual_alignment": [],
        }
        self.last_report = report
        self.set_phase(report, "wait_for_live_interfaces")
        self.wait_live(10.0)
        report["controller_subscriber_counts"] = self.assert_controller_graph()
        self.set_phase(report, "spine_to_pickup_height")
        report["motion_commanded"] = True
        report["spine"] = spine.move_absolute(float(self.config["spine"]["initial_target_m"]))
        report["phases_completed"].append("spine_ready")
        self.set_phase(report, "arms_to_pickup_posture")
        self.move_to_joint_posture("right", self.config["posture"]["right_parking_rad"])
        self.move_to_joint_posture("left", self.config["posture"]["left_pick_top_rad"])
        report["phases_completed"].append("arms_at_shanghai_reference_pickup_view")

        self.set_phase(report, "empty_close_baseline")
        empty = self.command_gripper(float(self.config["gripper"]["closed"]))
        self.command_gripper(float(self.config["gripper"]["open"]))
        report["empty_closed_baseline"] = empty
        report["phases_completed"].append("empty_close_baseline_recorded")

        mount = derive_mount_rotation(
            self.config["posture"]["left_pick_top_rad"],
            self.config["posture"]["left_tool_quaternion_xyzw"],
        )
        self.set_phase(report, "wrist_observation_and_alignment")
        aligned_q, bgr = self.visual_align(object_name, mount, report["visual_alignment"])
        image_path = self.output_dir / f"{object_name}-aligned-wrist.png"
        final_point = tuple(report["visual_alignment"][-1]["rim_point_px"])
        try:
            save_observation_image(bgr, object_name, final_point, image_path)
            report["aligned_wrist_image"] = str(image_path)
        except Exception as image_exc:
            warning = f"aligned_image_write: {type(image_exc).__name__}: {image_exc}"
            report.setdefault("report_warnings", []).append(warning)
            print(warning, file=sys.stderr, flush=True)
        report["phases_completed"].append("observation_and_visual_alignment_complete")

        descent_path = cartesian_waypoints(
            aligned_q, [0.0, 0.0, -descent_m], mount,
            float(self.config["motion"]["cartesian_waypoint_step_m"]),
        )
        descent_started = False
        lifted = False
        object_closed: list[float] | None = None
        retained: list[float] | None = None
        try:
            self.set_phase(report, "descent")
            descent_started = True
            self.move_arm_targets("left", descent_path)
            report["phases_completed"].append("descent_complete")
            self.set_phase(report, "close_and_lift")
            object_closed = self.command_gripper(float(self.config["gripper"]["closed"]))
            self.move_arm_targets("left", list(reversed([aligned_q] + descent_path[:-1])))
            lifted = True
            retained = self.settled_gripper_feedback(float(self.config["gripper"]["settle_timeout_s"]))
            report["phases_completed"].append("closed_and_lifted")
            report["grasp_evidence"] = gripper_contact_report(
                empty, object_closed, retained,
                float(self.config["gripper"]["minimum_contact_delta_from_empty_closed"]),
            )
        finally:
            primary = sys.exc_info()[1]
            if primary is not None:
                report.setdefault("failed_phase", report.get("active_phase", "unknown"))
            object_may_be_held = (
                object_closed is not None or report.get("failed_phase") == "close_and_lift"
            )
            if isinstance(primary, KeyboardInterrupt):
                report["recovery"] = {
                    "status": "skipped_after_operator_interrupt",
                    "object_may_be_held": object_may_be_held,
                }
            else:
                try:
                    if descent_started and not lifted:
                        self.set_phase(report, "recover_to_pickup_top")
                        self.move_to_joint_posture("left", aligned_q)
                        if object_closed is not None:
                            self.set_phase(report, "recover_release_and_retract")
                            self.move_arm_targets("left", descent_path)
                            self.command_gripper(float(self.config["gripper"]["open"]))
                            self.move_arm_targets(
                                "left", list(reversed([aligned_q] + descent_path[:-1]))
                            )
                            report["recovery"] = {
                                "status": "released_and_returned_to_pickup_top",
                                "object_may_be_held": False,
                            }
                        else:
                            report["recovery"] = {
                                "status": "returned_to_pickup_top",
                                "object_may_be_held": object_may_be_held,
                            }
                    elif object_closed is not None:
                        self.set_phase(report, "return_release_and_retract")
                        self.move_arm_targets("left", descent_path)
                        self.command_gripper(float(self.config["gripper"]["open"]))
                        self.move_arm_targets("left", list(reversed([aligned_q] + descent_path[:-1])))
                        report["recovery"] = {
                            "status": "released_and_returned_to_pickup_top",
                            "object_may_be_held": False,
                        }
                except Exception as recovery_exc:
                    report.setdefault("recovery_errors", []).append(
                        f"{type(recovery_exc).__name__}: {recovery_exc}"
                    )
                    report["recovery"] = {
                        "status": "failed",
                        "object_may_be_held": object_may_be_held,
                    }
                    if primary is None:
                        raise
        report["phases_completed"].append("released_and_returned_to_pickup_top")
        report["status"] = "passed" if report["grasp_evidence"]["accepted_as_held"] else "failed_contact_check"
        self.set_phase(report, "complete")
        report["duration_s"] = round(time.monotonic() - started, 3)
        self.spin_for(1.0)
        return report


def write_report(report: dict[str, Any], path: Path | None) -> None:
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered, flush=True)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(rendered + "\n", encoding="utf-8")
        temporary.replace(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", choices=OBJECTS, required=True)
    parser.add_argument("--execute", action="store_true", help="run the physical autonomous trial")
    parser.add_argument("--config", type=Path, default=Path(os.environ.get(
        "TMR_HAMBURG_GRASP_CYCLE_CONFIG", str(DEFAULT_CYCLE_CONFIG))))
    parser.add_argument("--venue-config", type=Path, default=DEFAULT_VENUE_CONFIG)
    parser.add_argument("--descent-m", type=float,
                        help="object-specific measured override; never inferred by room scaling")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/tmr_hamburg_grasp"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = args.output or args.output_dir / f"{args.object}-grasp-report.json"
    try:
        config = load_cycle_config(args.config)
        descent = float(
            args.descent_m
            if args.descent_m is not None
            else config["objects"][args.object]["descent_m"]
        )
        if not math.isfinite(descent) or not 0.05 <= descent <= 0.40:
            raise ValueError("--descent-m must be between 0.05 and 0.40")
    except Exception as exc:
        write_report({
            "status": "invalid_configuration",
            "motion_commanded": False,
            "failed_phase": "load_and_validate_configuration",
            "errors": [f"{type(exc).__name__}: {exc}"],
            "traceback": traceback.format_exc(),
        }, report_path)
        return 2
    if not args.execute:
        write_report(plan_report(config, args.object, args.descent_m), args.output)
        return 0

    try:
        venue = load_config(args.venue_config)
        config = resolve_cycle_venue_overrides(config, venue)
        environment, errors = environment_report(venue)
    except Exception as exc:
        write_report({
            "status": "invalid_environment",
            "motion_commanded": False,
            "failed_phase": "load_and_validate_venue",
            "errors": [f"{type(exc).__name__}: {exc}"],
            "traceback": traceback.format_exc(),
        }, report_path)
        return 2
    if errors:
        report = {"status": "blocked", "motion_commanded": False,
                  "environment": environment, "errors": errors}
        write_report(report, report_path)
        return 2

    rclpy = None
    node = None
    rclpy_started = False
    spine: SpineControl | None = None
    runner: NativeArmGraspCycle | None = None
    report: dict[str, Any] = {
        "status": "starting", "motion_commanded": False, "object": args.object
    }
    try:
        import rclpy as rclpy_module
        from rclpy.node import Node

        rclpy = rclpy_module
        rclpy.init(args=None)
        rclpy_started = True
        node = Node("tmr_task3_hamburg_autonomous_grasp")
        runner = NativeArmGraspCycle(node, venue, config, args.output_dir)
        spine = SpineControl(node, venue["interface_profile"]["spine"])
        report = runner.run(args.object, descent, spine)
    except KeyboardInterrupt:
        report = dict(runner.last_report) if runner is not None else {}
        report.setdefault("failed_phase", report.get("active_phase", "startup"))
        report.update({"status": "interrupted",
                       "motion_commanded": bool(report.get("motion_commanded", False)),
                       "errors": ["operator interrupted the physical trial"]})
    except Exception as exc:
        report = dict(runner.last_report) if runner is not None else {}
        report.setdefault("failed_phase", report.get("active_phase", "startup"))
        report.update({"status": "failed",
                       "motion_commanded": bool(report.get("motion_commanded", False)),
                       "errors": [f"{type(exc).__name__}: {exc}"],
                       "traceback": traceback.format_exc()})
    finally:
        cleanup_errors = []
        for label, operation in (
            ("spine_close", spine.close if spine is not None else None),
            ("node_destroy", node.destroy_node if node is not None else None),
            ("rclpy_shutdown", rclpy.shutdown if rclpy is not None and rclpy_started else None),
        ):
            if operation is None:
                continue
            try:
                operation()
            except Exception as cleanup_exc:
                cleanup_errors.append(f"{label}: {type(cleanup_exc).__name__}: {cleanup_exc}")
        if cleanup_errors:
            report.setdefault("cleanup_errors", []).extend(cleanup_errors)
            if report.get("status") == "passed":
                report["status"] = "failed_cleanup"
        if runner is not None:
            report["final_diagnostics"] = runner.diagnostic_snapshot()
        if spine is not None:
            report["spine_diagnostics"] = spine.diagnostic_snapshot()
    write_report(report, report_path)
    return 0 if report.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
