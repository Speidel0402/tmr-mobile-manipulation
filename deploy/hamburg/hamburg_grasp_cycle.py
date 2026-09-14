#!/usr/bin/env python3
"""Run one autonomous Hamburg observation-to-grasp-to-release trial.

The deployed Franka controller consumes relative, direction-mapped JointState
inputs on topics whose names contain ``gello``.  No GELLO leader is used here:
this node converts autonomous robot-space targets at the publisher boundary,
after first streaming a neutral activation sample.
"""

from __future__ import annotations

import argparse
from collections import deque
from copy import deepcopy
import importlib
import json
import math
import os
from pathlib import Path
import sys
import threading
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
from hamburg_preflight import DEFAULT_INTERFACE_CONFIG, environment_report, load_config
from spine_control import SpineControl


HERE = Path(__file__).resolve().parent
DEFAULT_CYCLE_CONFIG = HERE / "config" / "grasp-cycle-shanghai-reference.json"
DEFAULT_VENUE_CONFIG = HERE / "config" / "venue.json"
OBJECTS = ("cup", "bowl", "plate")
RELATIVE_GELLO_MODE = "relative_direction_mapped_gello"
ABSOLUTE_JOINT_MODE = "absolute_robot_joint_positions"
DEFAULT_GELLO_DIRECTION = [-1.0, -1.0, 1.0, 1.0, 1.0, 1.0, -1.0]
INPUT_CALLBACK_SPIN_BUDGET = 16


def competing_command_publishers(node: Any, topic: str) -> list[Any]:
    """Exclude at most one local endpoint, preserving duplicate-name conflicts."""
    own_identity = (node.get_namespace(), node.get_name())
    own_seen = False
    competing = []
    for info in node.get_publishers_info_by_topic(topic):
        if (info.node_namespace, info.node_name) == own_identity and not own_seen:
            own_seen = True
        else:
            competing.append(info)
    return competing


def encode_arm_command(
    robot_target: list[float],
    robot_at_activation: list[float],
    input_at_activation: list[float],
    direction: list[float],
) -> list[float]:
    """Encode an absolute robot target for Hamburg's relative GELLO input."""
    arrays = [
        np.asarray(values, dtype=float)
        for values in (robot_target, robot_at_activation, input_at_activation, direction)
    ]
    if any(values.shape != (7,) or not np.all(np.isfinite(values)) for values in arrays):
        raise ValueError("arm command target, references, and direction must be seven finite values")
    target, robot_zero, input_zero, signs = arrays
    if np.any(np.abs(signs) != 1.0):
        raise ValueError("arm command direction values must be -1 or 1")
    return (input_zero + signs * (target - robot_zero)).tolist()


def validate_arm_command_interface(interface: dict[str, Any]) -> None:
    if interface.get("message_type") != "sensor_msgs/msg/JointState":
        raise ValueError("arm command interface must be sensor_msgs/msg/JointState")
    mode = str(interface["control_mode"])
    if mode not in {RELATIVE_GELLO_MODE, ABSOLUTE_JOINT_MODE}:
        raise ValueError(f"unsupported arm command control_mode {mode!r}")
    if mode == RELATIVE_GELLO_MODE and interface.get(
        "requires_pre_activation_neutral_sample"
    ) is not True:
        raise ValueError(
            "relative GELLO mode requires a pre-activation neutral sample"
        )
    direction = interface["direction"]
    if (
        not isinstance(direction, list) or len(direction) != 7
        or any(float(value) not in {-1.0, 1.0} for value in direction)
    ):
        raise ValueError("arm command direction must contain seven values equal to -1 or 1")
    duration = float(interface["activation_sync_duration_s"])
    drift = float(interface["activation_sync_maximum_drift_rad"])
    if not math.isfinite(duration) or not 0.0 < duration <= 10.0:
        raise ValueError("arm activation sync duration must be in (0, 10] seconds")
    if not math.isfinite(drift) or not 0.0 < drift <= 0.20:
        raise ValueError("arm activation sync maximum drift must be in (0, 0.20] rad")


def validate_motion_config(motion: dict[str, Any]) -> None:
    positive_motion = (
        "publish_rate_hz", "maximum_joint_velocity_rad_s",
        "posture_joint_velocity_rad_s", "maximum_following_error_rad",
        "following_error_pause_rad", "following_error_resume_rad",
        "following_error_recovery_timeout_s", "joint_feedback_stale_timeout_s",
        "endpoint_tolerance_rad",
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
    if float(motion["joint_feedback_stale_timeout_s"]) > 5.0:
        raise ValueError("motion.joint_feedback_stale_timeout_s must not exceed 5 s")


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
    # Config copies made from previous Hamburg commits remain runnable.  The
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
    motion.setdefault("joint_feedback_stale_timeout_s", 1.0)
    validate_motion_config(motion)
    arm_command = data["arm_command_interface"]
    if arm_command.get("control_mode") == (
        "continuous joint target consumed by the deployed Franka joint-impedance controller"
    ):
        arm_command["control_mode"] = RELATIVE_GELLO_MODE
        arm_command["description"] = (
            "Hamburg relative, direction-mapped GELLO-teleop input driven by "
            "autonomous robot-space targets; no GELLO leader is required"
        )
    arm_command.setdefault("control_mode", RELATIVE_GELLO_MODE)
    arm_command.setdefault("direction", list(DEFAULT_GELLO_DIRECTION))
    arm_command.setdefault("requires_pre_activation_neutral_sample", True)
    arm_command.setdefault("activation_sync_duration_s", 1.0)
    arm_command.setdefault("activation_sync_maximum_drift_rad", 0.05)
    arm_command.setdefault(
        "stale_input_behavior",
        "controller zeroes torque and stops when the GELLO stream becomes stale",
    )
    validate_arm_command_interface(arm_command)
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
    profile_arm = venue["interface_profile"]["arm_command"]
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
    configured_arm = config["arm_command_interface"]
    if configured_arm["control_mode"] != profile_arm["semantics"]:
        mismatches.append(
            f"arm command semantics={configured_arm['control_mode']!r}, "
            f"interface_profile={profile_arm['semantics']!r}"
        )
    if [float(value) for value in configured_arm["direction"]] != [
        float(value) for value in profile_arm["direction"]
    ]:
        mismatches.append("arm command direction differs from the Hamburg interface profile")
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
        "TMR_HAMBURG_ARM_JOINT_FEEDBACK_STALE_TIMEOUT_S": (
            ("motion", "joint_feedback_stale_timeout_s"),
            profile["arm_motion"]["joint_feedback_stale_timeout_s"],
        ),
        "TMR_HAMBURG_ARM_COMMAND_SEMANTICS": (
            ("arm_command_interface", "control_mode"),
            profile["arm_command"]["semantics"],
        ),
        "TMR_HAMBURG_ARM_ACTIVATION_SYNC_DURATION_S": (
            ("arm_command_interface", "activation_sync_duration_s"),
            profile["arm_command"]["activation_sync_duration_s"],
        ),
        "TMR_HAMBURG_ARM_ACTIVATION_SYNC_MAXIMUM_DRIFT_RAD": (
            ("arm_command_interface", "activation_sync_maximum_drift_rad"),
            profile["arm_command"]["activation_sync_maximum_drift_rad"],
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
    validate_arm_command_interface(resolved["arm_command_interface"])
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
        "status": "plan",
        "operation": "Hamburg autonomous observation-grasp-release trial",
        "object": object_name,
        "command_interface": config["arm_command_interface"],
        "phases": [
            "publish measured joints as neutral GELLO inputs before controller activation",
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
        "applied_venue_overrides": config.get("applied_venue_overrides", {}),
        "selected_descent_m": descent,
        "arm_motion": {
            key: config["motion"][key] for key in (
                "posture_joint_velocity_rad_s", "maximum_joint_velocity_rad_s",
                "following_error_resume_rad", "following_error_pause_rad",
                "maximum_following_error_rad", "following_error_recovery_timeout_s",
                "joint_feedback_stale_timeout_s",
            )
        },
    }


class NativeArmGraspCycle:
    def __init__(self, node: Any, venue: dict[str, Any], config: dict[str, Any], output_dir: Path) -> None:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float32

        # Load vision's native extension before any controller neutral stream
        # starts; a cold import must not consume its freshness window.
        import_started = time.monotonic()
        importlib.import_module("cv2")
        self.opencv_import_duration_s = time.monotonic() - import_started
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
        self.robot_activation_reference: dict[str, list[float] | None] = {
            "left": None, "right": None
        }
        self.input_activation_reference: dict[str, list[float] | None] = {
            "left": None, "right": None
        }
        self.last_published_arm_input: dict[str, list[float] | None] = {
            "left": None, "right": None
        }
        self.last_arm_publish_at = 0.0
        self.arm_publish_count = 0
        self.arm_publications: dict[str, dict[str, Any] | None] = {"left": None, "right": None}
        self.input_service_last_at = 0.0
        self.input_service_maximum_gap_s = 0.0
        self.startup_wrist_decode_duration_s = 0.0
        self.arm_command_sync: dict[str, Any] | None = None
        self.arm_publish_lock = threading.RLock()
        self.arm_keepalive_stop = threading.Event()
        self.arm_keepalive_error: str | None = None
        self.arm_keepalive_thread = threading.Thread(
            target=self._arm_keepalive_loop,
            name="tmr_hamburg_arm_keepalive",
            daemon=True,
        )
        self.neutral_stream_started_at = 0.0
        self.neutral_initial_pose: dict[str, list[float]] | None = None
        self.neutral_peak_drift = {"left": 0.0, "right": 0.0}
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
        self.arm_keepalive_thread.start()

    def _arm_keepalive_loop(self) -> None:
        period = 1.0 / float(self.config["motion"]["publish_rate_hz"])
        while not self.arm_keepalive_stop.is_set():
            last_publish = self.last_arm_publish_at
            delay = period
            if last_publish > 0.0:
                delay = max(0.001, period - (time.monotonic() - last_publish))
            if self.arm_keepalive_stop.wait(delay):
                return
            # This thread owns normal arm publication, so the input executor
            # never waits on its Python publication lock.  Native asynchronous
            # DDS configuration reduces transport blocking; publication gaps
            # are still monitored because the Python thread alone cannot
            # isolate a native call that retains the GIL.
            last_publish = self.last_arm_publish_at
            if (
                last_publish > 0.0
                and time.monotonic() - last_publish < period * 0.9
            ):
                continue
            try:
                self.publish_arm_holds()
            except Exception as exc:
                self.arm_keepalive_error = f"{type(exc).__name__}: {exc}"
                print(json.dumps({
                    "event": "arm_keepalive_failed",
                    "error": self.arm_keepalive_error,
                }), file=sys.stderr, flush=True)
                return

    def stop_arm_keepalive(self) -> None:
        self.arm_keepalive_stop.set()
        self.arm_keepalive_thread.join(timeout=1.0)
        if self.arm_keepalive_thread.is_alive():
            raise RuntimeError("arm keepalive thread did not stop")
        self._raise_arm_keepalive_error()

    def _raise_arm_keepalive_error(self) -> None:
        if self.arm_keepalive_error is not None:
            raise RuntimeError(f"arm keepalive failed: {self.arm_keepalive_error}")

    def service_input_callbacks(self) -> None:
        """Drain a bounded batch without publishing or changing motion timing."""
        import rclpy

        now = time.monotonic()
        previous = getattr(self, "input_service_last_at", 0.0)
        if previous > 0.0:
            self.input_service_maximum_gap_s = max(
                getattr(self, "input_service_maximum_gap_s", 0.0), now - previous
            )
        self.input_service_last_at = now
        for _ in range(INPUT_CALLBACK_SPIN_BUDGET):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        self._raise_arm_keepalive_error()
        if getattr(self, "arm_command_sync", None) is not None:
            self._check_arm_publication_health()

    def spin_for(self, seconds: float) -> None:
        self._raise_arm_keepalive_error()
        deadline = time.monotonic() + seconds
        period = 1.0 / float(self.config["motion"]["publish_rate_hz"])
        while time.monotonic() < deadline:
            started = time.monotonic()
            # Drain several ready callbacks per tick.  The Hamburg sources run
            # near 1 kHz, while this control loop is intentionally 50 Hz and
            # has at least four live subscriptions (more in the full mission).
            # A bounded drain keeps each stream current without changing the
            # motion update rate.
            self.service_input_callbacks()
            remaining = period - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)

    def wait_live(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            if all(self.states.values()):
                self._start_neutral_arm_stream()
                self._check_neutral_arm_stream()
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
        decode_started = time.monotonic()
        try:
            decode_wrist_rgb(self.images[-1][1])
        except Exception as exc:
            raise RuntimeError(
                f"left wrist image is unusable before motion: {type(exc).__name__}: {exc}"
            ) from exc
        finally:
            self.startup_wrist_decode_duration_s = time.monotonic() - decode_started
        # The first decode lazily imports OpenCV.  Refresh callbacks after that
        # potentially slow operation before judging stream freshness.
        self.service_input_callbacks()
        self._start_neutral_arm_stream()
        self._check_neutral_arm_stream()

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
            "arm_command_mode": self.config["arm_command_interface"]["control_mode"],
            "robot_activation_reference": self.robot_activation_reference,
            "input_activation_reference": self.input_activation_reference,
            "last_published_arm_input": self.last_published_arm_input,
            "last_arm_publish_age_s": (
                round(now - self.last_arm_publish_at, 3)
                if self.last_arm_publish_at > 0.0 else None
            ),
            "arm_publish_count": self.arm_publish_count,
            "arm_publications": getattr(self, "arm_publications", {}),
            "maximum_input_service_gap_s": round(
                getattr(self, "input_service_maximum_gap_s", 0.0), 4
            ),
            "startup_wrist_decode_duration_s": round(
                getattr(self, "startup_wrist_decode_duration_s", 0.0), 4
            ),
            "opencv_import_duration_s": round(getattr(self, "opencv_import_duration_s", 0.0), 4),
            "publication_environment": {
                name: os.environ.get(name) for name in (
                    "RMW_FASTRTPS_PUBLICATION_MODE", "RMW_FASTRTPS_USE_QOS_FROM_XML"
                )
            },
            "arm_keepalive_thread_alive": self.arm_keepalive_thread.is_alive(),
            "arm_keepalive_error": self.arm_keepalive_error,
            "input_callbacks_per_control_tick": INPUT_CALLBACK_SPIN_BUDGET,
            "arm_command_sync": self.arm_command_sync,
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
            fresh = [self.images[-1]] if self.images and self.images[-1][0] > last_stamp else []
            for stamp, image in fresh:
                last_stamp = stamp
                fresh_frames += 1
                try:
                    frame = decode_wrist_rgb(image)
                except Exception as exc:
                    decode_errors.append(f"{type(exc).__name__}: {exc}")
                    continue
                finally:
                    self.service_input_callbacks()
                self._check_all_following()
                return frame
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

    def assert_controller_graph(self, *, require_subscribers: bool = True) -> dict[str, int]:
        topics = self.config["topics"]
        counts: dict[str, int] = {}
        for key in ("left_joint_target", "right_joint_target", "left_gripper_target"):
            count = len(self.node.get_subscriptions_info_by_topic(topics[key]))
            counts[key] = count
            if require_subscribers and count < 1:
                raise RuntimeError(f"no controller subscribes to {topics[key]}")
            external = competing_command_publishers(self.node, topics[key])
            if external:
                owners = sorted({f"{info.node_namespace}/{info.node_name}" for info in external})
                raise RuntimeError(f"competing command publisher on {topics[key]}: {owners}")
        return counts

    def publish_arm_holds(self) -> None:
        with self.arm_publish_lock:
            if not hasattr(self, "arm_publications"):
                self.arm_publications = {"left": None, "right": None}
            published_any = False
            for arm in ("left", "right"):
                values = self.commanded[arm]
                if values is None:
                    continue
                values = list(values)
                mode = self.config["arm_command_interface"]["control_mode"]
                if mode == RELATIVE_GELLO_MODE:
                    robot_zero = self.robot_activation_reference[arm]
                    input_zero = self.input_activation_reference[arm]
                    if robot_zero is None or input_zero is None:
                        continue
                    published = encode_arm_command(
                        values,
                        robot_zero,
                        input_zero,
                        self.config["arm_command_interface"]["direction"],
                    )
                else:
                    published = list(values)
                message = self.JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.header.frame_id = "tmr_hamburg_autonomous_gello_input"
                message.name = list(self.config["joint_names"][arm])
                message.position = published
                publish_started = time.monotonic()
                self.arm_publishers[arm].publish(message)
                completed = time.monotonic()
                previous = self.arm_publications[arm]
                duration = completed - publish_started
                # Replace a complete snapshot so the motion thread never sees
                # an acknowledgement paired with a different robot target.
                self.arm_publications[arm] = {
                    "robot_target": values,
                    "completed_at": completed,
                    "first_completed_at": previous["first_completed_at"] if previous else completed,
                    "count": previous["count"] + 1 if previous else 1,
                    "duration_s": duration,
                    "maximum_duration_s": max(previous["maximum_duration_s"], duration) if previous else duration,
                    "maximum_gap_s": max(previous["maximum_gap_s"], completed - previous["completed_at"]) if previous else 0.0,
                }
                self.last_published_arm_input[arm] = list(published)
                published_any = True
            if published_any:
                self.last_arm_publish_at = time.monotonic()
                self.arm_publish_count += 1

    def _start_neutral_arm_stream(self) -> None:
        """Start the pre-activation neutral stream at the first complete arm sample."""
        if getattr(self, "neutral_stream_started_at", 0.0) > 0.0:
            return
        initial = {arm: list(self.states[arm] or []) for arm in ("left", "right")}
        if any(len(values) != 7 for values in initial.values()):
            raise RuntimeError("cannot start neutral arm stream without both measured poses")
        self.neutral_initial_pose = initial
        self.neutral_peak_drift = {"left": 0.0, "right": 0.0}
        self.commanded = {arm: list(values) for arm, values in initial.items()}
        self.input_activation_reference = {
            arm: list(values) for arm, values in initial.items()
        }
        self.robot_activation_reference = {
            arm: list(values) for arm, values in initial.items()
        }
        self.neutral_stream_started_at = time.monotonic()

    def _check_neutral_arm_stream(self) -> None:
        initial = getattr(self, "neutral_initial_pose", None)
        if initial is None:
            raise RuntimeError("neutral arm stream has not been started")
        now = time.monotonic()
        maximum_drift = float(
            self.config["arm_command_interface"]["activation_sync_maximum_drift_rad"]
        )
        stale_timeout = float(self.config["motion"]["joint_feedback_stale_timeout_s"])
        for arm in ("left", "right"):
            age = now - self.state_times[arm]
            if age > stale_timeout:
                raise RuntimeError(
                    f"{arm} joint feedback became stale during neutral activation sync "
                    f"(age={age:.3f}s, limit={stale_timeout:.3f}s)"
                )
            measured = self.states[arm]
            assert measured is not None
            drift = max(abs(a - b) for a, b in zip(measured, initial[arm]))
            self.neutral_peak_drift[arm] = max(self.neutral_peak_drift[arm], drift)
            if drift > maximum_drift:
                raise RuntimeError(
                    f"{arm} moved {drift:.4f} rad during neutral GELLO activation sync; "
                    "start this entrypoint with the joint-impedance controller inactive "
                    "so it captures the neutral samples before following"
                )

    def synchronize_arm_command_interface(self) -> dict[str, Any]:
        """Publish a neutral input before motion and establish activation references.

        Hamburg's deployed controller captures both the robot and GELLO samples
        when it activates.  Broadcasting the current measured pose first makes
        that input neutral.  A fixed input is then maintained while the spine
        and other setup operations run, so activation may safely happen at any
        point before the first arm trajectory.
        """
        interface = self.config["arm_command_interface"]
        self._start_neutral_arm_stream()
        duration = float(interface["activation_sync_duration_s"])
        timeout = float(self.config["motion"]["joint_feedback_stale_timeout_s"])
        deadline = time.monotonic() + timeout
        while not all(getattr(self, "arm_publications", {}).get(arm) for arm in ("left", "right")):
            self.spin_for(min(0.02, max(0.0, deadline - time.monotonic())))
            self._check_neutral_arm_stream()
            if time.monotonic() >= deadline and not all(
                self.arm_publications.get(arm) for arm in ("left", "right")
            ):
                raise RuntimeError("neutral arm command stream was not published for both arms")
        # Count the synchronization interval from actual publication on both
        # topics, not from assigning the first target in the main thread.
        started = max(self.arm_publications[arm]["first_completed_at"] for arm in ("left", "right"))
        while time.monotonic() - started < duration:
            remaining = duration - (time.monotonic() - started)
            self.spin_for(min(0.05, max(0.0, remaining)))
            self._check_neutral_arm_stream()
        self._check_neutral_arm_stream()
        self._check_arm_publication_health()
        report = {
            "status": "neutral_stream_established",
            "control_mode": interface["control_mode"],
            "direction": list(interface["direction"]),
            "duration_s": round(time.monotonic() - started, 3),
            "input_at_activation": self.input_activation_reference,
            "robot_at_activation": self.robot_activation_reference,
            "peak_drift_rad": {
                arm: round(value, 5) for arm, value in self.neutral_peak_drift.items()
            },
            "continuous_stream_required": True,
            "joint_feedback_stale_timeout_s": float(
                self.config["motion"]["joint_feedback_stale_timeout_s"]
            ),
        }
        self.arm_command_sync = report
        print(json.dumps({"event": "arm_command_sync", **report}), file=sys.stderr, flush=True)
        return report

    def _check_arm_publication_health(self, selected_arm: str | None = None) -> None:
        now = time.monotonic()
        timeout = float(self.config["motion"]["joint_feedback_stale_timeout_s"])
        for arm in ((selected_arm,) if selected_arm is not None else ("left", "right")):
            publication = self.arm_publications[arm]
            if publication is None or now - publication["completed_at"] > timeout:
                age = now - publication["completed_at"] if publication else math.inf
                raise RuntimeError(
                    f"{arm} arm command publication became stale "
                    f"(age={age:.3f}s, limit={timeout:.3f}s)"
                )

    def _wait_for_arm_publication(self, arm: str, target: list[float]) -> None:
        """Wait for local publish return, not remote receipt, for each ramp point."""
        timeout = float(self.config["motion"]["joint_feedback_stale_timeout_s"])
        deadline = time.monotonic() + timeout
        while True:
            self._raise_arm_keepalive_error()
            publication = self.arm_publications[arm]
            if publication is not None and publication["robot_target"] == target:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"{arm} arm target publication timed out after {timeout:.3f}s")
            self.service_input_callbacks()
            self._check_all_following()
            time.sleep(min(0.002, max(0.0, deadline - time.monotonic())))

    def _check_following(self, arm: str) -> float:
        if getattr(self, "arm_command_sync", None) is not None:
            self._check_arm_publication_health(arm)
        measured = self.states[arm]
        commanded = self.commanded[arm]
        if measured is None or commanded is None:
            raise RuntimeError(f"{arm} joint feedback disappeared")
        age = time.monotonic() - self.state_times[arm]
        stale_timeout = float(self.config["motion"]["joint_feedback_stale_timeout_s"])
        if age > stale_timeout:
            raise RuntimeError(
                f"{arm} joint feedback became stale "
                f"(age={age:.3f}s, limit={stale_timeout:.3f}s)"
            )
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
                self._wait_for_arm_publication(arm, point)
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
            self._wait_for_arm_publication(arm, point)
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
            # Process the newest frame only.  A detector can take longer than
            # the camera period; draining an old batch starves joint callbacks.
            fresh = [self.images[-1]] if self.images and self.images[-1][0] > last_stamp else []
            for stamp, image in fresh:
                last_stamp = stamp
                fresh_frames += 1
                try:
                    last_bgr = decode_wrist_rgb(image)
                    points.append(detect_object(object_name, last_bgr))
                except Exception as exc:
                    detection_errors.append(f"{type(exc).__name__}: {exc}")
                    points.append(None)
                self.service_input_callbacks()
                self._check_all_following()
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
            "arm_command_interface": self.config["arm_command_interface"],
            "applied_venue_overrides": self.config.get("applied_venue_overrides", {}),
            "arm_motion": {
                key: self.config["motion"][key] for key in (
                    "posture_joint_velocity_rad_s", "maximum_joint_velocity_rad_s",
                    "following_error_resume_rad", "following_error_pause_rad",
                    "maximum_following_error_rad", "following_error_recovery_timeout_s",
                    "joint_feedback_stale_timeout_s",
                )
            },
            "selected_descent_m": descent_m,
            "phases_completed": [],
            "visual_alignment": [],
        }
        self.last_report = report
        self.set_phase(report, "preactivation_command_owner_check")
        report["preactivation_controller_graph"] = self.assert_controller_graph(
            require_subscribers=False
        )
        self.set_phase(report, "wait_for_live_interfaces")
        self.wait_live(10.0)
        self.set_phase(report, "neutral_arm_command_activation_sync")
        report["arm_command_sync"] = self.synchronize_arm_command_interface()
        report["controller_subscriber_counts"] = self.assert_controller_graph()
        report["phases_completed"].append("neutral_arm_command_stream_ready")
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
    parser.add_argument("--interface-config", type=Path, default=DEFAULT_INTERFACE_CONFIG)
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
        venue = load_config(args.venue_config, args.interface_config)
        config = resolve_cycle_venue_overrides(config, venue)
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
        spine = SpineControl(
            node,
            venue["interface_profile"]["spine"],
            heartbeat=runner.service_input_callbacks,
        )
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
            (
                "arm_keepalive_stop",
                runner.stop_arm_keepalive if runner is not None else None,
            ),
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
        for label, snapshot in (
            ("final_diagnostics", runner.diagnostic_snapshot if runner is not None else None),
            ("spine_diagnostics", spine.diagnostic_snapshot if spine is not None else None),
        ):
            if snapshot is None:
                continue
            try:
                report[label] = snapshot()
            except Exception as diagnostic_exc:
                report.setdefault("diagnostic_errors", []).append(
                    f"{label}: {type(diagnostic_exc).__name__}: {diagnostic_exc}"
                )
    write_report(report, report_path)
    return 0 if report.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
