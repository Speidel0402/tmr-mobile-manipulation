#!/usr/bin/env python3
"""Read-only, object-specific Hamburg grasp observation check.

This checks the native graph and the existing RGB detectors. It never moves an
arm, gripper, spine, or base and cannot establish physical grasp success.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

from hamburg_preflight import environment_report, load_config, probe_native_spine


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
GRASP_SCRIPTS = REPO_ROOT / "grasp" / "scripts"
OBJECTS = {"cup": 0.340, "bowl": 0.360, "plate": 0.375}
POSTURE_CONFIG = HERE / "config" / "grasp-observation.json"
EXPECTED_JOINTS = {
    arm: [f"{arm}_fr3v2_joint{i}" for i in range(1, 8)]
    for arm in ("left", "right")
}


def decode_wrist_rgb(message: Any):
    """Decode a ROS raw Image without cv_bridge or a camera relay."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    if (width, height) != (640, 480) or message.encoding not in {"rgb8", "bgr8"}:
        raise ValueError("left wrist camera must provide 640x480 rgb8/bgr8")
    if step < width * 3 or len(message.data) < step * height:
        raise ValueError("left wrist image data is truncated")
    import cv2
    import numpy as np

    rows = np.frombuffer(message.data, dtype=np.uint8, count=step * height).reshape(height, step)
    rgb = rows[:, : width * 3].reshape(height, width, 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else rgb.copy()


def detect_object(object_name: str, bgr: Any) -> tuple[float, float]:
    if str(GRASP_SCRIPTS) not in sys.path:
        sys.path.insert(0, str(GRASP_SCRIPTS))
    if object_name == "cup":
        from cup_rim_detector import detect_green_cup_right

        point, _detail = detect_green_cup_right(bgr)
        return float(point[0]), float(point[1])
    if object_name == "bowl":
        from detect_food_bowl import detect_food_bowl

        best, _candidates = detect_food_bowl(bgr)
    elif object_name == "plate":
        from detect_plate import detect_plate

        best, _candidates = detect_plate(bgr)
    else:
        raise ValueError(f"unsupported object {object_name!r}")
    return float(best["right_rim_x"]), float(best["right_rim_y"])


def evaluate_observations(points: list[tuple[float, float] | None]) -> tuple[dict[str, Any], list[str]]:
    """Require fresh, repeated detections before calling perception stable."""
    errors: list[str] = []
    valid = [point for point in points if point is not None]
    if len(points) < 5 or len(valid) < 3 or any(point is None for point in points[-2:]):
        errors.append("object detector needs five fresh frames, three detections, and two valid final frames")
    if valid:
        xs = sorted(point[0] for point in valid)
        ys = sorted(point[1] for point in valid)
        center = (xs[len(xs) // 2], ys[len(ys) // 2])
        spread = max(math.hypot(x - center[0], y - center[1]) for x, y in valid)
    else:
        center, spread = None, None
    if spread is not None and spread > 8.0:
        errors.append(f"object rim point unstable: {spread:.2f}px > 8.00px")
    return {
        "frames": len(points), "valid_detections": len(valid),
        "median_rim_point_px": center, "maximum_spread_px": spread,
    }, errors


def posture_report(latest: dict[str, Any], posture: dict[str, Any]) -> tuple[dict[str, Any], list[str]]:
    """Compare observed joints with the static Shanghai pickup view."""
    errors: list[str] = []
    report: dict[str, Any] = {}
    tolerance = float(posture["joint_tolerance_rad"])
    for arm, key in (("left", "left_pick_top_rad"), ("right", "right_parking_rad")):
        sample = latest.get(f"{arm}_arm_joint_state")
        if sample is None:
            errors.append(f"{arm} measured joint state unavailable")
            continue
        measured = dict(zip(sample["names"], sample["position"]))
        names = EXPECTED_JOINTS[arm]
        if not all(name in measured for name in names):
            errors.append(f"{arm} joint names do not match the configured pickup posture")
            continue
        target = posture[key]
        if len(target) != 7:
            raise ValueError(f"invalid {arm} pickup target length")
        deviations = [abs(float(measured[name]) - float(want)) for name, want in zip(names, target)]
        maximum = max(deviations)
        report[arm] = {"maximum_joint_error_rad": maximum, "within_tolerance": maximum <= tolerance}
        if not math.isfinite(maximum) or maximum > tolerance:
            errors.append(f"{arm} is not at the configured Shanghai pickup/parking posture")
    return report, errors


def run_check(config: dict[str, Any], object_name: str, timeout_s: float) -> dict[str, Any]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState

    if config["interface_profile"]["spine"]["interface"] != "action":
        raise ValueError("Hamburg grasp check requires the native spine action profile")
    streams = {item["name"]: item for item in config["streams"]}
    commands = {item["name"]: item for item in config["command_endpoints"]}
    posture_config = json.loads(POSTURE_CONFIG.read_text(encoding="utf-8"))
    rclpy.init(args=None)
    node = Node("tmr_task3_hamburg_grasp_check")
    latest: dict[str, Any] = {}
    frames: deque[tuple[int, Any]] = deque(maxlen=5)
    started = time.monotonic()
    errors: list[str] = []

    def on_joint(name: str):
        def callback(message: Any) -> None:
            latest[name] = {"topic": streams[name]["topic"], "received_at": time.monotonic(),
                            "names": list(message.name), "position": list(message.position)}
        return callback

    def on_image(name: str):
        def callback(message: Any) -> None:
            latest[name] = {"topic": streams[name]["topic"], "received_at": time.monotonic(), "width": int(message.width),
                            "height": int(message.height), "encoding": str(message.encoding),
                            "frame_id": str(message.header.frame_id)}
            if name == "left_wrist_camera":
                stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                if stamp > 0 and (not frames or stamp > frames[-1][0]):
                    frames.append((stamp, message))
        return callback

    try:
        for name in ("left_arm_joint_state", "right_arm_joint_state",
                     "left_gripper_joint_state", "right_gripper_joint_state"):
            node.create_subscription(JointState, streams[name]["topic"],
                                     on_joint(name), qos_profile_sensor_data)
        for name in ("left_wrist_camera", "head_camera"):
            node.create_subscription(Image, streams[name]["topic"],
                                     on_image(name), qos_profile_sensor_data)
        deadline = started + timeout_s
        required = {"left_arm_joint_state", "right_arm_joint_state",
                    "left_gripper_joint_state", "right_gripper_joint_state",
                    "left_wrist_camera", "head_camera"}
        while time.monotonic() < deadline and (not required.issubset(latest) or len(frames) < 5):
            rclpy.spin_once(node, timeout_sec=0.1)

        for name in sorted(required - latest.keys()):
            errors.append(f"missing live grasp stream: {name}")
        posture, posture_errors = posture_report(latest, posture_config)
        errors.extend(posture_errors)
        now = time.monotonic()
        for name, sample in latest.items():
            if now - sample["received_at"] > 0.75:
                errors.append(f"stale grasp stream: {name}")
        if "left_wrist_camera" in latest:
            sample = latest["left_wrist_camera"]
            if (sample["width"], sample["height"]) != (640, 480) or sample["encoding"] not in {"rgb8", "bgr8"}:
                errors.append("left wrist image size or encoding mismatch")
            if not sample["frame_id"] or any(x in sample["frame_id"].lower() for x in ("right", "zed")):
                errors.append("left wrist image frame identity is ambiguous")
        if "head_camera" in latest:
            sample = latest["head_camera"]
            head = streams["head_camera"]
            if (sample["width"], sample["height"], sample["encoding"]) != (
                head["expected_width"], head["expected_height"], head["expected_encoding"]
            ):
                errors.append("head camera does not match the configured Hamburg profile")
        subscriber_counts = {
            name: len(node.get_subscriptions_info_by_topic(commands[name]["topic"]))
            for name in ("left_arm_joint_target", "right_arm_joint_target",
                         "left_gripper_target", "right_gripper_target")
        }
        graph = dict(node.get_topic_names_and_types())
        for name, count in subscriber_counts.items():
            if count < 1:
                errors.append(f"no controller subscriber for {name}")
            topic = commands[name]["topic"]
            discovered = graph.get(topic, [])
            if len(discovered) != 1 or discovered[0] not in commands[name]["accepted_types"]:
                errors.append(f"command topic type mismatch for {name}: {discovered}")
        spine, spine_errors = probe_native_spine(node, config, timeout_s)
        errors.extend(spine_errors)
        position_m = (spine.get("service") or {}).get("position_m")
        if position_m is None or abs(position_m - float(config["interface_profile"]["spine"]["home_m"])) > float(posture_config["spine_home_tolerance_m"]):
            errors.append("spine is not at the configured Shanghai pickup height")

        points: list[tuple[float, float] | None] = []
        detector_errors: list[str] = []
        detector_ms: list[float] = []
        for _stamp, message in frames:
            try:
                t0 = time.monotonic()
                point = detect_object(object_name, decode_wrist_rgb(message))
                detector_ms.append(round((time.monotonic() - t0) * 1000.0, 2))
                points.append(point)
            except Exception as exc:
                points.append(None)
                detector_errors.append(f"{type(exc).__name__}: {exc}")
        detection, detection_errors = evaluate_observations(points)
        errors.extend(detection_errors)
        return {
            "schema_version": 1, "venue": config["venue"], "object": object_name,
            "status": "observation_ready" if not errors else "blocked",
            "motion_commanded": False, "grasp_executed": False,
            "physical_grasp_runnable": False,
            "single_ros_participant": True, "duration_s": round(time.monotonic() - started, 3),
            "streams": latest, "posture": posture,
            "controller_subscriber_counts": subscriber_counts,
            "native_spine": spine, "detection": detection,
            "detector_errors": detector_errors, "detector_ms": detector_ms,
            "errors": errors,
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", choices=tuple(OBJECTS), required=True)
    parser.add_argument("--plan", action="store_true", help="print the read-only check plan without ROS")
    parser.add_argument("--timeout-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    config = load_config(HERE / "config" / "venue.json")
    if args.plan:
        report = {"object": args.object, "operation": "Hamburg native grasp observation",
                  "legacy_descent_m_for_review": OBJECTS[args.object],
                  "motion_commanded": False, "grasp_executed": False,
                  "physical_grasp_runnable": False,
                  "requires": ["single Humble companion", "native spine action/service",
                               "fresh 640x480 left wrist RGB", "stable object rim detection"]}
    else:
        environment, errors = environment_report(config)
        if errors:
            report = {"object": args.object, "status": "blocked", "motion_commanded": False,
                      "grasp_executed": False, "physical_grasp_runnable": False,
                      "environment": environment, "errors": errors}
        else:
            try:
                report = run_check(config, args.object, args.timeout_s)
            except Exception as exc:
                report = {"object": args.object, "status": "blocked", "motion_commanded": False,
                          "grasp_executed": False, "physical_grasp_runnable": False,
                          "environment": environment,
                          "errors": [f"Hamburg grasp observation failed: {type(exc).__name__}: {exc}"]}
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered, flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(rendered + "\n", encoding="utf-8")
        temporary.replace(args.output)
    return 0 if args.plan or report.get("status") == "observation_ready" else 2


if __name__ == "__main__":
    raise SystemExit(main())
