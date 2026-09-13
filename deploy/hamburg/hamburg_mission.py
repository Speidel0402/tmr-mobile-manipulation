#!/usr/bin/env python3
"""Run the Hamburg Task 3 mission in one ROS node using native interfaces.

The default route values are the individually named Shanghai measurements.
They are exposed for Hamburg site overrides and are never uniformly scaled.
Run the three standalone grasp trials before executing this mission.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from hamburg_grasp_cycle import NativeArmGraspCycle, load_cycle_config, write_report
from hamburg_motion_core import cartesian_waypoints, derive_mount_rotation, gripper_contact_report
from hamburg_preflight import environment_report, load_config
from spine_control import SpineControl


HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
BASE_SCRIPTS = REPO_ROOT / "base" / "scripts"
if str(BASE_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(BASE_SCRIPTS))
from letter_card_vision import LetterCardRecognizer, annotate  # noqa: E402


DEFAULT_MISSION_CONFIG = HERE / "config" / "mission-shanghai-reference.json"
DEFAULT_GRASP_CONFIG = HERE / "config" / "grasp-cycle-shanghai-reference.json"
DEFAULT_VENUE_CONFIG = HERE / "config" / "venue.json"
OBJECTS = ("cup", "bowl", "plate")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_of(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def decode_raw_bgr(message: Any, width: int, height: int, encodings: list[str]):
    import cv2

    if (int(message.width), int(message.height)) != (width, height):
        raise ValueError(f"head image must be {width}x{height}")
    if message.encoding not in encodings:
        raise ValueError(f"head encoding {message.encoding!r} is not accepted")
    step = int(message.step)
    if step < width * 3 or len(message.data) < step * height:
        raise ValueError("head image data is truncated")
    rows = np.frombuffer(message.data, dtype=np.uint8, count=step * height).reshape(height, step)
    image = rows[:, : width * 3].reshape(height, width, 3)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else image.copy()


def validate_mission_config(config: dict[str, Any]) -> None:
    required = {
        "topics", "head_camera", "hamburg_measurements", "base_motion",
        "outbound_shanghai_reference", "post_pick_shanghai_reference",
        "letter_search_shanghai_reference", "placement_shanghai_reference",
        "return_to_pickup_shanghai_reference", "assignment",
    }
    missing = sorted(required - config.keys())
    if missing:
        raise ValueError("mission config is missing: " + ", ".join(missing))
    if set(config["assignment"]) != set(OBJECTS):
        raise ValueError("assignment must contain cup, bowl, and plate")
    if len(set(config["assignment"].values())) != 3:
        raise ValueError("object destination letters must be distinct")
    for group in ("outbound_shanghai_reference", "post_pick_shanghai_reference"):
        for stage in config[group]:
            if stage["kind"] not in {"translate", "rotate", "front_clearance"}:
                raise ValueError(f"unsupported route stage {stage['kind']!r}")


def plan_report(mission: dict[str, Any], grasp: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "plan",
        "motion_commanded": False,
        "operation": "Hamburg single-node three-utensil mission",
        "assignment": mission["assignment"],
        "single_ros_node_during_motion": True,
        "native_interfaces": {
            "base_state": mission["topics"]["odometry"],
            "base_command": mission["topics"]["base_command"],
            "head_image": mission["topics"]["head_image"],
            "spine": "MoveAbsolute action plus GetPosition service",
            "arm": grasp["arm_command_interface"],
        },
        "sequence": [
            "Shanghai-reference outbound stages, each independently overrideable",
            "cup observation-pick-lift -> B search-place -> measured return",
            "bowl observation-pick-lift -> A search-place -> measured return",
            "plate observation-pick-lift -> D search-place -> final stop",
        ],
        "route_profile_warning": mission["profile_warning"],
        "hamburg_measurements": mission["hamburg_measurements"],
        "grasp_parameter_scope": grasp["parameter_scope"],
        "outbound": mission["outbound_shanghai_reference"],
        "post_pick": mission["post_pick_shanghai_reference"],
        "letter_search": mission["letter_search_shanghai_reference"],
        "placement": mission["placement_shanghai_reference"],
        "return_to_pickup": mission["return_to_pickup_shanghai_reference"],
    }


class NativeBaseControl:
    def __init__(self, node: Any, arm: NativeArmGraspCycle, config: dict[str, Any]) -> None:
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, LaserScan

        self.node = node
        self.arm = arm
        self.config = config
        self.TwistStamped = TwistStamped
        self.pose: tuple[float, float, float] | None = None
        self.pose_at = 0.0
        self.command = [0.0, 0.0, 0.0]
        self.scans: dict[str, tuple[float, Any]] = {}
        self.head_frames: deque[tuple[int, Any]] = deque(maxlen=12)
        self.head_at = 0.0
        self.active_stage: str | None = None
        topics = config["topics"]

        def on_odom(message: Any) -> None:
            position = message.pose.pose.position
            self.pose = (float(position.x), float(position.y), yaw_of(message.pose.pose.orientation))
            self.pose_at = time.monotonic()

        def on_scan(name: str):
            def callback(message: Any) -> None:
                self.scans[name] = (time.monotonic(), message)
            return callback

        def on_head(message: Any) -> None:
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            if stamp <= 0:
                stamp = time.monotonic_ns()
            if not self.head_frames or stamp > self.head_frames[-1][0]:
                self.head_frames.append((stamp, message))
                self.head_at = time.monotonic()

        self.publisher = node.create_publisher(TwistStamped, topics["base_command"], 10)
        node.create_subscription(Odometry, topics["odometry"], on_odom, qos_profile_sensor_data)
        node.create_subscription(LaserScan, topics["front_lidar"], on_scan("front"), qos_profile_sensor_data)
        node.create_subscription(LaserScan, topics["rear_lidar"], on_scan("rear"), qos_profile_sensor_data)
        node.create_subscription(Image, topics["head_image"], on_head, qos_profile_sensor_data)

    def publish(self, vx: float, vy: float, wz: float) -> None:
        message = self.TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.publisher.publish(message)

    def stop(self, seconds: float = 0.5) -> None:
        self.command[:] = (0.0, 0.0, 0.0)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish(0.0, 0.0, 0.0)
            self.arm.spin_for(0.025)

    def wait_ready(self, timeout_s: float = 12.0) -> None:
        external = [
            info for info in self.node.get_publishers_info_by_topic(self.config["topics"]["base_command"])
            if info.node_name != self.node.get_name()
        ]
        if external:
            owners = sorted({f"{info.node_namespace}/{info.node_name}" for info in external})
            raise RuntimeError(f"competing base command publisher: {owners}")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.arm.spin_for(0.05)
            if self.pose is not None and set(self.scans) == {"front", "rear"} and len(self.head_frames) >= 2:
                if self.publisher.get_subscription_count() >= 1:
                    self.assert_fresh()
                    self.stop(0.2)
                    return
        raise RuntimeError("native odometry, dual LiDAR, head image, or base controller is unavailable")

    def assert_fresh(self) -> None:
        stale = float(self.config["base_motion"]["feedback_stale_s"])
        now = time.monotonic()
        if self.pose is None or now - self.pose_at > stale:
            raise RuntimeError("native odometry became stale")
        for name in ("front", "rear"):
            if name not in self.scans or now - self.scans[name][0] > stale:
                raise RuntimeError(f"{name} LiDAR became stale")

    def nearest_lidar_m(self, name: str, half_angle_deg: float = 25.0) -> float:
        _stamp, scan = self.scans[name]
        values = []
        half = math.radians(half_angle_deg)
        for index, value in enumerate(scan.ranges):
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if abs(wrap(angle)) <= half and math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                values.append(float(value))
        return min(values) if values else math.inf

    def front_clearance_estimate_m(self) -> float:
        """Reject isolated speckles while retaining a short-window table stop."""
        _stamp, scan = self.scans["front"]
        values = []
        for index, value in enumerate(scan.ranges):
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if abs(wrap(angle)) <= math.radians(25.0) and math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                values.append(float(value))
        if not values:
            return math.inf
        values.sort()
        return values[min(2, len(values) - 1)]

    def _fresh_pose(self) -> tuple[float, float, float]:
        self.assert_fresh()
        assert self.pose is not None
        return self.pose

    def _tick(self, desired: tuple[float, float, float], last_tick: float) -> float:
        minimum = float(self.config["base_motion"]["minimum_lidar_range_m"])
        relevant = []
        if desired[0] > 0.002:
            relevant.append(("front", self.nearest_lidar_m("front")))
        elif desired[0] < -0.002:
            relevant.append(("rear", self.nearest_lidar_m("rear")))
        if abs(desired[1]) > 0.002 or abs(desired[2]) > 0.02:
            relevant.extend((name, self.nearest_lidar_m(name, 50.0)) for name in ("front", "rear"))
        blocked = [(name, distance) for name, distance in relevant if distance < minimum]
        if blocked:
            self.stop(0.25)
            detail = ", ".join(f"{name}={distance:.3f}m" for name, distance in blocked)
            raise RuntimeError(f"LiDAR hard stop below {minimum:.3f} m: {detail}")
        now = time.monotonic()
        dt = clamp(now - last_tick, 0.02, 0.10)
        for index, (target, acceleration) in enumerate(zip(desired, (0.15, 0.15, 0.28))):
            maximum_step = acceleration * dt
            self.command[index] += clamp(target - self.command[index], -maximum_step, maximum_step)
        self.publish(*self.command)
        self.arm.spin_for(0.025)
        self.assert_fresh()
        return now

    def translate(self, forward_m: float, left_m: float) -> dict[str, Any]:
        start_x, start_y, start_yaw = self._fresh_pose()
        forward_axis = (math.cos(start_yaw), math.sin(start_yaw))
        left_axis = (-math.sin(start_yaw), math.cos(start_yaw))
        target_x = start_x + forward_m * forward_axis[0] + left_m * left_axis[0]
        target_y = start_y + forward_m * forward_axis[1] + left_m * left_axis[1]
        speed = float(self.config["base_motion"]["linear_speed_mps"])
        tolerance = float(self.config["base_motion"]["position_tolerance_m"])
        timeout = max(15.0, math.hypot(forward_m, left_m) / speed + 20.0)
        deadline = time.monotonic() + timeout
        progress_at, best = time.monotonic(), math.inf
        last_tick = time.monotonic()
        try:
            while time.monotonic() < deadline:
                x, y, yaw = self._fresh_pose()
                ex, ey = target_x - x, target_y - y
                error = math.hypot(ex, ey)
                yaw_error = wrap(start_yaw - yaw)
                if error <= tolerance and abs(math.degrees(yaw_error)) <= float(self.config["base_motion"]["yaw_tolerance_deg"]):
                    self.stop(0.25)
                    break
                if error < best - 0.006:
                    best, progress_at = error, time.monotonic()
                elif time.monotonic() - progress_at > float(self.config["base_motion"]["no_progress_timeout_s"]):
                    raise RuntimeError("base translation made no odometry progress")
                body_x = math.cos(yaw) * ex + math.sin(yaw) * ey
                body_y = -math.sin(yaw) * ex + math.cos(yaw) * ey
                last_tick = self._tick((clamp(body_x * 0.8, -speed, speed),
                                        clamp(body_y * 0.8, -speed, speed),
                                        clamp(yaw_error * 1.2, -0.08, 0.08)), last_tick)
            else:
                raise TimeoutError("base translation timed out")
        finally:
            self.stop()
        end_x, end_y, end_yaw = self._fresh_pose()
        actual_forward = (end_x - start_x) * forward_axis[0] + (end_y - start_y) * forward_axis[1]
        actual_left = (end_x - start_x) * left_axis[0] + (end_y - start_y) * left_axis[1]
        endpoint_error = math.hypot(actual_forward - forward_m, actual_left - left_m)
        if endpoint_error > 0.06:
            raise RuntimeError(f"base translation endpoint error {endpoint_error:.3f} m")
        return {"requested_forward_m": forward_m, "requested_left_m": left_m,
                "actual_forward_m": actual_forward, "actual_left_m": actual_left,
                "endpoint_error_m": endpoint_error, "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw))}

    def rotate(self, ccw_deg: float) -> dict[str, Any]:
        start_x, start_y, start_yaw = self._fresh_pose()
        requested = math.radians(ccw_deg)
        accumulated = 0.0
        previous = start_yaw
        speed = float(self.config["base_motion"]["angular_speed_rps"])
        deadline = time.monotonic() + max(20.0, abs(requested) / speed + 15.0)
        last_tick = time.monotonic()
        progress_at, best = time.monotonic(), math.inf
        try:
            while time.monotonic() < deadline:
                x, y, yaw = self._fresh_pose()
                accumulated += wrap(yaw - previous)
                previous = yaw
                error = requested - accumulated
                if abs(math.degrees(error)) <= float(self.config["base_motion"]["yaw_tolerance_deg"]):
                    self.stop(0.25)
                    break
                if abs(error) < best - math.radians(0.7):
                    best, progress_at = abs(error), time.monotonic()
                elif time.monotonic() - progress_at > float(self.config["base_motion"]["no_progress_timeout_s"]):
                    raise RuntimeError("base rotation made no odometry progress")
                world_ex, world_ey = start_x - x, start_y - y
                body_x = math.cos(yaw) * world_ex + math.sin(yaw) * world_ey
                body_y = -math.sin(yaw) * world_ex + math.cos(yaw) * world_ey
                last_tick = self._tick((clamp(body_x, -0.018, 0.018), clamp(body_y, -0.018, 0.018),
                                        clamp(error * 1.15, -speed, speed)), last_tick)
            else:
                raise TimeoutError("base rotation timed out")
        finally:
            self.stop()
        end_x, end_y, _ = self._fresh_pose()
        drift = math.hypot(end_x - start_x, end_y - start_y)
        angle_error = math.degrees(accumulated - requested)
        if abs(angle_error) > 3.0 or drift > 0.08:
            raise RuntimeError(f"base rotation error={angle_error:.2f} deg drift={drift:.3f} m")
        return {"requested_ccw_deg": ccw_deg, "actual_ccw_deg": math.degrees(accumulated),
                "error_deg": angle_error, "position_drift_m": drift}

    def approach_front_clearance(self, maximum_forward_m: float, clearance_m: float) -> dict[str, Any]:
        start = self._fresh_pose()
        speed = float(self.config["base_motion"]["linear_speed_mps"])
        last_tick = time.monotonic()
        deadline = time.monotonic() + max(15.0, maximum_forward_m / speed + 15.0)
        best_travel = 0.0
        progress_at = time.monotonic()
        while time.monotonic() < deadline:
            x, y, yaw = self._fresh_pose()
            travel = math.hypot(x - start[0], y - start[1])
            if travel > best_travel + 0.006:
                best_travel, progress_at = travel, time.monotonic()
            elif time.monotonic() - progress_at > float(self.config["base_motion"]["no_progress_timeout_s"]):
                raise RuntimeError("pickup-table approach made no odometry progress")
            nearest = self.front_clearance_estimate_m()
            minimum = float(self.config["base_motion"]["minimum_lidar_range_m"])
            if nearest < minimum:
                raise RuntimeError(f"front LiDAR hard stop at {nearest:.3f} m")
            if nearest <= clearance_m:
                self.stop()
                return {"travel_m": travel, "front_clearance_m": nearest, "target_clearance_m": clearance_m}
            if travel >= maximum_forward_m:
                raise RuntimeError("pickup table was not observed before the configured approach limit")
            yaw_error = wrap(start[2] - yaw)
            last_tick = self._tick((speed, 0.0, clamp(1.2 * yaw_error, -0.07, 0.07)), last_tick)
        raise TimeoutError("pickup-table approach timed out")

    def run_stages(self, stages: list[dict[str, Any]]) -> list[dict[str, Any]]:
        reports = []
        for stage in stages:
            self.active_stage = str(stage["name"])
            if stage["kind"] == "translate":
                result = self.translate(float(stage["forward_m"]), float(stage["left_m"]))
            elif stage["kind"] == "rotate":
                result = self.rotate(float(stage["ccw_deg"]))
            else:
                result = self.approach_front_clearance(float(stage["maximum_forward_m"]),
                                                       float(stage["front_clearance_m"]))
            reports.append({"stage": stage["name"], **result})
        self.active_stage = None
        return reports

    def search_letter(self, target_letter: str, output: Path) -> dict[str, Any]:
        import cv2

        search = self.config["letter_search_shanghai_reference"]
        recognizer = LetterCardRecognizer(alphabet=search["alphabet"],
                                          minimum_confidence=float(search["minimum_confidence"]),
                                          row_split_y_norm=float(search["row_split_y_normalized"]))
        start_x, start_y, start_yaw = self._fresh_pose()
        right_axis = (math.sin(start_yaw), -math.cos(start_yaw))
        forward_axis = (math.cos(start_yaw), math.sin(start_yaw))
        recent: deque[tuple[float, str, float]] = deque(maxlen=max(5, int(search["stable_frames"])))
        consecutive_misses = 0
        last_stamp = self.head_frames[-1][0] if self.head_frames else 0
        last_frame = None
        last_detections = []
        last_tick = time.monotonic()
        deadline = time.monotonic() + max(45.0, float(search["maximum_right_m"]) / float(search["search_speed_mps"]) + 20.0)
        while time.monotonic() < deadline:
            if time.monotonic() - self.head_at > 0.8:
                raise RuntimeError("head RGB stream became stale during letter search")
            x, y, yaw = self._fresh_pose()
            dx, dy = x - start_x, y - start_y
            right_m = dx * right_axis[0] + dy * right_axis[1]
            forward_drift = dx * forward_axis[0] + dy * forward_axis[1]
            fresh = [(stamp, frame) for stamp, frame in self.head_frames if stamp > last_stamp]
            for stamp, message in fresh:
                last_stamp = stamp
                last_frame = decode_raw_bgr(message, int(self.config["head_camera"]["width"]),
                                            int(self.config["head_camera"]["height"]),
                                            self.config["head_camera"]["encodings"])
                last_detections = recognizer.detect(last_frame) if right_m >= float(search["minimum_detection_right_m"]) else []
                matches = [item for item in last_detections if item.letter == target_letter]
                if matches:
                    best = max(matches, key=lambda item: item.confidence)
                    recent.append((float(best.center_x_norm), str(best.row), float(best.confidence)))
                    consecutive_misses = 0
                else:
                    consecutive_misses += 1
                    if consecutive_misses >= 3:
                        recent.clear()
            stable_count = int(search["stable_frames"])
            window = list(recent)[-stable_count:]
            if len(window) == stable_count:
                centers = np.asarray([item[0] for item in window])
                rows = [item[1] for item in window]
                error = float(np.mean(centers) - 0.5)
                if len(set(rows)) == 1 and float(np.ptp(centers)) <= 0.05 and abs(error) <= float(search["center_tolerance_normalized"]):
                    self.stop()
                    if float(search["post_center_right_m"]) > 0.0:
                        self.translate(0.0, -float(search["post_center_right_m"]))
                    final_x, final_y, _ = self._fresh_pose()
                    measured_right_m = ((final_x - start_x) * right_axis[0]
                                        + (final_y - start_y) * right_axis[1])
                    output.parent.mkdir(parents=True, exist_ok=True)
                    if last_frame is not None:
                        cv2.imwrite(str(output), annotate(last_frame, last_detections))
                    return {"target": target_letter, "row": rows[-1], "measured_right_m": measured_right_m,
                            "center_error_normalized": error, "evidence_image": str(output)}
            if right_m >= float(search["maximum_right_m"]):
                raise RuntimeError(f"letter {target_letter} was not centered within the configured right-search limit")
            desired_right = float(search["search_speed_mps"])
            if recent and consecutive_misses < 3:
                image_error = recent[-1][0] - 0.5
                desired_right = clamp(0.20 * image_error,
                                      -float(search["refine_speed_mps"]),
                                      float(search["refine_speed_mps"]))
                if 0.0 < abs(desired_right) < 0.012:
                    desired_right = math.copysign(0.012, desired_right)
            yaw_error = wrap(start_yaw - yaw)
            last_tick = self._tick((clamp(-0.9 * forward_drift, -0.025, 0.025), -desired_right,
                                    clamp(1.2 * yaw_error, -0.07, 0.07)), last_tick)
        raise TimeoutError(f"letter {target_letter} search timed out")


class HamburgMission:
    def __init__(self, arm: NativeArmGraspCycle, base: NativeBaseControl,
                 spine: SpineControl, mission: dict[str, Any], grasp: dict[str, Any], output_dir: Path) -> None:
        self.arm = arm
        self.base = base
        self.spine = spine
        self.mission = mission
        self.grasp = grasp
        self.output_dir = output_dir
        self.mount = derive_mount_rotation(grasp["posture"]["left_pick_top_rad"],
                                           grasp["posture"]["left_tool_quaternion_xyzw"])
        self.report: dict[str, Any] = {}

    def checkpoint(self, phase: str) -> None:
        self.report["active_phase"] = phase
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / "mission-progress.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def pick(self, object_name: str) -> tuple[dict[str, Any], list[float]]:
        empty = self.arm.command_gripper(float(self.grasp["gripper"]["closed"]))
        self.arm.command_gripper(float(self.grasp["gripper"]["open"]))
        align_records: list[dict[str, Any]] = []
        aligned_q, frame = self.arm.visual_align(object_name, self.mount, align_records)
        image = self.output_dir / f"mission-{object_name}-aligned.png"
        from hamburg_grasp_check import save_observation_image
        save_observation_image(frame, object_name, tuple(align_records[-1]["rim_point_px"]), image)
        descent = float(self.grasp["objects"][object_name]["descent_m"])
        path = cartesian_waypoints(aligned_q, [0.0, 0.0, -descent], self.mount,
                                   float(self.grasp["motion"]["cartesian_waypoint_step_m"]))
        try:
            self.arm.move_arm_targets("left", path)
            closed = self.arm.command_gripper(float(self.grasp["gripper"]["closed"]))
            self.arm.move_arm_targets("left", list(reversed([aligned_q] + path[:-1])))
            retained = self.arm.settled_gripper_feedback(float(self.grasp["gripper"]["settle_timeout_s"]))
            evidence = gripper_contact_report(empty, closed, retained,
                                              float(self.grasp["gripper"]["minimum_contact_delta_from_empty_closed"]))
            if not evidence["accepted_as_held"]:
                self.arm.move_arm_targets("left", path)
                self.arm.command_gripper(float(self.grasp["gripper"]["open"]))
                self.arm.move_arm_targets("left", list(reversed([aligned_q] + path[:-1])))
                raise RuntimeError(f"{object_name} did not pass the post-lift gripper contact check")
            return {"object": object_name, "descent_m": descent, "visual_alignment": align_records,
                    "grasp_evidence": evidence, "aligned_image": str(image)}, aligned_q
        except BaseException:
            self.arm.commanded["left"] = list(self.arm.states["left"] or aligned_q)
            self.arm.move_to_joint_posture("left", aligned_q)
            raise

    def place(self, aligned_q: list[float], row: str, object_name: str) -> dict[str, Any]:
        placement = self.mission["placement_shanghai_reference"]
        forward = float(placement[f"{row}_forward_m"])
        down = float(self.grasp["objects"][object_name]["descent_m"])
        forward_path = cartesian_waypoints(aligned_q, [forward, 0.0, 0.0], self.mount,
                                           float(self.grasp["motion"]["cartesian_waypoint_step_m"]))
        self.arm.move_arm_targets("left", forward_path)
        forward_q = list(self.arm.commanded["left"] or forward_path[-1])
        down_path = cartesian_waypoints(forward_q, [0.0, 0.0, -down], self.mount,
                                        float(self.grasp["motion"]["cartesian_waypoint_step_m"]))
        self.arm.move_arm_targets("left", down_path)
        opened = self.arm.command_gripper(float(self.grasp["gripper"]["open"]))
        self.arm.move_arm_targets("left", list(reversed([forward_q] + down_path[:-1])))
        self.arm.move_arm_targets("left", list(reversed([aligned_q] + forward_path[:-1])))
        return {"row": row, "forward_m": forward, "down_m": down, "open_feedback": opened}

    def run(self) -> dict[str, Any]:
        report: dict[str, Any] = {
            "schema_version": 1, "status": "running", "motion_commanded": True,
            "single_ros_node_during_motion": True, "assignment": self.mission["assignment"],
            "route_profile_warning": self.mission["profile_warning"],
            "hamburg_measurements": self.mission["hamburg_measurements"], "objects": {},
        }
        self.report = report
        self.checkpoint("wait_for_native_interfaces")
        self.arm.wait_live(10.0)
        self.arm.assert_controller_graph()
        self.base.wait_ready()
        self.checkpoint("initialize_spine_and_arms")
        report["spine"] = self.spine.move_absolute(float(self.grasp["spine"]["initial_target_m"]))
        self.arm.move_to_joint_posture("right", self.grasp["posture"]["right_parking_rad"])
        self.arm.move_to_joint_posture("left", self.grasp["posture"]["left_pick_top_rad"])
        self.checkpoint("outbound_shanghai_reference_route")
        report["outbound"] = self.base.run_stages(self.mission["outbound_shanghai_reference"])
        for index, object_name in enumerate(OBJECTS):
            destination = self.mission["assignment"][object_name]
            item: dict[str, Any] = {"destination": destination}
            report["objects"][object_name] = item
            self.checkpoint(f"{object_name}_observe_align_pick")
            item["pick"], aligned_q = self.pick(object_name)
            self.checkpoint(f"{object_name}_post_pick_route")
            item["post_pick_route"] = self.base.run_stages(self.mission["post_pick_shanghai_reference"])
            evidence = self.output_dir / f"mission-{object_name}-letter-{destination}.png"
            self.checkpoint(f"{object_name}_search_letter_{destination}")
            item["letter_search"] = self.base.search_letter(destination, evidence)
            self.checkpoint(f"{object_name}_place_at_{destination}")
            item["place"] = self.place(aligned_q, item["letter_search"]["row"], object_name)
            if index < len(OBJECTS) - 1:
                returning = self.mission["return_to_pickup_shanghai_reference"]
                measured = float(item["letter_search"]["measured_right_m"])
                stages = [
                    {"name": "left_by_measured_search", "kind": "translate", "forward_m": 0.0,
                     "left_m": measured + float(returning["extra_left_after_measured_search_m"])},
                    {"name": "clockwise_to_pickup", "kind": "rotate", "ccw_deg": -float(returning["clockwise_turn_deg"])},
                    {"name": "to_before_door", "kind": "translate", "forward_m": float(returning["to_before_door_m"]), "left_m": 0.0},
                    {"name": "through_door", "kind": "translate", "forward_m": float(returning["through_door_m"]), "left_m": 0.0},
                    {"name": "approach_pickup_table", "kind": "front_clearance",
                     "maximum_forward_m": float(returning["approach_pickup_maximum_m"]),
                     "front_clearance_m": float(returning["pickup_front_clearance_m"])},
                ]
                self.checkpoint(f"{object_name}_return_to_pickup")
                item["return_to_pickup"] = self.base.run_stages(stages)
        self.base.stop(1.0)
        report["status"] = "complete"
        report["zero_base_command_latched"] = True
        self.checkpoint("complete")
        return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--config", type=Path, default=Path(os.environ.get(
        "TMR_HAMBURG_MISSION_CONFIG", str(DEFAULT_MISSION_CONFIG))))
    parser.add_argument("--grasp-config", type=Path, default=Path(os.environ.get(
        "TMR_HAMBURG_GRASP_CYCLE_CONFIG", str(DEFAULT_GRASP_CONFIG))))
    parser.add_argument("--venue-config", type=Path, default=DEFAULT_VENUE_CONFIG)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/tmr_hamburg_mission"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mission = json.loads(args.config.read_text(encoding="utf-8"))
    validate_mission_config(mission)
    grasp = load_cycle_config(args.grasp_config)
    if not args.execute:
        write_report(plan_report(mission, grasp), args.output)
        return 0
    venue = load_config(args.venue_config)
    environment, errors = environment_report(venue)
    if errors:
        write_report({"status": "blocked", "motion_commanded": False,
                      "environment": environment, "errors": errors}, args.output)
        return 2
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("tmr_task3_hamburg_single_node_mission")
    spine: SpineControl | None = None
    base: NativeBaseControl | None = None
    mission_runner: HamburgMission | None = None
    try:
        arm = NativeArmGraspCycle(node, venue, grasp, args.output_dir)
        base = NativeBaseControl(node, arm, mission)
        spine = SpineControl(node, venue["interface_profile"]["spine"])
        mission_runner = HamburgMission(arm, base, spine, mission, grasp, args.output_dir)
        report = mission_runner.run()
    except KeyboardInterrupt:
        if base is not None:
            base.stop(1.0)
        report = dict(mission_runner.report) if mission_runner is not None else {}
        if base is not None and base.active_stage is not None:
            report["base_active_stage"] = base.active_stage
        report.update({"status": "interrupted", "motion_commanded": True,
                       "errors": ["operator interrupted the mission"], "zero_base_command_latched": True})
    except BaseException as exc:
        if base is not None:
            base.stop(1.0)
        report = dict(mission_runner.report) if mission_runner is not None else {}
        if base is not None and base.active_stage is not None:
            report["base_active_stage"] = base.active_stage
        report.update({"status": "failed", "motion_commanded": True,
                       "errors": [f"{type(exc).__name__}: {exc}"], "zero_base_command_latched": True})
    finally:
        if spine is not None:
            spine.close()
        node.destroy_node()
        rclpy.shutdown()
    write_report(report, args.output)
    return 0 if report.get("status") == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
