#!/usr/bin/env python3
"""Move robot-right, find a configured letter card, and center it in ZED RGB.

The base controller and vision loop run in one process on the base computer.
The controller learns the sign of image motion from a short odometry-measured
probe, so camera mounting skew does not need a trusted hand-derived transform.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Iterable

from letter_card_vision import LetterCardRecognizer, LetterDetection


ODOM_TOPIC = "/swerve_drive_controller/odom"
COMMAND_TOPIC = "/tmr_cycle/mission_cmd_vel"
CONTROLLER_COMMAND_TOPIC = "/swerve_drive_controller/cmd_vel"
LEASE_TOPIC = "/tmr_cycle/mission_active"
DEFAULT_CAMERA_TOPIC = "/head_camera/zed/rgb/color/rect/image/compressed"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class TargetObservation:
    right_m: float
    center_x_norm: float
    row: str
    confidence: float
    members: int


class TargetTracker:
    """Temporal consensus and same-letter group-centre estimation."""

    def __init__(
        self,
        target_letter: str,
        requested_row: str = "auto",
        history_size: int = 7,
        center_tolerance_norm: float = 0.055,
        stable_frames: int = 3,
    ) -> None:
        target = target_letter.strip().upper()
        if len(target) != 1 or not target.isalpha():
            raise ValueError("target_letter must be one A-Z character")
        if requested_row not in {"auto", "near", "far"}:
            raise ValueError("requested_row must be auto, near, or far")
        self.target_letter = target
        self.requested_row = requested_row
        self.center_tolerance_norm = float(center_tolerance_norm)
        self.stable_frames = int(stable_frames)
        self.history: deque[TargetObservation] = deque(maxlen=int(history_size))
        self.locked_row: str | None = None if requested_row == "auto" else requested_row
        self.consecutive_misses = 0

    def observe(
        self, detections: Iterable[LetterDetection], right_m: float
    ) -> TargetObservation | None:
        matches = [item for item in detections if item.letter == self.target_letter]
        if self.requested_row != "auto":
            matches = [item for item in matches if item.row == self.requested_row]
        if not matches:
            self.consecutive_misses += 1
            if self.consecutive_misses >= 2:
                self.history.clear()
            return None
        self.consecutive_misses = 0
        groups: dict[str, list[LetterDetection]] = {"near": [], "far": []}
        for item in matches:
            groups[item.row].append(item)
        if self.locked_row is None:
            row = max(groups, key=lambda name: sum(item.confidence for item in groups[name]))
            recent_rows = [item.row for item in self.history]
            if recent_rows.count(row) >= 1:
                self.locked_row = row
        else:
            row = self.locked_row
        selected = groups[row]
        if not selected:
            return None
        weights = [max(0.05, item.confidence) for item in selected]
        center = sum(item.center_x_norm * weight for item, weight in zip(selected, weights)) / sum(weights)
        observation = TargetObservation(
            right_m=float(right_m),
            center_x_norm=float(center),
            row=row,
            confidence=float(sum(weights) / len(weights)),
            members=len(selected),
        )
        self.history.append(observation)
        return observation

    def centered(self) -> tuple[bool, float | None, float | None]:
        if self.consecutive_misses or len(self.history) < self.stable_frames:
            return False, None, None
        recent = list(self.history)[-self.stable_frames :]
        if self.locked_row is not None and any(item.row != self.locked_row for item in recent):
            return False, None, None
        errors = [item.center_x_norm - 0.5 for item in recent]
        mean = sum(errors) / len(errors)
        spread = max(errors) - min(errors)
        stable = abs(mean) <= self.center_tolerance_norm and spread <= 0.032
        return stable, mean, spread


class AdaptiveCenterPolicy:
    """Learn d(image-x)/d(robot-right) and close the image-centering loop."""

    def __init__(self, search_speed_mps: float, refine_speed_mps: float) -> None:
        self.search_speed_mps = float(search_speed_mps)
        self.refine_speed_mps = float(refine_speed_mps)
        self.previous: TargetObservation | None = None
        self.image_gain_per_m: float | None = None
        self.last_direction = 1.0

    def command(self, observation: TargetObservation | None) -> float:
        if observation is None:
            return self.search_speed_mps if self.previous is None else 0.45 * self.search_speed_mps * self.last_direction
        if self.previous is not None and self.previous.members == observation.members:
            delta_m = observation.right_m - self.previous.right_m
            if abs(delta_m) >= 0.018:
                sample = (observation.center_x_norm - self.previous.center_x_norm) / delta_m
                if 0.04 <= abs(sample) <= 8.0:
                    self.image_gain_per_m = (
                        sample
                        if self.image_gain_per_m is None
                        else 0.65 * self.image_gain_per_m + 0.35 * sample
                    )
        self.previous = observation
        error = observation.center_x_norm - 0.5
        if abs(error) <= 0.055:
            return 0.0
        if self.image_gain_per_m is None or abs(self.image_gain_per_m) < 0.04:
            self.last_direction = 1.0
            return 0.55 * self.search_speed_mps
        desired_displacement = -error / self.image_gain_per_m
        speed = clamp(0.65 * desired_displacement, -self.refine_speed_mps, self.refine_speed_mps)
        if 0.0 < abs(speed) < 0.018:
            speed = math.copysign(0.018, speed)
        self.last_direction = math.copysign(1.0, speed)
        return speed


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def run_ros(args: argparse.Namespace) -> dict:
    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool

    class SearchNode(Node):
        def __init__(self) -> None:
            super().__init__("tmr_letter_guided_search")
            self.pose = None
            self.odom_at = 0.0
            self.frame_bytes: bytes | None = None
            self.frame_sequence = 0
            self.frame_at = 0.0
            self.frame_file_mtime_ns = -1
            self.command = [0.0, 0.0, 0.0]
            command_topic = CONTROLLER_COMMAND_TOPIC if args.direct_controller else COMMAND_TOPIC
            self.command_pub = self.create_publisher(TwistStamped, command_topic, 10)
            self.lease_pub = self.create_publisher(Bool, LEASE_TOPIC, 10)
            self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, qos_profile_sensor_data)
            if args.camera_file is None:
                self.create_subscription(
                    CompressedImage, args.camera_topic, self.on_frame, qos_profile_sensor_data
                )

        def on_odom(self, message) -> None:
            position = message.pose.pose.position
            self.pose = (
                float(position.x),
                float(position.y),
                yaw_from_quaternion(message.pose.pose.orientation),
            )
            self.odom_at = time.monotonic()

        def on_frame(self, message) -> None:
            self.frame_bytes = bytes(message.data)
            self.frame_sequence += 1
            self.frame_at = time.monotonic()

        def refresh_frame_file(self) -> None:
            if args.camera_file is None:
                return
            try:
                stat = args.camera_file.stat()
                if stat.st_mtime_ns == self.frame_file_mtime_ns:
                    return
                payload = args.camera_file.read_bytes()
            except (FileNotFoundError, OSError):
                return
            if payload:
                self.frame_bytes = payload
                self.frame_file_mtime_ns = stat.st_mtime_ns
                self.frame_sequence += 1
                self.frame_at = time.monotonic()

        def publish(self, vx: float, vy: float, wz: float) -> None:
            if not args.direct_controller:
                lease = Bool()
                lease.data = True
                self.lease_pub.publish(lease)
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = float(vx)
            message.twist.linear.y = float(vy)
            message.twist.angular.z = float(wz)
            self.command_pub.publish(message)

        def stop(self, samples: int = 24) -> None:
            self.command[:] = (0.0, 0.0, 0.0)
            for _ in range(samples):
                self.publish(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.025)

        def ready(self) -> None:
            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                self.refresh_frame_file()
                if (
                    self.pose is not None
                    and self.frame_bytes is not None
                    and time.monotonic() - self.odom_at <= 0.4
                    and time.monotonic() - self.frame_at <= 0.8
                    and self.command_pub.get_subscription_count() == 1
                ):
                    self.stop(6)
                    return
            raise RuntimeError("fresh odometry, ZED RGB, or exclusive velocity adapter unavailable")

        def fresh_pose(self):
            if self.pose is None or time.monotonic() - self.odom_at > 0.4:
                raise RuntimeError("odometry became stale")
            return self.pose

    rclpy.init()
    node = SearchNode()
    tracker = TargetTracker(
        args.target_letter,
        args.row,
        center_tolerance_norm=args.center_tolerance_norm,
        stable_frames=args.stable_frames,
    )
    policy = AdaptiveCenterPolicy(args.search_speed_mps, args.refine_speed_mps)
    recognizer = LetterCardRecognizer(
        alphabet=args.alphabet,
        minimum_confidence=args.minimum_confidence,
        row_split_y_norm=args.row_split_y_norm,
    )
    start = None
    latest_observation = None
    last_processed_sequence = -1
    decoded_frames = 0
    target_frames = 0
    last_target_at = None
    next_report = 0.0
    last_tick = time.monotonic()
    max_right_seen = 0.0
    motion_anchor = None
    motion_anchor_at = time.monotonic()
    try:
        node.ready()
        start = node.fresh_pose()
        start_x, start_y, start_yaw = start
        right_axis = (math.sin(start_yaw), -math.cos(start_yaw))
        forward_axis = (math.cos(start_yaw), math.sin(start_yaw))
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.refresh_frame_file()
            x, y, yaw = node.fresh_pose()
            dx, dy = x - start_x, y - start_y
            right_m = dx * right_axis[0] + dy * right_axis[1]
            forward_drift = dx * forward_axis[0] + dy * forward_axis[1]
            max_right_seen = max(max_right_seen, right_m)
            if node.frame_bytes is None or time.monotonic() - node.frame_at > 0.9:
                raise RuntimeError("ZED RGB stream became stale")
            if node.frame_sequence != last_processed_sequence:
                last_processed_sequence = node.frame_sequence
                array = np.frombuffer(node.frame_bytes, np.uint8)
                frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("ZED compressed frame decode failed")
                decoded_frames += 1
                detections = recognizer.detect(frame)
                latest_observation = tracker.observe(detections, right_m)
                if latest_observation is not None:
                    target_frames += 1
                    last_target_at = time.monotonic()
                    print(json.dumps({
                        "event": "target_observed",
                        "target": args.target_letter,
                        "row": latest_observation.row,
                        "center_x_norm": latest_observation.center_x_norm,
                        "confidence": latest_observation.confidence,
                        "members": latest_observation.members,
                        "right_m": right_m,
                    }), flush=True)
            centered, mean_error, spread = tracker.centered()
            if centered:
                node.stop(30)
                end_x, end_y, end_yaw = node.fresh_pose()
                actual_right = (end_x - start_x) * right_axis[0] + (end_y - start_y) * right_axis[1]
                return {
                    "status": "success",
                    "target_centered": True,
                    "target_letter": args.target_letter,
                    "row": tracker.locked_row,
                    "actual_right_m": actual_right,
                    "maximum_right_m": max_right_seen,
                    "center_error_norm": mean_error,
                    "center_spread_norm": spread,
                    "decoded_frames": decoded_frames,
                    "target_frames": target_frames,
                    "image_gain_per_m": policy.image_gain_per_m,
                    "zero_command_latched": True,
                    "start_odom": [start_x, start_y, start_yaw],
                    "end_odom": [end_x, end_y, end_yaw],
                    "yaw_error_deg": math.degrees(wrap(end_yaw - start_yaw)),
                }
            right_speed = policy.command(latest_observation)
            if last_target_at is not None and time.monotonic() - last_target_at > 1.4:
                right_speed = 0.35 * args.search_speed_mps * policy.last_direction
            if right_speed > 0.0 and right_m >= args.max_right_m - 0.025:
                node.stop(30)
                raise RuntimeError(
                    f"target {args.target_letter} not centered before {args.max_right_m:.3f} m right limit"
                )
            if right_speed < 0.0 and right_m <= -0.025:
                right_speed = 0.0
            if abs(right_speed) >= 0.018:
                if motion_anchor is None:
                    motion_anchor = (x, y)
                    motion_anchor_at = time.monotonic()
                elif math.hypot(x - motion_anchor[0], y - motion_anchor[1]) >= 0.008:
                    motion_anchor = (x, y)
                    motion_anchor_at = time.monotonic()
                elif time.monotonic() - motion_anchor_at > 3.0:
                    raise RuntimeError("no odometry progress during letter-guided motion")
            else:
                motion_anchor = (x, y)
                motion_anchor_at = time.monotonic()
            desired = (
                clamp(-0.85 * forward_drift, -0.025, 0.025),
                -right_speed,
                clamp(1.15 * wrap(start_yaw - yaw), -0.07, 0.07),
            )
            now = time.monotonic()
            dt = clamp(now - last_tick, 0.02, 0.10)
            last_tick = now
            for index, (target, acceleration) in enumerate(
                zip(desired, (0.15, 0.15, 0.42))
            ):
                step = acceleration * dt
                node.command[index] += clamp(target - node.command[index], -step, step)
            node.publish(*node.command)
            if now >= next_report:
                print(json.dumps({
                    "event": "search_progress",
                    "right_m": right_m,
                    "right_speed_mps": right_speed,
                    "target_seen": latest_observation is not None,
                    "gain": policy.image_gain_per_m,
                }), flush=True)
                next_report = now + 1.0
        raise TimeoutError("letter-guided search timed out")
    finally:
        try:
            node.stop(30)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--direct-controller",
        action="store_true",
        help="publish directly to the active swerve controller; the velocity adapter must be stopped",
    )
    parser.add_argument("--target-letter", default="B")
    parser.add_argument("--row", choices=("auto", "near", "far"), default="auto")
    parser.add_argument("--alphabet", default="ABE")
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument(
        "--camera-file",
        type=Path,
        help="read atomically refreshed JPEG frames from this file instead of subscribing in the control DDS domain",
    )
    parser.add_argument("--max-right-m", type=float, default=2.40)
    parser.add_argument("--search-speed-mps", type=float, default=0.055)
    parser.add_argument("--refine-speed-mps", type=float, default=0.040)
    parser.add_argument("--center-tolerance-norm", type=float, default=0.055)
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--minimum-confidence", type=float, default=0.22)
    parser.add_argument("--row-split-y-norm", type=float, default=0.52)
    parser.add_argument("--timeout-s", type=float, default=75.0)
    parser.add_argument("--state-file", type=Path)
    args = parser.parse_args()
    args.target_letter = args.target_letter.strip().upper()
    if len(args.target_letter) != 1 or args.target_letter not in args.alphabet.upper():
        parser.error("target-letter must be one character included in --alphabet")
    if not 0.20 <= args.max_right_m <= 2.40:
        parser.error("max-right-m must be in [0.20, 2.40]")
    return args


def dry_run(args: argparse.Namespace) -> dict:
    return {
        "status": "dry_run",
        "motion_enabled": False,
        "target_letter": args.target_letter,
        "requested_row": args.row,
        "maximum_right_m": args.max_right_m,
        "camera_topic": args.camera_topic,
        "camera_file": str(args.camera_file) if args.camera_file else None,
        "direct_controller": args.direct_controller,
        "vision": "white-card HSV -> perspective rectify -> glyph template -> temporal consensus",
        "centering": "online image-motion gain from odometry probe; signed low-speed refinement",
        "depth_required": False,
    }


def main() -> int:
    args = parse_args()
    if not args.execute:
        print(json.dumps(dry_run(args), indent=2), flush=True)
        return 0
    try:
        result = run_ros(args)
        if args.state_file:
            args.state_file.parent.mkdir(parents=True, exist_ok=True)
            args.state_file.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        report = {"status": "failed", "error": repr(exc), "zero_command_latched": True}
        if args.state_file:
            args.state_file.parent.mkdir(parents=True, exist_ok=True)
            args.state_file.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
