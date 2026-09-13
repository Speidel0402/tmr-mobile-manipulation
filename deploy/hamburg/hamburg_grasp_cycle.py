#!/usr/bin/env python3
"""Run one autonomous Hamburg observation-to-grasp-to-release trial.

The deployed Franka controller consumes JointState targets on topics whose
names contain ``gello``.  No GELLO leader is used here: this single ROS node
computes and streams autonomous targets directly to that controller input.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time
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


def load_cycle_config(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    required = {"topics", "joint_names", "posture", "motion", "vision", "spine", "gripper", "objects"}
    missing = sorted(required - data.keys())
    if missing:
        raise ValueError(f"grasp-cycle config is missing: {', '.join(missing)}")
    for object_name in OBJECTS:
        descent = float(data["objects"][object_name]["descent_m"])
        if not 0.05 <= descent <= 0.40:
            raise ValueError(f"{object_name} descent is outside the 0.05-0.40 m trial bound")
    return data


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
        self.commanded: dict[str, list[float] | None] = {"left": None, "right": None}
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
        self.commanded = {arm: list(self.states[arm] or []) for arm in ("left", "right")}

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

    def _check_following(self, arm: str) -> None:
        measured = self.states[arm]
        commanded = self.commanded[arm]
        if measured is None or commanded is None:
            raise RuntimeError(f"{arm} joint feedback disappeared")
        if time.monotonic() - self.state_times[arm] > 0.5:
            raise RuntimeError(f"{arm} joint feedback became stale")
        error = max(abs(a - b) for a, b in zip(measured, commanded))
        if error > float(self.config["motion"]["maximum_following_error_rad"]):
            raise RuntimeError(f"{arm} following error {error:.4f} rad exceeds limit")

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
                self._check_following(arm)
        tolerance = float(self.config["motion"]["endpoint_tolerance_rad"])
        timeout = float(self.config["motion"]["endpoint_timeout_s"])
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            measured = self.states[arm]
            if measured is not None and max(abs(a - b) for a, b in zip(measured, targets[-1])) <= tolerance:
                return
        raise RuntimeError(f"{arm} failed to settle at commanded target")

    def move_to_joint_posture(self, arm: str, target: list[float]) -> None:
        start = list(self.commanded[arm] or self.states[arm] or [])
        targets = smooth_joint_targets(
            start, target,
            float(self.config["motion"]["publish_rate_hz"]),
            float(self.config["motion"]["maximum_joint_velocity_rad_s"]),
        )
        # The interpolation above is already rate-limited; stream it directly.
        rate = float(self.config["motion"]["publish_rate_hz"])
        for point in targets:
            self.commanded[arm] = point
            self.spin_for(1.0 / rate)
            self._check_following(arm)
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
        return self.settled_gripper_feedback(float(self.config["gripper"]["settle_timeout_s"]))

    def settled_gripper_feedback(self, timeout_s: float) -> list[float]:
        spread_limit = float(self.config["gripper"]["settled_spread"])
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            recent = [values for stamp, values in self.gripper_samples if time.monotonic() - stamp <= 0.5]
            if len(recent) >= 4 and len({len(item) for item in recent}) == 1:
                array = np.asarray(recent, dtype=float)
                if float(np.max(np.ptp(array, axis=0))) <= spread_limit:
                    return np.median(array, axis=0).tolist()
        raise RuntimeError("left gripper feedback did not settle")

    def stable_detection(self, object_name: str, timeout_s: float = 8.0) -> tuple[tuple[float, float], Any]:
        points: deque[tuple[float, float] | None] = deque(maxlen=7)
        last_stamp = self.images[-1][0] if self.images else 0
        last_bgr = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_for(0.05)
            fresh = [(stamp, image) for stamp, image in self.images if stamp > last_stamp]
            for stamp, image in fresh:
                last_stamp = stamp
                try:
                    last_bgr = decode_wrist_rgb(image)
                    points.append(detect_object(object_name, last_bgr))
                except Exception:
                    points.append(None)
                valid = [point for point in points if point is not None]
                if len(valid) >= 5 and len(points) >= 5 and all(point is not None for point in list(points)[-2:]):
                    array = np.asarray(valid, dtype=float)
                    median = np.median(array, axis=0)
                    spread = float(np.max(np.linalg.norm(array - median, axis=1)))
                    if spread <= float(self.config["vision"]["maximum_stable_spread_px"]):
                        return (float(median[0]), float(median[1])), last_bgr
            self.images = deque([(stamp, image) for stamp, image in self.images if stamp >= last_stamp], maxlen=20)
        raise RuntimeError(f"no stable fresh {object_name} rim detection")

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
            "selected_descent_m": descent_m,
            "phases_completed": [],
            "visual_alignment": [],
        }
        self.last_report = report
        report["active_phase"] = "wait_for_live_interfaces"
        self.wait_live(10.0)
        report["controller_subscriber_counts"] = self.assert_controller_graph()
        report["active_phase"] = "spine_to_pickup_height"
        report["spine"] = spine.move_absolute(float(self.config["spine"]["initial_target_m"]))
        report["phases_completed"].append("spine_ready")
        report["motion_commanded"] = True
        report["active_phase"] = "arms_to_pickup_posture"
        self.move_to_joint_posture("right", self.config["posture"]["right_parking_rad"])
        self.move_to_joint_posture("left", self.config["posture"]["left_pick_top_rad"])
        report["phases_completed"].append("arms_at_shanghai_reference_pickup_view")

        report["active_phase"] = "empty_close_baseline"
        empty = self.command_gripper(float(self.config["gripper"]["closed"]))
        self.command_gripper(float(self.config["gripper"]["open"]))
        report["empty_closed_baseline"] = empty
        report["phases_completed"].append("empty_close_baseline_recorded")

        mount = derive_mount_rotation(
            self.config["posture"]["left_pick_top_rad"],
            self.config["posture"]["left_tool_quaternion_xyzw"],
        )
        report["active_phase"] = "wrist_observation_and_alignment"
        aligned_q, bgr = self.visual_align(object_name, mount, report["visual_alignment"])
        image_path = self.output_dir / f"{object_name}-aligned-wrist.png"
        final_point = tuple(report["visual_alignment"][-1]["rim_point_px"])
        save_observation_image(bgr, object_name, final_point, image_path)
        report["aligned_wrist_image"] = str(image_path)
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
            report["active_phase"] = "descent"
            descent_started = True
            self.move_arm_targets("left", descent_path)
            report["phases_completed"].append("descent_complete")
            report["active_phase"] = "close_and_lift"
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
            # A trial always leaves the utensil at the pickup point and the arm above it.
            if descent_started and not lifted:
                # Return through the known pickup-top joint target even if a
                # following-error exception interrupted the downward stream.
                self.move_to_joint_posture("left", aligned_q)
            elif object_closed is not None:
                report["active_phase"] = "return_release_and_retract"
                self.move_arm_targets("left", descent_path)
                self.command_gripper(float(self.config["gripper"]["open"]))
                self.move_arm_targets("left", list(reversed([aligned_q] + descent_path[:-1])))
        report["phases_completed"].append("released_and_returned_to_pickup_top")
        report["status"] = "passed" if report["grasp_evidence"]["accepted_as_held"] else "failed_contact_check"
        report["active_phase"] = "complete"
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
    config = load_cycle_config(args.config)
    descent = float(args.descent_m if args.descent_m is not None else config["objects"][args.object]["descent_m"])
    if not 0.05 <= descent <= 0.40:
        raise SystemExit("--descent-m must be between 0.05 and 0.40")
    if not args.execute:
        write_report(plan_report(config, args.object, args.descent_m), args.output)
        return 0

    venue = load_config(args.venue_config)
    environment, errors = environment_report(venue)
    if errors:
        report = {"status": "blocked", "motion_commanded": False,
                  "environment": environment, "errors": errors}
        write_report(report, args.output)
        return 2

    import rclpy
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("tmr_task3_hamburg_autonomous_grasp")
    spine: SpineControl | None = None
    runner: NativeArmGraspCycle | None = None
    try:
        runner = NativeArmGraspCycle(node, venue, config, args.output_dir)
        spine = SpineControl(node, venue["interface_profile"]["spine"])
        report = runner.run(args.object, descent, spine)
    except KeyboardInterrupt:
        report = dict(runner.last_report) if runner is not None else {}
        report.update({"status": "interrupted", "motion_commanded": True,
                       "errors": ["operator interrupted the physical trial"]})
    except Exception as exc:
        report = dict(runner.last_report) if runner is not None else {}
        report.update({"status": "failed", "motion_commanded": True,
                       "errors": [f"{type(exc).__name__}: {exc}"]})
    finally:
        if spine is not None:
            spine.close()
        node.destroy_node()
        rclpy.shutdown()
    write_report(report, args.output)
    return 0 if report.get("status") == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
