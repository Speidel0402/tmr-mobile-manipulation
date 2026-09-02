#!/usr/bin/env python3
"""Adaptive horizontal visual servo for the left wrist cup-rim point.

This controller locks end-effector height and orientation.  It learns the local
base-XY-to-image-UV Jacobian with two small probes, then uses short, shrinking
steps and Broyden updates.  It never commands vertical motion or the gripper.
"""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from geometry_msgs.msg import Pose
from lifecycle_msgs.msg import State
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from franka_msgs.action import ErrorRecovery, PTPMotion
from franka_msgs.msg import FrankaRobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK

JOINT_NAMES = [f"left_fr3v2_joint{i}" for i in range(1, 8)]


class ServoNode(Node):
    def __init__(self):
        super().__init__("adaptive_cup_edge_xy_servo")
        self.q = None
        self.robot_state = None
        self.lock_z = None
        self.lock_orientation = None
        self.max_joint_velocity = 0.07
        self.hold_target = None
        self.hold_pub = self.create_publisher(
            JointState, "/left/gello/joint_states", 1
        )
        self.create_subscription(
            JointState,
            "/left/franka_robot_state_broadcaster/measured_joint_states",
            self._on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FrankaRobotState,
            "/left/franka_robot_state_broadcaster/robot_state",
            self._on_robot_state,
            qos_profile_sensor_data,
        )
        self.fk_client = self.create_client(GetPositionFK, "/left_ik/compute_fk")
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/left_ik/compute_cartesian_path"
        )
        self.switch_client = self.create_client(
            SwitchController, "/left/controller_manager/switch_controller"
        )
        self.hardware_client = self.create_client(
            SetHardwareComponentState,
            "/left/controller_manager/set_hardware_component_state",
        )
        self.error_recovery = ActionClient(
            self, ErrorRecovery, "/left/action_server/error_recovery"
        )
        self.arm = ActionClient(self, PTPMotion, "/left/action_server/ptp_motion")

    def _on_joints(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            self.q = [float(positions[name]) for name in JOINT_NAMES]

    def _on_robot_state(self, message):
        self.robot_state = message

    def _set_hardware_state(self, state_id, label):
        request = SetHardwareComponentState.Request()
        request.name = "left_FrankaHardwareInterface"
        request.target_state = State()
        request.target_state.id = state_id
        request.target_state.label = label
        return self.call(self.hardware_client, request, timeout=12.0)

    def _active_error_names(self):
        if self.robot_state is None:
            return []
        errors = self.robot_state.current_errors
        return [
            name
            for name in errors.get_fields_and_field_types()
            if bool(getattr(errors, name))
        ]

    def _recover_robot_error(self):
        """Run Franka error recovery before hardware/controller activation."""
        if not self.error_recovery.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Franka error recovery action unavailable")
        future = self.error_recovery.send_goal_async(ErrorRecovery.Goal())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("Franka error recovery goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=12.0)
        wrapped = result_future.result() if result_future.done() else None
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped is None else int(wrapped.status)
            raise RuntimeError(f"Franka error recovery failed (status={status})")
        return True

    def ensure_runtime_ready(self):
        """Recover the left-arm lifecycle after a PTP controller hand-off."""
        if not self.hardware_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("hardware lifecycle service unavailable")
        if not self.switch_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError("controller switch service unavailable")

        # A healthy controller-manager must not be lifecycle-transitioned just
        # because this client started.  First accept a fresh live state; only
        # use the recovery path when both state streams are genuinely absent.
        live_deadline = time.monotonic() + 1.0
        while (self.q is None or self.robot_state is None) and time.monotonic() < live_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        state_was_live = self.q is not None and self.robot_state is not None
        recovered = False
        if state_was_live:
            active_errors = self._active_error_names()
            if active_errors:
                # A single transition sample is harmless; a persistent FCI
                # error must be cleared before lifecycle activation.
                time.sleep(0.35)
                rclpy.spin_once(self, timeout_sec=0.05)
                active_errors = self._active_error_names()
                if active_errors:
                    recovered = self._recover_robot_error()

        if not state_was_live or recovered:
            response = self._set_hardware_state(State.PRIMARY_STATE_ACTIVE, "active")
            if response.state.id != State.PRIMARY_STATE_ACTIVE:
                # The first ACTIVE request from UNCONFIGURED commonly performs
                # only configure and reports INACTIVE; a second request then
                # performs activation.
                response = self._set_hardware_state(
                    State.PRIMARY_STATE_ACTIVE, "active"
                )
            if response.state.id != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(
                    "left hardware activation failed "
                    f"(state={response.state.id}); restart the arm controller "
                    "process if this followed a Franka NetworkException"
                )

        # BEST_EFFORT makes this idempotent: a controller that is already
        # active must not prevent the missing publishers/hold controller from
        # being restored.
        # Seed the impedance controller with the measured PTP endpoint before
        # activation.  Without this refresh it can reuse an old cached target
        # and slowly pull the arm away during a long base translation.
        self.publish_hold_target(samples=3)
        request = SwitchController.Request()
        request.activate_controllers = [
            "joint_state_broadcaster",
            "franka_robot_state_broadcaster",
            "joint_impedance_controller",
        ]
        request.deactivate_controllers = []
        request.strictness = 1
        request.activate_asap = False
        request.timeout.sec = 5
        response = self.call(self.switch_client, request, timeout=8.0)
        if not response.ok:
            raise RuntimeError("left runtime controller recovery failed")
        self.publish_hold_target(samples=12)

        # Do not accept messages cached from before the lifecycle transition.
        self.q = None
        self.robot_state = None
        deadline = time.monotonic() + 12.0
        while (self.q is None or self.robot_state is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None or self.robot_state is None:
            raise RuntimeError("robot state unavailable after lifecycle recovery")
        self.gate()

    def publish_hold_target(self, samples=1):
        if self.hold_target is None:
            return
        message = JointState()
        message.name = JOINT_NAMES
        message.position = list(map(float, self.hold_target))
        for _ in range(int(samples)):
            message.header.stamp = self.get_clock().now().to_msg()
            self.hold_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_fresh_state(self, timeout=1.5):
        """Require state messages produced after this method was entered."""
        self.q = None
        self.robot_state = None
        deadline = time.monotonic() + float(timeout)
        while (self.q is None or self.robot_state is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None or self.robot_state is None:
            raise RuntimeError("left state streams stopped")

    def ensure_stable_runtime_after_ptp(self, attempts=3, settle_s=2.0):
        """Recover the hold controller after the PTP server has fully torn down.

        A PTP result can arrive before its controller cleanup.  Recovering the
        impedance controller immediately used to race that cleanup and leave
        the Franka hardware UNCONFIGURED a moment later.  This method waits for
        the late teardown, recovers, then proves that fresh state continues.
        """
        time.sleep(float(settle_s))
        last_error = None
        for attempt in range(1, int(attempts) + 1):
            try:
                # A stopped stream is expected if the late teardown already
                # happened; ensure_runtime_ready() performs lifecycle recovery.
                try:
                    self.wait_for_fresh_state(timeout=0.8)
                except RuntimeError:
                    pass
                self.ensure_runtime_ready()
                self.wait_for_fresh_state(timeout=1.5)
                time.sleep(0.8)
                self.wait_for_fresh_state(timeout=1.5)
                return attempt
            except Exception as exc:
                last_error = exc
                time.sleep(1.0)
        raise RuntimeError(f"stable left runtime recovery failed: {last_error!r}")

    def wait_ready(self):
        # The PTP action server can leave the hardware unconfigured during its
        # teardown.  Every run therefore starts from a known live ROS state.
        self.ensure_runtime_ready()
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("FK service unavailable")
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Cartesian path service unavailable")
        if not self.arm.wait_for_server(timeout_sec=4.0):
            raise RuntimeError("PTP action unavailable")
        pose = self.fk(list(self.q))
        self.lock_z = float(pose.position.z)
        self.lock_orientation = pose.orientation

    def set_impedance(self, active):
        request = SwitchController.Request()
        request.activate_controllers = ["joint_impedance_controller"] if active else []
        request.deactivate_controllers = [] if active else ["joint_impedance_controller"]
        request.strictness = 2
        request.activate_asap = False
        request.timeout.sec = 5
        response = self.call(self.switch_client, request, timeout=8.0)
        if not response.ok:
            raise RuntimeError(
                f"failed to {'activate' if active else 'deactivate'} joint impedance controller"
            )

    def call(self, client, request, timeout=20.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("ROS service timeout")
        return future.result()

    def fk(self, q):
        request = GetPositionFK.Request()
        request.header.frame_id = "base"
        request.fk_link_names = ["left_fr3v2_link8"]
        request.robot_state.joint_state.name = JOINT_NAMES
        request.robot_state.joint_state.position = q
        request.robot_state.is_diff = True
        response = self.call(self.fk_client, request)
        if not response.pose_stamped:
            raise RuntimeError("FK returned no pose")
        return response.pose_stamped[0].pose

    def gate(self):
        if self.robot_state is None:
            raise RuntimeError("robot state lost")
        active = self._active_error_names()
        if not active:
            return

        # Consecutive PTP segments can expose a one-cycle libfranka transition
        # pulse even though the hardware immediately returns to a clean active
        # state.  The old single-sample gate aborted halfway through descent.
        # Debounce briefly, but still reject every persistent error by name.
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.robot_state is None:
                continue
            active = self._active_error_names()
            if not active:
                return
        raise RuntimeError("persistent Franka error: " + ",".join(active))

    def move_ptp(self, q):
        for attempt in range(2):
            self.gate()
            goal = PTPMotion.Goal()
            goal.goal_joint_configuration = q
            goal.maximum_joint_velocities = [self.max_joint_velocity] * 7
            # The action server can remain in RUNNING indefinitely around a
            # 0.002 rad threshold even though the arm is visibly settled.
            # Use a practical action tolerance here; callers still verify the
            # measured joints and final Cartesian pose after completion.
            # The first pass avoids the old 0.002-rad endless wait.  If its
            # measured endpoint is still loose, one bounded precision pass
            # closes the residual instead of silently accumulating it.
            goal.goal_tolerance = 0.006 if attempt == 0 else 0.0025
            future = self.arm.send_goal_async(goal)
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.02)
            handle = future.result()
            if handle is None or not handle.accepted:
                status = "rejected"
            else:
                result_future = handle.get_result_async()
                deadline = time.monotonic() + (75.0 if attempt == 0 else 18.0)
                try:
                    while not result_future.done():
                        rclpy.spin_once(self, timeout_sec=0.02)
                        self.gate()
                        if time.monotonic() >= deadline:
                            cancel_future = handle.cancel_goal_async()
                            rclpy.spin_until_future_complete(
                                self, cancel_future, timeout_sec=5.0
                            )
                            raise RuntimeError("PTP action timeout")
                except KeyboardInterrupt:
                    cancel_future = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                    raise
                result = result_future.result().result
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.03)
                measured_error = max(
                    abs(float(actual) - float(target))
                    for actual, target in zip(self.q, q)
                )
                endpoint_tolerance = 0.004 if attempt == 0 else 0.003
                # The action server sometimes reports ABORTED during its late
                # controller teardown even though measured joints have already
                # reached the target and libfranka is clean.  The measured
                # endpoint plus the persistent-error gate is the authoritative
                # proof; an operator interrupt is handled separately above.
                if measured_error <= endpoint_tolerance:
                    self.gate()
                    self.hold_target = list(map(float, self.q))
                    return
                status = (
                    f"{result.target_status.status}, "
                    f"measured_joint_error={measured_error:.6f}"
                )
            # Controller hand-off can briefly leave libfranka in Idle.  Retry
            # once only when the state remains error-free; gate() aborts any
            # genuine reflex or communication fault.
            if attempt == 0:
                for _ in range(5):
                    rclpy.spin_once(self, timeout_sec=0.04)
                continue
            raise RuntimeError(f"arm motion failed (status={status})")

    def move_xy(self, delta_xy):
        delta_xy = np.asarray(delta_xy, dtype=float)
        if not np.all(np.isfinite(delta_xy)) or np.linalg.norm(delta_xy) > 0.065:
            raise RuntimeError(f"invalid XY step {delta_xy.tolist()}")
        q0 = list(self.q)
        start = self.fk(q0)
        target = Pose()
        target.position.x = start.position.x + float(delta_xy[0])
        target.position.y = start.position.y + float(delta_xy[1])
        target.position.z = self.lock_z
        target.orientation = self.lock_orientation

        request = GetCartesianPath.Request()
        request.header.frame_id = "base"
        request.start_state.joint_state.name = JOINT_NAMES
        request.start_state.joint_state.position = q0
        request.start_state.is_diff = True
        request.group_name = "left_arm"
        request.link_name = "left_fr3v2_link8"
        request.waypoints = [target]
        request.max_step = 0.012
        request.revolute_jump_threshold = 0.12
        request.avoid_collisions = True
        response = self.call(self.cartesian_client, request)
        path = [list(map(float, point.positions)) for point in response.solution.joint_trajectory.points]
        if response.error_code.val != 1 or response.fraction < 0.999 or not path:
            raise RuntimeError(
                f"horizontal path invalid: fraction={response.fraction:.4f}, code={response.error_code.val}"
            )
        for q in path:
            self.move_ptp(q)
        end = self.fk(list(self.q))
        actual = np.asarray(
            [end.position.x - start.position.x, end.position.y - start.position.y],
            dtype=float,
        )
        height_error = float(end.position.z - self.lock_z)
        if abs(height_error) > 0.002:
            raise RuntimeError(f"height drift {height_error:.6f}m")
        measured_q = np.asarray(
            [
                end.orientation.x,
                end.orientation.y,
                end.orientation.z,
                end.orientation.w,
            ],
            dtype=float,
        )
        locked_q = np.asarray(
            [
                self.lock_orientation.x,
                self.lock_orientation.y,
                self.lock_orientation.z,
                self.lock_orientation.w,
            ],
            dtype=float,
        )
        dot = abs(float(np.dot(measured_q / np.linalg.norm(measured_q), locked_q / np.linalg.norm(locked_q))))
        orientation_error_deg = math.degrees(
            2.0 * math.acos(min(1.0, max(-1.0, dot)))
        )
        if orientation_error_deg > 1.5:
            raise RuntimeError(
                f"orientation drift {orientation_error_deg:.3f}deg"
            )
        # Let the eye-in-hand camera finish mechanical settling and expose a
        # sharp frame before the next visual update.
        time.sleep(0.22)
        return actual, np.asarray([end.position.x, end.position.y], dtype=float), end


def detect_point(
    snapshot_url,
    edge="left",
    previous_stamp=None,
    timeout=3.0,
    target_category="cup",
    require_unique_classification=True,
):
    # Keep the ROS motion path importable in a minimal robot-side environment;
    # OpenCV is only required when visual detection actually runs.
    from find_cup_left_edge import (
        ellipse_points,
        load_snapshot,
        propose_rims,
        refine_outer_rim,
    )
    import cv2
    try:
        from three_object_detector import detect_three_objects
    except ImportError:
        detect_three_objects = None

    if target_category != "cup":
        raise RuntimeError("only the cup motion profile is calibrated")
    if require_unique_classification and detect_three_objects is None:
        raise RuntimeError("strict three-object classifier is unavailable")

    def hough_fallback(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2.0)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=80,
            param2=22,
            minRadius=20,
            maxRadius=80,
        )
        if circles is None:
            return None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        yy, xx = np.ogrid[:height, :width]
        candidates = []
        for center_x, center_y, radius in circles[0]:
            if center_y > 0.62 * height or radius < 24:
                continue
            inner = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= (0.62 * radius) ** 2
            if not np.any(inner):
                continue
            saturation = float(np.mean(hsv[:, :, 1][inner]))
            value = float(np.mean(hsv[:, :, 2][inner]))
            if saturation < 55.0 or value < 35.0:
                continue
            score = 2.0 * saturation + 0.35 * value - 0.35 * center_y + 0.1 * radius
            candidates.append((score, float(center_x), float(center_y), float(radius)))
        if not candidates:
            return None
        _, center_x, center_y, radius = max(candidates)
        point = np.asarray(
            [center_x - radius if edge == "left" else center_x + radius, center_y],
            dtype=float,
        )
        return point, 0.72, "hough_color_fallback"

    deadline = time.monotonic() + timeout
    last_error = "no frame"
    observations = []
    last_seen_stamp = -math.inf if previous_stamp is None else float(previous_stamp)
    while time.monotonic() < deadline:
        try:
            sample = load_snapshot("", snapshot_url)
            offset = abs(sample["rgb_stamp"] - sample["depth_stamp"])
            if offset > 0.035:
                last_error = f"RGB-D offset {offset:.6f}s"
                time.sleep(0.03)
                continue
            if sample["rgb_stamp"] <= last_seen_stamp + 1e-6:
                time.sleep(0.03)
                continue
            last_seen_stamp = float(sample["rgb_stamp"])
            bgr = sample["rgb"]
            classified = None
            if detect_three_objects is not None:
                scene = detect_three_objects(bgr)
                matches = [
                    item
                    for item in scene.get("objects", [])
                    if item["category"] == target_category
                    and float(item.get("confidence", 0.0)) >= 0.75
                ]
                # A clear, unique cup must not be rejected merely because the
                # bean bowl or plate has lower confidence.  Global three-class
                # validity remains available as a diagnostic.
                if len(matches) == 1:
                    classified = matches[0]
                if require_unique_classification and classified is None:
                    observations.clear()
                    last_error = "unique three-class RGB scene not valid"
                    time.sleep(0.03)
                    continue
            if classified is not None:
                ellipse_info = classified["rim_ellipse"]
                ellipse = (
                    tuple(ellipse_info["center_px"]),
                    tuple(ellipse_info["diameters_px"]),
                    float(ellipse_info["angle_deg"]),
                )
                rim = ellipse_points(ellipse)
                point = rim[
                    int(np.argmin(rim[:, 0]) if edge == "left" else np.argmax(rim[:, 0]))
                ]
                confidence = float(classified["confidence"])
                method = "unique_target_classification"
            else:
                # RGB cup evidence is the primary fallback.  The previous code
                # referenced an undefined depth_m here, so non-strict mode
                # always crashed instead of degrading gracefully.
                fallback = hough_fallback(bgr)
                if fallback is not None:
                    point, confidence, method = fallback
                else:
                    depth_m = sample["depth"].astype(np.float32) * float(sample["depth_scale_m"])
                    proposals, edges = propose_rims(bgr, depth_m)
                    if not proposals:
                        last_error = "cup rim not found"
                        time.sleep(0.03)
                        continue
                    proposal = proposals[0]
                    ellipse, _, confidence, method = refine_outer_rim(bgr, edges, proposal)
                    rim = ellipse_points(ellipse)
                    point = rim[
                        int(np.argmin(rim[:, 0]) if edge == "left" else np.argmax(rim[:, 0]))
                    ]
            if confidence < 0.65:
                last_error = f"low confidence {confidence:.3f}"
                time.sleep(0.03)
                continue
            observations.append({
                "point": point.astype(float),
                "confidence": float(confidence),
                "stamp": float(sample["rgb_stamp"]),
                "sync_offset": float(offset),
                "method": method,
                "category": target_category if classified is not None else None,
                "classification_valid": classified is not None,
            })
            # Accept only a three-frame spatial cluster.  This rejects the brief
            # inner/outer-rim switch that can occur in the first frame after a move.
            if len(observations) >= 3:
                points = np.asarray([item["point"] for item in observations], dtype=float)
                best_indices = []
                for index, candidate in enumerate(points):
                    indices = np.flatnonzero(np.linalg.norm(points - candidate, axis=1) <= 4.0)
                    if len(indices) > len(best_indices):
                        best_indices = indices.tolist()
                if len(best_indices) >= 3:
                    cluster = [observations[index] for index in best_indices]
                    if require_unique_classification and not all(
                        item["classification_valid"]
                        and item["category"] == target_category
                        and item["method"] == "unique_target_classification"
                        for item in cluster
                    ):
                        observations.clear()
                        last_error = "three consecutive strict classifications required"
                        continue
                    return {
                        "point": np.median(
                            np.asarray([item["point"] for item in cluster]), axis=0
                        ),
                        "confidence": float(np.median([item["confidence"] for item in cluster])),
                        "stamp": float(max(item["stamp"] for item in cluster)),
                        "sync_offset": float(max(item["sync_offset"] for item in cluster)),
                        "method": (
                            "unique_target_classification_3frame"
                            if require_unique_classification
                            else "three_frame_outer_ellipse"
                        ),
                        "category": target_category if require_unique_classification else None,
                        "classification_valid": bool(require_unique_classification),
                    }
            observations = observations[-7:]
            time.sleep(0.025)
        except Exception as exc:
            last_error = repr(exc)
            time.sleep(0.05)
    raise RuntimeError(f"fresh visual detection failed: {last_error}")


def emit(event, **values):
    payload = {"event": event, **values}
    print("SERVO=" + json.dumps(payload, separators=(",", ":")), flush=True)


def cap_norm(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit else vector * (limit / norm)


def adaptive_parameters(error_norm, trust):
    if error_norm > 120:
        gain, limit = 0.55, 0.045
    elif error_norm > 70:
        gain, limit = 0.50, 0.035
    elif error_norm > 35:
        gain, limit = 0.45, 0.020
    elif error_norm > 15:
        gain, limit = 0.40, 0.010
    else:
        gain, limit = 0.33, 0.005
    return gain * trust, limit


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-u", type=float, default=135.0)
    parser.add_argument("--target-v", type=float, default=109.0)
    parser.add_argument("--edge", choices=("left", "right"), default="left")
    parser.add_argument("--probe-m", type=float, default=0.010)
    parser.add_argument("--tolerance-px", type=float, default=5.0)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--near-target-px", type=float, default=8.0)
    parser.add_argument("--skip-impedance-handoff", action="store_true")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:18080/snapshot.npz")
    args = parser.parse_args()

    target = np.asarray([args.target_u, args.target_v], dtype=float)
    rclpy.init()
    node = ServoNode()
    impedance_deactivated = False
    try:
        node.wait_ready()
        initial_pose = node.fk(list(node.q))
        base_origin = np.asarray([initial_pose.position.x, initial_pose.position.y], dtype=float)
        emit("locked", z=node.lock_z, base_xy=base_origin.tolist(), target_px=target.tolist())

        first = detect_point(args.snapshot_url, edge=args.edge)
        p0, stamp = first["point"], first["stamp"]
        emit("observe", stage="initial", point_px=p0.tolist(), confidence=first["confidence"])

        if not args.skip_impedance_handoff:
            node.set_impedance(False)
            impedance_deactivated = True
            emit("controller", joint_impedance="inactive_for_ptp")

        actual_x, base_x, _ = node.move_xy([args.probe_m, 0.0])
        obs_x = detect_point(args.snapshot_url, edge=args.edge, previous_stamp=stamp)
        px, stamp = obs_x["point"], obs_x["stamp"]
        emit("probe", axis="x", actual_m=actual_x.tolist(), point_px=px.tolist())

        actual_to_y, base_y, _ = node.move_xy([-args.probe_m, args.probe_m])
        obs_y = detect_point(args.snapshot_url, edge=args.edge, previous_stamp=stamp)
        py, stamp = obs_y["point"], obs_y["stamp"]
        emit("probe", axis="y", actual_m=actual_to_y.tolist(), point_px=py.tolist())

        displacement_matrix = np.column_stack((base_x - base_origin, base_y - base_origin))
        pixel_matrix = np.column_stack((px - p0, py - p0))
        if abs(float(np.linalg.det(displacement_matrix))) < 2.5e-5:
            raise RuntimeError("probe displacement matrix is singular")
        jacobian = pixel_matrix @ np.linalg.inv(displacement_matrix)
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        condition = float(singular_values[0] / max(singular_values[-1], 1e-9))
        if singular_values[-1] < 80.0 or condition > 25.0:
            raise RuntimeError(
                f"image Jacobian unreliable: singular_values={singular_values.tolist()}, condition={condition:.3f}"
            )
        emit("jacobian", matrix=jacobian.tolist(), condition=condition)
        probe_jacobian = jacobian.copy()

        current_point = py
        current_base = base_y
        trust = 0.70
        converged = False
        for iteration in range(1, args.max_iterations + 1):
            error = target - current_point
            error_norm = float(np.linalg.norm(error))
            max_axis_error = float(np.max(np.abs(error)))
            if max_axis_error <= args.tolerance_px:
                confirm = detect_point(args.snapshot_url, edge=args.edge, previous_stamp=stamp)
                stamp = confirm["stamp"]
                confirm_error = target - confirm["point"]
                if float(np.max(np.abs(confirm_error))) <= args.tolerance_px + 1.0:
                    current_point = confirm["point"]
                    converged = True
                    emit("converged", iteration=iteration - 1, point_px=current_point.tolist(), error_px=confirm_error.tolist())
                    break
            gain, step_limit = adaptive_parameters(error_norm, trust)
            raw_step = np.linalg.pinv(jacobian, rcond=0.06) @ error
            step = cap_norm(raw_step * gain, step_limit)
            if not np.all(np.isfinite(step)):
                jacobian = probe_jacobian.copy()
                raw_step = np.linalg.pinv(jacobian, rcond=0.06) @ error
                step = cap_norm(raw_step * gain, step_limit)
            step_norm = float(np.linalg.norm(step))
            if step_norm < 0.0007:
                if max_axis_error <= args.near_target_px:
                    converged = True
                    emit("resolution_limited", point_px=current_point.tolist(), error_px=error.tolist(), requested_step_m=step.tolist())
                    break
                jacobian = probe_jacobian.copy()
                raw_step = np.linalg.pinv(jacobian, rcond=0.06) @ error
                step = cap_norm(raw_step * gain, step_limit)
                step_norm = float(np.linalg.norm(step))
                if step_norm > 1e-9 and step_norm < 0.0007:
                    step = step * (0.0007 / step_norm)
                elif step_norm <= 1e-9:
                    raise RuntimeError("visual correction unavailable after Jacobian reset")
                emit("jacobian_reset", iteration=iteration, reason="tiny_step")
            predicted_base = current_base + step
            if np.linalg.norm(predicted_base - base_origin) > 0.24:
                raise RuntimeError("horizontal search exceeded 0.24m radius")

            old_error_norm = error_norm
            actual_step, new_base, end_pose = node.move_xy(step)
            observation = detect_point(args.snapshot_url, edge=args.edge, previous_stamp=stamp)
            stamp = observation["stamp"]
            new_point = observation["point"]
            observed_pixel_step = new_point - current_point
            denominator = float(actual_step @ actual_step)
            if denominator > 2.25e-6 and float(np.linalg.norm(observed_pixel_step)) > 2.5:
                residual = observed_pixel_step - jacobian @ actual_step
                candidate = jacobian + np.outer(residual, actual_step) / denominator
                candidate = 0.75 * jacobian + 0.25 * candidate
                candidate_singular = np.linalg.svd(candidate, compute_uv=False)
                candidate_condition = float(candidate_singular[0] / max(candidate_singular[-1], 1e-9))
                if candidate_singular[-1] >= 60.0 and candidate_condition <= 20.0:
                    jacobian = candidate
                else:
                    emit("jacobian_update_ignored", iteration=iteration, condition=candidate_condition)
            new_error_norm = float(np.linalg.norm(target - new_point))
            improved = new_error_norm < old_error_norm
            trust = min(1.0, trust * 1.12) if improved else max(0.25, trust * 0.48)
            emit(
                "correct",
                iteration=iteration,
                command_m=step.tolist(),
                actual_m=actual_step.tolist(),
                point_px=new_point.tolist(),
                error_px=(target - new_point).tolist(),
                error_norm=new_error_norm,
                trust=trust,
                z=float(end_pose.position.z),
            )
            current_point, current_base = new_point, new_base

        if not converged:
            final_error = target - current_point
            if float(np.max(np.abs(final_error))) <= args.near_target_px:
                converged = True
                emit("near_target", point_px=current_point.tolist(), error_px=final_error.tolist())
            else:
                raise RuntimeError(f"did not converge; final error {final_error.tolist()}")

        final_pose = node.fk(list(node.q))
        emit(
            "done",
            point_px=current_point.tolist(),
            error_px=(target - current_point).tolist(),
            base_xy=[float(final_pose.position.x), float(final_pose.position.y)],
            z=float(final_pose.position.z),
            horizontal_displacement_m=[
                float(final_pose.position.x - initial_pose.position.x),
                float(final_pose.position.y - initial_pose.position.y),
            ],
            gripper_commanded=False,
            vertical_motion_commanded=False,
        )
    except Exception as exc:
        emit("abort", reason=repr(exc))
        raise
    finally:
        if impedance_deactivated:
            try:
                node.set_impedance(True)
                emit("controller", joint_impedance="restored")
            except Exception as exc:
                emit("controller_restore_failed", reason=repr(exc))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
