#!/usr/bin/env python3
"""Sequentially restore both FR3 arms to the unified pick-start posture.

The left target is the calibrated successful pick-top pose.  The right target
is the verified grasp initial pose.  No gripper command is sent by this script.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from franka_msgs.action import PTPMotion
from franka_msgs.msg import FrankaRobotState
from lifecycle_msgs.msg import State
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_HOLD_TOLERANCE_RAD = 0.018


def emit(event: str, **values) -> None:
    print("DUAL_INIT=" + json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


class ArmInitializer(Node):
    def __init__(self, arm: str, target: list[float], maximum_velocity: float) -> None:
        super().__init__(f"{arm}_pick_pose_initializer")
        self.arm_name = arm
        self.target = np.asarray(target, dtype=float)
        self.maximum_velocity = float(maximum_velocity)
        self.joint_names = [f"{arm}_fr3v2_joint{index}" for index in range(1, 8)]
        self.q = None
        self.robot_state = None
        self.hold_target = None
        self.hold_pub = self.create_publisher(
            JointState, f"/{arm}/gello/joint_states", 1
        )
        self.create_subscription(
            JointState,
            f"/{arm}/franka_robot_state_broadcaster/measured_joint_states",
            self._on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            FrankaRobotState,
            f"/{arm}/franka_robot_state_broadcaster/robot_state",
            self._on_robot_state,
            qos_profile_sensor_data,
        )
        self.switch_client = self.create_client(
            SwitchController, f"/{arm}/controller_manager/switch_controller"
        )
        self.hardware_client = self.create_client(
            SetHardwareComponentState,
            f"/{arm}/controller_manager/set_hardware_component_state",
        )
        self.action = ActionClient(self, PTPMotion, f"/{arm}/action_server/ptp_motion")

    def _on_joints(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.joint_names):
            self.q = [float(positions[name]) for name in self.joint_names]

    def _on_robot_state(self, message: FrankaRobotState) -> None:
        self.robot_state = message

    def call(self, client, request, timeout_s: float = 10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{self.arm_name} ROS service timeout")
        return future.result()

    def _set_hardware_active(self):
        request = SetHardwareComponentState.Request()
        request.name = f"{self.arm_name}_FrankaHardwareInterface"
        request.target_state = State()
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        request.target_state.label = "active"
        return self.call(self.hardware_client, request, timeout_s=12.0)

    def wait_fresh_state(self, timeout_s: float = 1.5) -> None:
        self.q = None
        self.robot_state = None
        deadline = time.monotonic() + timeout_s
        while (self.q is None or self.robot_state is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.q is None or self.robot_state is None:
            raise RuntimeError(f"{self.arm_name} state streams unavailable")

    def ensure_runtime_ready(self) -> None:
        if not self.hardware_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError(f"{self.arm_name} hardware lifecycle service unavailable")
        if not self.switch_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError(f"{self.arm_name} controller switch service unavailable")
        try:
            self.wait_fresh_state(timeout_s=1.0)
        except RuntimeError:
            response = self._set_hardware_active()
            if response.state.id != State.PRIMARY_STATE_ACTIVE:
                response = self._set_hardware_active()
            if response.state.id != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(f"{self.arm_name} hardware activation failed")
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
        if not self.call(self.switch_client, request, timeout_s=8.0).ok:
            raise RuntimeError(f"{self.arm_name} runtime controller recovery failed")
        self.publish_hold_target(samples=12)
        self.wait_fresh_state(timeout_s=12.0)

    def publish_hold_target(self, samples: int = 1) -> None:
        if self.hold_target is None:
            return
        message = JointState()
        message.name = self.joint_names
        message.position = list(map(float, self.hold_target))
        for _ in range(int(samples)):
            message.header.stamp = self.get_clock().now().to_msg()
            self.hold_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_impedance(self, active: bool) -> None:
        request = SwitchController.Request()
        request.activate_controllers = ["joint_impedance_controller"] if active else []
        request.deactivate_controllers = [] if active else ["joint_impedance_controller"]
        request.strictness = 2
        request.activate_asap = False
        request.timeout.sec = 5
        if not self.call(self.switch_client, request, timeout_s=8.0).ok:
            raise RuntimeError(f"{self.arm_name} impedance switch failed")

    def gate(self) -> None:
        if self.robot_state is None:
            raise RuntimeError(f"{self.arm_name} robot state lost")
        fields = self.robot_state.current_errors.get_fields_and_field_types()
        active = [name for name in fields if bool(getattr(self.robot_state.current_errors, name))]
        if not active:
            return
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.robot_state is not None:
                active = [
                    name
                    for name in fields
                    if bool(getattr(self.robot_state.current_errors, name))
                ]
                if not active:
                    return
        raise RuntimeError(f"persistent {self.arm_name} Franka error: " + ",".join(active))

    def move(self) -> dict:
        self.ensure_runtime_ready()
        if not self.action.wait_for_server(timeout_sec=6.0):
            raise RuntimeError(f"{self.arm_name} PTP action unavailable")
        assert self.q is not None
        start = np.asarray(self.q, dtype=float)
        start_error = float(np.max(np.abs(start - self.target)))
        if start_error <= 0.012:
            emit("already_at_target", arm=self.arm_name, maximum_error_rad=start_error)
            return self._stable_report(start, moved=False)

        self.gate()
        impedance_off = False
        try:
            self.set_impedance(False)
            impedance_off = True
            time.sleep(0.35)
            goal = PTPMotion.Goal()
            goal.goal_joint_configuration = self.target.tolist()
            goal.maximum_joint_velocities = [self.maximum_velocity] * 7
            goal.goal_tolerance = 0.006
            future = self.action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{self.arm_name} PTP goal rejected")
            result_future = handle.get_result_async()
            deadline = time.monotonic() + 45.0
            while not result_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)
                self.gate()
            if not result_future.done():
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            for _ in range(8):
                rclpy.spin_once(self, timeout_sec=0.04)
            if self.q is None:
                raise RuntimeError(f"{self.arm_name} measured endpoint unavailable")
            measured_error = float(np.max(np.abs(np.asarray(self.q) - self.target)))
            if measured_error > JOINT_HOLD_TOLERANCE_RAD:
                raise RuntimeError(
                    f"{self.arm_name} PTP endpoint error {measured_error:.6f} rad"
                )
            self.hold_target = list(map(float, self.q))
            # PTP reports before controller teardown is always complete.
            time.sleep(2.0)
            last_error = None
            for attempt in range(1, 4):
                try:
                    self.ensure_runtime_ready()
                    self.wait_fresh_state(timeout_s=1.5)
                    time.sleep(0.8)
                    self.wait_fresh_state(timeout_s=1.5)
                    impedance_off = False
                    report = self._stable_report(start, moved=True)
                    report["stable_hold_recovery_attempt"] = attempt
                    return report
                except Exception as exc:
                    last_error = exc
                    time.sleep(1.0)
            raise RuntimeError(f"{self.arm_name} stable hold recovery failed: {last_error!r}")
        finally:
            if impedance_off:
                try:
                    self.ensure_runtime_ready()
                except Exception as exc:
                    emit("hold_restore_failed", arm=self.arm_name, error=repr(exc))

    def _stable_report(self, start: np.ndarray, moved: bool) -> dict:
        self.wait_fresh_state(timeout_s=1.5)
        assert self.q is not None
        measured = np.asarray(self.q, dtype=float)
        maximum_error = float(np.max(np.abs(measured - self.target)))
        if maximum_error > JOINT_HOLD_TOLERANCE_RAD:
            raise RuntimeError(
                f"{self.arm_name} stable hold error {maximum_error:.6f} rad"
            )
        return {
            "arm": self.arm_name,
            "moved": moved,
            "start_joint_positions_rad": start.tolist(),
            "target_joint_positions_rad": self.target.tolist(),
            "measured_joint_positions_rad": measured.tolist(),
            "maximum_joint_error_rad": maximum_error,
            "stable_hold": True,
        }


def load_targets(path: Path) -> dict[str, list[float]]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    targets = {}
    for arm in ("left", "right"):
        positions = [float(item) for item in value[arm]["positions"]]
        if len(positions) != 7 or not all(math.isfinite(item) for item in positions):
            raise RuntimeError(f"invalid {arm} target in {path}")
        targets[arm] = positions
    return targets


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "grasp_initial_state.yaml",
    )
    parser.add_argument("--maximum-velocity", type=float, default=0.06)
    args = parser.parse_args()
    targets = load_targets(args.config)
    summary = {
        "status": "dry_run",
        "order": ["left", "right"],
        "targets": targets,
        "gripper_commanded": False,
    }
    if not args.execute:
        print(json.dumps(summary, indent=2), flush=True)
        return 0

    rclpy.init()
    reports = []
    try:
        for arm_name in ("left", "right"):
            emit("arm_start", arm=arm_name)
            node = ArmInitializer(arm_name, targets[arm_name], args.maximum_velocity)
            try:
                report = node.move()
                reports.append(report)
                emit("arm_complete", **report)
            finally:
                node.destroy_node()
        result = {
            "status": "success",
            "order": ["left", "right"],
            "reports": reports,
            "both_stable_hold": all(item["stable_hold"] for item in reports),
            "gripper_commanded": False,
        }
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc), "reports": reports}, indent=2), flush=True)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
