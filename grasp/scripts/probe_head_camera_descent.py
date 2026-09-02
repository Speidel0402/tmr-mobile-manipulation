#!/usr/bin/env python3
"""Capture ZED RGB before/after a bounded vertical probe, then restore top."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from run_streamed_live_pick_cycle import (
    ServoNode,
    command_gripper,
    move_vertical,
    read_settled_pose,
    validate_top,
)


TOPIC = "/head_camera/zed/rgb/color/rect/image/compressed"


class HeadCapture(Node):
    def __init__(self) -> None:
        super().__init__("head_camera_descent_probe_capture")
        self.frame: np.ndarray | None = None
        self.stamp_ns = 0
        self.create_subscription(
            CompressedImage, TOPIC, self._on_image, qos_profile_sensor_data
        )

    def _on_image(self, message: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if image is not None:
            self.frame = image
            self.stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )

    def fresh_frame(self, previous_stamp_ns: int = 0, timeout_s: float = 6.0) -> tuple[np.ndarray, int]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.frame is not None and self.stamp_ns > previous_stamp_ns:
                return self.frame.copy(), self.stamp_ns
        raise RuntimeError("fresh head-camera RGB frame unavailable")


def emit(event: str, **values) -> None:
    print(
        "HEAD_Z_PROBE="
        + json.dumps({"event": event, **values}, separators=(",", ":")),
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-down-m", type=float, default=0.18)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/head_z_probe"))
    args = parser.parse_args()
    if not 0.05 <= args.probe_down_m <= 0.23:
        raise SystemExit("probe distance must remain in [0.05, 0.23] m")

    rclpy.init()
    arm = ServoNode()
    gripper = Node("head_camera_descent_probe_gripper")
    camera = HeadCapture()
    impedance_off = False
    top_z: float | None = None
    low_z: float | None = None
    stamp_ns = 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    try:
        arm.wait_ready()
        opened = command_gripper(gripper, 0.0, "open_for_head_z_probe")
        if opened["position"] > 0.05:
            raise RuntimeError("gripper did not open for head-camera probe")

        arm.set_impedance(False)
        impedance_off = True
        top_pose = read_settled_pose(arm)
        validate_top(top_pose)
        top_z = float(top_pose.position.z)
        top, stamp_ns = camera.fresh_frame()
        top_path = args.output_dir / "top.jpg"
        if not cv2.imwrite(str(top_path), top):
            raise RuntimeError("failed to save top frame")

        low_pose, actual_down = move_vertical(arm, -args.probe_down_m)
        low_z = float(low_pose.position.z)
        time.sleep(0.20)
        low, stamp_ns = camera.fresh_frame(stamp_ns)
        low_path = args.output_dir / "low.jpg"
        if not cv2.imwrite(str(low_path), low):
            raise RuntimeError("failed to save low frame")

        restored, actual_up = move_vertical(arm, top_z - low_z)
        emit(
            "complete",
            requested_down_m=-args.probe_down_m,
            actual_down_m=actual_down,
            actual_up_m=actual_up,
            top_z_m=top_z,
            low_z_m=low_z,
            restored_z_m=float(restored.position.z),
            top_image=str(top_path),
            low_image=str(low_path),
        )
        return 0
    finally:
        if impedance_off:
            if top_z is not None and low_z is not None:
                current = arm.fk(list(arm.q))
                recovery = top_z - float(current.position.z)
                if 0.001 < recovery <= 0.24:
                    move_vertical(arm, recovery)
            arm.ensure_stable_runtime_after_ptp()
        camera.destroy_node()
        gripper.destroy_node()
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
