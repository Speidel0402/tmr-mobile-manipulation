#!/usr/bin/env python3
"""Publish pipeline-compatible ROS RGB topics from V4L2 wrists and HTTP ZED."""

from __future__ import annotations

import argparse
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraBridge(Node):
    def __init__(self, left_device: str, right_device: str, main_url: str) -> None:
        super().__init__("tmr_pipeline_camera_bridge")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {"left": None, "right": None}
        self.main_jpeg: bytes | None = None
        self.left_pub = self.create_publisher(
            Image, "/wrist_camera_left/color/image_raw", qos_profile_sensor_data
        )
        self.right_pub = self.create_publisher(
            Image, "/wrist_camera_right/color/image_raw", qos_profile_sensor_data
        )
        self.main_pub = self.create_publisher(
            CompressedImage,
            "/head_camera/zed/rgb/color/rect/image/compressed",
            qos_profile_sensor_data,
        )
        threading.Thread(target=self.capture, args=("left", left_device), daemon=True).start()
        threading.Thread(target=self.capture, args=("right", right_device), daemon=True).start()
        threading.Thread(target=self.fetch_main, args=(main_url,), daemon=True).start()
        self.create_timer(1.0 / 15.0, self.publish)

    def capture(self, name: str, device: str) -> None:
        while rclpy.ok():
            camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_FPS, 15)
            # V4L2 auto white balance was observed to swing the same cup from
            # HSV hue 18 to 108 within seconds.  Lock both D405 streams to the
            # device's calibrated neutral setting so display and detection do
            # not change while the scene is stationary.
            camera.set(cv2.CAP_PROP_AUTO_WB, 0.0)
            camera.set(cv2.CAP_PROP_WB_TEMPERATURE, 4600.0)
            if not camera.isOpened():
                camera.release()
                time.sleep(1.0)
                continue
            while rclpy.ok():
                ok, frame = camera.read()
                if not ok:
                    break
                # The D405 V4L2 YUYV conversion on this host returns RGB
                # channel order even though OpenCV normally advertises BGR.
                # Normalize it before publishing a correctly labelled bgr8
                # ROS image; otherwise neutral surfaces appear cyan/blue.
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # Direct V4L2 capture bypasses the RealSense ISP. Fixed gains
                # and saturation measured against the head ZED keep both wrist
                # cameras visually consistent without scene-dependent drift.
                frame = np.clip(
                    frame.astype(np.float32)
                    * np.asarray((1.566, 0.99, 0.770), dtype=np.float32),
                    0,
                    255,
                ).astype(np.uint8)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                hsv[:, :, 1] = np.clip(
                    hsv[:, :, 1].astype(np.float32) * 0.70, 0, 255
                ).astype(np.uint8)
                frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                with self.lock:
                    self.frames[name] = frame
            camera.release()
            time.sleep(0.5)

    def fetch_main(self, url: str) -> None:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while rclpy.ok():
            try:
                with opener.open(url, timeout=1.5) as response:
                    payload = response.read(8_000_001)
                if payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9"):
                    with self.lock:
                        self.main_jpeg = payload
            except Exception:
                pass
            time.sleep(0.05)

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            left = None if self.frames["left"] is None else self.frames["left"].copy()
            right = None if self.frames["right"] is None else self.frames["right"].copy()
            main = self.main_jpeg
        for frame, publisher, frame_id in (
            (left, self.left_pub, "wrist_camera_left_color_optical_frame"),
            (right, self.right_pub, "wrist_camera_right_color_optical_frame"),
        ):
            if frame is not None:
                message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                message.header.stamp = stamp
                message.header.frame_id = frame_id
                publisher.publish(message)
        if main is not None:
            message = CompressedImage()
            message.header.stamp = stamp
            message.header.frame_id = "head_camera_left_camera_optical_frame"
            message.format = "jpeg"
            message.data = main
            self.main_pub.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-device", default="/dev/video20")
    parser.add_argument("--right-device", default="/dev/video14")
    parser.add_argument("--main-url", default="http://172.16.0.50:18082/tmr_zed_latest.jpg")
    args = parser.parse_args()
    rclpy.init()
    node = CameraBridge(args.left_device, args.right_device, args.main_url)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
