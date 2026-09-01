#!/usr/bin/env python3
import threading
import time
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraStreams(Node):
    def __init__(self):
        super().__init__("temporary_camera_mjpeg_viewer")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {"rgb": None, "depth": None}
        self.raw = {"rgb": None, "depth": None, "camera_k": None}
        self.stamps = {"rgb": 0.0, "depth": 0.0}
        self.create_subscription(
            Image,
            "/wrist_camera_left/color/image_raw",
            self.on_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/wrist_camera_left/aligned_depth_to_color/image_raw",
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/wrist_camera_left/color/camera_info",
            self.on_camera_info,
            qos_profile_sensor_data,
        )

    def _store_jpeg(self, name, image):
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            with self.lock:
                self.frames[name] = encoded.tobytes()

    def on_rgb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.raw["rgb"] = image.copy()
            self.stamps["rgb"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self._store_jpeg("rgb", image)

    def on_depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        if depth.dtype == np.float32:
            depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0) * 1000.0
        else:
            depth_mm = depth.astype(np.float32)
        with self.lock:
            self.raw["depth"] = depth.copy()
            self.stamps["depth"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        scaled = np.clip(depth_mm, 0, 2000) * (255.0 / 2000.0)
        colored = cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)
        colored[depth_mm <= 0] = 0
        self._store_jpeg("depth", colored)

    def on_camera_info(self, msg):
        with self.lock:
            self.raw["camera_k"] = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)


class Handler(BaseHTTPRequestHandler):
    streams = None

    def log_message(self, *_args):
        pass

    def do_GET(self):
        if self.path == "/snapshot.npz":
            with self.streams.lock:
                rgb = None if self.streams.raw["rgb"] is None else self.streams.raw["rgb"].copy()
                depth = None if self.streams.raw["depth"] is None else self.streams.raw["depth"].copy()
                camera_k = None if self.streams.raw["camera_k"] is None else self.streams.raw["camera_k"].copy()
                rgb_stamp = self.streams.stamps["rgb"]
                depth_stamp = self.streams.stamps["depth"]
            if rgb is None:
                self.send_error(503, "RGB snapshot is not ready")
                return
            payload = BytesIO()
            snapshot = {
                "rgb": rgb,
                "rgb_stamp": np.asarray(rgb_stamp),
            }
            # Depth is optional for the competition RGB edge-servo path.  Keep
            # it when available for offline tools, but never block a grasp on
            # a disabled or late depth stream.
            if depth is not None:
                snapshot["depth"] = depth
                snapshot["depth_stamp"] = np.asarray(depth_stamp)
                snapshot["depth_scale_m"] = np.asarray(
                    1.0 if depth.dtype == np.float32 else 0.001
                )
            if camera_k is not None:
                snapshot["camera_k"] = camera_k
            np.savez_compressed(payload, **snapshot)
            body = payload.getvalue()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ("/", "/index.html"):
            body = """<!doctype html><meta charset=utf-8>
<title>Wrist Camera Live</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}h2{margin:14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}.card{background:#222;padding:10px;border-radius:8px}
img{width:100%;height:auto;background:#000}p{color:#aaa;margin:8px 0 0}</style>
<h2>Left Wrist D405 — Live</h2><div class=grid>
<div class=card><h3>RGB</h3><img src=/rgb.mjpg></div>
<div class=card><h3>Depth (0–2 m pseudo-color)</h3><img src=/depth.mjpg><p>Black = invalid; warm/cool colors indicate distance.</p></div>
</div>""".encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        names = {"/rgb.mjpg": "rgb", "/depth.mjpg": "depth"}
        if self.path not in names:
            self.send_error(404)
            return
        name = names[self.path]
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while True:
                with self.streams.lock:
                    frame = self.streams.frames[name]
                if frame:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
                time.sleep(1 / 20)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    rclpy.init()
    streams = CameraStreams()
    Handler.streams = streams
    threading.Thread(target=rclpy.spin, args=(streams,), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", 18080), Handler)
    print("Camera viewer listening on http://127.0.0.1:18080", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        streams.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
