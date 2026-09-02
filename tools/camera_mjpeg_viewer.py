#!/usr/bin/env python3
import argparse
import json
import threading
import time
import uuid
from io import BytesIO
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


CAMERA_ROLE = "left_wrist"
RGB_TOPIC = "/wrist_camera_left/color/image_raw"
DEPTH_TOPIC = "/wrist_camera_left/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/wrist_camera_left/color/camera_info"
VIEWER_TITLE = "Left Wrist D405"
MAX_RGB_FRAME_AGE_S = 0.75
MAX_DEPTH_FRAME_AGE_S = 1.50


class CameraStreams(Node):
    def __init__(self):
        super().__init__(f"temporary_camera_mjpeg_viewer_{CAMERA_ROLE}")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {"rgb": None, "depth": None}
        self.raw = {"rgb": None, "depth": None, "camera_k": None}
        self.stamps = {"rgb": 0.0, "depth": 0.0}
        self.received_monotonic = {"rgb": 0.0, "depth": 0.0}
        self.sequences = {"rgb": 0, "depth": 0}
        self.rgb_frame_id = ""
        self.session_id = uuid.uuid4().hex
        self.create_subscription(
            Image,
            RGB_TOPIC,
            self.on_rgb,
            qos_profile_sensor_data,
        )
        if DEPTH_TOPIC:
            self.create_subscription(
                Image,
                DEPTH_TOPIC,
                self.on_depth,
                qos_profile_sensor_data,
            )
        if CAMERA_INFO_TOPIC:
            self.create_subscription(
                CameraInfo,
                CAMERA_INFO_TOPIC,
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
            self.received_monotonic["rgb"] = time.monotonic()
            self.sequences["rgb"] += 1
            self.rgb_frame_id = str(msg.header.frame_id)
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
            self.received_monotonic["depth"] = time.monotonic()
            self.sequences["depth"] += 1
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
        path = urlsplit(self.path).path
        if path == "/healthz":
            now = time.monotonic()
            with self.streams.lock:
                updated = self.streams.received_monotonic["rgb"]
                sequence = self.streams.sequences["rgb"]
                frame_id = self.streams.rgb_frame_id
                session_id = self.streams.session_id
            age = None if updated == 0.0 else now - updated
            healthy = age is not None and age <= MAX_RGB_FRAME_AGE_S
            body = json.dumps(
                {
                    "status": "ok" if healthy else "stale",
                    "camera_role": CAMERA_ROLE,
                    "rgb_topic": RGB_TOPIC,
                    "rgb_frame_id": frame_id,
                    "camera_session_id": session_id,
                    "rgb_sequence": sequence,
                    "rgb_age_s": None if age is None else round(age, 4),
                    "maximum_rgb_age_s": MAX_RGB_FRAME_AGE_S,
                },
                separators=(",", ":"),
            ).encode()
            self.send_response(200 if healthy else 503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/snapshot.npz":
            with self.streams.lock:
                rgb = None if self.streams.raw["rgb"] is None else self.streams.raw["rgb"].copy()
                depth = None if self.streams.raw["depth"] is None else self.streams.raw["depth"].copy()
                camera_k = None if self.streams.raw["camera_k"] is None else self.streams.raw["camera_k"].copy()
                rgb_stamp = self.streams.stamps["rgb"]
                rgb_updated = self.streams.received_monotonic["rgb"]
                rgb_age_s = None if rgb_updated == 0.0 else time.monotonic() - rgb_updated
                rgb_sequence = self.streams.sequences["rgb"]
                depth_stamp = self.streams.stamps["depth"]
                rgb_frame_id = self.streams.rgb_frame_id
                session_id = self.streams.session_id
            if rgb is None:
                self.send_error(503, "RGB snapshot is not ready")
                return
            if rgb_age_s is None or rgb_age_s > MAX_RGB_FRAME_AGE_S:
                age_text = "unavailable" if rgb_age_s is None else f"{rgb_age_s:.3f}s old"
                self.send_error(503, f"RGB frame is stale ({age_text})")
                return
            payload = BytesIO()
            snapshot = {
                "rgb": rgb,
                "rgb_stamp": np.asarray(rgb_stamp),
                "rgb_age_s": np.asarray(rgb_age_s),
                "rgb_sequence": np.asarray(rgb_sequence, dtype=np.int64),
                # These scalar strings let the pick process reject a stale
                # viewer, a ZED/right-camera viewer, or a viewer restart in the
                # middle of visual servoing.
                "camera_role": np.asarray(CAMERA_ROLE),
                "rgb_topic": np.asarray(RGB_TOPIC),
                "rgb_frame_id": np.asarray(rgb_frame_id),
                "camera_session_id": np.asarray(session_id),
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
        if path in ("/", "/index.html"):
            depth_card = "" if not DEPTH_TOPIC else """
<div class=card><h3>Depth (0–2 m pseudo-color)</h3><img src=/depth.mjpg><p>Black = invalid; warm/cool colors indicate distance.</p></div>"""
            body = f"""<!doctype html><meta charset=utf-8>
<title>{VIEWER_TITLE} Live</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}h2{margin:14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}.card{background:#222;padding:10px;border-radius:8px}
img{width:100%;height:auto;background:#000}p{color:#aaa;margin:8px 0 0}.stale{opacity:.18}</style>
<h2>{VIEWER_TITLE} — Live</h2><div class=grid>
<div class=card><h3>RGB</h3><img id=rgb data-src=/rgb.mjpg src=/rgb.mjpg><p id=status>Waiting for a fresh frame…</p></div>
{depth_card}
</div><script>
let wasHealthy=false;
async function monitor(){{
  const image=document.getElementById('rgb'), label=document.getElementById('status');
  try{{
    const response=await fetch('/healthz?ts='+Date.now(),{{cache:'no-store'}});
    const value=await response.json(), healthy=response.ok && value.status==='ok';
    image.classList.toggle('stale',!healthy);
    label.textContent=healthy ? `LIVE · ${{value.rgb_age_s}} s · #${{value.rgb_sequence}}` : 'STALE — reconnecting';
    if(healthy && !wasHealthy) image.src=image.dataset.src+'?ts='+Date.now();
    wasHealthy=healthy;
  }}catch(error){{ image.classList.add('stale'); label.textContent='DISCONNECTED — reconnecting'; wasHealthy=false; }}
}}
setInterval(monitor,500); monitor();
</script>""".encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        names = {"/rgb.mjpg": "rgb", "/depth.mjpg": "depth"}
        if path not in names:
            self.send_error(404)
            return
        name = names[path]
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            last_sequence = -1
            while True:
                with self.streams.lock:
                    frame = self.streams.frames[name]
                    updated = self.streams.received_monotonic[name]
                    sequence = self.streams.sequences[name]
                maximum_age = MAX_RGB_FRAME_AGE_S if name == "rgb" else MAX_DEPTH_FRAME_AGE_S
                age = None if updated == 0.0 else time.monotonic() - updated
                if age is not None and age > maximum_age:
                    break
                if frame and age is not None and sequence != last_sequence:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
                    self.wfile.flush()
                    last_sequence = sequence
                time.sleep(1 / 50)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    global CAMERA_ROLE, RGB_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC, VIEWER_TITLE
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--camera-role", default=CAMERA_ROLE)
    parser.add_argument("--rgb-topic", default=RGB_TOPIC)
    parser.add_argument("--depth-topic", default=DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=CAMERA_INFO_TOPIC)
    parser.add_argument("--title", default=VIEWER_TITLE)
    args = parser.parse_args()
    CAMERA_ROLE = args.camera_role
    RGB_TOPIC = args.rgb_topic
    DEPTH_TOPIC = args.depth_topic
    CAMERA_INFO_TOPIC = args.camera_info_topic
    VIEWER_TITLE = args.title
    rclpy.init()
    streams = CameraStreams()
    Handler.streams = streams
    threading.Thread(target=rclpy.spin, args=(streams,), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Camera viewer listening on http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        streams.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
