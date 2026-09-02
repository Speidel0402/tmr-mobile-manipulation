#!/usr/bin/env python3
"""Serve head ZED plus both wrist RGB streams without mixing DDS domains.

The wrist cameras remain ROS subscribers in the arm/control DDS domain.  The
head ZED frame is fetched from the base computer's tiny HTTP JPEG exporter,
which is fed independently from the ZED/vision DDS domain.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WRIST_TOPICS = {
    "left": "/wrist_camera_left/color/image_raw",
    "right": "/wrist_camera_right/color/image_raw",
}
MAX_FRAME_AGE_S = {"main": 1.50, "left": 0.75, "right": 0.75}


class Streams(Node):
    def __init__(self, main_url: str, main_period_s: float) -> None:
        super().__init__("tmr_three_camera_web_viewer")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = {"main": None, "left": None, "right": None}
        self.updated_monotonic = {"main": 0.0, "left": 0.0, "right": 0.0}
        self.sequences = {"main": 0, "left": 0, "right": 0}
        self.main_url = main_url
        self.main_period_s = main_period_s
        self.main_source_marker = None
        for name, topic in WRIST_TOPICS.items():
            self.create_subscription(
                Image,
                topic,
                lambda message, stream=name: self.on_wrist(stream, message),
                qos_profile_sensor_data,
            )
        threading.Thread(target=self.poll_main, daemon=True).start()

    def on_wrist(self, name: str, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            with self.lock:
                self.frames[name] = encoded.tobytes()
                self.updated_monotonic[name] = time.monotonic()
                self.sequences[name] += 1

    def poll_main(self) -> None:
        # This fetcher is deliberately independent from the wrist ROS
        # context.  A DDS restart must not terminate the head-camera bridge.
        while True:
            started = time.monotonic()
            try:
                with urllib.request.urlopen(self.main_url, timeout=1.0) as response:
                    source_marker = response.headers.get("Last-Modified")
                    payload = response.read(8_000_001)
                if (
                    16 <= len(payload) <= 8_000_000
                    and payload.startswith(b"\xff\xd8")
                    and payload.endswith(b"\xff\xd9")
                    and source_marker
                    and source_marker != self.main_source_marker
                ):
                    with self.lock:
                        self.frames["main"] = payload
                        self.updated_monotonic["main"] = time.monotonic()
                        self.sequences["main"] += 1
                        self.main_source_marker = source_marker
            except Exception:
                pass
            remaining = self.main_period_s - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


class Handler(BaseHTTPRequestHandler):
    streams: Streams

    def log_message(self, *_args) -> None:
        pass

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            body = """<!doctype html><meta charset=utf-8>
<title>TMR Camera Live</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}h2{margin:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px;padding:12px}.card{background:#222;padding:10px;border-radius:8px}img{width:100%;height:auto;background:#000}.stale{opacity:.18}p{color:#aaa}</style>
<h2>TMR Cameras - Live</h2><div class=grid>
<div class=card><h3>Main ZED RGB</h3><img id=main data-src=/main.mjpg src=/main.mjpg><p id=main-status>Waiting…</p></div>
<div class=card><h3>Left Wrist D405 RGB</h3><img id=left data-src=/left.mjpg src=/left.mjpg><p id=left-status>Waiting…</p></div>
<div class=card><h3>Right Wrist D405 RGB</h3><img id=right data-src=/right.mjpg src=/right.mjpg><p id=right-status>Waiting…</p></div>
</div><script>
const previous={main:false,left:false,right:false};
async function monitor(){
  try{
    const response=await fetch('/status.json?ts='+Date.now(),{cache:'no-store'}), value=await response.json();
    for(const name of ['main','left','right']){
      const image=document.getElementById(name), label=document.getElementById(name+'-status');
      const healthy=Boolean(value.healthy[name]);
      image.classList.toggle('stale',!healthy);
      label.textContent=healthy ? `LIVE · ${value.frame_age_s[name]} s · #${value.sequence[name]}` : 'STALE — reconnecting';
      if(healthy && !previous[name]) image.src=image.dataset.src+'?ts='+Date.now();
      previous[name]=healthy;
    }
  }catch(error){
    for(const name of ['main','left','right']){
      document.getElementById(name).classList.add('stale');
      document.getElementById(name+'-status').textContent='DISCONNECTED — reconnecting'; previous[name]=false;
    }
  }
}
setInterval(monitor,500); monitor();
</script>""".encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/status.json":
            now = time.monotonic()
            with self.streams.lock:
                ages = {
                    name: None if updated == 0.0 else round(now - updated, 3)
                    for name, updated in self.streams.updated_monotonic.items()
                }
                sequences = dict(self.streams.sequences)
            healthy = {
                name: age is not None and age <= MAX_FRAME_AGE_S[name]
                for name, age in ages.items()
            }
            body = json.dumps(
                {"frame_age_s": ages, "sequence": sequences, "healthy": healthy},
                separators=(",", ":"),
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        stream = {
            "/main.mjpg": "main",
            "/left.mjpg": "left",
            "/right.mjpg": "right",
        }.get(path)
        if stream is None:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            last_sequence = -1
            while True:
                with self.streams.lock:
                    frame = self.streams.frames[stream]
                    updated = self.streams.updated_monotonic[stream]
                    sequence = self.streams.sequences[stream]
                age = None if updated == 0.0 else time.monotonic() - updated
                if age is not None and age > MAX_FRAME_AGE_S[stream]:
                    break
                if frame and age is not None and sequence != last_sequence:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
                    self.wfile.flush()
                    last_sequence = sequence
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18081)
    parser.add_argument(
        "--main-url",
        default="http://172.16.0.50:18082/tmr_zed_latest.jpg",
    )
    parser.add_argument("--main-period-s", type=float, default=0.10)
    args = parser.parse_args()
    rclpy.init()
    streams = Streams(args.main_url, max(0.05, args.main_period_s))
    Handler.streams = streams
    threading.Thread(target=rclpy.spin, args=(streams,), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Three-camera viewer: http://0.0.0.0:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        streams.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
