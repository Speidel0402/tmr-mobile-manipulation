#!/usr/bin/env python3
"""Save simultaneous head ZED and left-wrist RGB frames without motion."""

import argparse
import json
from pathlib import Path

import cv2
import rclpy

from run_streamed_live_pick_cycle import HeadRgbObserver, snapshot_rgb


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/pick_cameras"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    head = HeadRgbObserver()
    try:
        head_rgb, head_stamp = head.fresh()
        wrist_rgb, wrist_stamp, session_id = snapshot_rgb()
        head_path = args.output_dir / "head_zed.jpg"
        wrist_path = args.output_dir / "left_wrist.jpg"
        if not cv2.imwrite(str(head_path), head_rgb):
            raise RuntimeError("failed to save head ZED frame")
        if not cv2.imwrite(str(wrist_path), wrist_rgb):
            raise RuntimeError("failed to save wrist frame")
        print(
            json.dumps(
                {
                    "head_image": str(head_path),
                    "head_stamp_ns": head_stamp,
                    "wrist_image": str(wrist_path),
                    "wrist_stamp": wrist_stamp,
                    "wrist_session_id": session_id,
                },
                indent=2,
            ),
            flush=True,
        )
    finally:
        head.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
