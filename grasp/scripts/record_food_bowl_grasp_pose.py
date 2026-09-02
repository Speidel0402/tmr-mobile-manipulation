#!/usr/bin/env python3
"""Record a manually taught food-bowl grasp pose, then open-image-close once."""

import argparse
import json
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node

from detect_food_bowl import detect_food_bowl
from run_streamed_live_pick_cycle import (
    HeadRgbObserver,
    close_result_is_real,
    command_gripper,
    snapshot_rgb,
)
from servo_cup_edge_xy import ServoNode


def pose_dict(pose):
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir", type=Path, default=Path("/tmp/food_bowl_calibration")
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    arm = ServoNode()
    gripper = Node("record_food_bowl_grasp_pose")
    head = HeadRgbObserver()
    record = {}
    try:
        arm.wait_ready()
        taught_pose = arm.fk(list(arm.q))
        record["joint_positions_rad"] = [float(value) for value in arm.q]
        record["tool_pose_base"] = pose_dict(taught_pose)

        opened = command_gripper(gripper, 0.0, "open_at_taught_food_bowl_pose")
        if opened["position"] > 0.05 or not opened["reached_goal"]:
            raise RuntimeError("gripper failed to open at taught pose: " + repr(opened))
        record["open"] = opened

        head_image, head_stamp_ns = head.fresh()
        wrist_image, wrist_stamp, session_id = snapshot_rgb()
        head_path = args.output_dir / "head_zed_open.jpg"
        wrist_path = args.output_dir / "left_wrist_open.jpg"
        if not cv2.imwrite(str(head_path), head_image):
            raise RuntimeError("failed to save head image")
        if not cv2.imwrite(str(wrist_path), wrist_image):
            raise RuntimeError("failed to save wrist image")
        record["images"] = {
            "head": str(head_path),
            "head_stamp_ns": int(head_stamp_ns),
            "wrist": str(wrist_path),
            "wrist_stamp": float(wrist_stamp),
            "wrist_session_id": session_id,
        }
        try:
            best, candidates = detect_food_bowl(wrist_image)
            record["automatic_visual_measurement"] = {
                "best": best,
                "candidates": candidates,
            }
        except Exception as exc:
            record["automatic_visual_measurement_error"] = repr(exc)

        closed = command_gripper(gripper, 0.8, "single_close_at_taught_food_bowl_pose")
        valid, progress, verdict = close_result_is_real(closed)
        closed["close_progress"] = progress
        closed["verdict"] = verdict
        closed["accepted_by_existing_policy"] = bool(valid)
        record["close"] = closed
        record["final_joint_positions_rad"] = [float(value) for value in arm.q]
        record["final_tool_pose_base"] = pose_dict(arm.fk(list(arm.q)))
        record_path = args.output_dir / "record.json"
        record_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
        print(json.dumps({"record": str(record_path), **record}, indent=2), flush=True)
    finally:
        head.destroy_node()
        gripper.destroy_node()
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
