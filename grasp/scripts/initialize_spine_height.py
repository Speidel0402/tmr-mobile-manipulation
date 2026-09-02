#!/usr/bin/env python3
"""Restore the user-confirmed competition spine height and verify it."""

from __future__ import annotations

import argparse
import json

import rclpy
from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import GetPosition, SwitchOn
from rclpy.action import ActionClient
from rclpy.node import Node


TARGET_POSITION_M = 0.7
POSITION_TOLERANCE_M = 0.003


class SpineInitializer(Node):
    def __init__(self) -> None:
        super().__init__("competition_spine_height_initializer")
        self.switch_on = self.create_client(SwitchOn, "/franka_spine_node/switch_on")
        self.get_position = self.create_client(
            GetPosition, "/franka_spine_node/get_position"
        )
        self.move = ActionClient(
            self, MoveAbsolute, "/franka_spine_node/move_absolute"
        )

    def call(self, client, request, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("spine service unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError("spine service timeout")
        return future.result()

    def position(self) -> float:
        response = self.call(self.get_position, GetPosition.Request())
        if not response.success:
            raise RuntimeError("spine position query failed")
        return float(response.position)

    def execute(self, target_m: float, velocity: float) -> dict:
        start = self.position()
        switched = self.call(self.switch_on, SwitchOn.Request())
        if not switched.success:
            raise RuntimeError(f"spine switch-on failed: {switched.message}")
        if not self.move.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("spine move action unavailable")
        goal = MoveAbsolute.Goal()
        goal.position = float(target_m)
        goal.velocity = float(velocity)
        goal.acceleration = 0.1
        goal.deceleration = 0.1
        future = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("spine height goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("spine height goal timeout")
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f"spine move failed: {result.error}")
        measured = self.position()
        error = measured - float(target_m)
        if abs(error) > POSITION_TOLERANCE_M:
            raise RuntimeError(f"spine height error {error:.6f}m")
        return {
            "status": "success",
            "moved": True,
            "start_position_m": start,
            "target_position_m": float(target_m),
            "measured_position_m": measured,
            "position_error_m": error,
            "stop_by": result.stop_by,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--target-m", type=float, default=TARGET_POSITION_M)
    parser.add_argument("--velocity", type=float, default=0.05)
    args = parser.parse_args()
    if not args.execute:
        print(
            json.dumps(
                {
                    "status": "dry_run",
                    "motion_enabled": False,
                    "target_position_m": args.target_m,
                    "velocity_mps": args.velocity,
                },
                indent=2,
            )
        )
        return 0
    rclpy.init()
    node = SpineInitializer()
    try:
        print(json.dumps(node.execute(args.target_m, args.velocity), indent=2), flush=True)
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
