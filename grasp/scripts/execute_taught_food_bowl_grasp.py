#!/usr/bin/env python3
"""Open, descend at the manually taught XY, close once, and hold low."""

import json

import rclpy
from rclpy.node import Node

from run_streamed_live_pick_cycle import (
    close_result_is_real,
    command_gripper,
    move_vertical,
)
from servo_cup_edge_xy import ServoNode


DOWN_M = 0.360


def emit(event, **values):
    print(
        "TAUGHT_BOWL="
        + json.dumps({"event": event, **values}, separators=(",", ":")),
        flush=True,
    )


def main():
    rclpy.init()
    arm = ServoNode()
    gripper = Node("execute_taught_food_bowl_grasp")
    impedance_off = False
    try:
        arm.wait_ready()
        start = arm.fk(list(arm.q))
        opened = command_gripper(gripper, 0.0, "open_before_taught_descent")
        if opened["position"] > 0.05 or not opened["reached_goal"]:
            raise RuntimeError("gripper did not open before descent: " + repr(opened))
        emit("open_complete", z=float(start.position.z), open_position=opened["position"])

        arm.set_impedance(False)
        impedance_off = True
        low, actual = move_vertical(arm, -DOWN_M)
        emit("down_complete", requested_m=-DOWN_M, actual_m=actual, z=float(low.position.z))

        closed = command_gripper(gripper, 0.8, "single_close_after_taught_descent")
        valid, progress, verdict = close_result_is_real(closed)
        emit(
            "close_complete",
            position=closed["position"],
            stalled=closed["stalled"],
            status=closed["status"],
            progress=progress,
            verdict=verdict,
            accepted_by_existing_policy=valid,
        )
    finally:
        if impedance_off:
            attempt = arm.ensure_stable_runtime_after_ptp()
            held = arm.fk(list(arm.q))
            emit("low_hold", recovery_attempt=attempt, z=float(held.position.z))
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
