#!/usr/bin/env python3
"""Lower the held cup by the calibrated distance and release it once."""

import json

import rclpy
from rclpy.node import Node

from run_streamed_live_pick_cycle import command_gripper, move_vertical
from servo_cup_edge_xy import ServoNode


DOWN_M = 0.340


def emit(event, **values):
    print(
        "RELEASE=" + json.dumps({"event": event, **values}, separators=(",", ":")),
        flush=True,
    )


def main():
    rclpy.init()
    arm = ServoNode()
    gripper = Node("release_cup_at_fixed_height")
    impedance_off = False
    try:
        arm.wait_ready()
        start = arm.fk(list(arm.q))
        emit("start", z=float(start.position.z), gripper_commanded=False)
        arm.set_impedance(False)
        impedance_off = True
        low, actual = move_vertical(arm, -DOWN_M)
        emit("down_complete", requested_m=-DOWN_M, actual_m=actual, z=float(low.position.z))
        opened = command_gripper(gripper, 0.0, "open_after_fixed_descent")
        if opened["position"] > 0.05 or not opened["reached_goal"]:
            raise RuntimeError("cup release was not verified: " + repr(opened))
        emit("released", open_position=opened["position"], z=float(low.position.z))
    finally:
        if impedance_off:
            attempt = arm.ensure_stable_runtime_after_ptp()
            held = arm.fk(list(arm.q))
            emit("controller_hold", recovery_attempt=attempt, z=float(held.position.z))
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
