#!/usr/bin/env python3
"""Return the left tool vertically to the calibrated pick height."""

import json

import rclpy

from run_streamed_live_pick_cycle import REFERENCE_Z, move_vertical
from servo_cup_edge_xy import ServoNode


def emit(event, **values):
    print(
        "RESTORE_HEIGHT="
        + json.dumps({"event": event, **values}, separators=(",", ":")),
        flush=True,
    )


def main():
    rclpy.init()
    arm = ServoNode()
    impedance_off = False
    try:
        arm.wait_ready()
        start = arm.fk(list(arm.q))
        requested = float(REFERENCE_Z - start.position.z)
        if not 0.0 <= requested <= 0.38:
            raise RuntimeError(f"standard-height correction outside range: {requested:.6f}m")
        arm.set_impedance(False)
        impedance_off = True
        end, actual = move_vertical(arm, requested)
        emit(
            "height_restored",
            start_z=float(start.position.z),
            target_z=float(REFERENCE_Z),
            actual_up_m=actual,
            end_z=float(end.position.z),
            gripper_commanded=False,
        )
    finally:
        if impedance_off:
            attempt = arm.ensure_stable_runtime_after_ptp()
            held = arm.fk(list(arm.q))
            emit("controller_hold", recovery_attempt=attempt, z=float(held.position.z))
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
