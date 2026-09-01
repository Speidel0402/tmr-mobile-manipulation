#!/usr/bin/env python3
"""Restore only the left arm to the proven start pose of the live pick cycle."""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy

from servo_cup_edge_xy import ServoNode


TARGET = np.asarray(
    [
        -1.71976900100708,
        -1.6329213380813599,
        1.8240526914596558,
        -2.447446823120117,
        2.177191972732544,
        0.8496646285057068,
        -3.05077862739563,
    ],
    dtype=float,
)


def main() -> int:
    rclpy.init()
    arm = ServoNode()
    impedance_off = False
    try:
        arm.wait_ready()
        start = np.asarray(arm.q, dtype=float)
        arm.set_impedance(False)
        impedance_off = True
        time.sleep(0.35)
        arm.max_joint_velocity = 0.06
        arm.move_ptp(TARGET.tolist())
        for _ in range(8):
            rclpy.spin_once(arm, timeout_sec=0.04)
        before_hold = np.asarray(arm.q, dtype=float)
        error = before_hold - TARGET
        if float(np.max(np.abs(error))) > 0.008:
            raise RuntimeError(
                f"left pick-initial endpoint error {float(np.max(np.abs(error))):.6f} rad"
            )
        time.sleep(1.0)
        arm.ensure_runtime_ready()
        impedance_off = False
        for _ in range(8):
            rclpy.spin_once(arm, timeout_sec=0.04)
        after_hold = np.asarray(arm.q, dtype=float)
        hold_error = after_hold - TARGET
        result = {
            "status": "success",
            "start_joint_positions_rad": start.tolist(),
            "target_joint_positions_rad": TARGET.tolist(),
            "measured_joint_positions_rad": after_hold.tolist(),
            "maximum_joint_error_rad": float(np.max(np.abs(hold_error))),
            "gripper_commanded": False,
        }
        if result["maximum_joint_error_rad"] > 0.012:
            raise RuntimeError(
                f"left impedance handoff moved endpoint by {result['maximum_joint_error_rad']:.6f} rad"
            )
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except BaseException as exc:
        print(json.dumps({"status": "failed", "error": repr(exc)}, indent=2), flush=True)
        return 1
    finally:
        if impedance_off:
            try:
                time.sleep(1.0)
                arm.ensure_runtime_ready()
            except Exception as exc:
                print(json.dumps({"event": "hold_restore_failed", "error": repr(exc)}), flush=True)
        arm.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
