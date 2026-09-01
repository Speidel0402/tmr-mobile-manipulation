#!/usr/bin/env python3
"""Restore the left arm to a previously locked ground-relative FK height."""

from __future__ import annotations

import argparse
import json
import time

import rclpy

from servo_cup_edge_xy import ServoNode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-z", type=float, required=True)
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--max-delta-m", type=float, default=0.03)
    args = parser.parse_args()

    rclpy.init()
    node = ServoNode()
    impedance_disabled = False
    try:
        node.wait_ready()
        start = node.fk(list(node.q))
        delta_z = float(args.target_z - start.position.z)
        if abs(delta_z) > args.max_delta_m:
            raise RuntimeError(
                f"height correction {delta_z:.6f}m exceeds limit {args.max_delta_m:.6f}m"
            )

        # move_xy uses lock_z as the Cartesian target while preserving the
        # current orientation, so the same helper can make a short horizontal
        # correction on the original ground-relative height plane.
        node.lock_z = float(args.target_z)
        node.set_impedance(False)
        impedance_disabled = True
        actual_xy, base_xy, end = node.move_xy([args.dx, args.dy])
        node.set_impedance(True)
        impedance_disabled = False
        settle_deadline = time.monotonic() + 0.8
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        held = node.fk(list(node.q))
        print(
            json.dumps(
                {
                    "start_xyz": [start.position.x, start.position.y, start.position.z],
                    "target_z": args.target_z,
                    "requested_delta_z": delta_z,
                    "requested_xy": [args.dx, args.dy],
                    "actual_xy": actual_xy.tolist(),
                    "end_xyz": [end.position.x, end.position.y, end.position.z],
                    "held_xyz": [
                        held.position.x,
                        held.position.y,
                        held.position.z,
                    ],
                    "base_xy": base_xy.tolist(),
                },
                indent=2,
            )
        )
        return 0
    finally:
        if impedance_disabled:
            node.set_impedance(True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
