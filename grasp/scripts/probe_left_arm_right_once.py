#!/usr/bin/env python3
"""Move the left tool 3 cm toward robot-right while holding Z/orientation."""

import json

import rclpy

from servo_cup_edge_xy import ServoNode


def main() -> int:
    rclpy.init()
    node = ServoNode()
    impedance_disabled = False
    try:
        node.wait_ready()
        start = node.fk(list(node.q))
        node.set_impedance(False)
        impedance_disabled = True
        actual_xy, _, end = node.move_xy([0.0, -0.03])
        node.set_impedance(True)
        impedance_disabled = False
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)
        held = node.fk(list(node.q))
        print(
            json.dumps(
                {
                    "status": "success",
                    "requested_base_xy_m": [0.0, -0.03],
                    "actual_base_xy_m": actual_xy.tolist(),
                    "start_xyz_m": [start.position.x, start.position.y, start.position.z],
                    "end_xyz_m": [end.position.x, end.position.y, end.position.z],
                    "held_xyz_m": [held.position.x, held.position.y, held.position.z],
                },
                indent=2,
            ),
            flush=True,
        )
        return 0
    finally:
        if impedance_disabled:
            try:
                node.set_impedance(True)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
