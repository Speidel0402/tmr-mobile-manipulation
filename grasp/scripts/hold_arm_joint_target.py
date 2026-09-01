#!/usr/bin/env python3
"""Publish a timestamped FR3 joint target long enough for impedance hand-off."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def joints(text):
    values = [float(item) for item in text.split(",")]
    if len(values) != 7 or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError("expected seven finite comma-separated joints")
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--joints", type=joints, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=50.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node(f"{args.arm}_transport_hold_publisher")
    publisher = node.create_publisher(JointState, f"/{args.arm}/gello/joint_states", 1)
    message = JointState()
    message.name = [f"{args.arm}_fr3v2_joint{index}" for index in range(1, 8)]
    message.position = args.joints
    deadline = time.monotonic() + args.duration
    period = 1.0 / args.rate
    try:
        while time.monotonic() < deadline:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
