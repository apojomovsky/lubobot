#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a small deterministic /cmd_vel sequence for localization tests."
    )
    parser.add_argument("--topic", default="/cmd_vel", help="Twist topic to publish to")
    parser.add_argument(
        "--rate", type=float, default=10.0, help="Publish rate in Hz during each segment"
    )
    return parser.parse_args()


class CmdVelSequence(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("cmd_vel_sequence")
        self.publisher = self.create_publisher(Twist, topic, 10)

    def publish_segment(self, linear_x: float, angular_z: float, duration_s: float, rate_hz: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        interval = 1.0 / rate_hz
        end_time = time.monotonic() + duration_s

        while time.monotonic() < end_time:
            self.publisher.publish(twist)
            time.sleep(interval)

    def stop(self, repeats: int = 5, rate_hz: float = 10.0) -> None:
        twist = Twist()
        interval = 1.0 / rate_hz
        for _ in range(repeats):
            self.publisher.publish(twist)
            time.sleep(interval)


def main() -> int:
    args = parse_args()
    rclpy.init()

    node = CmdVelSequence(args.topic)

    try:
        # Compact sequence for limited space.
        sequence = [
            (0.0, 0.0, 5.0),
            (0.0, 0.45, 5.0),
            (0.05, 0.0, 5.0),
            (0.0, -0.45, 5.0),
            (0.05, 0.0, 5.0),
            (0.0, 0.0, 5.0),
        ]

        for linear_x, angular_z, duration_s in sequence:
            node.publish_segment(linear_x, angular_z, duration_s, args.rate)
        node.stop(rate_hz=args.rate)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
