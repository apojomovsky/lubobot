#!/usr/bin/env python3

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def fmt_vec(vec: tuple[float, float, float]) -> str:
    return f"({vec[0]: .3f}, {vec[1]: .3f}, {vec[2]: .3f})"


def dominant_axis(vec: tuple[float, float, float]) -> tuple[str, float]:
    labels = ("x", "y", "z")
    values = [vec[0], vec[1], vec[2]]
    idx = max(range(3), key=lambda i: abs(values[i]))
    return labels[idx], values[idx]


@dataclass
class SampleWindow:
    accel: list[tuple[float, float, float]] = field(default_factory=list)
    gyro: list[tuple[float, float, float]] = field(default_factory=list)
    rpy: list[tuple[float, float, float]] = field(default_factory=list)

    def add_raw(self, msg: Imu) -> None:
        self.accel.append(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        )
        self.gyro.append(
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
        )

    def add_filtered(self, msg: Imu) -> None:
        self.rpy.append(
            quat_to_rpy(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
        )

    def mean_vec(self, items: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        if not items:
            return (0.0, 0.0, 0.0)
        return (
            statistics.fmean(v[0] for v in items),
            statistics.fmean(v[1] for v in items),
            statistics.fmean(v[2] for v in items),
        )

    @property
    def accel_mean(self) -> tuple[float, float, float]:
        return self.mean_vec(self.accel)

    @property
    def gyro_mean(self) -> tuple[float, float, float]:
        return self.mean_vec(self.gyro)

    @property
    def rpy_mean(self) -> tuple[float, float, float]:
        return self.mean_vec(self.rpy)


PHASES = [
    ("still_flat", "Place the robot flat on the floor and keep it still."),
    ("yaw_left", "Rotate the robot slowly counter-clockwise by hand."),
    ("yaw_right", "Rotate the robot slowly clockwise by hand."),
    ("pitch_up", "Tilt the robot nose up and hold briefly."),
    ("pitch_down", "Tilt the robot nose down and hold briefly."),
    ("roll_left_up", "Lift the robot's left side and hold briefly."),
    ("roll_right_up", "Lift the robot's right side and hold briefly."),
]


class ImuProbe(Node):
    def __init__(self, raw_topic: str, filtered_topic: str) -> None:
        super().__init__("imu_frame_probe")
        self.raw_window: SampleWindow | None = None
        self.filtered_window: SampleWindow | None = None
        self.create_subscription(Imu, raw_topic, self.on_raw, 50)
        self.create_subscription(Imu, filtered_topic, self.on_filtered, 50)

    def on_raw(self, msg: Imu) -> None:
        if self.raw_window is not None:
            self.raw_window.add_raw(msg)

    def on_filtered(self, msg: Imu) -> None:
        if self.filtered_window is not None:
            self.filtered_window.add_filtered(msg)


def collect_window(node: ImuProbe, duration: float) -> tuple[SampleWindow, SampleWindow]:
    raw = SampleWindow()
    filtered = SampleWindow()
    node.raw_window = raw
    node.filtered_window = filtered

    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.raw_window = None
    node.filtered_window = None
    return raw, filtered


def summarize(name: str, raw: SampleWindow, filtered: SampleWindow, baseline: SampleWindow | None) -> list[str]:
    lines = [f"[{name}]"]
    lines.append(f"  samples raw={len(raw.accel)} filtered={len(filtered.rpy)}")
    lines.append(f"  accel_mean   {fmt_vec(raw.accel_mean)}")
    lines.append(f"  gyro_mean    {fmt_vec(raw.gyro_mean)}")
    if filtered.rpy:
        roll, pitch, yaw = filtered.rpy_mean
        lines.append(f"  rpy_mean     ({roll: .3f}, {pitch: .3f}, {yaw: .3f})")

    if baseline is not None:
        accel_delta = tuple(a - b for a, b in zip(raw.accel_mean, baseline.accel_mean))
        gyro_delta = tuple(a - b for a, b in zip(raw.gyro_mean, baseline.gyro_mean))
        accel_axis, accel_value = dominant_axis(accel_delta)
        gyro_axis, gyro_value = dominant_axis(gyro_delta)
        lines.append(f"  accel_delta  {fmt_vec(accel_delta)} dominant={accel_axis} ({accel_value: .3f})")
        lines.append(f"  gyro_delta   {fmt_vec(gyro_delta)} dominant={gyro_axis} ({gyro_value: .3f})")

    return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Guided IMU frame probe.")
    parser.add_argument("--raw-topic", default="/imu/data_raw")
    parser.add_argument("--filtered-topic", default="/imu/data")
    parser.add_argument("--duration", type=float, default=4.0, help="Seconds per phase")
    parser.add_argument(
        "--output",
        default="",
        help="Optional output file path; defaults to ros2_ws/log/imu_frame_probe_<timestamp>.txt when available",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    workspace_log = Path("/workspaces/ros2_ws/log")
    if workspace_log.is_dir():
        workspace_log.mkdir(parents=True, exist_ok=True)
        return workspace_log / f"imu_frame_probe_{stamp}.txt"
    return Path.cwd() / f"imu_frame_probe_{stamp}.txt"


def main() -> int:
    args = parse_args()
    output_path = Path(args.output) if args.output else default_output_path()

    rclpy.init()
    node = ImuProbe(args.raw_topic, args.filtered_topic)

    try:
        all_lines: list[str] = []
        baseline_raw: SampleWindow | None = None

        print("IMU frame probe starting.")
        print(f"Raw topic: {args.raw_topic}")
        print(f"Filtered topic: {args.filtered_topic}")
        print(f"Output file: {output_path}")
        print("")

        for name, instruction in PHASES:
            input(f"{instruction} Press Enter to record for {args.duration:.1f}s...")
            raw, filtered = collect_window(node, args.duration)
            if name == "still_flat":
                baseline_raw = raw
            lines = summarize(name, raw, filtered, baseline_raw if baseline_raw is not None else None)
            print("\n".join(lines))
            print("")
            all_lines.extend(lines)
            all_lines.append("")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(all_lines), encoding="ascii")
        print(f"Wrote summary to {output_path}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
