#!/usr/bin/env python3
"""Publish one or more Operation Touchdown UAV target coordinates.

This is a testing convenience for the formal mission supervisor.  It only
publishes ``geometry_msgs/PointStamped`` on ``/ugv/uav_target``; it never
publishes motor commands or starts navigation by itself.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
UGV_NAV_SRC = ROOT / "ros2_ws" / "src" / "ugv_nav"
if str(UGV_NAV_SRC) not in sys.path:
    sys.path.insert(0, str(UGV_NAV_SRC))

from ugv_nav_core.nav2_bridge import target_units_scale  # noqa: E402


@dataclass(frozen=True)
class UavTargetSpec:
    x_m: float
    y_m: float
    frame_id: str
    topic: str
    repeat_hz: float
    count: int
    discovery_wait_s: float


def _finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a number, got {text!r}") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"expected a finite number, got {text!r}")
    return value


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--x", required=True, type=_finite_float, help="Target x coordinate.")
    parser.add_argument("--y", required=True, type=_finite_float, help="Target y coordinate.")
    parser.add_argument("--units", default="meters", help="Input units: meters/m or yards/yd.")
    parser.add_argument("--frame", default="map", help="PointStamped frame_id. Default: map.")
    parser.add_argument("--topic", default="/ugv/uav_target", help="Output PointStamped topic.")
    parser.add_argument("--repeat-hz", type=float, default=1.0, help="Publish rate when count > 1 or count=0.")
    parser.add_argument("--count", type=int, default=1, help="Number of messages to publish; 0 means until Ctrl-C.")
    parser.add_argument(
        "--discovery-wait-s",
        type=float,
        default=0.5,
        help="Wait after creating the publisher so subscribers can discover it.",
    )
    return parser.parse_args(argv)


def spec_from_args(args: argparse.Namespace) -> UavTargetSpec:
    scale = target_units_scale(str(args.units))
    if scale is None:
        raise ValueError(f"unsupported units {args.units!r}; use meters or yards")
    frame_id = str(args.frame).strip()
    if not frame_id:
        raise ValueError("frame must not be empty")
    topic = str(args.topic).strip()
    if not topic.startswith("/"):
        raise ValueError("topic must be an absolute ROS topic, for example /ugv/uav_target")
    repeat_hz = float(args.repeat_hz)
    if not math.isfinite(repeat_hz) or repeat_hz <= 0.0:
        raise ValueError("repeat-hz must be finite and > 0")
    count = int(args.count)
    if count < 0:
        raise ValueError("count must be >= 0")
    discovery_wait_s = float(args.discovery_wait_s)
    if not math.isfinite(discovery_wait_s) or discovery_wait_s < 0.0:
        raise ValueError("discovery-wait-s must be finite and >= 0")
    x_m = float(args.x) * scale
    y_m = float(args.y) * scale
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("converted target coordinate is not finite")
    return UavTargetSpec(
        x_m=x_m,
        y_m=y_m,
        frame_id=frame_id,
        topic=topic,
        repeat_hz=repeat_hz,
        count=count,
        discovery_wait_s=discovery_wait_s,
    )


def publish_target(spec: UavTargetSpec) -> None:
    import rclpy
    from geometry_msgs.msg import PointStamped

    rclpy.init()
    node = rclpy.create_node("ugv_send_uav_target")
    publisher = node.create_publisher(PointStamped, spec.topic, 10)
    try:
        if spec.discovery_wait_s > 0.0:
            time.sleep(spec.discovery_wait_s)
        period_s = 1.0 / spec.repeat_hz
        sent = 0
        while rclpy.ok() and (spec.count == 0 or sent < spec.count):
            msg = PointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = spec.frame_id
            msg.point.x = spec.x_m
            msg.point.y = spec.y_m
            msg.point.z = 0.0
            publisher.publish(msg)
            sent += 1
            node.get_logger().info(
                f"Published UAV target {sent if spec.count else sent}/"
                f"{spec.count if spec.count else 'inf'}: "
                f"x={spec.x_m:.3f}m y={spec.y_m:.3f}m frame={spec.frame_id} topic={spec.topic}"
            )
            if spec.count != 0 and sent >= spec.count:
                break
            time.sleep(period_s)
        time.sleep(0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        spec = spec_from_args(parse_args(argv))
    except (argparse.ArgumentTypeError, ValueError) as exc:
        print(f"send_uav_target: {exc}", file=sys.stderr)
        return 2
    try:
        publish_target(spec)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
