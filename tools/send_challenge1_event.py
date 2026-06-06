#!/usr/bin/env python3
"""Publish Challenge 1 UAV launched/landed event signals for testing.

The Challenge 1 controller listens to two simple Bool topics:

* /ugv/uav_launched
* /ugv/uav_landed

The real UAV/ESP integration can publish the same topics.  This script is only
a field-test convenience and never publishes motor commands.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence


DEFAULT_TOPICS = {
    "launched": "/ugv/uav_launched",
    "landed": "/ugv/uav_landed",
}


@dataclass(frozen=True)
class Challenge1EventSpec:
    event: str
    topic: str
    value: bool
    repeat_hz: float
    count: int
    discovery_wait_s: float


def _bool_arg(text: str) -> bool:
    value = str(text).strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {text!r}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--event", choices=["launched", "landed"], default="landed")
    parser.add_argument("--topic", default="", help="Override output Bool topic.")
    parser.add_argument("--value", type=_bool_arg, default=True)
    parser.add_argument("--repeat-hz", type=float, default=2.0)
    parser.add_argument("--count", type=int, default=3, help="Number of messages; 0 means until Ctrl-C.")
    parser.add_argument("--discovery-wait-s", type=float, default=0.5)
    return parser.parse_args(argv)


def spec_from_args(args: argparse.Namespace) -> Challenge1EventSpec:
    event = str(args.event)
    topic = str(args.topic).strip() or DEFAULT_TOPICS[event]
    if not topic.startswith("/"):
        raise ValueError("topic must be an absolute ROS topic, for example /ugv/uav_landed")
    repeat_hz = float(args.repeat_hz)
    if repeat_hz <= 0.0:
        raise ValueError("repeat-hz must be > 0")
    count = int(args.count)
    if count < 0:
        raise ValueError("count must be >= 0")
    discovery_wait_s = float(args.discovery_wait_s)
    if discovery_wait_s < 0.0:
        raise ValueError("discovery-wait-s must be >= 0")
    return Challenge1EventSpec(
        event=event,
        topic=topic,
        value=bool(args.value),
        repeat_hz=repeat_hz,
        count=count,
        discovery_wait_s=discovery_wait_s,
    )


def publish_event(spec: Challenge1EventSpec) -> None:
    import rclpy
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("ugv_send_challenge1_event")
    publisher = node.create_publisher(Bool, spec.topic, 10)
    try:
        if spec.discovery_wait_s > 0.0:
            time.sleep(spec.discovery_wait_s)
        period_s = 1.0 / spec.repeat_hz
        sent = 0
        while rclpy.ok() and (spec.count == 0 or sent < spec.count):
            msg = Bool()
            msg.data = spec.value
            publisher.publish(msg)
            sent += 1
            node.get_logger().info(
                f"Published Challenge 1 {spec.event}={spec.value} "
                f"{sent if spec.count else sent}/{spec.count if spec.count else 'inf'} "
                f"on {spec.topic}"
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
        print(f"send_challenge1_event: {exc}", file=sys.stderr)
        return 2
    try:
        publish_event(spec)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
