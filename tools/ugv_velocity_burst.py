#!/usr/bin/env python3
"""Publish a bounded velocity burst to the motor bridge command topic."""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Iterable, Optional


DEFAULT_COMMAND_TOPIC = "/ugv_nav_cmd"


def build_velocity_payload(v_mps: float, omega_radps: float, reason: str) -> str:
    return json.dumps(
        {
            "mode": "VELOCITY",
            "command_type": "velocity",
            "controller": "ugv_velocity_burst",
            "v_mps": float(v_mps),
            "omega_radps": float(omega_radps),
            "reason": str(reason),
        },
        sort_keys=True,
    )


def build_stop_payload(reason: str = "velocity_burst_stop") -> str:
    return json.dumps(
        {
            "mode": "STOP",
            "command_type": "stop",
            "controller": "ugv_velocity_burst",
            "v_mps": 0.0,
            "omega_radps": 0.0,
            "reason": str(reason),
        },
        sort_keys=True,
    )


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--v-mps", type=float, default=0.12)
    parser.add_argument("--omega-radps", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--reason", default="velocity_burst")
    parser.add_argument("--yes", action="store_true", help="confirm the robot is clear to move")
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    duration_s = max(0.05, min(10.0, float(args.duration_s)))
    hz = max(1.0, min(50.0, float(args.hz)))
    period_s = 1.0 / hz

    print("WARNING: this sends real velocity commands to the motor bridge.", file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    import rclpy  # type: ignore
    from std_msgs.msg import String  # type: ignore

    rclpy.init()
    node = rclpy.create_node("ugv_velocity_burst")
    pub = node.create_publisher(String, str(args.topic), 10)
    velocity = String(data=build_velocity_payload(args.v_mps, args.omega_radps, args.reason))
    stop = String(data=build_stop_payload())

    deadline = time.monotonic() + duration_s
    try:
        time.sleep(0.25)
        while time.monotonic() < deadline and rclpy.ok():
            pub.publish(velocity)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    finally:
        for _ in range(5):
            pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
