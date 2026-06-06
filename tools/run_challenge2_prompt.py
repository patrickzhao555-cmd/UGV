#!/usr/bin/env python3
"""Prompt for the UGV Challenge 2 start pose, then launch the real controller."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from typing import Optional, Sequence


DEFAULT_MOTOR_PORT = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00"


def _finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a number, got {text!r}") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"expected a finite number, got {text!r}")
    return value


def _prompt_float(label: str, default: Optional[float] = None) -> float:
    while True:
        suffix = "" if default is None else f" [{default:g}]"
        text = input(f"{label}{suffix}: ").strip()
        if not text and default is not None:
            return float(default)
        try:
            return _finite_float(text)
        except argparse.ArgumentTypeError as exc:
            print(exc, file=sys.stderr)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--start-x-m", type=_finite_float)
    parser.add_argument("--start-y-m", type=_finite_float)
    parser.add_argument("--start-yaw-deg", type=_finite_float)
    parser.add_argument("--motor-port", default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--speed-mps", type=_finite_float, default=0.24)
    parser.add_argument("--stop-radius-m", type=_finite_float, default=0.75)
    parser.add_argument("--dry-run", action="store_true", help="Print the ros2 launch command without running it.")
    parser.add_argument(
        "extra_launch_args",
        nargs=argparse.REMAINDER,
        help="Optional extra ros2 launch args after --, for example -- nav_debug_ignore_nav_frame:=false",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    start_x_m = args.start_x_m
    start_y_m = args.start_y_m
    start_yaw_deg = args.start_yaw_deg
    if start_x_m is None:
        start_x_m = _prompt_float("UGV start field x (m)")
    if start_y_m is None:
        start_y_m = _prompt_float("UGV start field y (m)")
    if start_yaw_deg is None:
        start_yaw_deg = _prompt_float("UGV heading yaw (deg, field +x = 0, left turn positive)")

    extra = list(args.extra_launch_args)
    if extra and extra[0] == "--":
        extra = extra[1:]

    command = [
        "ros2",
        "launch",
        "ugv_sensor_sync",
        "competition_bringup.launch.py",
        "nav_controller_mode:=challenge2_align_straight",
        f"nav_challenge2_start_pose_set:=true",
        f"nav_challenge2_start_x_m:={start_x_m:.6f}",
        f"nav_challenge2_start_y_m:={start_y_m:.6f}",
        f"nav_challenge2_start_yaw_deg:={start_yaw_deg:.6f}",
        f"nav_challenge2_speed_mps:={float(args.speed_mps):.6f}",
        f"nav_challenge2_stop_radius_m:={float(args.stop_radius_m):.6f}",
        "start_zed:=true",
        "start_lidar:=false",
        "start_lidar_filter:=false",
        "start_fusion:=false",
        "nav_debug_ignore_nav_frame:=true",
        f"motor_port:={str(args.motor_port)}",
        *extra,
    ]

    print("Launching Challenge 2 with explicit UGV start pose:")
    print(" ".join(command))
    if args.dry_run:
        return 0
    try:
        return int(subprocess.call(command))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
