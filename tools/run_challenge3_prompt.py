#!/usr/bin/env python3
"""Prompt for the UGV Challenge 3 start pose, then launch corridor bypass."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from typing import Optional, Sequence


DEFAULT_MOTOR_PORT = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00"
DEFAULT_UAV_ESP_PORT = "/dev/ttyUSB1"


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
    parser.add_argument("--cruise-speed-mps", type=_finite_float, default=0.24)
    parser.add_argument("--hard-turn-speed-mps", type=_finite_float, default=1.70)
    parser.add_argument("--hard-turn-omega-radps", type=_finite_float, default=7.80)
    parser.add_argument("--lane-offsets-m", default="0.0,1.6,-1.6,2.2,-2.2")
    parser.add_argument("--lidar-forward-fov-deg", type=_finite_float, default=230.0)
    parser.add_argument("--lidar-min-cluster-points", type=int, default=3)
    parser.add_argument("--lidar-cluster-max-gap-m", type=_finite_float, default=0.35)
    parser.add_argument("--require-scan", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--esp-target", action="store_true", help="Start the ESP serial target receiver.")
    parser.add_argument("--uav-esp-port", default=DEFAULT_UAV_ESP_PORT)
    parser.add_argument("--uav-esp-baud", type=int, default=115200)
    parser.add_argument("--uav-esp-serial-protocol", choices=("binary14", "line", "auto"), default="binary14")
    parser.add_argument(
        "--uav-esp-require-checksum",
        action="store_true",
        help="Require NMEA-style *XX XOR checksums on ESP target lines.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the ros2 launch command without running it.")
    parser.add_argument(
        "extra_launch_args",
        nargs=argparse.REMAINDER,
        help="Optional extra ros2 launch args after --, for example -- challenge3_obstacle_lookahead_m:=4.0",
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

    esp_launch_args = []
    if args.esp_target:
        esp_launch_args = [
            "start_uav_target_receiver:=true",
            "uav_target_input_mode:=serial",
            f"uav_esp_serial_port:={str(args.uav_esp_port)}",
            f"uav_esp_serial_baud:={int(args.uav_esp_baud)}",
            f"uav_esp_serial_protocol:={str(args.uav_esp_serial_protocol)}",
            f"uav_esp_require_checksum:={'true' if args.uav_esp_require_checksum else 'false'}",
        ]

    command = [
        "ros2",
        "launch",
        "ugv_sensor_sync",
        "competition_bringup.launch.py",
        "start_nav:=false",
        "start_challenge3_corridor:=true",
        "start_zed:=true",
        "start_lidar:=true",
        "start_lidar_filter:=true",
        "start_fusion:=false",
        "nav_debug_ignore_nav_frame:=true",
        f"lidar_filter_forward_fov_deg:={float(args.lidar_forward_fov_deg):.6f}",
        "challenge3_start_pose_set:=true",
        f"challenge3_start_x_m:={start_x_m:.6f}",
        f"challenge3_start_y_m:={start_y_m:.6f}",
        f"challenge3_start_yaw_deg:={start_yaw_deg:.6f}",
        f"challenge3_cruise_speed_mps:={float(args.cruise_speed_mps):.6f}",
        f"challenge3_hard_turn_speed_mps:={float(args.hard_turn_speed_mps):.6f}",
        f"challenge3_hard_turn_max_omega_radps:={float(args.hard_turn_omega_radps):.6f}",
        f"challenge3_lane_offsets_m:={str(args.lane_offsets_m)}",
        f"challenge3_lidar_min_cluster_points:={max(1, int(args.lidar_min_cluster_points))}",
        f"challenge3_lidar_cluster_max_gap_m:={float(args.lidar_cluster_max_gap_m):.6f}",
        f"challenge3_require_scan:={'true' if args.require_scan else 'false'}",
        f"motor_port:={str(args.motor_port)}",
        *esp_launch_args,
        *extra,
    ]

    print("Launching Challenge 3 large-radius corridor bypass:")
    print(" ".join(command))
    if not args.esp_target:
        print("Manual target fallback remains active: publish /ugv/uav_target with tools/send_uav_target.py")
    if args.dry_run:
        return 0
    try:
        return int(subprocess.call(command))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
