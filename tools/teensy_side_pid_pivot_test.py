#!/usr/bin/env python3
"""Bounded pivot test for the Teensy side-PID firmware."""

import argparse
import sys
import time

import serial


WARNING = "WARNING: first run must be with wheels off the ground."


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--omega-radps", type=float, default=0.35)
    parser.add_argument("--duration-s", type=float, default=0.8)
    parser.add_argument("--track-width-m", type=float, default=0.425)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0825)
    parser.add_argument("--ticks-per-rev", type=int, default=3200)
    parser.add_argument("--control-hz", type=float, default=100.0)
    parser.add_argument("--left-motor-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-motor-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--fl-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--fr-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rl-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rr-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args()


def write_line(dev: serial.Serial, line: str) -> None:
    dev.write(line.encode("ascii"))
    dev.flush()


def send_param(dev: serial.Serial, name: str, value: float, timeout_s: float = 0.8) -> None:
    write_line(dev, f"CMD PARAM {name} {value}\n")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = dev.readline().decode("utf-8", errors="replace").strip()
        if line == f"PARAM,{name},ok":
            return
        if line == f"PARAM,{name},unknown":
            raise RuntimeError(f"Teensy rejected parameter {name}")
    raise TimeoutError(f"Timed out waiting for PARAM ACK: {name}")


def sync_params(dev: serial.Serial, args: argparse.Namespace) -> None:
    params = [
        ("track_width_m", args.track_width_m),
        ("wheel_radius_m", args.wheel_radius_m),
        ("ticks_per_rev", args.ticks_per_rev),
        ("control_hz", args.control_hz),
        ("left_motor_sign", args.left_motor_sign),
        ("right_motor_sign", args.right_motor_sign),
        ("fl_encoder_sign", args.fl_encoder_sign),
        ("fr_encoder_sign", args.fr_encoder_sign),
        ("rl_encoder_sign", args.rl_encoder_sign),
        ("rr_encoder_sign", args.rr_encoder_sign),
    ]
    for name, value in params:
        send_param(dev, name, value)


def main() -> int:
    args = parse_args()
    duration_s = max(0.05, min(2.0, float(args.duration_s)))
    omega = abs(float(args.omega_radps))
    if args.direction == "right":
        omega = -omega

    print(WARNING, file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.2) as dev:
        write_line(dev, "CMD STOP\n")
        time.sleep(0.2)
        sync_params(dev, args)
        write_line(dev, f"CMD V 0.000000 {omega:.6f}\n")
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            line = dev.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("S,"):
                print(line)
        write_line(dev, "CMD STOP\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
