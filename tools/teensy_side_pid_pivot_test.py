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
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args()


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
        dev.write(b"CMD STOP\n")
        dev.flush()
        time.sleep(0.2)
        dev.write(f"CMD V 0.000000 {omega:.6f}\n".encode("ascii"))
        dev.flush()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            line = dev.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("S,"):
                print(line)
        dev.write(b"CMD STOP\n")
        dev.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
