#!/usr/bin/env python3
"""Bounded velocity step test for the Teensy side-PID firmware."""

import argparse
import sys
import time
from typing import Optional

import serial


WARNING = "WARNING: first run must be with wheels off the ground."


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--v-mps", type=float, default=0.12)
    parser.add_argument("--omega-radps", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--kp", type=float)
    parser.add_argument("--ki", type=float)
    parser.add_argument("--kd", type=float)
    parser.add_argument("--ff-us-per-tps", type=float)
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args()


def write_line(dev: serial.Serial, line: str) -> None:
    dev.write(line.encode("ascii"))
    dev.flush()


def maybe_param(dev: serial.Serial, name: str, value: Optional[float]) -> None:
    if value is not None:
        write_line(dev, f"CMD PARAM {name} {float(value):.6f}\n")
        time.sleep(0.05)


def main() -> int:
    args = parse_args()
    duration_s = max(0.05, min(3.0, float(args.duration_s)))
    print(WARNING, file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=0.2) as dev:
        write_line(dev, "CMD STOP\n")
        time.sleep(0.2)
        maybe_param(dev, "kp", args.kp)
        maybe_param(dev, "ki", args.ki)
        maybe_param(dev, "kd", args.kd)
        maybe_param(dev, "ff_us_per_tps", args.ff_us_per_tps)
        write_line(dev, f"CMD V {float(args.v_mps):.6f} {float(args.omega_radps):.6f}\n")
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            line = dev.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("S,"):
                print(line)
        write_line(dev, "CMD STOP\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
