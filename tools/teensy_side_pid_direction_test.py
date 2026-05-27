#!/usr/bin/env python3
"""Bounded RAW2 direction test for the Teensy side-PID firmware."""

import argparse
import sys
import time

from teensy_serial_test_utils import (
    add_common_serial_args,
    open_teensy_serial,
    run_with_serial_errors,
    settle_serial,
    sync_standard_params,
    write_line,
)


WARNING = "WARNING: first run must be with wheels off the ground."


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--left-us", type=int, default=1560)
    parser.add_argument("--right-us", type=int, default=1560)
    parser.add_argument("--duration-s", type=float, default=0.6)
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
    add_common_serial_args(parser)
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    duration_s = max(0.05, min(2.0, float(args.duration_s)))
    print(WARNING, file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    with open_teensy_serial(args.port, args.baud) as dev:
        settle_serial(dev, args.settle_s)
        write_line(dev, "CMD STOP\n")
        time.sleep(0.3)
        sync_standard_params(dev, args)
        write_line(dev, f"CMD RAW2 {int(args.left_us)} {int(args.right_us)}\n")
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            line = dev.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("S,"):
                print(line)
        write_line(dev, "CMD STOP\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(run_with_serial_errors(main))
