#!/usr/bin/env python3
"""Bounded velocity step test for the Teensy side-PID firmware."""

import argparse
import sys
from typing import Optional

from teensy_serial_test_utils import (
    add_common_serial_args,
    open_teensy_serial,
    prepare_direct_bench_session,
    quiet_status_stream,
    run_with_serial_errors,
    safe_stop_and_restore_stream,
    send_param,
    stream_status_while_refreshing_command,
    sync_standard_params,
)


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
    parser.add_argument("--pid-output-limit-us", type=float)
    parser.add_argument("--track-width-m", type=float, default=0.416)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0825)
    parser.add_argument("--ticks-per-rev", type=int, default=3200)
    parser.add_argument("--control-hz", type=float, default=50.0)
    parser.add_argument("--left-motor-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-motor-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--fl-encoder-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--fr-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rl-encoder-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--rr-encoder-sign", type=int, choices=(-1, 1), default=1)
    add_common_serial_args(parser)
    parser.add_argument("--yes", action="store_true", help="confirm wheels are off ground and run")
    return parser.parse_args()


def maybe_param(dev, args: argparse.Namespace, name: str, value: Optional[float]) -> None:
    if value is not None:
        send_param(dev, name, float(value), args.param_timeout_s, args.param_retries)


def main() -> int:
    args = parse_args()
    duration_s = max(0.05, min(3.0, float(args.duration_s)))
    print(WARNING, file=sys.stderr)
    if not args.yes:
        print("Refusing to run without --yes.", file=sys.stderr)
        return 2

    with open_teensy_serial(args.port, args.baud) as dev:
        prepare_direct_bench_session(dev, args)
        try:
            sync_standard_params(dev, args)
            maybe_param(dev, args, "kp", args.kp)
            maybe_param(dev, args, "ki", args.ki)
            maybe_param(dev, args, "kd", args.kd)
            maybe_param(dev, args, "ff_us_per_tps", args.ff_us_per_tps)
            maybe_param(dev, args, "pid_output_limit_us", args.pid_output_limit_us)
            quiet_status_stream(dev, True)
            stream_status_while_refreshing_command(
                dev,
                f"CMD V {float(args.v_mps):.6f} {float(args.omega_radps):.6f}\n",
                duration_s,
                args.refresh_period_s,
            )
        finally:
            safe_stop_and_restore_stream(dev)
    return 0


if __name__ == "__main__":
    raise SystemExit(run_with_serial_errors(main))
