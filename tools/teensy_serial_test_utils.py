#!/usr/bin/env python3
"""Shared helpers for direct Teensy serial bench tools."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Callable

import serial


def add_common_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--param-timeout-s", type=float, default=2.0)
    parser.add_argument("--settle-s", type=float, default=0.6)


def open_teensy_serial(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(port, baud, timeout=0.1, write_timeout=0.2, exclusive=True)
    except TypeError:
        return serial.Serial(port, baud, timeout=0.1, write_timeout=0.2)


def write_line(dev: serial.Serial, line: str) -> None:
    dev.write(line.encode("ascii"))
    dev.flush()


def settle_serial(dev: serial.Serial, settle_s: float) -> None:
    time.sleep(max(0.0, float(settle_s)))
    try:
        dev.reset_input_buffer()
    except serial.SerialException:
        raise
    except Exception:
        pass


def send_param(dev: serial.Serial, name: str, value: Any, timeout_s: float) -> None:
    try:
        dev.reset_input_buffer()
    except Exception:
        pass
    write_line(dev, f"CMD PARAM {name} {value}\n")
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() < deadline:
        line = dev.readline().decode("utf-8", errors="replace").strip()
        if line == f"PARAM,{name},ok":
            return
        if line == f"PARAM,{name},unknown":
            raise RuntimeError(f"Teensy rejected parameter {name}")
    raise TimeoutError(f"Timed out waiting for PARAM ACK: {name}")


def sync_standard_params(dev: serial.Serial, args: argparse.Namespace) -> None:
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
        send_param(dev, name, value, args.param_timeout_s)


def run_with_serial_errors(fn: Callable[[], int]) -> int:
    try:
        return fn()
    except (serial.SerialException, TimeoutError, RuntimeError) as exc:
        print(f"Serial bench test failed: {exc}", file=sys.stderr)
        print(
            "Close ROS motor bridge, Arduino Serial Monitor, and any other process using the Teensy port, then retry.",
            file=sys.stderr,
        )
        return 1
