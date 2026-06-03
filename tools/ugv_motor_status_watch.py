#!/usr/bin/env python3
"""Compact motor status watcher for left/right side PID debugging."""

from __future__ import annotations

import argparse
import json
import time
from typing import Any, Dict, Iterable, Optional


DEFAULT_STATUS_TOPIC = "/motor_controller/status"


def _float_field(data: Dict[str, Any], key: str) -> Optional[float]:
    value = data.get(key)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _fmt(value: Optional[float], width: int = 7, precision: int = 3) -> str:
    if value is None:
        return " " * (width - 1) + "-"
    return f"{value:{width}.{precision}f}"


def _fmt_int(value: Any, width: int = 4) -> str:
    try:
        return f"{int(value):{width}d}"
    except (TypeError, ValueError):
        return " " * (width - 1) + "-"


def summarize_status(data: Dict[str, Any]) -> str:
    target_l = _float_field(data, "target_left_mps")
    target_r = _float_field(data, "target_right_mps")
    measured_l = _float_field(data, "measured_left_mps")
    measured_r = _float_field(data, "measured_right_mps")
    err_l = None if target_l is None or measured_l is None else target_l - measured_l
    err_r = None if target_r is None or measured_r is None else target_r - measured_r
    return (
        f"cmd v={_fmt(_float_field(data, 'commanded_v_mps'), 6, 3)} "
        f"w={_fmt(_float_field(data, 'commanded_omega_radps'), 7, 3)} | "
        f"target L/R={_fmt(target_l)}/{_fmt(target_r)} m/s | "
        f"meas L/R={_fmt(measured_l)}/{_fmt(measured_r)} m/s | "
        f"err L/R={_fmt(err_l)}/{_fmt(err_r)} | "
        f"pwm L/R={_fmt_int(data.get('left_pwm'))}/{_fmt_int(data.get('right_pwm'))} | "
        f"fault={data.get('fault_reason') or data.get('fault') or 'none'}"
    )


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print compact /motor_controller/status side PID telemetry.")
    parser.add_argument("--topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--hz", type=float, default=5.0)
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> None:
    import rclpy  # type: ignore
    from std_msgs.msg import String  # type: ignore

    args = parse_args(argv)
    min_period_s = 1.0 / max(0.2, float(args.hz))
    last_print_s = 0.0

    rclpy.init()
    node = rclpy.create_node("ugv_motor_status_watch")

    def callback(msg: String) -> None:
        nonlocal last_print_s
        now_s = time.monotonic()
        if now_s - last_print_s < min_period_s:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        print(summarize_status(data), flush=True)
        last_print_s = now_s

    node.create_subscription(String, str(args.topic), callback, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
