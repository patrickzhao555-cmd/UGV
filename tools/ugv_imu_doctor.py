#!/usr/bin/env python3
"""Check whether the UGV heading IMU is actually available on ROS 2."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from typing import Any, Iterable, Optional


def _finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def rate_hz(times_s: Iterable[float]) -> float:
    samples = list(times_s)
    if len(samples) < 2:
        return 0.0
    elapsed_s = samples[-1] - samples[0]
    if elapsed_s <= 0.0:
        return 0.0
    return float(len(samples) - 1) / elapsed_s


def diagnose_imu_health(
    *,
    imu_messages: int,
    imu_rate_hz: float,
    min_rate_hz: float,
    zed_status: Optional[dict[str, Any]],
) -> tuple[bool, str]:
    if imu_messages > 0 and imu_rate_hz >= max(0.0, float(min_rate_hz)):
        return True, "imu_ok"
    if zed_status is None:
        return False, "zed_status_missing"
    zed_count = int(zed_status.get("imu_count") or 0)
    zed_error = str(zed_status.get("last_imu_error") or "")
    if zed_count <= 0:
        if zed_error:
            return False, f"zed_imu_not_publishing:{zed_error}"
        return False, "zed_imu_not_publishing"
    if imu_messages <= 0:
        return False, "imu_topic_missing_or_qos_mismatch"
    return False, "imu_rate_low"


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-topic", default="/zed/imu")
    parser.add_argument("--zed-status-topic", default="/zed/status")
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--min-rate-hz", type=float, default=20.0)
    parser.add_argument("--qos", choices=["sensor_data", "default"], default="sensor_data")
    return parser.parse_args(list(argv) if argv is not None else None)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)

    import rclpy  # type: ignore
    from rclpy.qos import qos_profile_sensor_data  # type: ignore
    from sensor_msgs.msg import Imu  # type: ignore
    from std_msgs.msg import String  # type: ignore

    rclpy.init()
    node = rclpy.create_node("ugv_imu_doctor")
    qos = qos_profile_sensor_data if args.qos == "sensor_data" else 10
    arrivals_s: deque[float] = deque()
    zed_status: Optional[dict[str, Any]] = None
    last_imu: Optional[Imu] = None

    def imu_callback(msg: Imu) -> None:
        nonlocal last_imu
        arrivals_s.append(time.monotonic())
        last_imu = msg

    def zed_status_callback(msg: String) -> None:
        nonlocal zed_status
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"raw": msg.data}
        zed_status = payload if isinstance(payload, dict) else {"raw": payload}

    node.create_subscription(Imu, str(args.imu_topic), imu_callback, qos)
    node.create_subscription(String, str(args.zed_status_topic), zed_status_callback, 10)

    deadline = time.monotonic() + max(0.5, float(args.duration_s))
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    observed_rate = rate_hz(arrivals_s)
    ok, reason = diagnose_imu_health(
        imu_messages=len(arrivals_s),
        imu_rate_hz=observed_rate,
        min_rate_hz=float(args.min_rate_hz),
        zed_status=zed_status,
    )

    print("UGV IMU doctor")
    print(f"  imu_topic={args.imu_topic} qos={args.qos}")
    print(f"  imu_messages={len(arrivals_s)} imu_rate_hz={observed_rate:.2f} min_rate_hz={float(args.min_rate_hz):.2f}")
    if last_imu is not None:
        print(
            "  last_angular_velocity_radps="
            f"({last_imu.angular_velocity.x:.5f}, {last_imu.angular_velocity.y:.5f}, {last_imu.angular_velocity.z:.5f})"
        )
        print(
            "  last_linear_accel_mps2="
            f"({last_imu.linear_acceleration.x:.3f}, {last_imu.linear_acceleration.y:.3f}, {last_imu.linear_acceleration.z:.3f})"
        )
    if zed_status is None:
        print(f"  zed_status_topic={args.zed_status_topic} missing")
    else:
        print(
            f"  zed_status_topic={args.zed_status_topic} "
            f"camera_model={zed_status.get('camera_model')} "
            f"zed_imu_count={zed_status.get('imu_count')} "
            f"zed_imu_rate_hz={zed_status.get('imu_rate_hz')} "
            f"zed_imu_age_s={zed_status.get('imu_age_s')} "
            f"zed_failures={zed_status.get('imu_publish_failures')} "
            f"zed_error={zed_status.get('last_imu_error')}"
        )
        raw_degps = zed_status.get("last_imu_ang_degps")
        if isinstance(raw_degps, list):
            print(f"  zed_raw_gyro_degps={raw_degps}")

    print(f"  verdict={'PASS' if ok else 'FAIL'} reason={reason}")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
