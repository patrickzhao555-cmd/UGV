#!/usr/bin/env python3
"""Estimate effective track width from a measured turn.

Run on the robot in a clear area. The tool sends a controlled velocity turn,
reads odometry yaw from /ugv_nav_status, then asks for the measured physical yaw
change. It prints a recommended ROBOT_TRACK_WIDTH_M.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any, Dict, Optional


def wrap_degrees(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def finite_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def pose_yaw_deg_from_status(payload: str) -> Optional[float]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    pose = data.get("pose_m")
    if not isinstance(pose, list) or len(pose) < 3:
        return None
    return finite_float(pose[2])


def track_width_from_status(payload: str) -> Optional[float]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    for key in ("active_robot_track_width_m", "robot_track_width_m"):
        value = finite_float(data.get(key))
        if value is not None and value > 0.0:
            return value
    closed_loop = data.get("competition_closed_loop")
    if isinstance(closed_loop, dict):
        value = finite_float(closed_loop.get("active_robot_track_width_m"))
        if value is not None and value > 0.0:
            return value
    return None


def recommended_track_width_m(current_track_width_m: float, odom_yaw_delta_deg: float, measured_yaw_delta_deg: float) -> float:
    measured = float(measured_yaw_delta_deg)
    if abs(measured) < 1e-6:
        raise ValueError("measured_yaw_delta_deg must be nonzero")
    return float(current_track_width_m) * float(odom_yaw_delta_deg) / measured


def velocity_command(v_mps: float, omega_radps: float, reason: str) -> str:
    return json.dumps(
        {
            "mode": "VELOCITY",
            "command_type": "velocity",
            "v_mps": float(v_mps),
            "omega_radps": float(omega_radps),
            "controller": "velocity",
            "reason": reason,
        }
    )


def stop_command(reason: str) -> str:
    return json.dumps({"mode": "STOP", "command_type": "stop", "reason": reason})


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate effective ROBOT_TRACK_WIDTH_M from odom yaw vs physical yaw.")
    parser.add_argument("--cmd-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--current-track-width-m", type=float, default=0.6096)
    parser.add_argument("--v-mps", type=float, default=0.0)
    parser.add_argument("--omega-radps", type=float, default=0.25)
    parser.add_argument("--duration-s", type=float, default=2.5)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--command-period-s", type=float, default=0.05)
    parser.add_argument("--measured-yaw-delta-deg", type=float, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.dry_run:
        print(velocity_command(args.v_mps, args.omega_radps, "turn_odom_calibration"))
        print(stop_command("turn_odom_calibration_done"))
        return

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class CalibrationNode(Node):
        def __init__(self) -> None:
            super().__init__("turn_odom_calibration")
            self.pub = self.create_publisher(String, args.cmd_topic, 10)
            self.latest_payload = ""
            self.latest_yaw_deg: Optional[float] = None
            self.latest_track_width_m: Optional[float] = None
            self.create_subscription(String, args.status_topic, self._status_cb, 10)

        def _status_cb(self, msg: String) -> None:
            self.latest_payload = msg.data
            yaw = pose_yaw_deg_from_status(msg.data)
            if yaw is not None:
                self.latest_yaw_deg = yaw
            track_width = track_width_from_status(msg.data)
            if track_width is not None:
                self.latest_track_width_m = track_width

        def publish_json(self, payload: str) -> None:
            msg = String()
            msg.data = payload
            self.pub.publish(msg)

    def wait_for_yaw(node: CalibrationNode, timeout_s: float = 5.0) -> float:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest_yaw_deg is not None:
                return float(node.latest_yaw_deg)
        raise RuntimeError(f"No yaw received on {args.status_topic}")

    rclpy.init()
    node = CalibrationNode()
    try:
        start_yaw = wait_for_yaw(node)
        current_track_width = node.latest_track_width_m or max(1e-6, float(args.current_track_width_m))
        print(f"Start odom yaw: {start_yaw:.3f} deg")
        print(f"Active track width: {current_track_width:.4f} m")

        end_time = time.time() + max(0.1, float(args.duration_s))
        command_payload = velocity_command(args.v_mps, args.omega_radps, "turn_odom_calibration")
        while time.time() < end_time:
            node.publish_json(command_payload)
            rclpy.spin_once(node, timeout_sec=max(0.01, float(args.command_period_s)))

        node.publish_json(stop_command("turn_odom_calibration_done"))
        settle_end = time.time() + max(0.0, float(args.settle_s))
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        end_yaw = wait_for_yaw(node, timeout_s=0.5)
    finally:
        try:
            node.publish_json(stop_command("turn_odom_calibration_shutdown"))
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    odom_delta = wrap_degrees(end_yaw - start_yaw)
    print(f"End odom yaw: {end_yaw:.3f} deg")
    print(f"Odom yaw delta: {odom_delta:.3f} deg")

    measured_delta = args.measured_yaw_delta_deg
    if measured_delta is None:
        measured_delta = float(input("Measured physical yaw change in degrees, signed to match turn direction: ").strip())
    rec = recommended_track_width_m(current_track_width, odom_delta, measured_delta)
    print(f"Physical yaw delta: {float(measured_delta):.3f} deg")
    print(f"Recommended ROBOT_TRACK_WIDTH_M={rec:.4f}")
    print(f"Formula: {current_track_width:.4f} * {odom_delta:.3f} / {float(measured_delta):.3f}")


if __name__ == "__main__":
    main()
