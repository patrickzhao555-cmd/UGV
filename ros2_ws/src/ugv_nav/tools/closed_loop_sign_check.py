#!/usr/bin/env python3
"""Bench helper for checking competition closed-loop sign conventions.

This tool publishes small velocity commands and watches /ugv_nav_status pose
updates. Run it only in a clear bench/ground setup where short motion is safe.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Dict, List, Optional, Tuple


def wrap_to_pi(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def pose_from_status(payload: str) -> Optional[Tuple[float, float, float]]:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    pose = data.get("pose_m")
    if not isinstance(pose, list) or len(pose) < 3:
        return None
    try:
        return float(pose[0]), float(pose[1]), math.radians(float(pose[2]))
    except (TypeError, ValueError):
        return None


def delta_pose(before: Tuple[float, float, float], after: Tuple[float, float, float]) -> Dict[str, float]:
    return {
        "dx_m": after[0] - before[0],
        "dy_m": after[1] - before[1],
        "dyaw_deg": math.degrees(wrap_to_pi(after[2] - before[2])),
    }


def recommend_signs(pos_delta: Dict[str, float], neg_delta: Dict[str, float]) -> Dict[str, float]:
    pos_yaw = pos_delta["dyaw_deg"]
    neg_yaw = neg_delta["dyaw_deg"]
    omega_sign = 1.0 if pos_yaw >= abs(neg_yaw) * -0.25 else -1.0
    if pos_yaw < 0.0 and neg_yaw > 0.0:
        omega_sign = -1.0

    # For the lower-left round2/round3 sweep frame, positive omega while moving
    # forward should generally arc toward increasing y.
    lane_correction_sign = 1.0 if pos_delta["dy_m"] >= neg_delta["dy_m"] else -1.0
    return {
        "NAV_OMEGA_COMMAND_SIGN": omega_sign,
        "NAV_HEADING_ERROR_SIGN": 1.0,
        "NAV_LANE_ERROR_SIGN": 1.0,
        "NAV_LANE_CORRECTION_SIGN": lane_correction_sign,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Check closed-loop omega/lane sign response from /ugv_nav_status odom.")
    parser.add_argument("--cmd-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--v-mps", type=float, default=0.08)
    parser.add_argument("--omega-radps", type=float, default=0.18)
    parser.add_argument("--duration-s", type=float, default=1.2)
    parser.add_argument("--settle-s", type=float, default=0.6)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.dry_run:
        print("Dry run only. Commands that would be sent:")
        for omega in (0.0, args.omega_radps, -args.omega_radps):
            print(json.dumps({"mode": "VELOCITY", "command_type": "velocity", "v_mps": args.v_mps, "omega_radps": omega, "controller": "velocity"}))
        print(json.dumps({"mode": "STOP", "command_type": "stop"}))
        return

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SignCheckNode(Node):
        def __init__(self) -> None:
            super().__init__("closed_loop_sign_check")
            self.pub = self.create_publisher(String, args.cmd_topic, 10)
            self.poses: List[Tuple[float, float, float]] = []
            self.create_subscription(String, args.status_topic, self._status_cb, 10)

        def _status_cb(self, msg: String) -> None:
            pose = pose_from_status(msg.data)
            if pose is not None:
                self.poses.append(pose)
                self.poses = self.poses[-100:]

        def send_velocity(self, omega: float) -> None:
            msg = String()
            msg.data = json.dumps(
                {
                    "mode": "VELOCITY",
                    "command_type": "velocity",
                    "v_mps": float(args.v_mps),
                    "omega_radps": float(omega),
                    "controller": "velocity",
                    "reason": "closed_loop_sign_check",
                }
            )
            self.pub.publish(msg)

        def stop(self) -> None:
            msg = String()
            msg.data = json.dumps({"mode": "STOP", "command_type": "stop", "reason": "closed_loop_sign_check_done"})
            self.pub.publish(msg)

    def wait_for_pose(node: SignCheckNode, timeout_s: float = 3.0) -> Tuple[float, float, float]:
        end = time.time() + timeout_s
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.poses:
                return node.poses[-1]
        raise RuntimeError(f"No pose received on {args.status_topic}")

    def run_segment(node: SignCheckNode, omega: float) -> Dict[str, float]:
        before = wait_for_pose(node)
        end = time.time() + max(0.1, float(args.duration_s))
        while time.time() < end:
            node.send_velocity(omega)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.stop()
        settle_end = time.time() + max(0.0, float(args.settle_s))
        while time.time() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        after = wait_for_pose(node, timeout_s=0.5)
        return delta_pose(before, after)

    rclpy.init()
    node = SignCheckNode()
    try:
        wait_for_pose(node)
        print("Sending small forward command...")
        forward_delta = run_segment(node, 0.0)
        print("Sending small positive omega command...")
        positive_delta = run_segment(node, abs(float(args.omega_radps)))
        print("Sending small negative omega command...")
        negative_delta = run_segment(node, -abs(float(args.omega_radps)))
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rec = recommend_signs(positive_delta, negative_delta)
    print("Observed deltas:")
    print(f"  forward:       {forward_delta}")
    print(f"  positive omega:{positive_delta}")
    print(f"  negative omega:{negative_delta}")
    print("Recommended environment settings:")
    for key, value in rec.items():
        print(f"  {key}={value:.1f}")


if __name__ == "__main__":
    main()
