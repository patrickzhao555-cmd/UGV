#!/usr/bin/env python3
"""Clean-slate navigation placeholder.

The old multi-mode navigator was intentionally removed on the cleanup branch.
This file remains only as a safe ROS entrypoint while the new Jetson high-level
navigation stack is designed.

It publishes STOP commands by default and never emits raw PWM. The future active
contract is velocity-only JSON:

    {"command_type": "velocity", "v_mps": <float>, "omega_radps": <float>}
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ControlCommand:
    command_type: str = "stop"
    v_mps: float = 0.0
    omega_radps: float = 0.0
    reason: str = "clean_slate_nav_placeholder"

    def to_json(self) -> str:
        mode = "STOP" if self.command_type == "stop" else "VELOCITY"
        return json.dumps(
            {
                "mode": mode,
                "command_type": self.command_type,
                "controller": "clean_nav_placeholder",
                "v_mps": float(self.v_mps),
                "omega_radps": float(self.omega_radps),
                "reason": self.reason,
            },
            sort_keys=True,
        )


def build_stop_command(reason: str = "clean_slate_nav_placeholder") -> ControlCommand:
    return ControlCommand(command_type="stop", reason=reason)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Clean-slate UGV navigation placeholder. Publishes STOP in real mode.",
        allow_abbrev=False,
    )
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--command-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--nav-status-period-s", type=float, default=1.0)
    parser.add_argument("--once", action="store_true", help="publish one STOP and exit in real mode")
    args, unknown = parser.parse_known_args()
    if unknown:
        print(f"Ignoring legacy navigation arguments: {' '.join(unknown)}")
    return args


def run_sim() -> None:
    print("Clean navigation placeholder: old sim/navigation stack removed on this branch.")


def run_real(args: argparse.Namespace) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python packages are required for real mode: {exc}") from exc

    class CleanNavNode(Node):
        def __init__(self) -> None:
            super().__init__("ugv_clean_nav_placeholder")
            self.cmd_pub = self.create_publisher(String, args.command_topic, 10)
            self.status_pub = self.create_publisher(String, args.status_topic, 10)
            self.timer = self.create_timer(max(0.1, float(args.nav_status_period_s)), self.tick)
            self.get_logger().warn(
                "Old navigation stack was removed on cleanup/two-side-pid-runtime; publishing STOP only."
            )

        def tick(self) -> None:
            cmd = build_stop_command()
            self.cmd_pub.publish(String(data=cmd.to_json()))
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "nav_runtime": "clean_slate_placeholder",
                            "active": False,
                            "command_contract": "velocity_only",
                            "last_command": json.loads(cmd.to_json()),
                            "timestamp_s": round(time.time(), 3),
                        },
                        sort_keys=True,
                    )
                )
            )
            if args.once:
                raise KeyboardInterrupt

    rclpy.init()
    node = CleanNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    args = parse_args()
    if args.mode == "real":
        run_real(args)
    else:
        run_sim()


if __name__ == "__main__":
    main()
