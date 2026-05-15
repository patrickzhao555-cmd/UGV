#!/usr/bin/env python3
import json
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ugv_sensor_sync.msg import EncoderTicksStamped


MOTION_HELP = (
    "forward, backward, turn_left, turn_right, left_forward, left_backward, "
    "right_forward, right_backward, raw, sequence"
)


class MotorDirectTest(Node):
    """Publish bounded direct motor commands for drivetrain bring-up tests."""

    def __init__(self):
        super().__init__("motor_direct_test")

        self.declare_parameter("motion", "forward")
        self.declare_parameter("raw", 0.22)
        self.declare_parameter("raw_left", 0.0)
        self.declare_parameter("raw_right", 0.0)
        self.declare_parameter("duration_s", 0.8)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("startup_wait_s", 1.5)
        self.declare_parameter("stop_after_s", 0.6)
        self.declare_parameter("command_topic", "/ugv_nav_cmd")
        self.declare_parameter("status_topic", "/motor_controller/status")
        self.declare_parameter("encoder_topic", "/encoder_ticks_stamped")
        self.declare_parameter("yes", False)

        self.motion = str(self.get_parameter("motion").value).strip().lower()
        self.raw = self._clamp_raw(float(self.get_parameter("raw").value))
        self.raw_left = self._clamp_raw(float(self.get_parameter("raw_left").value))
        self.raw_right = self._clamp_raw(float(self.get_parameter("raw_right").value))
        self.duration_s = self._clamp_duration(float(self.get_parameter("duration_s").value))
        self.rate_hz = max(2.0, min(50.0, float(self.get_parameter("rate_hz").value)))
        self.startup_wait_s = max(0.0, min(10.0, float(self.get_parameter("startup_wait_s").value)))
        self.stop_after_s = max(0.2, min(2.0, float(self.get_parameter("stop_after_s").value)))
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.yes = bool(self.get_parameter("yes").value)

        self.pub = self.create_publisher(String, self.command_topic, 10)
        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        self.create_subscription(EncoderTicksStamped, self.encoder_topic, self._encoder_callback, 10)

        self.latest_status: Optional[dict] = None
        self.first_encoder: Optional[Tuple[int, int, int, int, int, int]] = None
        self.latest_encoder: Optional[Tuple[int, int, int, int, int, int]] = None
        self.sent_motion_packets = 0
        self.sent_stop_packets = 0
        self.start_time = time.monotonic()
        self.phase = "startup_wait"
        self.done = False

        if not self.yes:
            self.get_logger().error(
                "Refusing to move motors without yes:=true. Lift the UGV or clear the area, "
                "then rerun with yes:=true."
            )
            self.done = True
            return

        self.command = self._build_command(self.motion)
        self.get_logger().warn(
            f"Direct motor test armed: motion={self.motion}, raw_left={self.command['raw_left']:.3f}, "
            f"raw_right={self.command['raw_right']:.3f}, duration={self.duration_s:.2f}s. "
            "No navigation/LiDAR safety is active."
        )
        self.get_logger().warn(
            "Use this with the robot lifted, or with a clear floor and an emergency stop ready."
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    @staticmethod
    def _clamp_raw(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-0.45, min(0.45, value))

    @staticmethod
    def _clamp_duration(value: float) -> float:
        if not math.isfinite(value):
            return 0.8
        return max(0.1, min(3.0, value))

    def _build_command(self, motion: str) -> Dict[str, float]:
        raw = abs(self.raw)
        commands: Dict[str, Tuple[str, float, float]] = {
            "forward": ("FORWARD", raw, raw),
            "backward": ("BACKWARD", -raw, -raw),
            "turn_left": ("TURN_LEFT", -raw, raw),
            "turn_right": ("TURN_RIGHT", raw, -raw),
            "left_forward": ("LEFT_FORWARD", raw, 0.0),
            "left_backward": ("LEFT_BACKWARD", -raw, 0.0),
            "right_forward": ("RIGHT_FORWARD", 0.0, raw),
            "right_backward": ("RIGHT_BACKWARD", 0.0, -raw),
            "raw": ("RAW", self.raw_left, self.raw_right),
        }
        if motion not in commands and motion != "sequence":
            raise ValueError(f"Unsupported motion {motion!r}. Expected one of: {MOTION_HELP}")
        if motion == "sequence":
            mode, left, right = "FORWARD", raw, raw
        else:
            mode, left, right = commands[motion]
        return {
            "mode": mode,
            "turn_deg": 0.0,
            "move_m": 0.0,
            "raw_left": float(left),
            "raw_right": float(right),
            "reason": f"direct motor test {motion}",
            "controller": "motor_direct_test",
        }

    def _status_callback(self, msg: String) -> None:
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def _encoder_callback(self, msg: EncoderTicksStamped) -> None:
        vals = (
            int(msg.left_ticks),
            int(msg.right_ticks),
            int(msg.front_left_ticks),
            int(msg.front_right_ticks),
            int(msg.rear_left_ticks),
            int(msg.rear_right_ticks),
        )
        if self.first_encoder is None:
            self.first_encoder = vals
        self.latest_encoder = vals

    def _publish(self, packet: dict) -> None:
        self.pub.publish(String(data=json.dumps(packet)))

    def _stop_packet(self, reason: str = "direct motor test stop") -> dict:
        return {
            "mode": "STOP",
            "turn_deg": 0.0,
            "move_m": 0.0,
            "raw_left": 0.0,
            "raw_right": 0.0,
            "reason": reason,
            "controller": "motor_direct_test",
        }

    def _sequence_command(self, elapsed: float) -> dict:
        quarter = self.duration_s / 4.0
        raw = abs(self.raw)
        if elapsed < quarter:
            left, right, mode = raw, raw, "FORWARD"
        elif elapsed < 2.0 * quarter:
            left, right, mode = -raw, -raw, "BACKWARD"
        elif elapsed < 3.0 * quarter:
            left, right, mode = -raw, raw, "TURN_LEFT"
        else:
            left, right, mode = raw, -raw, "TURN_RIGHT"
        return {
            "mode": mode,
            "turn_deg": 0.0,
            "move_m": 0.0,
            "raw_left": left,
            "raw_right": right,
            "reason": f"direct motor test sequence {mode.lower()}",
            "controller": "motor_direct_test",
        }

    def _tick(self) -> None:
        if self.done:
            return

        now = time.monotonic()
        elapsed = now - self.start_time
        if self.phase == "startup_wait":
            self._publish(self._stop_packet("direct motor test pre-start stop"))
            self.sent_stop_packets += 1
            if elapsed >= self.startup_wait_s:
                self.phase = "moving"
                self.phase_start = now
                status = self.latest_status or {}
                self.get_logger().info(
                    "Starting motion. Motor status: "
                    f"connected={status.get('connected')} dry_run={status.get('dry_run')} "
                    f"last_pwm={status.get('last_pwm')} target_pwm={status.get('target_pwm')}"
                )
            return

        if self.phase == "moving":
            move_elapsed = now - self.phase_start
            packet = self._sequence_command(move_elapsed) if self.motion == "sequence" else self.command
            self._publish(packet)
            self.sent_motion_packets += 1
            if move_elapsed >= self.duration_s:
                self.phase = "stopping"
                self.phase_start = now
            return

        if self.phase == "stopping":
            self._publish(self._stop_packet())
            self.sent_stop_packets += 1
            if now - self.phase_start >= self.stop_after_s:
                self._report_summary()
                self.done = True
                rclpy.shutdown()

    def _report_summary(self) -> None:
        delta_text = "encoder_delta=unavailable"
        if self.first_encoder is not None and self.latest_encoder is not None:
            delta = tuple(b - a for a, b in zip(self.first_encoder, self.latest_encoder))
            delta_text = (
                f"encoder_delta left/right={delta[0]}/{delta[1]}, "
                f"raw fl/fr/rl/rr={delta[2]}/{delta[3]}/{delta[4]}/{delta[5]}"
            )
        status = self.latest_status or {}
        self.get_logger().info(
            f"Direct motor test complete: motion_packets={self.sent_motion_packets}, "
            f"stop_packets={self.sent_stop_packets}, {delta_text}, "
            f"last_pwm={status.get('last_pwm')}, target_pwm={status.get('target_pwm')}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorDirectTest()
        if node.done:
            return
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except ValueError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
    finally:
        if node is not None:
            for _ in range(5):
                node._publish(node._stop_packet("direct motor test shutdown stop"))
                time.sleep(0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
