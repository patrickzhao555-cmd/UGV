#!/usr/bin/env python3
"""Receive Operation Touchdown UAV target coordinates from terminal or ESP.

This node is an input adapter only. It publishes validated marker-center
coordinates to ``/ugv/uav_target`` as ``geometry_msgs/PointStamped`` in meters.
The mission supervisor remains responsible for costmap, field-boundary, and
navigation decisions.
"""

from __future__ import annotations

import json
import queue
import sys
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - depends on robot image packages
    serial = None

from ugv_nav_core.uav_target_input import ParsedUavTarget, parse_uav_target_line


VALID_INPUT_MODES = {"disabled", "terminal", "serial", "both"}


class UavTargetReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_uav_target_receiver")

        self.declare_parameter("input_mode", "disabled")
        self.declare_parameter("output_topic", "/ugv/uav_target")
        self.declare_parameter("status_topic", "/ugv/uav_target_receiver/status")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("target_units", "meters")
        self.declare_parameter("require_checksum", False)
        self.declare_parameter("drop_duplicate_sequences", True)
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_reconnect_period_s", 1.0)
        self.declare_parameter("serial_read_timeout_s", 0.05)
        self.declare_parameter("serial_max_line_bytes", 256)
        self.declare_parameter("terminal_prompt", True)
        self.declare_parameter("status_period_s", 0.5)

        self.input_mode = str(self.get_parameter("input_mode").value).strip().lower()
        if self.input_mode not in VALID_INPUT_MODES:
            self.get_logger().error(f"Unsupported input_mode={self.input_mode!r}; using disabled.")
            self.input_mode = "disabled"
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value).strip() or "map"
        self.target_units = str(self.get_parameter("target_units").value).strip() or "meters"
        self.require_checksum = bool(self.get_parameter("require_checksum").value)
        self.drop_duplicate_sequences = bool(self.get_parameter("drop_duplicate_sequences").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.serial_baud = int(self.get_parameter("serial_baud").value)
        self.serial_reconnect_period_s = max(0.1, float(self.get_parameter("serial_reconnect_period_s").value))
        self.serial_read_timeout_s = max(0.0, float(self.get_parameter("serial_read_timeout_s").value))
        self.serial_max_line_bytes = max(32, int(self.get_parameter("serial_max_line_bytes").value))
        self.terminal_prompt = bool(self.get_parameter("terminal_prompt").value)
        self.status_period_s = max(0.1, float(self.get_parameter("status_period_s").value))

        self.target_pub = self.create_publisher(PointStamped, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.accepted_count = 0
        self.rejected_count = 0
        self.duplicate_count = 0
        self.last_seq: Optional[int] = None
        self.last_target: Optional[dict] = None
        self.last_error: Optional[str] = None
        self.last_source: Optional[str] = None
        self.last_line_s: Optional[float] = None
        self.serial_connected = False
        self.serial_error: Optional[str] = None
        self._serial_device = None
        self._serial_buffer = b""
        self._last_serial_attempt_s = 0.0
        self._terminal_queue: "queue.Queue[str]" = queue.Queue()
        self._terminal_thread: Optional[threading.Thread] = None
        self._shutdown = False

        if self._terminal_enabled:
            self._start_terminal_thread()

        self.create_timer(0.02, self._poll_inputs)
        self.create_timer(self.status_period_s, self._publish_status)

        self.get_logger().warn(
            "UAV target receiver started "
            f"(mode={self.input_mode}, output={self.output_topic}, frame={self.frame_id}, "
            f"units={self.target_units}, serial={self.serial_port}@{self.serial_baud})"
        )

    @property
    def _terminal_enabled(self) -> bool:
        return self.input_mode in {"terminal", "both"}

    @property
    def _serial_enabled(self) -> bool:
        return self.input_mode in {"serial", "both"}

    def destroy_node(self) -> bool:
        self._shutdown = True
        self._close_serial()
        return super().destroy_node()

    def _start_terminal_thread(self) -> None:
        self._terminal_thread = threading.Thread(target=self._terminal_loop, name="uav_target_terminal", daemon=True)
        self._terminal_thread.start()

    def _terminal_loop(self) -> None:
        if self.terminal_prompt:
            print(
                "UGV target terminal input: enter 'x y', 'x,y', 'TARGET,x,y', "
                "or JSON {'x_m':5,'y_m':7}. Ctrl-D/Ctrl-C to stop.",
                file=sys.stderr,
                flush=True,
            )
        while not self._shutdown:
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if line == "":
                return
            self._terminal_queue.put(line)

    def _poll_inputs(self) -> None:
        while True:
            try:
                line = self._terminal_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line, source="terminal")

        if self._serial_enabled:
            self._poll_serial()

    def _poll_serial(self) -> None:
        if serial is None:
            self.serial_connected = False
            self.serial_error = "pyserial_missing"
            return
        if self._serial_device is None:
            self._try_open_serial()
            return
        try:
            chunk = self._serial_device.read(self.serial_max_line_bytes)
        except serial.SerialException as exc:
            self.serial_error = f"serial_read_failed:{exc}"
            self._close_serial()
            return
        if not chunk:
            return
        self._serial_buffer += chunk
        if len(self._serial_buffer) > self.serial_max_line_bytes * 4:
            self._serial_buffer = self._serial_buffer[-self.serial_max_line_bytes :]
            self.serial_error = "serial_line_overflow"
        while b"\n" in self._serial_buffer:
            raw_line, self._serial_buffer = self._serial_buffer.split(b"\n", 1)
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if line:
                self._handle_line(line, source="serial")

    def _try_open_serial(self) -> None:
        now_s = time.monotonic()
        if now_s - self._last_serial_attempt_s < self.serial_reconnect_period_s:
            return
        self._last_serial_attempt_s = now_s
        try:
            self._serial_device = serial.Serial(
                self.serial_port,
                self.serial_baud,
                timeout=self.serial_read_timeout_s,
                write_timeout=0.1,
            )
            self.serial_connected = True
            self.serial_error = None
            self._serial_buffer = b""
            self.get_logger().info(f"UAV ESP serial opened: {self.serial_port} @ {self.serial_baud}")
        except serial.SerialException as exc:
            self.serial_connected = False
            self.serial_error = f"serial_open_failed:{exc}"
            self._serial_device = None

    def _close_serial(self) -> None:
        if self._serial_device is not None:
            try:
                self._serial_device.close()
            except Exception:
                pass
        self._serial_device = None
        self.serial_connected = False
        self._serial_buffer = b""

    def _handle_line(self, line: str, *, source: str) -> None:
        parsed = parse_uav_target_line(
            line,
            default_units=self.target_units,
            require_checksum=self.require_checksum,
        )
        self.last_line_s = time.monotonic()
        self.last_source = source
        if not parsed.accepted:
            self.rejected_count += 1
            self.last_error = f"{source}:{parsed.reason}"
            self._publish_status()
            self.get_logger().warn(f"Rejected {source} UAV target line: {parsed.reason}")
            return
        if self._is_duplicate(parsed):
            self.duplicate_count += 1
            self.last_error = f"{source}:duplicate_sequence"
            self._publish_status()
            return
        self._publish_target(parsed, source=source)

    def _is_duplicate(self, parsed: ParsedUavTarget) -> bool:
        if not self.drop_duplicate_sequences or parsed.seq is None:
            return False
        if self.last_seq is not None and parsed.seq <= self.last_seq:
            return True
        self.last_seq = parsed.seq
        return False

    def _publish_target(self, parsed: ParsedUavTarget, *, source: str) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(parsed.x_m)
        msg.point.y = float(parsed.y_m)
        msg.point.z = 0.0
        self.target_pub.publish(msg)

        self.accepted_count += 1
        self.last_error = None
        self.last_target = {
            "x_m": float(parsed.x_m),
            "y_m": float(parsed.y_m),
            "seq": parsed.seq,
            "source": source,
            "input_format": parsed.source_format,
        }
        self.get_logger().info(
            f"Published UAV target from {source}: x={parsed.x_m:.3f}m y={parsed.y_m:.3f}m seq={parsed.seq}"
        )
        self._publish_status()

    def _publish_status(self) -> None:
        payload = {
            "node": "ugv_uav_target_receiver",
            "input_mode": self.input_mode,
            "terminal_enabled": self._terminal_enabled,
            "serial_enabled": self._serial_enabled,
            "serial_connected": self.serial_connected,
            "serial_port": self.serial_port,
            "serial_baud": self.serial_baud,
            "serial_error": self.serial_error,
            "output_topic": self.output_topic,
            "frame_id": self.frame_id,
            "target_units": self.target_units,
            "require_checksum": self.require_checksum,
            "drop_duplicate_sequences": self.drop_duplicate_sequences,
            "accepted_count": self.accepted_count,
            "rejected_count": self.rejected_count,
            "duplicate_count": self.duplicate_count,
            "last_seq": self.last_seq,
            "last_target": self.last_target,
            "last_source": self.last_source,
            "last_error": self.last_error,
            "last_line_age_s": None if self.last_line_s is None else max(0.0, time.monotonic() - self.last_line_s),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = UavTargetReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
