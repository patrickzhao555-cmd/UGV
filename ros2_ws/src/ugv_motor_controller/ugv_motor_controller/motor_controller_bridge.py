#!/usr/bin/env python3
import json
import time
from typing import Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String

from ugv_sensor_sync.msg import EncoderTicksStamped


class MotorControllerBridge(Node):
    def __init__(self):
        super().__init__('motor_controller_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('command_topic', '/ugv_nav_cmd')
        self.declare_parameter('encoder_topic', '/encoder_ticks')
        self.declare_parameter('encoder_stamped_topic', '/encoder_ticks_stamped')
        self.declare_parameter('raw_encoder_topic', '/motor_controller/raw_encoders')
        self.declare_parameter('connected_topic', '/motor_controller/connected')
        self.declare_parameter('status_topic', '/motor_controller/status')
        self.declare_parameter('encoder_frame_id', 'base_link')
        self.declare_parameter('pwm_neutral_us', 1500)
        self.declare_parameter('pwm_min_us', 1100)
        self.declare_parameter('pwm_max_us', 1900)
        self.declare_parameter('raw_command_scale_us', 900.0)
        self.declare_parameter('pwm_slew_rate_us_per_s', 2400.0)
        self.declare_parameter('command_deadband', 0.03)
        self.declare_parameter('command_timeout_s', 0.75)
        self.declare_parameter('command_refresh_period_s', 0.1)
        self.declare_parameter('serial_retry_period_s', 1.0)
        self.declare_parameter('poll_period_s', 0.02)
        self.declare_parameter('status_period_s', 0.5)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('invert_left_command', False)
        self.declare_parameter('invert_right_command', False)
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', False)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.command_topic = self.get_parameter('command_topic').value
        self.encoder_topic = self.get_parameter('encoder_topic').value
        self.encoder_stamped_topic = self.get_parameter('encoder_stamped_topic').value
        self.raw_encoder_topic = self.get_parameter('raw_encoder_topic').value
        self.connected_topic = self.get_parameter('connected_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.encoder_frame_id = self.get_parameter('encoder_frame_id').value
        self.pwm_neutral_us = int(self.get_parameter('pwm_neutral_us').value)
        self.pwm_min_us = int(self.get_parameter('pwm_min_us').value)
        self.pwm_max_us = int(self.get_parameter('pwm_max_us').value)
        self.raw_command_scale_us = float(self.get_parameter('raw_command_scale_us').value)
        self.pwm_slew_rate_us_per_s = float(self.get_parameter('pwm_slew_rate_us_per_s').value)
        self.command_deadband = float(self.get_parameter('command_deadband').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)
        self.command_refresh_period_s = float(self.get_parameter('command_refresh_period_s').value)
        self.serial_retry_period_s = float(self.get_parameter('serial_retry_period_s').value)
        self.poll_period_s = float(self.get_parameter('poll_period_s').value)
        self.status_period_s = float(self.get_parameter('status_period_s').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.invert_left_command = bool(self.get_parameter('invert_left_command').value)
        self.invert_right_command = bool(self.get_parameter('invert_right_command').value)
        self.invert_left_encoder = bool(self.get_parameter('invert_left_encoder').value)
        self.invert_right_encoder = bool(self.get_parameter('invert_right_encoder').value)

        self.serial_device: Optional[serial.Serial] = None
        self.last_serial_attempt = 0.0
        self.last_command_received = 0.0
        self.last_stop_sent = False
        self.target_pwm_command = (self.pwm_neutral_us, self.pwm_neutral_us)
        self.last_pwm_command = (self.pwm_neutral_us, self.pwm_neutral_us)
        self.last_encoder_pair: Optional[Tuple[int, int]] = None
        self.last_raw_encoder_quad: Optional[Tuple[int, int, int, int]] = None
        self.last_teensy_ms: Optional[int] = None
        self.last_status_log = 0.0
        self.last_status_publish = 0.0
        self.last_parse_error_log = 0.0
        self.last_connected_state: Optional[bool] = None
        self.serial_rx_buffer = ''
        self.last_command_mode: Optional[str] = None
        self.last_pwm_send_time = 0.0

        self.encoder_pub = self.create_publisher(Int32MultiArray, self.encoder_topic, 10)
        self.encoder_stamped_pub = self.create_publisher(EncoderTicksStamped, self.encoder_stamped_topic, 10)
        self.raw_encoder_pub = self.create_publisher(Int32MultiArray, self.raw_encoder_topic, 10)
        self.connected_pub = self.create_publisher(Bool, self.connected_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        # Match the Teensy encoder cadence (~50 Hz) so the serial poll timer
        # does not monopolize the single-threaded executor and starve commands.
        self.create_timer(max(self.poll_period_s, 0.02), self.poll)

        self.get_logger().info(
            f'Motor controller bridge starting '
            f'(port={self.port}, baud={self.baud}, command_topic={self.command_topic}, '
            f'encoder_stamped_topic={self.encoder_stamped_topic}, dry_run={self.dry_run})'
        )

    def command_callback(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._log_parse_issue(f'Invalid /ugv_nav_cmd JSON: {exc}')
            return

        left_raw, right_raw = self._extract_raw_drive(obj)
        left_pwm = self._raw_to_pwm(left_raw, invert=self.invert_left_command)
        right_pwm = self._raw_to_pwm(right_raw, invert=self.invert_right_command)

        self.last_command_received = time.monotonic()
        self.last_stop_sent = False
        self.target_pwm_command = (left_pwm, right_pwm)
        self.last_command_mode = str(obj.get('mode', 'RAW'))

        if self.serial_device is None:
            self._publish_status(
                connected=False,
                extra={
                    'reason': 'serial disconnected',
                    'last_command_mode': self.last_command_mode,
                },
            )
            return

        send_reason = 'nav stop command' if self.last_command_mode == 'STOP' else 'nav command'
        self._send_pwm_command(left_pwm, right_pwm, reason=send_reason)
        self._publish_status(
            connected=True,
            extra={
                'event': 'nav command received',
                'last_command_mode': self.last_command_mode,
            },
        )

    def poll(self) -> None:
        self._ensure_serial()
        self._refresh_active_command()
        self._handle_command_timeout()
        self._drain_serial()
        self._publish_connected()
        self._maybe_publish_periodic_status()

    def _ensure_serial(self) -> None:
        if self.serial_device is not None:
            return

        now = time.monotonic()
        if now - self.last_serial_attempt < self.serial_retry_period_s:
            return
        self.last_serial_attempt = now

        try:
            self.serial_device = serial.Serial(
                self.port,
                self.baud,
                timeout=0.0,
                write_timeout=0.1,
            )
            self.get_logger().info(f'Motor controller serial connected: {self.port} @ {self.baud}')
            self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason='startup stop')
        except serial.SerialException as exc:
            self.serial_device = None
            self._throttled_info(f'Waiting for motor controller serial: {exc}')

    def _handle_command_timeout(self) -> None:
        if self.serial_device is None:
            return
        if self.last_stop_sent:
            return
        if self.last_command_received == 0.0:
            self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason='initial hold')
            self.last_stop_sent = True
            return

        if time.monotonic() - self.last_command_received > self.command_timeout_s:
            self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason='command timeout stop')
            self.last_stop_sent = True

    def _refresh_active_command(self) -> None:
        if self.serial_device is None:
            return
        if self.last_stop_sent:
            return
        if self.last_command_received <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_command_received > self.command_timeout_s:
            return
        if now - self.last_pwm_send_time < self.command_refresh_period_s:
            return
        self._send_pwm_command(*self.target_pwm_command, reason='command refresh')

    def _drain_serial(self) -> None:
        if self.serial_device is None:
            return

        try:
            while True:
                waiting = self.serial_device.in_waiting
                if waiting <= 0:
                    break
                chunk = self.serial_device.read(waiting)
                if not chunk:
                    break
                self.serial_rx_buffer += chunk.decode('utf-8', errors='ignore')

                while '\n' in self.serial_rx_buffer:
                    line, self.serial_rx_buffer = self.serial_rx_buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._handle_serial_line(line)

                if len(self.serial_rx_buffer) > 4096:
                    self.serial_rx_buffer = self.serial_rx_buffer[-1024:]
        except serial.SerialException as exc:
            self.get_logger().warn(f'Motor controller serial lost: {exc}')
            self._close_serial()

    def _handle_serial_line(self, line: str) -> None:
        if line.startswith('ENC ') or line.startswith('DBG ENC '):
            self._handle_debug_encoder_line(line)
            return
        if not line.startswith('E'):
            self._throttled_info(f'Motor controller debug: {line}')
            return

        payload = line[1:]
        parts = payload.split(',')
        if len(parts) not in {4, 5}:
            self._log_parse_issue(f'Unexpected encoder line: {line!r}')
            return

        try:
            fl = int(parts[0])
            fr = int(parts[1])
            rl = int(parts[2])
            rr = int(parts[3])
            teensy_ms = int(parts[4]) if len(parts) == 5 else None
        except ValueError:
            self._log_parse_issue(f'Failed to parse encoder line: {line!r}')
            return

        self._publish_encoder_values(
            fl=fl,
            fr=fr,
            rl=rl,
            rr=rr,
            teensy_ms=teensy_ms,
        )

    def _handle_debug_encoder_line(self, line: str) -> None:
        body = line[4:] if line.startswith('DBG ') else line
        try:
            ticks_part = body[4:].split(' pwm=', 1)[0]
            fl_s, fr_s, rl_s, rr_s = ticks_part.split(',')
            self._publish_encoder_values(
                fl=int(fl_s),
                fr=int(fr_s),
                rl=int(rl_s),
                rr=int(rr_s),
                teensy_ms=None,
            )
        except (ValueError, IndexError):
            self._log_parse_issue(f'Failed to parse debug encoder line: {line!r}')

    def _publish_encoder_values(
        self,
        *,
        fl: int,
        fr: int,
        rl: int,
        rr: int,
        teensy_ms: Optional[int],
    ) -> None:
        left = int(round((fl + rl) / 2.0))
        right = int(round((fr + rr) / 2.0))
        if self.invert_left_encoder:
            left = -left
        if self.invert_right_encoder:
            right = -right

        self.last_encoder_pair = (left, right)
        self.last_raw_encoder_quad = (fl, fr, rl, rr)
        self.last_teensy_ms = teensy_ms

        enc_msg = Int32MultiArray()
        enc_msg.data = [left, right]
        self.encoder_pub.publish(enc_msg)

        enc_stamped_msg = EncoderTicksStamped()
        enc_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        enc_stamped_msg.header.frame_id = self.encoder_frame_id
        enc_stamped_msg.left_ticks = left
        enc_stamped_msg.right_ticks = right
        enc_stamped_msg.front_left_ticks = fl
        enc_stamped_msg.front_right_ticks = fr
        enc_stamped_msg.rear_left_ticks = rl
        enc_stamped_msg.rear_right_ticks = rr
        enc_stamped_msg.controller_millis = 0 if teensy_ms is None else max(0, teensy_ms)
        self.encoder_stamped_pub.publish(enc_stamped_msg)

        raw_msg = Int32MultiArray()
        raw_msg.data = [fl, fr, rl, rr]
        self.raw_encoder_pub.publish(raw_msg)

        self._publish_status(
            connected=True,
            extra={
                'left_ticks': left,
                'right_ticks': right,
                'raw_ticks': [fl, fr, rl, rr],
                'teensy_ms': teensy_ms,
                'last_pwm': list(self.last_pwm_command),
                'target_pwm': list(self.target_pwm_command),
            },
        )

    def _extract_raw_drive(self, obj: dict) -> Tuple[float, float]:
        if 'raw_left' in obj and 'raw_right' in obj:
            return float(obj['raw_left']), float(obj['raw_right'])

        mode = str(obj.get('mode', 'STOP')).upper()
        if mode == 'FORWARD':
            return 0.35, 0.35
        if mode == 'BACKWARD':
            return -0.35, -0.35
        if mode == 'TURN_LEFT':
            return -0.30, 0.30
        if mode == 'TURN_RIGHT':
            return 0.30, -0.30
        return 0.0, 0.0

    def _raw_to_pwm(self, raw_value: float, invert: bool = False) -> int:
        if invert:
            raw_value = -raw_value
        if abs(raw_value) < self.command_deadband:
            raw_value = 0.0

        pwm = self.pwm_neutral_us + raw_value * self.raw_command_scale_us
        return int(max(self.pwm_min_us, min(self.pwm_max_us, round(pwm))))

    def _send_pwm_command(self, left_pwm: int, right_pwm: int, reason: str) -> None:
        if self.serial_device is None:
            return

        self.target_pwm_command = (left_pwm, right_pwm)
        left_pwm, right_pwm = self._apply_pwm_slew(left_pwm, right_pwm, reason)
        self.last_pwm_command = (left_pwm, right_pwm)
        if self.dry_run:
            self.last_pwm_send_time = time.monotonic()
            self._throttled_info(f'DRY RUN motor command ({reason}): {left_pwm}, {right_pwm}')
            return

        line = f'M{left_pwm},{right_pwm}\n'
        try:
            self.serial_device.write(line.encode('utf-8'))
            self.serial_device.flush()
            self.last_pwm_command = (left_pwm, right_pwm)
            self.last_pwm_send_time = time.monotonic()
            self._throttled_info(f'Sent motor command ({reason}): {left_pwm}, {right_pwm}')
        except serial.SerialException as exc:
            self.get_logger().warn(f'Failed to send motor command: {exc}')
            self._close_serial()

    def _publish_connected(self) -> None:
        connected = self.serial_device is not None
        self.connected_pub.publish(Bool(data=connected))
        if self.last_connected_state is None or connected != self.last_connected_state:
            self._publish_status(
                connected=connected,
                extra={'event': 'serial connected' if connected else 'serial disconnected'},
            )
            self.last_connected_state = connected

    def _publish_status(self, connected: bool, extra: Optional[dict] = None) -> None:
        status = {
            'connected': connected,
            'port': self.port,
            'baud': self.baud,
            'dry_run': self.dry_run,
            'last_pwm': list(self.last_pwm_command),
            'target_pwm': list(self.target_pwm_command),
            'pwm_slew_rate_us_per_s': self.pwm_slew_rate_us_per_s,
            'command_age_s': self._command_age_s(),
        }
        if self.last_encoder_pair is not None:
            status['encoder_ticks'] = list(self.last_encoder_pair)
        if self.last_teensy_ms is not None:
            status['teensy_ms'] = self.last_teensy_ms
        if self.last_command_mode is not None:
            status['last_command_mode'] = self.last_command_mode
        if extra:
            status.update(extra)
        self.status_pub.publish(String(data=json.dumps(status)))
        self.last_status_publish = time.monotonic()

    def _close_serial(self) -> None:
        if self.serial_device is not None:
            try:
                self.serial_device.close()
            except Exception:
                pass
        self.serial_device = None
        self.serial_rx_buffer = ''

    def _throttled_info(self, msg: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_status_log >= period_s:
            self.get_logger().info(msg)
            self.last_status_log = now

    def _log_parse_issue(self, msg: str) -> None:
        now = time.monotonic()
        if now - self.last_parse_error_log >= 1.0:
            self.get_logger().warn(msg)
            self.last_parse_error_log = now

    def _maybe_publish_periodic_status(self) -> None:
        now = time.monotonic()
        if now - self.last_status_publish >= self.status_period_s:
            self._publish_status(connected=self.serial_device is not None)

    def _command_age_s(self) -> Optional[float]:
        if self.last_command_received <= 0.0:
            return None
        return round(max(0.0, time.monotonic() - self.last_command_received), 3)

    def _apply_pwm_slew(self, left_pwm: int, right_pwm: int, reason: str) -> Tuple[int, int]:
        immediate_reasons = ('stop', 'timeout', 'shutdown', 'startup', 'initial hold')
        if self.pwm_slew_rate_us_per_s <= 0.0 or any(token in reason for token in immediate_reasons):
            return left_pwm, right_pwm

        now = time.monotonic()
        dt = self.command_refresh_period_s if self.last_pwm_send_time <= 0.0 else max(0.0, now - self.last_pwm_send_time)
        max_delta = max(1, int(round(self.pwm_slew_rate_us_per_s * dt)))

        def step(prev: int, target: int) -> int:
            delta = target - prev
            if abs(delta) <= max_delta:
                return target
            return prev + max_delta if delta > 0 else prev - max_delta

        return step(self.last_pwm_command[0], left_pwm), step(self.last_pwm_command[1], right_pwm)

    def destroy_node(self) -> None:
        try:
            self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason='shutdown stop')
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
