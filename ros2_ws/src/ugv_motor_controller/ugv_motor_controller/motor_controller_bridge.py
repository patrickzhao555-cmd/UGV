#!/usr/bin/env python3
import json
import time
from typing import Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String

from ugv_sensor_sync.msg import EncoderTicksStamped
from ugv_motor_controller.velocity_control import (
    VelocityPidConfig,
    WheelVelocityPid,
    EncoderSpeedSample,
    active_command_refresh_due,
    apply_velocity_raw_fallback_floor,
    command_age_s,
    command_is_timed_out,
    encoder_speed_is_fresh,
    extract_raw_drive,
    is_stop_command,
    reset_velocity_pid_pair,
    select_drive_command,
    stale_encoder_control_mode,
    update_encoder_wheel_speed_estimate,
    velocity_to_wheel_speeds,
)


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
        self.declare_parameter('velocity_control_enabled', False)
        self.declare_parameter('prefer_velocity_fields', True)
        self.declare_parameter('track_width_m', 0.6096)
        self.declare_parameter('wheel_radius_m', 0.06)
        self.declare_parameter('ticks_per_rev', 1000)
        self.declare_parameter('velocity_kp', 0.80)
        self.declare_parameter('velocity_ki', 0.0)
        self.declare_parameter('velocity_kd', 0.02)
        self.declare_parameter('velocity_integral_limit', 0.30)
        self.declare_parameter('velocity_feedforward_raw_per_mps', 1.35)
        self.declare_parameter('velocity_min_target_mps', 0.02)
        self.declare_parameter('velocity_max_target_mps', 0.60)
        self.declare_parameter('velocity_control_period_s', 0.05)
        self.declare_parameter('velocity_stale_encoder_timeout_s', 0.25)
        self.declare_parameter('velocity_fallback_to_raw_without_encoder', False)
        self.declare_parameter('velocity_encoder_speed_filter_alpha', 0.65)
        self.declare_parameter('velocity_encoder_speed_max_mps', 2.0)
        self.declare_parameter('velocity_encoder_speed_min_dt_s', 0.015)
        self.declare_parameter('velocity_raw_fallback_floor_enabled', False)
        self.declare_parameter('velocity_raw_fallback_min_wheel_raw', 0.0)
        self.declare_parameter('velocity_raw_fallback_min_target_raw', 0.001)

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
        self.command_timeout_s = max(0.0, float(self.get_parameter('command_timeout_s').value))
        self.command_refresh_period_s = max(0.0, float(self.get_parameter('command_refresh_period_s').value))
        self.serial_retry_period_s = float(self.get_parameter('serial_retry_period_s').value)
        self.poll_period_s = float(self.get_parameter('poll_period_s').value)
        self.status_period_s = float(self.get_parameter('status_period_s').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.invert_left_command = bool(self.get_parameter('invert_left_command').value)
        self.invert_right_command = bool(self.get_parameter('invert_right_command').value)
        self.invert_left_encoder = bool(self.get_parameter('invert_left_encoder').value)
        self.invert_right_encoder = bool(self.get_parameter('invert_right_encoder').value)
        self.velocity_control_enabled = bool(self.get_parameter('velocity_control_enabled').value)
        self.prefer_velocity_fields = bool(self.get_parameter('prefer_velocity_fields').value)
        self.track_width_m = max(0.05, float(self.get_parameter('track_width_m').value))
        self.wheel_radius_m = max(0.005, float(self.get_parameter('wheel_radius_m').value))
        self.ticks_per_rev = max(1, int(self.get_parameter('ticks_per_rev').value))
        self.velocity_control_period_s = max(0.01, float(self.get_parameter('velocity_control_period_s').value))
        self.velocity_stale_encoder_timeout_s = max(
            0.02,
            float(self.get_parameter('velocity_stale_encoder_timeout_s').value),
        )
        self.velocity_fallback_to_raw_without_encoder = bool(
            self.get_parameter('velocity_fallback_to_raw_without_encoder').value
        )
        self.velocity_encoder_speed_filter_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter('velocity_encoder_speed_filter_alpha').value)),
        )
        self.velocity_encoder_speed_max_mps = max(
            0.05,
            float(self.get_parameter('velocity_encoder_speed_max_mps').value),
        )
        self.velocity_encoder_speed_min_dt_s = max(
            0.0,
            float(self.get_parameter('velocity_encoder_speed_min_dt_s').value),
        )
        self.velocity_raw_fallback_floor_enabled = bool(
            self.get_parameter('velocity_raw_fallback_floor_enabled').value
        )
        self.velocity_raw_fallback_min_wheel_raw = max(
            0.0,
            float(self.get_parameter('velocity_raw_fallback_min_wheel_raw').value),
        )
        self.velocity_raw_fallback_min_target_raw = max(
            0.0,
            float(self.get_parameter('velocity_raw_fallback_min_target_raw').value),
        )
        pid_cfg = VelocityPidConfig(
            kp=float(self.get_parameter('velocity_kp').value),
            ki=float(self.get_parameter('velocity_ki').value),
            kd=float(self.get_parameter('velocity_kd').value),
            integral_limit=max(0.0, float(self.get_parameter('velocity_integral_limit').value)),
            feedforward_raw_per_mps=float(self.get_parameter('velocity_feedforward_raw_per_mps').value),
            min_target_mps=max(0.0, float(self.get_parameter('velocity_min_target_mps').value)),
            max_target_mps=max(0.01, float(self.get_parameter('velocity_max_target_mps').value)),
            max_raw=1.0,
        )
        self.left_velocity_pid = WheelVelocityPid(pid_cfg)
        self.right_velocity_pid = WheelVelocityPid(pid_cfg)

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
        self.last_command_type: Optional[str] = None
        self.control_mode = 'raw'
        self.selected_raw_left = 0.0
        self.selected_raw_right = 0.0
        self.velocity_raw_fallback_floor_applied_left = False
        self.velocity_raw_fallback_floor_applied_right = False
        self.active_velocity_command: Optional[Tuple[float, float]] = None
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.measured_left_mps = 0.0
        self.measured_right_mps = 0.0
        self.encoder_speed_baseline_sample: Optional[EncoderSpeedSample] = None
        self.encoder_speed_delta_available = False
        self.last_encoder_speed_time = 0.0
        self.last_encoder_speed_dt_s: Optional[float] = None
        self.last_encoder_speed_dt_source: Optional[str] = None
        self.last_encoder_speed_anomaly: Optional[str] = None
        self.encoder_speed_skipped_samples = 0
        self.last_encoder_speed_accumulated_dt_s: Optional[float] = None
        self.last_velocity_pid_time = 0.0
        self.last_velocity_pid_left = None
        self.last_velocity_pid_right = None
        self.last_velocity_error_left_mps = 0.0
        self.last_velocity_error_right_mps = 0.0
        self.last_velocity_safe_reason: Optional[str] = None
        self.last_pwm_send_time = 0.0
        self.timeout_stop_count = 0
        self.command_refresh_count = 0

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
            f'encoder_stamped_topic={self.encoder_stamped_topic}, dry_run={self.dry_run}, '
            f'invert_left_command={self.invert_left_command}, '
            f'invert_right_command={self.invert_right_command}, '
            f'invert_left_encoder={self.invert_left_encoder}, '
            f'invert_right_encoder={self.invert_right_encoder}, '
            f'velocity_control_enabled={self.velocity_control_enabled}, '
            f'prefer_velocity_fields={self.prefer_velocity_fields})'
        )

    def command_callback(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._log_parse_issue(f'Invalid /ugv_nav_cmd JSON: {exc}')
            return

        self.last_command_mode = str(obj.get('mode', 'RAW'))
        self.last_command_type = str(obj.get('command_type', 'raw')).lower()

        if is_stop_command(obj):
            self.last_command_received = time.monotonic()
            self.last_stop_sent = False
            self._stop_immediately(reason='nav stop command')
            self._publish_status(
                connected=self.serial_device is not None,
                extra={
                    'event': 'nav stop received',
                    'last_command_mode': self.last_command_mode,
                    'last_command_type': self.last_command_type,
                },
            )
            return

        try:
            command_path, velocity_cmd, raw_cmd = select_drive_command(
                obj,
                velocity_control_enabled=self.velocity_control_enabled,
                prefer_velocity_fields=self.prefer_velocity_fields,
            )
        except (TypeError, ValueError) as exc:
            self._log_parse_issue(f'Invalid /ugv_nav_cmd drive values: {exc}')
            return

        if command_path == 'velocity' and velocity_cmd is not None:
            self.last_command_received = time.monotonic()
            self.last_stop_sent = False
            self.active_velocity_command = velocity_cmd
            self.control_mode = 'velocity_pid'
            self.selected_raw_left = 0.0
            self.selected_raw_right = 0.0
            self.velocity_raw_fallback_floor_applied_left = False
            self.velocity_raw_fallback_floor_applied_right = False
            self.last_velocity_safe_reason = None
            self._update_velocity_control(reason='nav velocity command')
        else:
            self.active_velocity_command = None
            self.last_velocity_safe_reason = None
            self.target_left_mps = 0.0
            self.target_right_mps = 0.0
            self.last_velocity_error_left_mps = 0.0
            self.last_velocity_error_right_mps = 0.0
            self.last_velocity_pid_left = None
            self.last_velocity_pid_right = None
            self.last_velocity_pid_time = 0.0
            reset_velocity_pid_pair(self.left_velocity_pid, self.right_velocity_pid)
            if raw_cmd is None:
                self._log_parse_issue('Invalid /ugv_nav_cmd raw drive values: missing raw command')
                return
            left_raw, right_raw = raw_cmd
            velocity_raw_fallback = self.last_command_type == 'velocity'
            floor_result = apply_velocity_raw_fallback_floor(
                left_raw,
                right_raw,
                enabled=self.velocity_raw_fallback_floor_enabled,
                command_type=self.last_command_type or 'raw',
                mode=self.last_command_mode or 'RAW',
                min_wheel_raw=self.velocity_raw_fallback_min_wheel_raw,
                min_target_raw=self.velocity_raw_fallback_min_target_raw,
            )
            left_raw = floor_result.left_raw
            right_raw = floor_result.right_raw
            self.selected_raw_left = left_raw
            self.selected_raw_right = right_raw
            self.velocity_raw_fallback_floor_applied_left = floor_result.applied_left
            self.velocity_raw_fallback_floor_applied_right = floor_result.applied_right
            self.control_mode = 'raw_velocity_fallback' if velocity_raw_fallback else 'raw'
            self.last_command_received = time.monotonic()
            self.last_stop_sent = False
            left_pwm = self._raw_to_pwm(left_raw, invert=self.invert_left_command)
            right_pwm = self._raw_to_pwm(right_raw, invert=self.invert_right_command)
            self.target_pwm_command = (left_pwm, right_pwm)
            if self.serial_device is not None:
                self._send_pwm_command(left_pwm, right_pwm, reason='nav raw command')

        if self.serial_device is None:
            self._publish_status(
                connected=False,
                extra={
                    'reason': 'serial disconnected',
                    'last_command_mode': self.last_command_mode,
                    'last_command_type': self.last_command_type,
                },
            )
            return

        self._publish_status(
            connected=True,
            extra={
                'event': 'nav command received',
                'last_command_mode': self.last_command_mode,
                'last_command_type': self.last_command_type,
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
        if self.last_stop_sent:
            return
        if self.last_command_received == 0.0:
            self._stop_immediately(reason='initial hold')
            self.last_stop_sent = True
            return

        if command_is_timed_out(self.last_command_received, time.monotonic(), self.command_timeout_s):
            self.timeout_stop_count += 1
            self._stop_immediately(reason='command timeout stop')
            self.last_stop_sent = True

    def _refresh_active_command(self) -> None:
        if self.last_stop_sent:
            return
        if self.last_command_received <= 0.0:
            return
        now = time.monotonic()
        if command_is_timed_out(self.last_command_received, now, self.command_timeout_s):
            return
        if self.active_velocity_command is not None:
            if now - self.last_velocity_pid_time >= self.velocity_control_period_s:
                self._update_velocity_control(reason='velocity refresh')
            return
        if self.serial_device is None:
            return
        if not active_command_refresh_due(
            self.last_command_received,
            self.last_pwm_send_time,
            now,
            timeout_s=self.command_timeout_s,
            refresh_period_s=self.command_refresh_period_s,
        ):
            return
        self.command_refresh_count += 1
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

        now = time.monotonic()
        current_speed_sample = EncoderSpeedSample(
            left_ticks=left,
            right_ticks=right,
            host_time_s=now,
            controller_millis=teensy_ms,
        )
        speed_update = update_encoder_wheel_speed_estimate(
            self.encoder_speed_baseline_sample,
            current_speed_sample,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
            previous_left_mps=self.measured_left_mps,
            previous_right_mps=self.measured_right_mps,
            filter_alpha=self.velocity_encoder_speed_filter_alpha,
            max_abs_speed_mps=self.velocity_encoder_speed_max_mps,
            min_host_dt_s=self.velocity_encoder_speed_min_dt_s,
        )
        self.encoder_speed_baseline_sample = speed_update.baseline_sample
        self.last_encoder_speed_accumulated_dt_s = speed_update.accumulated_dt_s
        if speed_update.skipped:
            self.encoder_speed_skipped_samples += 1
            self.last_encoder_speed_dt_source = speed_update.dt_source
            self.last_encoder_speed_anomaly = speed_update.anomaly
        elif speed_update.estimate is not None:
            speed_estimate = speed_update.estimate
            self.measured_left_mps = speed_estimate.left_mps
            self.measured_right_mps = speed_estimate.right_mps
            self.last_encoder_speed_dt_s = speed_estimate.dt_s
            self.last_encoder_speed_dt_source = speed_estimate.dt_source
            self.last_encoder_speed_anomaly = speed_estimate.anomaly
            self.last_encoder_speed_time = now
            self.encoder_speed_delta_available = True
        else:
            self.measured_left_mps = 0.0
            self.measured_right_mps = 0.0
            self.last_encoder_speed_dt_s = None
            self.last_encoder_speed_dt_source = None
            self.last_encoder_speed_anomaly = None
            self.last_encoder_speed_time = 0.0
            self.encoder_speed_delta_available = False

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

    def _encoder_speed_is_fresh(self) -> bool:
        if not self.encoder_speed_delta_available:
            return False
        return encoder_speed_is_fresh(
            self.last_encoder_speed_time,
            now=time.monotonic(),
            timeout_s=self.velocity_stale_encoder_timeout_s,
        )
    def _stop_immediately(self, reason: str) -> None:
        self.active_velocity_command = None
        self.control_mode = 'stopped'
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.selected_raw_left = 0.0
        self.selected_raw_right = 0.0
        self.velocity_raw_fallback_floor_applied_left = False
        self.velocity_raw_fallback_floor_applied_right = False
        self.last_velocity_error_left_mps = 0.0
        self.last_velocity_error_right_mps = 0.0
        self.last_velocity_pid_left = None
        self.last_velocity_pid_right = None
        self.last_velocity_pid_time = 0.0
        self.last_velocity_safe_reason = reason
        reset_velocity_pid_pair(self.left_velocity_pid, self.right_velocity_pid)
        self.target_pwm_command = (self.pwm_neutral_us, self.pwm_neutral_us)
        self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason=reason)

    def _update_velocity_control(self, reason: str) -> None:
        if self.active_velocity_command is None:
            return

        v_mps, omega_radps = self.active_velocity_command
        self.target_left_mps, self.target_right_mps = velocity_to_wheel_speeds(
            v_mps,
            omega_radps,
            self.track_width_m,
        )

        if not self._encoder_speed_is_fresh():
            self.last_velocity_error_left_mps = self.target_left_mps - self.measured_left_mps
            self.last_velocity_error_right_mps = self.target_right_mps - self.measured_right_mps
            self.last_velocity_pid_left = None
            self.last_velocity_pid_right = None
            if self.velocity_fallback_to_raw_without_encoder:
                left_raw, right_raw = velocity_to_wheel_speeds(
                    v_mps,
                    omega_radps,
                    self.track_width_m,
                )
                left_raw *= self.left_velocity_pid.config.feedforward_raw_per_mps
                right_raw *= self.right_velocity_pid.config.feedforward_raw_per_mps
                left_pwm = self._raw_to_pwm(left_raw, invert=self.invert_left_command)
                right_pwm = self._raw_to_pwm(right_raw, invert=self.invert_right_command)
                self.control_mode = stale_encoder_control_mode(
                    fallback_to_raw_without_encoder=self.velocity_fallback_to_raw_without_encoder
                )
                self.last_velocity_safe_reason = 'encoder stale; raw fallback enabled'
                self.target_pwm_command = (left_pwm, right_pwm)
                self._send_pwm_command(left_pwm, right_pwm, reason=reason)
            else:
                self.control_mode = stale_encoder_control_mode(
                    fallback_to_raw_without_encoder=self.velocity_fallback_to_raw_without_encoder
                )
                self.last_velocity_safe_reason = 'encoder missing or stale'
                reset_velocity_pid_pair(self.left_velocity_pid, self.right_velocity_pid)
                self.target_pwm_command = (self.pwm_neutral_us, self.pwm_neutral_us)
                self._send_pwm_command(self.pwm_neutral_us, self.pwm_neutral_us, reason='velocity encoder stale stop')
            self.last_velocity_pid_time = time.monotonic()
            return

        now = time.monotonic()
        dt_s = self.velocity_control_period_s if self.last_velocity_pid_time <= 0.0 else max(
            1e-6,
            now - self.last_velocity_pid_time,
        )
        self.last_velocity_pid_time = now
        self.control_mode = 'velocity_pid'
        self.last_velocity_safe_reason = None

        left_pid = self.left_velocity_pid.update(self.target_left_mps, self.measured_left_mps, dt_s)
        right_pid = self.right_velocity_pid.update(self.target_right_mps, self.measured_right_mps, dt_s)
        self.last_velocity_pid_left = left_pid
        self.last_velocity_pid_right = right_pid
        self.last_velocity_error_left_mps = left_pid.error_mps
        self.last_velocity_error_right_mps = right_pid.error_mps

        left_pwm = self._raw_to_pwm(left_pid.output_raw, invert=self.invert_left_command)
        right_pwm = self._raw_to_pwm(right_pid.output_raw, invert=self.invert_right_command)
        self.target_pwm_command = (left_pwm, right_pwm)
        self._send_pwm_command(left_pwm, right_pwm, reason=reason)

    def _extract_raw_drive(self, obj: dict) -> Tuple[float, float]:
        return extract_raw_drive(obj)

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
            'control_mode': self.control_mode,
            'velocity_control_enabled': self.velocity_control_enabled,
            'prefer_velocity_fields': self.prefer_velocity_fields,
            'velocity_raw_fallback_floor_enabled': self.velocity_raw_fallback_floor_enabled,
            'velocity_raw_fallback_min_wheel_raw': round(self.velocity_raw_fallback_min_wheel_raw, 4),
            'velocity_raw_fallback_min_target_raw': round(self.velocity_raw_fallback_min_target_raw, 4),
            'velocity_raw_fallback_floor_applied_left': bool(self.velocity_raw_fallback_floor_applied_left),
            'velocity_raw_fallback_floor_applied_right': bool(self.velocity_raw_fallback_floor_applied_right),
            'selected_raw_left': round(self.selected_raw_left, 4),
            'selected_raw_right': round(self.selected_raw_right, 4),
            'command_timeout_s': round(self.command_timeout_s, 3),
            'command_refresh_period_s': round(self.command_refresh_period_s, 3),
            'timeout_stop_count': int(self.timeout_stop_count),
            'command_refresh_count': int(self.command_refresh_count),
            'last_command_time_s': None if self.last_command_received <= 0.0 else round(self.last_command_received, 3),
            'last_motor_send_time_s': None if self.last_pwm_send_time <= 0.0 else round(self.last_pwm_send_time, 3),
            'last_motor_send_age_s': self._last_motor_send_age_s(),
            'target_left_mps': round(self.target_left_mps, 4),
            'target_right_mps': round(self.target_right_mps, 4),
            'measured_left_mps': round(self.measured_left_mps, 4),
            'measured_right_mps': round(self.measured_right_mps, 4),
            'velocity_error_left_mps': round(self.last_velocity_error_left_mps, 4),
            'velocity_error_right_mps': round(self.last_velocity_error_right_mps, 4),
            'pid_left': self.last_velocity_pid_left.as_dict() if self.last_velocity_pid_left is not None else None,
            'pid_right': self.last_velocity_pid_right.as_dict() if self.last_velocity_pid_right is not None else None,
            'encoder_speed_age_s': self._encoder_speed_age_s(),
            'encoder_speed_dt_s': None if self.last_encoder_speed_dt_s is None else round(self.last_encoder_speed_dt_s, 4),
            'encoder_speed_dt_source': self.last_encoder_speed_dt_source,
            'encoder_speed_anomaly': self.last_encoder_speed_anomaly,
            'encoder_speed_min_dt_s': round(self.velocity_encoder_speed_min_dt_s, 4),
            'encoder_speed_skipped_samples': int(self.encoder_speed_skipped_samples),
            'encoder_speed_accumulated_dt_s': (
                None
                if self.last_encoder_speed_accumulated_dt_s is None
                else round(self.last_encoder_speed_accumulated_dt_s, 4)
            ),
            'velocity_safe_reason': self.last_velocity_safe_reason,
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
        if self.last_command_type is not None:
            status['last_command_type'] = self.last_command_type
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
        age = command_age_s(self.last_command_received, time.monotonic())
        if age is None:
            return None
        return round(age, 3)

    def _last_motor_send_age_s(self) -> Optional[float]:
        if self.last_pwm_send_time <= 0.0:
            return None
        return round(max(0.0, time.monotonic() - self.last_pwm_send_time), 3)

    def _encoder_speed_age_s(self) -> Optional[float]:
        if not self.encoder_speed_delta_available or self.last_encoder_speed_time <= 0.0:
            return None
        return round(max(0.0, time.monotonic() - self.last_encoder_speed_time), 3)

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
