#!/usr/bin/env python3
"""Thin ROS-to-Teensy bridge for the active Teensy motor controller.

This node intentionally does not implement motor PID. The robot has two
goBILDA speed controllers and four Pololu encoder feedback channels, so Teensy
owns left/right side velocity PID and the Jetson only forwards chassis intent
as ``CMD V <v_mps> <omega_radps>``.
"""

from __future__ import annotations

import json
import os
import time
from collections import OrderedDict
from typing import Any, Dict, Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String

from ugv_motor_controller.teensy_side_pid import (
    TeensyParamSyncTracker,
    TeensySidePidStatus,
    build_teensy_param_command,
    build_teensy_stop_command,
    build_teensy_velocity_command,
    extract_velocity_command,
    is_stop_command,
    parse_teensy_param_ack_line,
    parse_teensy_side_pid_status_line,
    side_mismatch_flags,
    ticks_per_sec_to_mps,
    velocity_to_side_speeds,
)
from ugv_sensor_sync.msg import EncoderTicksStamped


def _sign_param(value: Any) -> int:
    return -1 if int(value) < 0 else 1


class MotorControllerBridge(Node):
    def __init__(self) -> None:
        super().__init__("motor_controller_bridge")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_topic", "/ugv_nav_cmd")
        self.declare_parameter("encoder_topic", "/encoder_ticks")
        self.declare_parameter("encoder_stamped_topic", "/encoder_ticks_stamped")
        self.declare_parameter("raw_encoder_topic", "/motor_controller/raw_encoders")
        self.declare_parameter("connected_topic", "/motor_controller/connected")
        self.declare_parameter("status_topic", "/motor_controller/status")
        self.declare_parameter("encoder_frame_id", "base_link")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("command_timeout_s", 0.75)
        self.declare_parameter("command_refresh_period_s", 0.10)
        self.declare_parameter("serial_retry_period_s", 1.0)
        self.declare_parameter("poll_period_s", 0.02)
        self.declare_parameter("status_period_s", 0.5)
        self.declare_parameter("track_width_m", 0.425)
        self.declare_parameter("wheel_radius_m", float(os.environ.get("MOTOR_WHEEL_RADIUS_M", "0.0825")))
        self.declare_parameter("ticks_per_rev", int(os.environ.get("MOTOR_TICKS_PER_REV", "3200")))
        self.declare_parameter("pwm_min_us", 1100)
        self.declare_parameter("pwm_neutral_us", 1500)
        self.declare_parameter("pwm_max_us", 1900)
        self.declare_parameter("pwm_slew_rate_us_per_s", 2400.0)
        self.declare_parameter("teensy_control_hz", 50.0)
        self.declare_parameter("teensy_pid_kp", 0.05)
        self.declare_parameter("teensy_pid_ki", 0.0)
        self.declare_parameter("teensy_pid_kd", 0.0)
        self.declare_parameter("teensy_pid_feedforward_us_per_tps", 0.04)
        self.declare_parameter("teensy_pid_static_ff_us", 170.0)
        self.declare_parameter("teensy_pid_output_limit_us", 350.0)
        self.declare_parameter("teensy_pid_min_target_tps", 2.0)
        self.declare_parameter("teensy_left_motor_sign", 1)
        self.declare_parameter("teensy_right_motor_sign", -1)
        self.declare_parameter("teensy_fl_encoder_sign", -1)
        self.declare_parameter("teensy_fr_encoder_sign", 1)
        self.declare_parameter("teensy_rl_encoder_sign", -1)
        self.declare_parameter("teensy_rr_encoder_sign", 1)
        self.declare_parameter("teensy_stall_fault_enabled", True)
        self.declare_parameter("teensy_stall_target_tps", 15.0)
        self.declare_parameter("teensy_stall_near_zero_tps", 2.0)
        self.declare_parameter("teensy_stall_moving_peer_tps", 12.0)
        self.declare_parameter("teensy_stall_pwm_delta_us", 120.0)
        self.declare_parameter("teensy_stall_timeout_ms", 300)
        self.declare_parameter("teensy_sign_mismatch_tps", 10.0)
        self.declare_parameter("teensy_sign_mismatch_target_tps", 100.0)
        self.declare_parameter("teensy_sign_mismatch_timeout_ms", 250)
        self.declare_parameter("teensy_side_mismatch_fault_enabled", True)
        self.declare_parameter("teensy_side_mismatch_warn_tps", 80.0)
        self.declare_parameter("teensy_side_mismatch_fault_tps", 180.0)
        self.declare_parameter("teensy_encoder_jump_fault_enabled", True)
        self.declare_parameter("teensy_encoder_jump_tps", 12000.0)
        self.declare_parameter("teensy_pid_param_ack_timeout_s", 1.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.encoder_stamped_topic = str(self.get_parameter("encoder_stamped_topic").value)
        self.raw_encoder_topic = str(self.get_parameter("raw_encoder_topic").value)
        self.connected_topic = str(self.get_parameter("connected_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.encoder_frame_id = str(self.get_parameter("encoder_frame_id").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_timeout_s = max(0.05, float(self.get_parameter("command_timeout_s").value))
        self.command_refresh_period_s = max(0.02, float(self.get_parameter("command_refresh_period_s").value))
        self.serial_retry_period_s = max(0.05, float(self.get_parameter("serial_retry_period_s").value))
        self.poll_period_s = max(0.02, float(self.get_parameter("poll_period_s").value))
        self.status_period_s = max(0.1, float(self.get_parameter("status_period_s").value))
        self.track_width_m = max(0.05, float(self.get_parameter("track_width_m").value))
        self.wheel_radius_m = max(0.005, float(self.get_parameter("wheel_radius_m").value))
        self.ticks_per_rev = max(1, int(self.get_parameter("ticks_per_rev").value))
        self.pwm_min_us = int(self.get_parameter("pwm_min_us").value)
        self.pwm_neutral_us = int(self.get_parameter("pwm_neutral_us").value)
        self.pwm_max_us = int(self.get_parameter("pwm_max_us").value)
        self.pwm_slew_rate_us_per_s = max(0.0, float(self.get_parameter("pwm_slew_rate_us_per_s").value))
        self.teensy_control_hz = max(20.0, min(200.0, float(self.get_parameter("teensy_control_hz").value)))
        self.teensy_pid_kp = float(self.get_parameter("teensy_pid_kp").value)
        self.teensy_pid_ki = float(self.get_parameter("teensy_pid_ki").value)
        self.teensy_pid_kd = float(self.get_parameter("teensy_pid_kd").value)
        self.teensy_pid_feedforward_us_per_tps = float(
            self.get_parameter("teensy_pid_feedforward_us_per_tps").value
        )
        self.teensy_pid_static_ff_us = max(0.0, float(self.get_parameter("teensy_pid_static_ff_us").value))
        self.teensy_pid_output_limit_us = max(1.0, float(self.get_parameter("teensy_pid_output_limit_us").value))
        self.teensy_pid_min_target_tps = max(0.0, float(self.get_parameter("teensy_pid_min_target_tps").value))
        self.teensy_left_motor_sign = _sign_param(self.get_parameter("teensy_left_motor_sign").value)
        self.teensy_right_motor_sign = _sign_param(self.get_parameter("teensy_right_motor_sign").value)
        self.teensy_fl_encoder_sign = _sign_param(self.get_parameter("teensy_fl_encoder_sign").value)
        self.teensy_fr_encoder_sign = _sign_param(self.get_parameter("teensy_fr_encoder_sign").value)
        self.teensy_rl_encoder_sign = _sign_param(self.get_parameter("teensy_rl_encoder_sign").value)
        self.teensy_rr_encoder_sign = _sign_param(self.get_parameter("teensy_rr_encoder_sign").value)
        self.teensy_stall_fault_enabled = bool(self.get_parameter("teensy_stall_fault_enabled").value)
        self.teensy_stall_target_tps = max(0.0, float(self.get_parameter("teensy_stall_target_tps").value))
        self.teensy_stall_near_zero_tps = max(0.0, float(self.get_parameter("teensy_stall_near_zero_tps").value))
        self.teensy_stall_moving_peer_tps = max(0.0, float(self.get_parameter("teensy_stall_moving_peer_tps").value))
        self.teensy_stall_pwm_delta_us = max(0.0, float(self.get_parameter("teensy_stall_pwm_delta_us").value))
        self.teensy_stall_timeout_ms = max(0, int(self.get_parameter("teensy_stall_timeout_ms").value))
        self.teensy_sign_mismatch_tps = max(0.0, float(self.get_parameter("teensy_sign_mismatch_tps").value))
        self.teensy_sign_mismatch_target_tps = max(
            0.0,
            float(self.get_parameter("teensy_sign_mismatch_target_tps").value),
        )
        self.teensy_sign_mismatch_timeout_ms = max(
            0,
            int(self.get_parameter("teensy_sign_mismatch_timeout_ms").value),
        )
        self.teensy_side_mismatch_fault_enabled = bool(
            self.get_parameter("teensy_side_mismatch_fault_enabled").value
        )
        self.teensy_side_mismatch_warn_tps = max(
            0.0,
            float(self.get_parameter("teensy_side_mismatch_warn_tps").value),
        )
        self.teensy_side_mismatch_fault_tps = max(
            0.0,
            float(self.get_parameter("teensy_side_mismatch_fault_tps").value),
        )
        self.teensy_encoder_jump_fault_enabled = bool(
            self.get_parameter("teensy_encoder_jump_fault_enabled").value
        )
        self.teensy_encoder_jump_tps = max(0.0, float(self.get_parameter("teensy_encoder_jump_tps").value))
        self.teensy_pid_param_ack_timeout_s = max(
            0.05,
            float(self.get_parameter("teensy_pid_param_ack_timeout_s").value),
        )

        self.serial_device: Optional[serial.Serial] = None
        self.dry_run_connected = False
        self.serial_rx_buffer = ""
        self.last_serial_attempt_s = 0.0
        self.last_connected_state: Optional[bool] = None
        self.last_command_received_s = 0.0
        self.last_command_sent_s = 0.0
        self.last_status_publish_s = 0.0
        self.last_log_s = 0.0
        self.timeout_stop_sent = False
        self.active_velocity_command: Optional[Tuple[float, float]] = None
        self.target_left_mps = 0.0
        self.target_right_mps = 0.0
        self.measured_left_mps = 0.0
        self.measured_right_mps = 0.0
        self.last_encoder_quad: Optional[Tuple[int, int, int, int]] = None
        self.last_encoder_pair: Optional[Tuple[int, int]] = None
        self.last_teensy_ms: Optional[int] = None
        self.last_teensy_status: Optional[TeensySidePidStatus] = None
        self.teensy_pid_param_sync = TeensyParamSyncTracker()
        self.teensy_pid_params_synced = False
        self.teensy_pid_param_sync_count = 0
        self.teensy_pid_param_sync_last_s: Optional[float] = None
        self.teensy_pid_param_sync_pending: list[str] = []
        self.teensy_pid_param_sync_failed = False
        self.teensy_pid_param_sync_reason = "not_started"

        self.encoder_pub = self.create_publisher(Int32MultiArray, self.encoder_topic, 10)
        self.encoder_stamped_pub = self.create_publisher(EncoderTicksStamped, self.encoder_stamped_topic, 10)
        self.raw_encoder_pub = self.create_publisher(Int32MultiArray, self.raw_encoder_topic, 10)
        self.connected_pub = self.create_publisher(Bool, self.connected_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)
        self.create_timer(self.poll_period_s, self.poll)

        self.get_logger().info(
            "Motor bridge active path: Jetson velocity command -> Teensy two-controller four-encoder side PID "
            f"(port={self.port}, dry_run={self.dry_run}, radius={self.wheel_radius_m}, "
            f"ticks_per_rev={self.ticks_per_rev}, motor_signs="
            f"{self.teensy_left_motor_sign}/{self.teensy_right_motor_sign}, "
            f"kp={self.teensy_pid_kp}, ff={self.teensy_pid_feedforward_us_per_tps}, "
            f"static_ff={self.teensy_pid_static_ff_us})"
        )

    def command_callback(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._log_warn(f"Invalid /ugv_nav_cmd JSON: {exc}")
            return

        if is_stop_command(obj):
            self.last_command_received_s = time.monotonic()
            self.active_velocity_command = None
            self.target_left_mps = 0.0
            self.target_right_mps = 0.0
            self.timeout_stop_sent = True
            self._send_stop("nav stop command")
            self._publish_status(event="nav stop received")
            return

        try:
            velocity = extract_velocity_command(obj)
        except (TypeError, ValueError) as exc:
            self._log_warn(f"Invalid velocity command: {exc}")
            return
        if velocity is None:
            self._log_warn("Ignoring non-velocity /ugv_nav_cmd; clean runtime accepts command_type=velocity only")
            return

        self.last_command_received_s = time.monotonic()
        self.timeout_stop_sent = False
        self.active_velocity_command = velocity
        self.target_left_mps, self.target_right_mps = velocity_to_side_speeds(
            velocity[0],
            velocity[1],
            self.track_width_m,
        )
        if self._transport_available() and self._motion_allowed():
            self._write_command(
                build_teensy_velocity_command(*velocity),
                reason="nav velocity command",
                log_payload=f"CMD V {velocity[0]:.4f} {velocity[1]:.4f}",
            )
        self._publish_status(event="nav velocity received")

    def poll(self) -> None:
        self._ensure_serial()
        self._drain_serial()
        self._refresh_active_command()
        self._handle_command_timeout()
        self._handle_param_sync_timeout()
        self._publish_connected()
        self._maybe_publish_periodic_status()

    def _ensure_serial(self) -> None:
        if self.serial_device is not None or self.dry_run_connected:
            return
        now = time.monotonic()
        if now - self.last_serial_attempt_s < self.serial_retry_period_s:
            return
        self.last_serial_attempt_s = now
        if self.dry_run:
            self.dry_run_connected = True
            self._sync_teensy_pid_params("dry-run startup parameter sync")
            return
        try:
            self.serial_device = serial.Serial(self.port, self.baud, timeout=0.0, write_timeout=0.1)
            self.get_logger().info(f"Motor serial connected: {self.port} @ {self.baud}")
            self._send_stop("startup stop")
            self._sync_teensy_pid_params("startup parameter sync")
        except serial.SerialException as exc:
            self.serial_device = None
            self._throttled_info(f"Waiting for motor serial: {exc}")

    def _refresh_active_command(self) -> None:
        if not self._transport_available() or self.active_velocity_command is None or not self._motion_allowed():
            return
        now = time.monotonic()
        if now - self.last_command_received_s > self.command_timeout_s:
            return
        if now - self.last_command_sent_s < self.command_refresh_period_s:
            return
        self._write_command(
            build_teensy_velocity_command(*self.active_velocity_command),
            reason="velocity refresh",
            log_payload=f"CMD V {self.active_velocity_command[0]:.4f} {self.active_velocity_command[1]:.4f}",
        )

    def _handle_command_timeout(self) -> None:
        if self.timeout_stop_sent:
            return
        if self.last_command_received_s <= 0.0:
            self.timeout_stop_sent = True
            self._send_stop("initial hold")
            return
        if time.monotonic() - self.last_command_received_s > self.command_timeout_s:
            self.timeout_stop_sent = True
            self.active_velocity_command = None
            self.target_left_mps = 0.0
            self.target_right_mps = 0.0
            self._send_stop("command timeout")

    def _drain_serial(self) -> None:
        if self.serial_device is None:
            return
        try:
            while self.serial_device.in_waiting > 0:
                chunk = self.serial_device.read(self.serial_device.in_waiting)
                self.serial_rx_buffer += chunk.decode("utf-8", errors="ignore")
                while "\n" in self.serial_rx_buffer:
                    line, self.serial_rx_buffer = self.serial_rx_buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self._handle_serial_line(line)
                if len(self.serial_rx_buffer) > 4096:
                    self.serial_rx_buffer = self.serial_rx_buffer[-1024:]
        except serial.SerialException as exc:
            self.get_logger().warn(f"Motor serial lost: {exc}")
            self._close_serial()

    def _handle_serial_line(self, line: str) -> None:
        if line.startswith("PARAM,"):
            self._handle_param_ack(line)
            return
        if line.startswith("S,"):
            self._handle_status(line)
            return
        if line.startswith("E"):
            return
        if line.startswith("PARAMS,"):
            self._publish_status(event="teensy params", teensy_params=line)
            return
        self._throttled_info(f"Teensy debug: {line}")

    def _handle_param_ack(self, line: str) -> None:
        try:
            ack = parse_teensy_param_ack_line(line)
        except ValueError as exc:
            self._log_warn(str(exc))
            return
        was_synced = self.teensy_pid_params_synced
        completed = self.teensy_pid_param_sync.handle_ack(ack, time.monotonic())
        self._copy_param_sync_state()
        if self.teensy_pid_param_sync_failed:
            self.get_logger().warn(f"Teensy parameter sync failed: {self.teensy_pid_param_sync_reason}")
            self._publish_status(event="teensy param sync failed")
            return
        if completed and not was_synced:
            self.teensy_pid_param_sync_count += self.teensy_pid_param_sync.acked_count
            self.teensy_pid_param_sync_last_s = self.teensy_pid_param_sync.completed_s
            self.get_logger().info("Teensy parameter sync ACKed")
            self._publish_status(event="teensy param sync complete")

    def _handle_status(self, line: str) -> None:
        try:
            status = parse_teensy_side_pid_status_line(line)
        except ValueError as exc:
            self._log_warn(str(exc))
            return
        self.last_teensy_status = status
        self.measured_left_mps = ticks_per_sec_to_mps(
            status.left_measured_tps,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
        )
        self.measured_right_mps = ticks_per_sec_to_mps(
            status.right_measured_tps,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
        )
        self._publish_encoder_values(status)

    def _publish_encoder_values(self, status: TeensySidePidStatus) -> None:
        left_ticks, right_ticks = status.side_ticks
        self.last_encoder_pair = (left_ticks, right_ticks)
        self.last_encoder_quad = status.wheel_ticks
        self.last_teensy_ms = status.controller_millis

        enc = Int32MultiArray()
        enc.data = [left_ticks, right_ticks]
        self.encoder_pub.publish(enc)

        stamped = EncoderTicksStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.encoder_frame_id
        stamped.left_ticks = left_ticks
        stamped.right_ticks = right_ticks
        stamped.front_left_ticks = status.fl_ticks
        stamped.front_right_ticks = status.fr_ticks
        stamped.rear_left_ticks = status.rl_ticks
        stamped.rear_right_ticks = status.rr_ticks
        stamped.controller_millis = max(0, status.controller_millis)
        self.encoder_stamped_pub.publish(stamped)

        raw = Int32MultiArray()
        raw.data = list(status.wheel_ticks)
        self.raw_encoder_pub.publish(raw)
        status_dict = status.as_status_dict()
        left_warn, left_fault = side_mismatch_flags(
            status.fl_tps,
            status.rl_tps,
            warn_tps=self.teensy_side_mismatch_warn_tps,
            fault_tps=self.teensy_side_mismatch_fault_tps,
        )
        right_warn, right_fault = side_mismatch_flags(
            status.fr_tps,
            status.rr_tps,
            warn_tps=self.teensy_side_mismatch_warn_tps,
            fault_tps=self.teensy_side_mismatch_fault_tps,
        )
        status_dict["left_front_rear_mismatch_warn"] = left_warn
        status_dict["right_front_rear_mismatch_warn"] = right_warn
        status_dict["left_front_rear_mismatch_fault_threshold"] = left_fault
        status_dict["right_front_rear_mismatch_fault_threshold"] = right_fault
        self._publish_status(**status_dict)

    def _teensy_pid_param_values(self) -> OrderedDict[str, Any]:
        return OrderedDict(
            [
                ("track_width_m", self.track_width_m),
                ("wheel_radius_m", self.wheel_radius_m),
                ("ticks_per_rev", self.ticks_per_rev),
                ("kp", self.teensy_pid_kp),
                ("ki", self.teensy_pid_ki),
                ("kd", self.teensy_pid_kd),
                ("command_timeout_ms", int(round(self.command_timeout_s * 1000.0))),
                ("pwm_min_us", self.pwm_min_us),
                ("pwm_neutral_us", self.pwm_neutral_us),
                ("pwm_max_us", self.pwm_max_us),
                ("pwm_slew_us_per_s", self.pwm_slew_rate_us_per_s),
                ("control_hz", self.teensy_control_hz),
                ("left_motor_sign", self.teensy_left_motor_sign),
                ("right_motor_sign", self.teensy_right_motor_sign),
                ("fl_encoder_sign", self.teensy_fl_encoder_sign),
                ("fr_encoder_sign", self.teensy_fr_encoder_sign),
                ("rl_encoder_sign", self.teensy_rl_encoder_sign),
                ("rr_encoder_sign", self.teensy_rr_encoder_sign),
                ("ff_us_per_tps", self.teensy_pid_feedforward_us_per_tps),
                ("static_ff_us", self.teensy_pid_static_ff_us),
                ("pid_output_limit_us", self.teensy_pid_output_limit_us),
                ("min_target_tps", self.teensy_pid_min_target_tps),
                ("stall_fault_enabled", 1 if self.teensy_stall_fault_enabled else 0),
                ("stall_target_tps", self.teensy_stall_target_tps),
                ("stall_near_zero_tps", self.teensy_stall_near_zero_tps),
                ("stall_moving_peer_tps", self.teensy_stall_moving_peer_tps),
                ("stall_pwm_delta_us", self.teensy_stall_pwm_delta_us),
                ("stall_timeout_ms", self.teensy_stall_timeout_ms),
                ("sign_mismatch_tps", self.teensy_sign_mismatch_tps),
                ("sign_mismatch_target_tps", self.teensy_sign_mismatch_target_tps),
                ("sign_mismatch_timeout_ms", self.teensy_sign_mismatch_timeout_ms),
                ("side_mismatch_fault_enabled", 1 if self.teensy_side_mismatch_fault_enabled else 0),
                ("side_mismatch_warn_tps", self.teensy_side_mismatch_warn_tps),
                ("side_mismatch_fault_tps", self.teensy_side_mismatch_fault_tps),
                ("encoder_jump_fault_enabled", 1 if self.teensy_encoder_jump_fault_enabled else 0),
                ("encoder_jump_tps", self.teensy_encoder_jump_tps),
            ]
        )

    def _sync_teensy_pid_params(self, reason: str) -> None:
        params = self._teensy_pid_param_values()
        self.teensy_pid_param_sync.start(params.keys(), time.monotonic())
        self._copy_param_sync_state()
        if self.dry_run_connected and self.serial_device is None:
            for name in params:
                self.teensy_pid_param_sync.handle_ack(type("Ack", (), {"name": name, "ok": True})(), time.monotonic())
            self._copy_param_sync_state()
            self.teensy_pid_param_sync_count += self.teensy_pid_param_sync.acked_count
            self.teensy_pid_param_sync_last_s = self.teensy_pid_param_sync.completed_s
            return
        for name, value in params.items():
            if not self._write_command(
                build_teensy_param_command(name, value),
                reason=reason,
                log_payload=f"CMD PARAM {name} {value}",
            ):
                self.teensy_pid_param_sync.mark_write_failed(name)
                self._copy_param_sync_state()
                return
        self._copy_param_sync_state()

    def _copy_param_sync_state(self) -> None:
        self.teensy_pid_params_synced = bool(self.teensy_pid_param_sync.synced)
        self.teensy_pid_param_sync_pending = list(self.teensy_pid_param_sync.pending_names)
        self.teensy_pid_param_sync_failed = bool(self.teensy_pid_param_sync.failed)
        self.teensy_pid_param_sync_reason = self.teensy_pid_param_sync.reason

    def _handle_param_sync_timeout(self) -> None:
        if not self.teensy_pid_param_sync.check_timeout(time.monotonic(), self.teensy_pid_param_ack_timeout_s):
            return
        self._copy_param_sync_state()
        self.get_logger().warn(f"Teensy parameter sync timeout: {self.teensy_pid_param_sync_reason}")
        self._publish_status(event="teensy param sync timeout")

    def _motion_allowed(self) -> bool:
        return bool(self.teensy_pid_params_synced)

    def _transport_available(self) -> bool:
        return self.serial_device is not None or self.dry_run_connected

    def _send_stop(self, reason: str) -> None:
        self._write_command(build_teensy_stop_command(), reason=reason, log_payload="CMD STOP")

    def _write_command(self, line: str, *, reason: str, log_payload: str) -> bool:
        if self.dry_run_connected and self.serial_device is None:
            self.last_command_sent_s = time.monotonic()
            self._throttled_info(f"DRY RUN motor command ({reason}): {log_payload}", period_s=0.5)
            return True
        if self.serial_device is None:
            return False
        try:
            self.serial_device.write(line.encode("utf-8"))
            self.serial_device.flush()
            self.last_command_sent_s = time.monotonic()
            self._throttled_info(f"Sent motor command ({reason}): {log_payload}", period_s=0.5)
            return True
        except serial.SerialException as exc:
            self.get_logger().warn(f"Failed to send motor command: {exc}")
            self._close_serial()
            return False

    def _publish_connected(self) -> None:
        connected = self.serial_device is not None or self.dry_run_connected
        self.connected_pub.publish(Bool(data=connected))
        if self.last_connected_state is None or connected != self.last_connected_state:
            self._publish_status(event="serial connected" if connected else "serial disconnected")
            self.last_connected_state = connected

    def _publish_status(self, **extra: Any) -> None:
        connected = self.serial_device is not None or self.dry_run_connected
        status: Dict[str, Any] = {
            "connected": connected,
            "port": self.port,
            "baud": self.baud,
            "dry_run": self.dry_run,
            "control_mode": "two_controller_four_encoder_side_pid",
            "motor_hardware": "four_pololu_37d_50_1_motors_two_gobilda_1x15a_pwm_controllers",
            "actuator_outputs": 2,
            "encoder_channels": 4,
            "encoder_feedback": "pololu_motor_quadrature",
            "same_side_sync": "diagnostic_only",
            "accepted_command_contract": "velocity_only",
            "teensy_pid_params_synced": bool(self.teensy_pid_params_synced),
            "teensy_pid_param_sync_pending": list(self.teensy_pid_param_sync_pending),
            "teensy_pid_param_sync_failed": bool(self.teensy_pid_param_sync_failed),
            "teensy_pid_param_sync_reason": self.teensy_pid_param_sync_reason,
            "teensy_pid_param_sync_count": int(self.teensy_pid_param_sync_count),
            "teensy_pid_param_sync_last_s": self.teensy_pid_param_sync_last_s,
            "track_width_m": round(self.track_width_m, 4),
            "wheel_radius_m": round(self.wheel_radius_m, 4),
            "ticks_per_rev": int(self.ticks_per_rev),
            "teensy_control_hz": round(self.teensy_control_hz, 3),
            "teensy_pid_kp": round(self.teensy_pid_kp, 6),
            "teensy_pid_ki": round(self.teensy_pid_ki, 6),
            "teensy_pid_kd": round(self.teensy_pid_kd, 6),
            "teensy_pid_feedforward_us_per_tps": round(self.teensy_pid_feedforward_us_per_tps, 6),
            "teensy_pid_static_ff_us": round(self.teensy_pid_static_ff_us, 3),
            "teensy_side_mismatch_warn_tps": round(self.teensy_side_mismatch_warn_tps, 3),
            "teensy_side_mismatch_fault_tps": round(self.teensy_side_mismatch_fault_tps, 3),
            "teensy_sign_mismatch_tps": round(self.teensy_sign_mismatch_tps, 3),
            "teensy_sign_mismatch_target_tps": round(self.teensy_sign_mismatch_target_tps, 3),
            "teensy_sign_mismatch_timeout_ms": int(self.teensy_sign_mismatch_timeout_ms),
            "teensy_encoder_jump_tps": round(self.teensy_encoder_jump_tps, 3),
            "target_left_mps": round(self.target_left_mps, 4),
            "target_right_mps": round(self.target_right_mps, 4),
            "measured_left_mps": round(self.measured_left_mps, 4),
            "measured_right_mps": round(self.measured_right_mps, 4),
            "command_age_s": self._command_age_s(),
        }
        if self.last_encoder_pair is not None:
            status["encoder_ticks"] = list(self.last_encoder_pair)
        if self.last_encoder_quad is not None:
            status["raw_ticks"] = list(self.last_encoder_quad)
        if self.last_teensy_ms is not None:
            status["teensy_ms"] = self.last_teensy_ms
        status.update(extra)
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        self.last_status_publish_s = time.monotonic()

    def _maybe_publish_periodic_status(self) -> None:
        if time.monotonic() - self.last_status_publish_s >= self.status_period_s:
            self._publish_status()

    def _command_age_s(self) -> Optional[float]:
        if self.last_command_received_s <= 0.0:
            return None
        return round(max(0.0, time.monotonic() - self.last_command_received_s), 3)

    def _close_serial(self) -> None:
        if self.serial_device is not None:
            try:
                self.serial_device.close()
            except Exception:
                pass
        self.serial_device = None
        self.dry_run_connected = False
        self.serial_rx_buffer = ""
        self.teensy_pid_param_sync.mark_disconnected()
        self._copy_param_sync_state()

    def _throttled_info(self, msg: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_log_s >= period_s:
            self.get_logger().info(msg)
            self.last_log_s = now

    def _log_warn(self, msg: str) -> None:
        self.get_logger().warn(msg)

    def destroy_node(self) -> None:
        try:
            self._send_stop("shutdown stop")
        finally:
            self._close_serial()
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MotorControllerBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
