#!/usr/bin/env python3
"""Safe Jetson chassis controller entrypoint.

Default real mode stays in ``idle`` and publishes STOP. Explicit test modes
publish velocity-only JSON:

    {"command_type": "velocity", "v_mps": <float>, "omega_radps": <float>}
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Deque, Dict, Optional, Sequence

from ugv_nav_core.chassis_controller import (
    ChassisControllerConfig,
    ChassisEstimatorState,
    GyroBiasCalibrationState,
    PivotControllerState,
    bounded_duration,
    compute_straight_omega,
    evaluate_pivot_clearance,
    evaluate_safety,
    min_finite_range,
    pivot_encoder_gyro_disagreement,
    reset_gyro_bias_calibration,
    reset_pivot,
    start_pivot,
    step_profiled_pivot,
    update_encoder_heading,
    update_gyro_bias_calibration,
    update_gyro_heading,
)
from ugv_nav_core.mission_controller import (
    MissionPlan,
    MissionSegment,
    MissionTelemetryRecorder,
    StuckMonitorState,
    apply_competition_speed_rule,
    classify_mission_safety,
    encoder_average_distance_m,
    heading_error_rms,
    load_mission_plan,
    mission_segment_start_hold_reason,
    motion_rule_ok,
    normalized_imu_qos,
    reset_stuck_monitor,
    segment_timeout_s,
    straight_omega_with_slew,
    telemetry_force_flush_key,
    update_stuck_monitor,
)


@dataclass(frozen=True)
class ControlCommand:
    command_type: str = "stop"
    v_mps: float = 0.0
    omega_radps: float = 0.0
    reason: str = "idle"
    controller: str = "ugv_chassis_controller"

    def to_json(self) -> str:
        mode = "STOP" if self.command_type == "stop" else "VELOCITY"
        return json.dumps(
            {
                "mode": mode,
                "command_type": self.command_type,
                "controller": self.controller,
                "v_mps": float(self.v_mps),
                "omega_radps": float(self.omega_radps),
                "reason": self.reason,
            },
            sort_keys=True,
        )


def build_stop_command(reason: str = "idle") -> ControlCommand:
    return ControlCommand(command_type="stop", reason=reason)


def build_velocity_command(v_mps: float, omega_radps: float, reason: str) -> ControlCommand:
    return ControlCommand(
        command_type="velocity",
        v_mps=float(v_mps),
        omega_radps=float(omega_radps),
        reason=reason,
    )


def parse_bool(value: str) -> bool:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe UGV chassis controller. Default mode publishes STOP.",
        allow_abbrev=False,
    )
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument(
        "--controller-mode",
        choices=["idle", "straight_test", "pivot_test", "mission_sequence"],
        default="idle",
    )
    parser.add_argument("--command-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--nav-frame-topic", default="/sensors/nav_frame")
    parser.add_argument("--imu-topic", default="/zed/imu")
    parser.add_argument("--imu-qos", default="sensor_data")
    parser.add_argument("--imu-yaw-axis", choices=["x", "y", "z"], default="z")
    parser.add_argument("--imu-yaw-sign", type=float, default=1.0)
    parser.add_argument("--motor-status-topic", default="/motor_controller/status")
    parser.add_argument("--nav-status-period-s", type=float, default=0.25)
    parser.add_argument("--control-period-s", type=float, default=0.02)
    parser.add_argument("--straight-speed-mps", type=float, default=0.20)
    parser.add_argument("--straight-duration-s", type=float, default=2.0)
    parser.add_argument("--pivot-angle-deg", type=float, default=90.0)
    parser.add_argument("--max-omega-radps", type=float, default=0.45)
    parser.add_argument("--heading-kp", type=float, default=1.2)
    parser.add_argument("--heading-kd", type=float, default=0.15)
    parser.add_argument("--pivot-kp", type=float, default=1.0)
    parser.add_argument("--heading-deadband-rad", type=float, default=0.025)
    parser.add_argument("--stop-clearance-m", type=float, default=0.45)
    parser.add_argument("--sensor-timeout-s", type=float, default=0.30)
    parser.add_argument("--imu-timeout-s", type=float, default=0.12)
    parser.add_argument("--motor-status-timeout-s", type=float, default=0.50)
    parser.add_argument("--max-test-duration-s", type=float, default=3.0)
    parser.add_argument("--gyro-bias-calibration-s", type=float, default=1.5)
    parser.add_argument("--gyro-bias-max-std-radps", type=float, default=0.03)
    parser.add_argument("--gyro-bias-warn-abs-radps", type=float, default=0.10)
    parser.add_argument("--gyro-bias-max-encoder-delta-ticks", type=int, default=2)
    parser.add_argument("--pivot-max-omega-radps", type=float, default=0.35)
    parser.add_argument("--pivot-min-omega-radps", type=float, default=0.16)
    parser.add_argument("--pivot-breakaway-omega-radps", type=float, default=0.18)
    parser.add_argument("--pivot-breakaway-s", type=float, default=0.20)
    parser.add_argument("--pivot-accel-limit-radps2", type=float, default=0.80)
    parser.add_argument("--pivot-decel-limit-radps2", type=float, default=0.60)
    parser.add_argument("--pivot-approach-error-rad", type=float, default=0.25)
    parser.add_argument("--pivot-min-omega-disable-error-rad", type=float, default=0.10)
    parser.add_argument("--pivot-kp-approach", type=float, default=0.75)
    parser.add_argument("--pivot-kd-yaw-rate", type=float, default=0.10)
    parser.add_argument("--pivot-settle-error-rad", type=float, default=0.035)
    parser.add_argument("--pivot-settle-yaw-rate-radps", type=float, default=0.05)
    parser.add_argument("--pivot-settle-time-s", type=float, default=0.35)
    parser.add_argument("--pivot-brake-s", type=float, default=0.15)
    parser.add_argument("--pivot-timeout-s", type=float, default=4.0)
    parser.add_argument("--pivot-max-correction-retries", type=int, default=1)
    parser.add_argument("--pivot-clearance-m", type=float, default=0.35)
    parser.add_argument("--slip-disagreement-rad", type=float, default=0.35)
    parser.add_argument("--mission-file", default="")
    parser.add_argument("--competition-min-speed-mps", type=float, default=0.089408)
    parser.add_argument("--mission-default-speed-mps", type=float, default=0.15)
    parser.add_argument("--mission-reliable-speed-mps", type=float, default=0.15)
    parser.add_argument("--mission-slow-speed-mps", type=float, default=0.09)
    parser.add_argument("--mission-emergency-stop-clearance-m", type=float, default=0.18)
    parser.add_argument("--mission-critical-sensor-timeout-s", type=float, default=1.0)
    parser.add_argument("--mission-straight-max-omega-radps", type=float, default=0.20)
    parser.add_argument("--mission-straight-omega-slew-radps2", type=float, default=0.80)
    parser.add_argument("--debug-allow-sub-min-crawl", type=parse_bool, default=False)
    parser.add_argument("--debug-allow-unknown-args", type=parse_bool, default=False)
    parser.add_argument("--debug-allow-unknown-pivot-clearance", type=parse_bool, default=False)
    parser.add_argument("--mission-stop-on-degraded-obstacle", type=parse_bool, default=True)
    parser.add_argument("--mission-telemetry-active-hz", type=float, default=50.0)
    parser.add_argument("--mission-telemetry-flush-period-s", type=float, default=0.50)
    parser.add_argument("--mission-telemetry-flush-max-records", type=int, default=25)
    parser.add_argument("--imu-rate-window-s", type=float, default=2.0)
    parser.add_argument("--stuck-detection-enabled", type=parse_bool, default=True)
    parser.add_argument("--straight-stuck-timeout-s", type=float, default=0.50)
    parser.add_argument("--pivot-stuck-timeout-s", type=float, default=0.45)
    parser.add_argument("--pivot-breakaway-retry-scale", type=float, default=1.25)
    parser.add_argument("--telemetry-enabled", type=parse_bool, default=True)
    parser.add_argument("--telemetry-dir", default="~/.ros/ugv_mission_logs")
    parser.add_argument("--track-width-m", type=float, default=0.425)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0825)
    parser.add_argument("--ticks-per-rev", type=float, default=3200.0)
    parser.add_argument("--once", action="store_true", help="publish one command/status cycle and exit in real mode")
    args, unknown = parser.parse_known_args(argv)
    try:
        args.imu_qos = normalized_imu_qos(args.imu_qos)
    except ValueError as exc:
        parser.error(str(exc))
    if unknown:
        if bool(args.debug_allow_unknown_args):
            print(f"Ignoring unsupported navigation arguments: {' '.join(unknown)}")
        else:
            parser.error(f"unsupported navigation arguments: {' '.join(unknown)}")
    return args


def config_from_args(args: argparse.Namespace) -> ChassisControllerConfig:
    return ChassisControllerConfig(
        straight_speed_mps=float(args.straight_speed_mps),
        straight_duration_s=float(args.straight_duration_s),
        pivot_angle_deg=float(args.pivot_angle_deg),
        max_omega_radps=float(args.max_omega_radps),
        heading_kp=float(args.heading_kp),
        heading_kd=float(args.heading_kd),
        pivot_kp=float(args.pivot_kp),
        heading_deadband_rad=float(args.heading_deadband_rad),
        stop_clearance_m=float(args.stop_clearance_m),
        sensor_timeout_s=float(args.sensor_timeout_s),
        imu_timeout_s=float(args.imu_timeout_s),
        motor_status_timeout_s=float(args.motor_status_timeout_s),
        max_test_duration_s=float(args.max_test_duration_s),
        gyro_bias_calibration_s=float(args.gyro_bias_calibration_s),
        gyro_bias_max_std_radps=float(args.gyro_bias_max_std_radps),
        gyro_bias_warn_abs_radps=float(args.gyro_bias_warn_abs_radps),
        gyro_bias_max_encoder_delta_ticks=int(args.gyro_bias_max_encoder_delta_ticks),
        pivot_max_omega_radps=float(args.pivot_max_omega_radps),
        pivot_min_omega_radps=float(args.pivot_min_omega_radps),
        pivot_breakaway_omega_radps=float(args.pivot_breakaway_omega_radps),
        pivot_breakaway_s=float(args.pivot_breakaway_s),
        pivot_accel_limit_radps2=float(args.pivot_accel_limit_radps2),
        pivot_decel_limit_radps2=float(args.pivot_decel_limit_radps2),
        pivot_approach_error_rad=float(args.pivot_approach_error_rad),
        pivot_min_omega_disable_error_rad=float(args.pivot_min_omega_disable_error_rad),
        pivot_kp_approach=float(args.pivot_kp_approach),
        pivot_kd_yaw_rate=float(args.pivot_kd_yaw_rate),
        pivot_settle_error_rad=float(args.pivot_settle_error_rad),
        pivot_settle_yaw_rate_radps=float(args.pivot_settle_yaw_rate_radps),
        pivot_settle_time_s=float(args.pivot_settle_time_s),
        pivot_brake_s=float(args.pivot_brake_s),
        pivot_timeout_s=float(args.pivot_timeout_s),
        pivot_max_correction_retries=int(args.pivot_max_correction_retries),
        pivot_clearance_m=float(args.pivot_clearance_m),
        slip_disagreement_rad=float(args.slip_disagreement_rad),
        competition_min_speed_mps=float(args.competition_min_speed_mps),
        mission_default_speed_mps=float(args.mission_default_speed_mps),
        mission_reliable_speed_mps=float(args.mission_reliable_speed_mps),
        mission_slow_speed_mps=float(args.mission_slow_speed_mps),
        mission_emergency_stop_clearance_m=float(args.mission_emergency_stop_clearance_m),
        mission_critical_sensor_timeout_s=float(args.mission_critical_sensor_timeout_s),
        mission_straight_max_omega_radps=float(args.mission_straight_max_omega_radps),
        mission_straight_omega_slew_radps2=float(args.mission_straight_omega_slew_radps2),
        debug_allow_sub_min_crawl=bool(args.debug_allow_sub_min_crawl),
        debug_allow_unknown_pivot_clearance=bool(args.debug_allow_unknown_pivot_clearance),
        mission_stop_on_degraded_obstacle=bool(args.mission_stop_on_degraded_obstacle),
        mission_telemetry_active_hz=float(args.mission_telemetry_active_hz),
        mission_telemetry_flush_period_s=float(args.mission_telemetry_flush_period_s),
        mission_telemetry_flush_max_records=int(args.mission_telemetry_flush_max_records),
        imu_rate_window_s=float(args.imu_rate_window_s),
        stuck_detection_enabled=bool(args.stuck_detection_enabled),
        straight_stuck_timeout_s=float(args.straight_stuck_timeout_s),
        pivot_stuck_timeout_s=float(args.pivot_stuck_timeout_s),
        pivot_breakaway_retry_scale=float(args.pivot_breakaway_retry_scale),
        track_width_m=float(args.track_width_m),
        wheel_radius_m=float(args.wheel_radius_m),
        ticks_per_rev=float(args.ticks_per_rev),
    )


def run_sim() -> None:
    print("Clean navigation chassis controller: real mode is idle unless a test or mission mode is selected.")


def run_real(args: argparse.Namespace) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        from ugv_sensor_sync.msg import NavSensorFrame
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python packages are required for real mode: {exc}") from exc

    config = config_from_args(args)
    mission_plan: Optional[MissionPlan] = None
    if str(args.controller_mode) == "mission_sequence":
        if not str(args.mission_file).strip():
            raise SystemExit("--mission-file is required for controller-mode=mission_sequence")
        try:
            mission_plan = load_mission_plan(str(args.mission_file))
        except (OSError, ValueError) as exc:
            raise SystemExit(f"Failed to load mission file: {exc}") from exc

    class ChassisControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("ugv_chassis_controller")
            self.config = config
            self.mode = str(args.controller_mode)
            self.estimator = ChassisEstimatorState()
            self.gyro_bias = GyroBiasCalibrationState()
            self.pivot = PivotControllerState()
            self.last_sensor_s: Optional[float] = None
            self.last_imu_s: Optional[float] = None
            self.last_motor_status_s: Optional[float] = None
            self.motor_status: Optional[Dict[str, Any]] = None
            self.near_obstacle = False
            self.front_clearance_m: Optional[float] = None
            self.pivot_clearance_m: Optional[float] = None
            self.last_scan_ranges: Optional[list[float]] = None
            self.raw_yaw_rate_radps = 0.0
            self.yaw_rate_radps = 0.0
            self.imu_dt_integrated = False
            self.mode_start_s: Optional[float] = None
            self.target_heading_rad: Optional[float] = None
            self.encoder_gyro_disagreement_rad = 0.0
            self.slip_detected = False
            self.last_status_publish_s = 0.0
            self.last_telemetry_write_s = 0.0
            self.last_telemetry_force_flush_key: Optional[str] = None
            self.last_safety_log_s = 0.0
            self.last_safety_reason: Optional[str] = None
            self.last_command_payload: Dict[str, Any] = json.loads(build_stop_command().to_json())
            self.current_left_ticks: Optional[int] = None
            self.current_right_ticks: Optional[int] = None
            self.encoder_available = False
            self.mission_plan = mission_plan
            self.mission_state = "idle"
            self.mission_safety_level = "ok"
            self.mission_safety_reason = "idle"
            self.segment_index: Optional[int] = None
            self.segment_start_s: Optional[float] = None
            self.segment_start_left_ticks: Optional[int] = None
            self.segment_start_right_ticks: Optional[int] = None
            self.segment_distance_m = 0.0
            self.target_distance_m: Optional[float] = None
            self.previous_straight_omega_radps = 0.0
            self.last_mission_update_s: Optional[float] = None
            self.straight_heading_error_samples: list[float] = []
            self.sub_min_speed_command_blocked = False
            self.imu_arrival_times_s: Deque[float] = deque()
            self.imu_dt_samples_s: Deque[tuple[float, float]] = deque()
            self.imu_skipped_integrations = 0
            self.imu_max_dt_s = 0.0
            self.stuck_monitor = StuckMonitorState()
            self.stuck_detected = False
            self.stuck_reason = "ok"
            self.pivot_breakaway_scale = 1.0
            self.telemetry = MissionTelemetryRecorder(
                enabled=bool(args.telemetry_enabled) and self.mission_plan is not None,
                telemetry_dir=str(args.telemetry_dir),
                mission_id=self.mission_plan.mission_id if self.mission_plan is not None else "mission",
                flush_period_s=float(self.config.mission_telemetry_flush_period_s),
                flush_max_records=int(self.config.mission_telemetry_flush_max_records),
            )

            self.imu_axis = str(args.imu_yaw_axis)
            self.imu_sign = -1.0 if float(args.imu_yaw_sign) < 0.0 else 1.0
            self.imu_qos_name = normalized_imu_qos(str(args.imu_qos))
            imu_qos = qos_profile_sensor_data if self.imu_qos_name == "sensor_data" else 50

            self.cmd_pub = self.create_publisher(String, args.command_topic, 10)
            self.status_pub = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(NavSensorFrame, args.nav_frame_topic, self.nav_frame_callback, 10)
            self.create_subscription(Imu, args.imu_topic, self.imu_callback, imu_qos)
            self.create_subscription(String, args.motor_status_topic, self.motor_status_callback, 10)
            self.timer = self.create_timer(max(0.01, float(args.control_period_s)), self.tick)
            self.get_logger().warn(
                f"Chassis controller started in {self.mode}; only velocity/STOP JSON is published."
            )
            self.get_logger().info(
                "Chassis sanity: "
                f"control_hz={1.0 / max(0.01, float(args.control_period_s)):.1f}, "
                f"imu_topic={args.imu_topic}, imu_qos={self.imu_qos_name}, "
                f"competition_min_speed={self.config.competition_min_speed_mps:.6f}m/s, "
                f"heading_kp={self.config.heading_kp:.3f}, heading_kd={self.config.heading_kd:.3f}, "
                f"pivot_max_omega={self.config.pivot_max_omega_radps:.3f}, "
                f"stop_clearance={self.config.stop_clearance_m:.3f}, "
                f"pivot_clearance={self.config.pivot_clearance_m:.3f}"
            )

        def nav_frame_callback(self, msg: Any) -> None:
            now_s = time.monotonic()
            self.last_sensor_s = now_s
            self.near_obstacle = bool(msg.near_obstacle)
            self.front_clearance_m = float(msg.front_clearance_m)
            self.last_scan_ranges = [float(value) for value in getattr(msg.scan, "ranges", [])]
            self.pivot_clearance_m = min_finite_range(self.last_scan_ranges)
            self.current_left_ticks = int(msg.left_encoder_ticks)
            self.current_right_ticks = int(msg.right_encoder_ticks)
            self.encoder_available = bool(msg.encoder_available)
            self.estimator = update_encoder_heading(
                self.estimator,
                left_ticks=self.current_left_ticks,
                right_ticks=self.current_right_ticks,
                encoder_available=self.encoder_available,
                config=self.config,
            )

        def imu_callback(self, msg: Any) -> None:
            now_s = time.monotonic()
            stamp_s = self._stamp_to_seconds(msg.header.stamp) or now_s
            self.last_imu_s = now_s
            self._record_imu_arrival(now_s)
            angular_velocity = msg.angular_velocity
            raw_value = float(getattr(angular_velocity, self.imu_axis))
            self.raw_yaw_rate_radps = self.imu_sign * raw_value
            bias = self.gyro_bias.bias_radps if self.gyro_bias.ready else 0.0
            previous_stamp_s = self.estimator.last_stamp_s
            self.estimator, self.yaw_rate_radps, self.imu_dt_integrated = update_gyro_heading(
                self.estimator,
                stamp_s=stamp_s,
                raw_yaw_rate_radps=self.raw_yaw_rate_radps,
                gyro_bias_radps=bias,
            )
            if previous_stamp_s is not None:
                dt_s = float(stamp_s) - float(previous_stamp_s)
                if math.isfinite(dt_s) and dt_s > 0.0:
                    self.imu_dt_samples_s.append((now_s, dt_s))
                    self.imu_max_dt_s = max(self.imu_max_dt_s, dt_s)
                if not self.imu_dt_integrated:
                    self.imu_skipped_integrations += 1
            self._trim_imu_diagnostics(now_s)

        def motor_status_callback(self, msg: Any) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"Ignoring invalid motor status JSON: {exc}")
                return
            if isinstance(payload, dict):
                self.motor_status = payload
                self.last_motor_status_s = time.monotonic()

        @staticmethod
        def _stamp_to_seconds(stamp: Any) -> Optional[float]:
            sec = int(getattr(stamp, "sec", 0))
            nanosec = int(getattr(stamp, "nanosec", 0))
            if sec == 0 and nanosec == 0:
                return None
            return float(sec) + float(nanosec) * 1e-9

        def _record_imu_arrival(self, now_s: float) -> None:
            self.imu_arrival_times_s.append(float(now_s))
            self._trim_imu_diagnostics(now_s)

        def _trim_imu_diagnostics(self, now_s: float) -> None:
            window_s = max(0.1, float(self.config.imu_rate_window_s))
            while self.imu_arrival_times_s and now_s - self.imu_arrival_times_s[0] > window_s:
                self.imu_arrival_times_s.popleft()
            while self.imu_dt_samples_s and now_s - self.imu_dt_samples_s[0][0] > window_s:
                self.imu_dt_samples_s.popleft()
            self.imu_max_dt_s = max((dt for _, dt in self.imu_dt_samples_s), default=0.0)

        def _imu_rate_hz(self) -> float:
            if len(self.imu_arrival_times_s) < 2:
                return 0.0
            elapsed_s = self.imu_arrival_times_s[-1] - self.imu_arrival_times_s[0]
            if elapsed_s <= 0.0:
                return 0.0
            return float(len(self.imu_arrival_times_s) - 1) / elapsed_s

        def _start_straight_if_needed(self, now_s: float) -> None:
            if self.mode_start_s is not None:
                return
            self.mode_start_s = now_s
            self.target_heading_rad = self.estimator.heading_rad

        def _start_pivot_if_needed(self, now_s: float) -> None:
            if self.pivot.state != "idle":
                return
            self.mode_start_s = now_s
            start_pivot(
                self.pivot,
                now_s=now_s,
                current_heading_rad=self.estimator.heading_rad,
                encoder_heading_rad=self.estimator.encoder_heading_rad,
                target_angle_rad=math.radians(self.config.pivot_angle_deg),
            )
            self.target_heading_rad = self.pivot.target_heading_rad

        def _reset_test_state(self) -> None:
            self.mode_start_s = None
            self.target_heading_rad = None
            reset_pivot(self.pivot)
            self.encoder_gyro_disagreement_rad = 0.0
            self.slip_detected = False

        def _reset_bias_and_heading(self) -> None:
            reset_gyro_bias_calibration(self.gyro_bias)
            self.estimator.gyro_heading_rad = 0.0
            self.estimator.last_stamp_s = None
            self.estimator.gyro_available = False

        def _calibrate_gyro_bias_if_needed(self, now_s: float) -> tuple[bool, str]:
            if self.gyro_bias.ready:
                return True, "gyro_bias_ready"
            result = update_gyro_bias_calibration(
                self.gyro_bias,
                now_s=now_s,
                raw_yaw_rate_radps=self.raw_yaw_rate_radps,
                left_ticks=self.estimator.last_left_ticks,
                right_ticks=self.estimator.last_right_ticks,
                config=self.config,
            )
            if result.unstable:
                self.get_logger().warn(
                    f"Gyro bias calibration rejected: {result.reason} "
                    f"(bias={result.bias_radps:.5f}, std={result.std_radps:.5f})"
                )
                return False, result.reason
            if result.ready:
                self.estimator.gyro_heading_rad = 0.0
                self.estimator.gyro_available = True
                self.estimator.last_stamp_s = None
                if result.reason == "gyro_bias_large":
                    self.get_logger().warn(
                        f"Gyro bias calibrated but large: {result.bias_radps:.5f} rad/s "
                        f"(std={result.std_radps:.5f})"
                    )
                else:
                    self.get_logger().info(
                        f"Gyro bias calibrated: {result.bias_radps:.5f} rad/s "
                        f"(std={result.std_radps:.5f})"
                    )
                return True, result.reason
            return False, result.reason

        def _evaluate_active_safety(self, now_s: float) -> str:
            safety = evaluate_safety(
                now_s=now_s,
                last_sensor_s=self.last_sensor_s,
                last_imu_s=self.last_imu_s,
                last_motor_status_s=self.last_motor_status_s,
                motor_status=self.motor_status,
                near_obstacle=self.near_obstacle,
                front_clearance_m=self.front_clearance_m,
                config=self.config,
                require_imu=True,
            )
            if not safety.safe:
                return safety.reason
            if self.mode == "pivot_test":
                pivot_safety = evaluate_pivot_clearance(self.last_scan_ranges, self.config)
                if not pivot_safety.safe:
                    return pivot_safety.reason
            return "ok"

        def _reset_mission_state(self) -> None:
            self.mission_state = "idle"
            self.mission_safety_level = "ok"
            self.mission_safety_reason = "idle"
            self.segment_index = None
            self.segment_start_s = None
            self.segment_start_left_ticks = None
            self.segment_start_right_ticks = None
            self.segment_distance_m = 0.0
            self.target_distance_m = None
            self.previous_straight_omega_radps = 0.0
            self.last_mission_update_s = None
            self.straight_heading_error_samples.clear()
            self.target_heading_rad = None
            self.last_telemetry_force_flush_key = None
            self.stuck_detected = False
            self.stuck_reason = "ok"
            reset_stuck_monitor(self.stuck_monitor)
            self.pivot_breakaway_scale = 1.0
            reset_pivot(self.pivot)

        def _current_segment(self) -> Optional[MissionSegment]:
            if self.mission_plan is None or self.segment_index is None:
                return None
            if self.segment_index < 0 or self.segment_index >= len(self.mission_plan.segments):
                return None
            return self.mission_plan.segments[self.segment_index]

        def _start_mission_segment(self, now_s: float) -> None:
            segment = self._current_segment()
            if segment is None:
                self.mission_state = "mission_complete"
                self.target_distance_m = None
                return
            self.segment_start_s = now_s
            self.segment_start_left_ticks = self.current_left_ticks
            self.segment_start_right_ticks = self.current_right_ticks
            self.segment_distance_m = 0.0
            self.previous_straight_omega_radps = 0.0
            self.last_mission_update_s = now_s
            self.straight_heading_error_samples.clear()
            self.last_telemetry_force_flush_key = None
            self.stuck_detected = False
            self.stuck_reason = "ok"
            reset_stuck_monitor(self.stuck_monitor)
            self.pivot_breakaway_scale = 1.0
            if segment.segment_type == "straight":
                self.target_heading_rad = self.estimator.heading_rad
                self.target_distance_m = segment.distance_m
                self.mission_state = "straight_active"
            elif segment.segment_type == "pivot":
                reset_pivot(self.pivot)
                start_pivot(
                    self.pivot,
                    now_s=now_s,
                    current_heading_rad=self.estimator.heading_rad,
                    encoder_heading_rad=self.estimator.encoder_heading_rad,
                    target_angle_rad=math.radians(segment.angle_deg),
                )
                self.target_heading_rad = self.pivot.target_heading_rad
                self.target_distance_m = None
                self.mission_state = "pivot_active"
            else:
                self.target_heading_rad = self.estimator.heading_rad
                self.target_distance_m = None
                self.mission_state = "wait_active"

        def _finish_mission_segment(self, reason: str) -> ControlCommand:
            if self.segment_index is None:
                self.mission_state = "mission_complete"
                return build_stop_command("mission_complete")
            self.segment_index += 1
            reset_pivot(self.pivot)
            self.previous_straight_omega_radps = 0.0
            reset_stuck_monitor(self.stuck_monitor)
            self.stuck_detected = False
            self.stuck_reason = "ok"
            self.pivot_breakaway_scale = 1.0
            self.segment_start_s = None
            self.target_heading_rad = None
            self.target_distance_m = None
            self.last_telemetry_force_flush_key = None
            if self.mission_plan is None or self.segment_index >= len(self.mission_plan.segments):
                self.mission_state = "mission_complete"
                return build_stop_command("mission_complete")
            self.mission_state = "segment_complete"
            return build_stop_command(reason)

        def _mission_segment_distance(self) -> float:
            self.segment_distance_m = encoder_average_distance_m(
                start_left_ticks=self.segment_start_left_ticks,
                start_right_ticks=self.segment_start_right_ticks,
                current_left_ticks=self.current_left_ticks,
                current_right_ticks=self.current_right_ticks,
                config=self.config,
            )
            return self.segment_distance_m

        def _evaluate_mission_safety(self, now_s: float) -> None:
            decision = classify_mission_safety(
                now_s=now_s,
                last_sensor_s=self.last_sensor_s,
                last_imu_s=self.last_imu_s,
                last_motor_status_s=self.last_motor_status_s,
                motor_status=self.motor_status,
                near_obstacle=self.near_obstacle,
                front_clearance_m=self.front_clearance_m,
                config=self.config,
                require_imu=True,
            )
            self.mission_safety_level = decision.level
            self.mission_safety_reason = decision.reason

        def _segment_key(self) -> str:
            segment_type = None if self._current_segment() is None else self._current_segment().segment_type
            return f"{self.segment_index}:{segment_type}:{self.segment_start_s}"

        def _stuck_stop_or_abort(
            self,
            *,
            now_s: float,
            command_kind: str,
            cmd: ControlCommand,
        ) -> Optional[ControlCommand]:
            result = update_stuck_monitor(
                self.stuck_monitor,
                now_s=now_s,
                segment_key=self._segment_key(),
                command_kind=command_kind,
                v_cmd_mps=cmd.v_mps,
                omega_cmd_radps=cmd.omega_radps,
                yaw_rate_radps=self.yaw_rate_radps,
                motor_status=self.motor_status,
                config=self.config,
            )
            self.stuck_detected = result.stuck
            self.stuck_reason = result.reason
            if not result.stuck:
                return None
            if self.stuck_monitor.recovery_count < 1:
                self.stuck_monitor.recovery_count += 1
                retry_reason = f"{result.reason}_retry"
                self.stuck_reason = retry_reason
                if command_kind == "pivot":
                    self.pivot_breakaway_scale *= max(1.0, float(self.config.pivot_breakaway_retry_scale))
                    if self.pivot.target_heading_rad is not None:
                        self.pivot.state = "breakaway"
                        self.pivot.state_start_s = now_s
                        self.pivot.last_update_s = now_s
                        self.pivot.previous_omega_radps = 0.0
                        self.pivot.settle_start_s = None
                        self.pivot.retry_reason = result.reason
                self._log_safety_stop(retry_reason, now_s)
                reset_stuck_monitor(self.stuck_monitor, keep_recovery_count=True)
                return build_stop_command(retry_reason)

            self.mission_state = "abort"
            self.mission_safety_level = "critical"
            self.mission_safety_reason = result.reason
            self._log_safety_stop(result.reason, now_s)
            return build_stop_command(result.reason)

        def _tick_mission(self, now_s: float) -> tuple[ControlCommand, float, float, str]:
            heading = self.estimator.heading_rad
            heading_error = 0.0
            self.sub_min_speed_command_blocked = False
            if self.mission_plan is None:
                self.mission_state = "abort"
                self.mission_safety_level = "critical"
                self.mission_safety_reason = "mission_missing"
                return build_stop_command("mission_missing"), heading, heading_error, "mission_missing"
            if self.mission_state == "mission_complete":
                return build_stop_command("mission_complete"), heading, heading_error, "ok"
            if self.mission_state == "abort":
                return build_stop_command(self.mission_safety_reason), heading, heading_error, self.mission_safety_reason

            if self.mission_state == "idle":
                self.mission_state = "preflight"
            self._evaluate_mission_safety(now_s)
            if self.mission_safety_level == "critical":
                if self.segment_index is None and self.mission_state in {"preflight", "gyro_bias_calibration"}:
                    self._log_safety_stop(self.mission_safety_reason, now_s)
                    return (
                        build_stop_command(self.mission_safety_reason),
                        heading,
                        heading_error,
                        self.mission_safety_reason,
                    )
                self.mission_state = "abort"
                self._log_safety_stop(self.mission_safety_reason, now_s)
                return build_stop_command(self.mission_safety_reason), heading, heading_error, self.mission_safety_reason

            calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
            if not calibrated:
                self.mission_state = "gyro_bias_calibration"
                return build_stop_command(calibration_reason), heading, heading_error, calibration_reason

            if self.segment_index is None:
                self.segment_index = 0
                self.mission_state = "segment_start"
            if self.mission_state in {"segment_start", "segment_complete"}:
                segment = self._current_segment()
                if segment is None:
                    self.mission_state = "mission_complete"
                    return build_stop_command("mission_complete"), heading, heading_error, "ok"
                pivot_clearance_reason = None
                if segment.segment_type == "pivot":
                    pivot_safety = evaluate_pivot_clearance(self.last_scan_ranges, self.config)
                    if not pivot_safety.safe:
                        pivot_clearance_reason = pivot_safety.reason
                hold_reason = mission_segment_start_hold_reason(
                    segment,
                    safety_level=self.mission_safety_level,
                    safety_reason=self.mission_safety_reason,
                    encoder_available=self.encoder_available,
                    left_ticks=self.current_left_ticks,
                    right_ticks=self.current_right_ticks,
                    pivot_clearance_reason=pivot_clearance_reason,
                    config=self.config,
                )
                if hold_reason is not None:
                    self.mission_safety_level = "degraded"
                    self.mission_safety_reason = hold_reason
                    self._log_safety_stop(hold_reason, now_s)
                    return build_stop_command(hold_reason), heading, heading_error, hold_reason
                self._start_mission_segment(now_s)
                return build_stop_command("segment_start"), heading, heading_error, "ok"
            segment = self._current_segment()
            if segment is None:
                self.mission_state = "mission_complete"
                return build_stop_command("mission_complete"), heading, heading_error, "ok"

            dt_s = 0.0 if self.last_mission_update_s is None else max(0.0, now_s - self.last_mission_update_s)
            self.last_mission_update_s = now_s
            segment_elapsed_s = 0.0 if self.segment_start_s is None else now_s - self.segment_start_s
            timeout_s = segment_timeout_s(segment, self.config)
            if timeout_s is not None and segment_elapsed_s > timeout_s:
                self.mission_state = "abort"
                self.mission_safety_level = "critical"
                self.mission_safety_reason = "segment_timeout"
                return build_stop_command("segment_timeout"), heading, heading_error, "segment_timeout"

            if self.mission_safety_level == "degraded":
                if self.mission_safety_reason in {"sensor_missing", "sensor_stale", "front_clearance_invalid"}:
                    return build_stop_command(self.mission_safety_reason), heading, heading_error, self.mission_safety_reason
                if segment.segment_type == "straight":
                    if (
                        self.config.mission_stop_on_degraded_obstacle
                        and self.mission_safety_reason in {"near_obstacle", "front_clearance_low"}
                    ):
                        return (
                            build_stop_command(self.mission_safety_reason),
                            heading,
                            heading_error,
                            self.mission_safety_reason,
                        )
                elif segment.segment_type == "pivot":
                    pivot_safety = evaluate_pivot_clearance(self.last_scan_ranges, self.config)
                    if not pivot_safety.safe:
                        return build_stop_command(pivot_safety.reason), heading, heading_error, pivot_safety.reason

            if segment.segment_type == "straight":
                distance_m = self._mission_segment_distance()
                if distance_m >= segment.distance_m:
                    return self._finish_mission_segment("segment_complete"), heading, heading_error, "ok"
                if not self.encoder_available:
                    self.mission_safety_level = "degraded"
                    self.mission_safety_reason = "encoder_unavailable"
                    return build_stop_command("encoder_unavailable"), heading, heading_error, "encoder_unavailable"
                target_heading = self.target_heading_rad if self.target_heading_rad is not None else heading
                heading_error = math.atan2(math.sin(target_heading - heading), math.cos(target_heading - heading))
                self.straight_heading_error_samples.append(heading_error)
                if self.mission_safety_level == "degraded":
                    requested_v = self.config.mission_slow_speed_mps
                    reason = f"mission_degraded:{self.mission_safety_reason}"
                else:
                    requested_v = (
                        self.config.mission_default_speed_mps
                        if segment.speed_mps is None
                        else float(segment.speed_mps)
                    )
                    reason = "mission_straight"
                raw_requested_v = requested_v
                if self.config.debug_allow_sub_min_crawl:
                    v_cmd = 0.0 if abs(requested_v) < 1e-6 else float(requested_v)
                else:
                    v_cmd = apply_competition_speed_rule(
                        requested_v,
                        allow_stop=False,
                        min_speed_mps=self.config.competition_min_speed_mps,
                    )
                self.sub_min_speed_command_blocked = (
                    abs(raw_requested_v) >= 1e-6
                    and abs(raw_requested_v) < self.config.competition_min_speed_mps
                    and abs(v_cmd) >= self.config.competition_min_speed_mps
                )
                omega = straight_omega_with_slew(
                    heading_error_rad=heading_error,
                    yaw_rate_radps=self.yaw_rate_radps,
                    previous_omega_radps=self.previous_straight_omega_radps,
                    dt_s=dt_s,
                    config=self.config,
                )
                self.previous_straight_omega_radps = omega
                cmd = build_velocity_command(v_cmd, omega, reason)
                stuck_cmd = self._stuck_stop_or_abort(now_s=now_s, command_kind="straight", cmd=cmd)
                if stuck_cmd is not None:
                    return stuck_cmd, heading, heading_error, self.stuck_reason
                return cmd, heading, heading_error, "ok"

            if segment.segment_type == "pivot":
                pivot_safety = evaluate_pivot_clearance(self.last_scan_ranges, self.config)
                if not pivot_safety.safe:
                    self.mission_safety_level = "degraded"
                    self.mission_safety_reason = pivot_safety.reason
                    return build_stop_command(pivot_safety.reason), heading, heading_error, pivot_safety.reason
                pivot_config = self.config
                if segment.max_omega_radps is not None:
                    pivot_config = replace(
                        self.config,
                        pivot_max_omega_radps=min(
                            self.config.pivot_max_omega_radps,
                            float(segment.max_omega_radps),
                        ),
                    )
                if segment.timeout_s is not None:
                    pivot_config = replace(pivot_config, pivot_timeout_s=float(segment.timeout_s))
                if self.pivot_breakaway_scale != 1.0:
                    pivot_config = replace(
                        pivot_config,
                        pivot_breakaway_omega_radps=(
                            pivot_config.pivot_breakaway_omega_radps * self.pivot_breakaway_scale
                        ),
                    )
                step = step_profiled_pivot(
                    self.pivot,
                    now_s=now_s,
                    heading_rad=heading,
                    yaw_rate_radps=self.yaw_rate_radps,
                    encoder_heading_rad=self.estimator.encoder_heading_rad,
                    config=pivot_config,
                )
                heading_error = step.heading_error_rad
                self.target_heading_rad = self.pivot.target_heading_rad
                self.encoder_gyro_disagreement_rad = pivot_encoder_gyro_disagreement(
                    pivot_start_gyro_heading_rad=self.pivot.start_heading_rad,
                    pivot_start_encoder_heading_rad=self.pivot.start_encoder_heading_rad,
                    current_gyro_heading_rad=self.estimator.gyro_heading_rad,
                    current_encoder_heading_rad=self.estimator.encoder_heading_rad,
                )
                self.slip_detected = self.encoder_gyro_disagreement_rad > self.config.slip_disagreement_rad
                if step.complete:
                    return self._finish_mission_segment("segment_complete"), heading, heading_error, "ok"
                if step.command_type == "velocity":
                    cmd = build_velocity_command(0.0, step.omega_radps, step.reason)
                    stuck_cmd = self._stuck_stop_or_abort(now_s=now_s, command_kind="pivot", cmd=cmd)
                    if stuck_cmd is not None:
                        return stuck_cmd, heading, heading_error, self.stuck_reason
                    return cmd, heading, heading_error, "ok"
                return build_stop_command(step.reason), heading, heading_error, "ok"

            wait_s = max(0.0, float(segment.wait_s))
            if segment_elapsed_s >= wait_s:
                return self._finish_mission_segment("segment_complete"), heading, heading_error, "ok"
            return build_stop_command("mission_wait"), heading, heading_error, "ok"

        def tick(self) -> None:
            now_s = time.monotonic()
            heading = self.estimator.heading_rad
            heading_error = 0.0
            safety_state = "idle"
            cmd = build_stop_command("idle")

            if self.mode == "idle":
                self._reset_test_state()
                self._reset_mission_state()
            elif self.mode == "mission_sequence":
                cmd, heading, heading_error, safety_state = self._tick_mission(now_s)
            else:
                safety_state = self._evaluate_active_safety(now_s)
                if safety_state != "ok":
                    self._reset_test_state()
                    self._reset_bias_and_heading()
                    self._log_safety_stop(safety_state, now_s)
                    cmd = build_stop_command(safety_state)
                else:
                    calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
                    if not calibrated:
                        safety_state = calibration_reason
                        self._reset_test_state()
                        cmd = build_stop_command(calibration_reason)
                    else:
                        heading = self.estimator.heading_rad
                        if self.mode == "straight_test":
                            self._start_straight_if_needed(now_s)
                            mode_start_s = (
                                float(self.mode_start_s) if self.mode_start_s is not None else float(now_s)
                            )
                            elapsed_s = now_s - mode_start_s
                            max_duration_s = bounded_duration(self.config.max_test_duration_s, self.config)
                            duration_s = bounded_duration(self.config.straight_duration_s, self.config)
                            if elapsed_s >= max_duration_s:
                                cmd = build_stop_command("test_duration_elapsed")
                            elif elapsed_s >= duration_s:
                                cmd = build_stop_command("straight_test_complete")
                            else:
                                omega, heading_error = compute_straight_omega(
                                    target_heading_rad=float(self.target_heading_rad),
                                    heading_rad=heading,
                                    yaw_rate_radps=self.yaw_rate_radps,
                                    config=self.config,
                                )
                                cmd = build_velocity_command(
                                    self.config.straight_speed_mps,
                                    omega,
                                    "straight_heading_hold",
                                )
                        elif self.mode == "pivot_test":
                            self._start_pivot_if_needed(now_s)
                            step = step_profiled_pivot(
                                self.pivot,
                                now_s=now_s,
                                heading_rad=heading,
                                yaw_rate_radps=self.yaw_rate_radps,
                                encoder_heading_rad=self.estimator.encoder_heading_rad,
                                config=self.config,
                            )
                            heading_error = step.heading_error_rad
                            self.target_heading_rad = self.pivot.target_heading_rad
                            self.encoder_gyro_disagreement_rad = pivot_encoder_gyro_disagreement(
                                pivot_start_gyro_heading_rad=self.pivot.start_heading_rad,
                                pivot_start_encoder_heading_rad=self.pivot.start_encoder_heading_rad,
                                current_gyro_heading_rad=self.estimator.gyro_heading_rad,
                                current_encoder_heading_rad=self.estimator.encoder_heading_rad,
                            )
                            self.slip_detected = (
                                self.encoder_gyro_disagreement_rad > self.config.slip_disagreement_rad
                            )
                            if step.command_type == "velocity":
                                cmd = build_velocity_command(0.0, step.omega_radps, step.reason)
                            else:
                                cmd = build_stop_command(step.reason)

            self._publish_command(cmd)
            self._publish_status_if_needed(now_s, heading, heading_error, safety_state)
            self._write_telemetry_if_needed(now_s, heading, heading_error, safety_state)
            if args.once:
                raise KeyboardInterrupt

        def _publish_command(self, cmd: ControlCommand) -> None:
            payload = cmd.to_json()
            self.last_command_payload = json.loads(payload)
            self.cmd_pub.publish(String(data=payload))

        def _log_safety_stop(self, reason: str, now_s: float) -> None:
            if reason == self.last_safety_reason and now_s - self.last_safety_log_s < 1.0:
                return
            self.get_logger().warn(f"Chassis test holding STOP: {reason}")
            self.last_safety_reason = reason
            self.last_safety_log_s = now_s

        def _publish_status_if_needed(
            self,
            now_s: float,
            heading: float,
            heading_error: float,
            safety_state: str,
        ) -> None:
            period_s = max(0.05, float(args.nav_status_period_s))
            if now_s - self.last_status_publish_s < period_s:
                return
            status = self._build_status_record(now_s, heading, heading_error, safety_state)
            self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status_publish_s = now_s

        def _write_telemetry_if_needed(
            self,
            now_s: float,
            heading: float,
            heading_error: float,
            safety_state: str,
        ) -> None:
            if self.mission_plan is None:
                return
            active = self.mission_state in {"straight_active", "pivot_active", "wait_active"}
            active_period_s = 1.0 / max(1.0, float(self.config.mission_telemetry_active_hz))
            idle_period_s = max(0.05, float(args.nav_status_period_s))
            period_s = active_period_s if active else idle_period_s
            force_key = telemetry_force_flush_key(
                mission_state=self.mission_state,
                safety_level=self.mission_safety_level,
                safety_reason=self.mission_safety_reason,
                segment_index=self.segment_index,
            )
            force_flush = force_key is not None and force_key != self.last_telemetry_force_flush_key
            if not force_flush and now_s - self.last_telemetry_write_s < period_s:
                return
            self.telemetry.write(
                self._build_status_record(now_s, heading, heading_error, safety_state),
                now_s=now_s,
                force_flush=force_flush,
            )
            if force_flush:
                self.last_telemetry_force_flush_key = force_key
            self.last_telemetry_write_s = now_s

        def _build_status_record(
            self,
            now_s: float,
            heading: float,
            heading_error: float,
            safety_state: str,
        ) -> Dict[str, Any]:
            pivot_state = self.pivot.state
            if self.mode == "pivot_test":
                if safety_state in {
                    "gyro_bias_calibration",
                    "gyro_bias_unstable",
                    "gyro_bias_encoder_motion",
                }:
                    pivot_state = "gyro_bias_calibration"
                elif safety_state not in {"ok", "idle"}:
                    pivot_state = "precheck" if self.pivot.state == "idle" else "abort"
            last_v = float(self.last_command_payload.get("v_mps", 0.0))
            last_omega = float(self.last_command_payload.get("omega_radps", 0.0))
            straight_limit = max(0.0, min(float(self.config.max_omega_radps), float(self.config.mission_straight_max_omega_radps)))
            pivot_limit = max(0.0, min(float(self.config.max_omega_radps), float(self.config.pivot_max_omega_radps)))
            omega_limit = pivot_limit if pivot_state not in {"idle", "complete"} else straight_limit
            pivot_state_elapsed_s = (
                None if self.pivot.state_start_s is None else round(max(0.0, now_s - self.pivot.state_start_s), 3)
            )
            pivot_clearance_known = self.last_scan_ranges is not None and self.pivot_clearance_m is not None
            return {
                "nav_runtime": "chassis_controller",
                "controller_mode": self.mode,
                "active": self.last_command_payload.get("command_type") == "velocity",
                "mission_active": self.mode == "mission_sequence" and self.mission_state not in {"idle", "mission_complete"},
                "mission_state": self.mission_state,
                "segment_index": self.segment_index,
                "segment_type": None if self._current_segment() is None else self._current_segment().segment_type,
                "segment_distance_m": self.segment_distance_m,
                "target_distance_m": self.target_distance_m,
                "competition_min_speed_mps": self.config.competition_min_speed_mps,
                "motion_rule_ok": motion_rule_ok(
                    last_v,
                    min_speed_mps=self.config.competition_min_speed_mps,
                ),
                "sub_min_speed_command_blocked": self.sub_min_speed_command_blocked,
                "below_reliable_motion_speed": (
                    abs(last_v) >= 1e-6
                    and abs(last_v) < self.config.mission_reliable_speed_mps
                ),
                "pivot_state": pivot_state,
                "pivot_state_elapsed_s": pivot_state_elapsed_s,
                "target_heading_rad": self.target_heading_rad,
                "estimated_heading_rad": heading,
                "gyro_heading_rad": self.estimator.gyro_heading_rad,
                "encoder_heading_rad": self.estimator.encoder_heading_rad,
                "encoder_gyro_disagreement_rad": self.encoder_gyro_disagreement_rad,
                "slip_detected": self.slip_detected,
                "heading_error_rad": heading_error,
                "raw_yaw_rate_radps": self.raw_yaw_rate_radps,
                "yaw_rate_radps": self.yaw_rate_radps,
                "gyro_bias_radps": self.gyro_bias.bias_radps,
                "gyro_bias_std_radps": self.gyro_bias.std_radps,
                "gyro_bias_ready": self.gyro_bias.ready,
                "gyro_bias_sample_count": len(self.gyro_bias.samples),
                "gyro_bias_status": "ready" if self.gyro_bias.ready else safety_state,
                "pivot_retry_count": self.pivot.retry_count,
                "pivot_direction": self.pivot.direction,
                "pivot_retry_reason": self.pivot.retry_reason,
                "pivot_overshoot_rad": self.pivot.overshoot_rad,
                "pivot_final_error_rad": self.pivot.final_error_rad,
                "v_mps": last_v,
                "omega_radps": last_omega,
                "omega_saturated": abs(last_omega) >= max(0.0, omega_limit - 1e-6) and abs(last_omega) > 1e-6,
                "safety_state": safety_state,
                "safety_level": self.mission_safety_level if self.mode == "mission_sequence" else (
                    "ok" if safety_state in {"idle", "ok"} else "critical"
                ),
                "safety_reason": self.mission_safety_reason if self.mode == "mission_sequence" else safety_state,
                "straight_heading_error_rms": heading_error_rms(self.straight_heading_error_samples),
                "near_obstacle": self.near_obstacle,
                "front_clearance_m": self.front_clearance_m,
                "pivot_clearance_m": self.pivot_clearance_m,
                "pivot_clearance_known": pivot_clearance_known,
                "sensor_age_s": None if self.last_sensor_s is None else round(now_s - self.last_sensor_s, 3),
                "imu_age_s": None if self.last_imu_s is None else round(now_s - self.last_imu_s, 3),
                "imu_rate_hz": round(self._imu_rate_hz(), 3),
                "imu_max_dt_s": round(self.imu_max_dt_s, 4),
                "imu_skipped_integrations": self.imu_skipped_integrations,
                "imu_qos": self.imu_qos_name,
                "motor_status_age_s": (
                    None if self.last_motor_status_s is None else round(now_s - self.last_motor_status_s, 3)
                ),
                "stuck_detected": self.stuck_detected,
                "stuck_reason": self.stuck_reason,
                "stuck_recovery_count": self.stuck_monitor.recovery_count,
                "last_command": self.last_command_payload,
                "telemetry_path": None if self.telemetry.path is None else str(self.telemetry.path),
                "telemetry_enabled": bool(self.telemetry.enabled) and not bool(self.telemetry.failed),
                "telemetry_error": self.telemetry.error,
                "timestamp_s": round(time.time(), 3),
            }

    rclpy.init()
    node: Optional[ChassisControllerNode] = None
    try:
        node = ChassisControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.telemetry.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    args = parse_args()
    if args.mode == "real":
        run_real(args)
    else:
        run_sim()


if __name__ == "__main__":
    main()
