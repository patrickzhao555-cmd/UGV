#!/usr/bin/env python3
"""Safe Jetson chassis controller entrypoint.

Competition real mode uses IMU + encoder closed-loop trajectory tracking.
Calibration modes still publish the same velocity-only JSON:

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
    CurveControllerState,
    GyroBiasCalibrationState,
    PivotControllerState,
    bounded_duration,
    clamp,
    compute_straight_omega,
    curve_speed_radius_omega,
    evaluate_pivot_clearance,
    evaluate_safety,
    min_finite_range,
    motor_status_ready,
    pivot_encoder_gyro_disagreement,
    reset_curve,
    reset_gyro_bias_calibration,
    reset_pivot,
    start_curve,
    start_pivot,
    step_curve,
    step_profiled_pivot,
    update_encoder_heading,
    update_gyro_bias_calibration,
    update_gyro_heading,
    wrap_pi,
)
from ugv_nav_core.mission_controller import (
    MOVING_TARGET_SPEED_MPS,
    MissionPlan,
    MissionSegment,
    MissionTelemetryRecorder,
    StuckMonitorState,
    apply_competition_speed_rule,
    average_abs_measured_speed_mps,
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
from ugv_nav_core.trajectory_tracker import (
    TrackerConfig,
    TrackerState,
    reset_tracker,
    start_tracker_goal,
    step_tracker,
    tracker_status,
    update_tracker_odometry,
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


def encoder_ticks_from_motor_status(payload: Any) -> Optional[tuple[int, int]]:
    if not isinstance(payload, dict):
        return None
    ticks = payload.get("encoder_ticks")
    if isinstance(ticks, (list, tuple)) and len(ticks) >= 2:
        try:
            return int(ticks[0]), int(ticks[1])
        except (TypeError, ValueError):
            return None
    if "left_ticks" in payload and "right_ticks" in payload:
        try:
            return int(payload["left_ticks"]), int(payload["right_ticks"])
        except (TypeError, ValueError):
            return None
    return None


def challenge2_target_bearing_rad(target_x_m: float, target_y_m: float) -> float:
    x = float(target_x_m)
    y = float(target_y_m)
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("challenge2 target coordinates must be finite")
    return math.atan2(y, x)


def challenge2_target_distance_m(
    target_x_m: float,
    target_y_m: float,
    *,
    pose_x_m: float = 0.0,
    pose_y_m: float = 0.0,
) -> float:
    return math.hypot(float(target_x_m) - float(pose_x_m), float(target_y_m) - float(pose_y_m))


def challenge2_cross_track_error_m(
    target_x_m: float,
    target_y_m: float,
    *,
    pose_x_m: float,
    pose_y_m: float,
) -> float:
    target_len = math.hypot(float(target_x_m), float(target_y_m))
    if target_len <= 1e-9:
        return 0.0
    ux = float(target_x_m) / target_len
    uy = float(target_y_m) / target_len
    return ux * float(pose_y_m) - uy * float(pose_x_m)


def challenge2_landing_requirement_met(
    landed_s: Optional[float],
    now_s: float,
    required_s: float,
) -> bool:
    if landed_s is None:
        return False
    return float(now_s) - float(landed_s) >= max(0.0, float(required_s))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe UGV chassis controller. Default mode publishes STOP.",
        allow_abbrev=False,
    )
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument(
        "--controller-mode",
        choices=[
            "idle",
            "straight_test",
            "pivot_test",
            "curve_test",
            "mission_sequence",
            "competition_tracker",
            "challenge1_landing_platform",
            "challenge2_align_straight",
        ],
        default="idle",
    )
    parser.add_argument("--allow-legacy-controller", type=parse_bool, default=False)
    parser.add_argument("--command-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--nav-frame-topic", default="/sensors/nav_frame")
    parser.add_argument("--imu-topic", default="/zed/imu")
    parser.add_argument("--imu-qos", default="sensor_data")
    parser.add_argument("--imu-yaw-axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--imu-yaw-sign", type=float, default=-1.0)
    parser.add_argument("--imu-min-rate-hz", type=float, default=20.0)
    parser.add_argument("--motor-status-topic", default="/motor_controller/status")
    parser.add_argument("--encoder-stamped-topic", default="/encoder_ticks_stamped")
    parser.add_argument("--zed-status-topic", default="/zed/status")
    parser.add_argument("--allow-encoder-heading-fallback", type=parse_bool, default=False)
    parser.add_argument("--nav-status-period-s", type=float, default=0.25)
    parser.add_argument("--control-period-s", type=float, default=0.02)
    parser.add_argument("--straight-speed-mps", type=float, default=0.20)
    parser.add_argument("--straight-duration-s", type=float, default=2.0)
    parser.add_argument("--pivot-angle-deg", type=float, default=90.0)
    parser.add_argument("--curve-angle-deg", type=float, default=90.0)
    parser.add_argument("--curve-speed-mps", type=float, default=0.15)
    parser.add_argument("--curve-radius-m", type=float, default=1.0)
    parser.add_argument("--curve-omega-radps", type=float, default=0.0)
    parser.add_argument("--curve-direction", choices=["left", "right"], default="left")
    parser.add_argument("--arc-min-turn-radius-m", type=float, default=0.75)
    parser.add_argument("--arc-max-omega-radps", type=float, default=0.45)
    parser.add_argument("--curve-omega-slew-radps2", type=float, default=0.80)
    parser.add_argument("--curve-timeout-s", type=float, default=0.0)
    parser.add_argument("--curve-no-progress-timeout-s", type=float, default=1.5)
    parser.add_argument("--curve-min-progress-rad", type=float, default=0.025)
    parser.add_argument("--curve-approach-error-rad", type=float, default=0.25)
    parser.add_argument("--curve-kp-approach", type=float, default=0.90)
    parser.add_argument("--curve-kd-yaw-rate", type=float, default=0.08)
    parser.add_argument("--curve-min-omega-radps", type=float, default=0.14)
    parser.add_argument("--curve-min-omega-disable-error-rad", type=float, default=0.08)
    parser.add_argument("--allow-side-reverse", type=parse_bool, default=False)
    parser.add_argument("--target-topic", default="/ugv/uav_target")
    parser.add_argument("--uav-launched-topic", default="/ugv/uav_launched")
    parser.add_argument("--uav-landed-topic", default="/ugv/uav_landed")
    parser.add_argument("--manual-target-x-m", type=float, default=0.0)
    parser.add_argument("--manual-target-y-m", type=float, default=0.0)
    parser.add_argument("--tracking-enabled", type=parse_bool, default=True)
    parser.add_argument("--target-stop-radius-m", type=float, default=0.75)
    parser.add_argument("--tracking-lookahead-min-m", type=float, default=0.45)
    parser.add_argument("--tracking-lookahead-max-m", type=float, default=1.20)
    parser.add_argument("--tracking-lookahead-speed-gain", type=float, default=1.8)
    parser.add_argument("--tracking-nominal-speed-mps", type=float, default=0.25)
    parser.add_argument("--tracking-max-speed-mps", type=float, default=0.42)
    parser.add_argument("--tracking-max-omega-radps", type=float, default=0.85)
    parser.add_argument("--tracking-heading-kp", type=float, default=0.85)
    parser.add_argument("--tracking-cross-track-kp", type=float, default=0.75)
    parser.add_argument("--tracking-slowdown-distance-m", type=float, default=1.20)
    parser.add_argument("--obstacle-warn-m", type=float, default=2.0)
    parser.add_argument("--obstacle-stop-m", type=float, default=1.0)
    parser.add_argument("--bypass-offset-m", type=float, default=1.1)
    parser.add_argument("--bypass-forward-m", type=float, default=2.0)
    parser.add_argument("--bypass-rejoin-ahead-m", type=float, default=2.0)
    parser.add_argument("--challenge1-auto-start", type=parse_bool, default=True)
    parser.add_argument("--challenge1-speed-mps", type=float, default=0.24)
    parser.add_argument("--challenge1-post-landing-s", type=float, default=40.0)
    parser.add_argument("--challenge1-timeout-s", type=float, default=420.0)
    parser.add_argument("--challenge1-max-distance-m", type=float, default=0.0)
    parser.add_argument("--challenge1-stop-on-obstacle", type=parse_bool, default=True)
    parser.add_argument("--challenge2-speed-mps", type=float, default=0.24)
    parser.add_argument("--challenge2-approach-speed-mps", type=float, default=0.12)
    parser.add_argument("--challenge2-slowdown-distance-m", type=float, default=1.5)
    parser.add_argument("--challenge2-stop-radius-m", type=float, default=0.75)
    parser.add_argument("--challenge2-post-landing-s", type=float, default=10.0)
    parser.add_argument("--challenge2-pivot-max-omega-radps", type=float, default=0.85)
    parser.add_argument("--challenge2-pivot-timeout-s", type=float, default=25.0)
    parser.add_argument("--challenge2-pivot-settle-error-rad", type=float, default=0.035)
    parser.add_argument("--challenge2-pivot-settle-time-s", type=float, default=0.35)
    parser.add_argument("--challenge2-heading-kp", type=float, default=0.85)
    parser.add_argument("--challenge2-cross-track-kp", type=float, default=0.75)
    parser.add_argument("--challenge2-max-omega-radps", type=float, default=0.35)
    parser.add_argument("--max-omega-radps", type=float, default=0.45)
    parser.add_argument("--heading-kp", type=float, default=0.6)
    parser.add_argument("--heading-kd", type=float, default=0.08)
    parser.add_argument("--pivot-kp", type=float, default=1.0)
    parser.add_argument("--heading-deadband-rad", type=float, default=0.025)
    parser.add_argument("--stop-clearance-m", type=float, default=0.45)
    parser.add_argument("--sensor-timeout-s", type=float, default=0.30)
    parser.add_argument("--imu-timeout-s", type=float, default=0.30)
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
    parser.add_argument("--competition-min-speed-mps", type=float, default=0.0894)
    parser.add_argument("--competition-moving-target-speed-mps", type=float, default=MOVING_TARGET_SPEED_MPS)
    parser.add_argument("--competition-continuous-motion-enabled", type=parse_bool, default=True)
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
    parser.add_argument("--debug-ignore-nav-frame", type=parse_bool, default=False)
    parser.add_argument("--debug-ignore-obstacles", type=parse_bool, default=False)
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
    parser.add_argument("--track-width-m", type=float, default=0.416)
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
    if abs(float(args.pivot_angle_deg)) > 180.0:
        parser.error("--pivot-angle-deg supports -180..180 degrees; split larger rotations into multiple pivots")
    if abs(float(args.curve_angle_deg)) > 180.0:
        parser.error("--curve-angle-deg supports -180..180 degrees; split larger curves into multiple segments")
    return args


def config_from_args(args: argparse.Namespace) -> ChassisControllerConfig:
    return ChassisControllerConfig(
        straight_speed_mps=float(args.straight_speed_mps),
        straight_duration_s=float(args.straight_duration_s),
        pivot_angle_deg=float(args.pivot_angle_deg),
        curve_angle_deg=float(args.curve_angle_deg),
        curve_speed_mps=float(args.curve_speed_mps),
        curve_radius_m=float(args.curve_radius_m),
        curve_omega_radps=float(args.curve_omega_radps),
        curve_direction=str(args.curve_direction),
        arc_min_turn_radius_m=float(args.arc_min_turn_radius_m),
        arc_max_omega_radps=float(args.arc_max_omega_radps),
        curve_omega_slew_radps2=float(args.curve_omega_slew_radps2),
        curve_timeout_s=float(args.curve_timeout_s),
        curve_no_progress_timeout_s=float(args.curve_no_progress_timeout_s),
        curve_min_progress_rad=float(args.curve_min_progress_rad),
        curve_approach_error_rad=float(args.curve_approach_error_rad),
        curve_kp_approach=float(args.curve_kp_approach),
        curve_kd_yaw_rate=float(args.curve_kd_yaw_rate),
        curve_min_omega_radps=float(args.curve_min_omega_radps),
        curve_min_omega_disable_error_rad=float(args.curve_min_omega_disable_error_rad),
        allow_side_reverse=bool(args.allow_side_reverse),
        max_omega_radps=float(args.max_omega_radps),
        heading_kp=float(args.heading_kp),
        heading_kd=float(args.heading_kd),
        pivot_kp=float(args.pivot_kp),
        heading_deadband_rad=float(args.heading_deadband_rad),
        stop_clearance_m=float(args.stop_clearance_m),
        sensor_timeout_s=float(args.sensor_timeout_s),
        imu_timeout_s=float(args.imu_timeout_s),
        imu_min_rate_hz=float(args.imu_min_rate_hz),
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
        competition_moving_target_speed_mps=float(args.competition_moving_target_speed_mps),
        competition_continuous_motion_enabled=bool(args.competition_continuous_motion_enabled),
        mission_default_speed_mps=float(args.mission_default_speed_mps),
        mission_reliable_speed_mps=float(args.mission_reliable_speed_mps),
        mission_slow_speed_mps=float(args.mission_slow_speed_mps),
        mission_emergency_stop_clearance_m=float(args.mission_emergency_stop_clearance_m),
        mission_critical_sensor_timeout_s=float(args.mission_critical_sensor_timeout_s),
        mission_straight_max_omega_radps=float(args.mission_straight_max_omega_radps),
        mission_straight_omega_slew_radps2=float(args.mission_straight_omega_slew_radps2),
        debug_allow_sub_min_crawl=bool(args.debug_allow_sub_min_crawl),
        debug_allow_unknown_pivot_clearance=bool(args.debug_allow_unknown_pivot_clearance),
        debug_ignore_nav_frame=bool(args.debug_ignore_nav_frame),
        debug_ignore_obstacles=bool(args.debug_ignore_obstacles),
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


def validate_controller_mode(args: argparse.Namespace) -> None:
    mode = str(args.controller_mode)
    competition_modes = {
        "idle",
        "competition_tracker",
        "challenge1_landing_platform",
        "challenge2_align_straight",
    }
    if not bool(args.allow_legacy_controller) and mode not in competition_modes:
        raise SystemExit(
            f"controller-mode={mode} is a legacy/calibration controller. "
            "Use controller-mode=competition_tracker, challenge1_landing_platform, "
            "or challenge2_align_straight "
            "for closed-loop competition tracking, "
            "or pass --allow-legacy-controller true for intentional calibration/debug runs."
        )


def run_real(args: argparse.Namespace) -> None:
    try:
        import rclpy
        from geometry_msgs.msg import PointStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool
        from std_msgs.msg import String
        from ugv_sensor_sync.msg import EncoderTicksStamped, NavSensorFrame
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python packages are required for real mode: {exc}") from exc

    validate_controller_mode(args)
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
            self.curve = CurveControllerState()
            self.tracker = TrackerState()
            self.tracker_config = TrackerConfig(
                target_stop_radius_m=max(0.05, float(args.target_stop_radius_m)),
                lookahead_min_m=max(0.05, float(args.tracking_lookahead_min_m)),
                lookahead_max_m=max(0.05, float(args.tracking_lookahead_max_m)),
                lookahead_speed_gain=max(0.0, float(args.tracking_lookahead_speed_gain)),
                nominal_speed_mps=max(0.0, float(args.tracking_nominal_speed_mps)),
                max_speed_mps=max(0.0, float(args.tracking_max_speed_mps)),
                max_omega_radps=max(0.0, float(args.tracking_max_omega_radps)),
                heading_kp=float(args.tracking_heading_kp),
                cross_track_kp=float(args.tracking_cross_track_kp),
                slowdown_distance_m=max(0.05, float(args.tracking_slowdown_distance_m)),
                obstacle_warn_m=max(0.0, float(args.obstacle_warn_m)),
                obstacle_stop_m=max(0.0, float(args.obstacle_stop_m)),
                bypass_offset_m=max(0.0, float(args.bypass_offset_m)),
                bypass_forward_m=max(0.1, float(args.bypass_forward_m)),
                bypass_rejoin_ahead_m=max(0.1, float(args.bypass_rejoin_ahead_m)),
                track_width_m=float(args.track_width_m),
                wheel_radius_m=float(args.wheel_radius_m),
                ticks_per_rev=float(args.ticks_per_rev),
            )
            self.tracker_pending_target: Optional[tuple[float, float]] = None
            if abs(float(args.manual_target_x_m)) > 1e-9 or abs(float(args.manual_target_y_m)) > 1e-9:
                self.tracker_pending_target = (float(args.manual_target_x_m), float(args.manual_target_y_m))
            self.challenge2_tracker = TrackerState()
            self.challenge2_tracker_config = replace(
                self.tracker_config,
                target_stop_radius_m=max(0.05, float(args.challenge2_stop_radius_m)),
                nominal_speed_mps=max(0.0, float(args.challenge2_speed_mps)),
                max_speed_mps=max(0.0, max(float(args.challenge2_speed_mps), float(args.challenge2_approach_speed_mps))),
                max_omega_radps=max(0.0, float(args.challenge2_max_omega_radps)),
                heading_kp=float(args.challenge2_heading_kp),
                cross_track_kp=float(args.challenge2_cross_track_kp),
                slowdown_distance_m=max(0.05, float(args.challenge2_slowdown_distance_m)),
            )
            self.challenge2_target: Optional[tuple[float, float]] = (
                None if self.tracker_pending_target is None else tuple(self.tracker_pending_target)
            )
            self.challenge2_state = "WAIT_TARGET"
            self.challenge2_start_s: Optional[float] = None
            self.challenge2_landed_s: Optional[float] = None
            self.challenge2_uav_landed = False
            self.challenge2_landing_requirement_met = False
            self.challenge2_target_distance_m: Optional[float] = None
            self.challenge2_target_bearing_rad: Optional[float] = (
                None
                if self.challenge2_target is None
                else challenge2_target_bearing_rad(self.challenge2_target[0], self.challenge2_target[1])
            )
            self.challenge2_align_error_rad = 0.0
            self.challenge2_cross_track_error_m = 0.0
            self.challenge2_result_reason = "waiting_for_target"
            self.challenge2_last_omega_radps = 0.0
            self.challenge2_last_update_s: Optional[float] = None
            self.challenge1_state = "idle"
            self.challenge1_uav_launched = bool(args.challenge1_auto_start)
            self.challenge1_uav_landed = False
            self.challenge1_start_s: Optional[float] = None
            self.challenge1_landed_s: Optional[float] = None
            self.challenge1_complete_s: Optional[float] = None
            self.challenge1_start_left_ticks: Optional[int] = None
            self.challenge1_start_right_ticks: Optional[int] = None
            self.challenge1_distance_m = 0.0
            self.challenge1_last_omega_radps = 0.0
            self.last_sensor_s: Optional[float] = None
            self.last_imu_s: Optional[float] = None
            self.last_motor_status_s: Optional[float] = None
            self.last_encoder_feedback_s: Optional[float] = None
            self.last_zed_status_s: Optional[float] = None
            self.motor_status: Optional[Dict[str, Any]] = None
            self.zed_status: Optional[Dict[str, Any]] = None
            self.near_obstacle = False
            self.front_clearance_m: Optional[float] = None
            self.pivot_clearance_m: Optional[float] = None
            self.last_scan_ranges: Optional[list[float]] = None
            self.last_scan_angle_min = 0.0
            self.last_scan_angle_increment = 0.0
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
            self.create_subscription(PointStamped, args.target_topic, self.target_callback, 10)
            self.create_subscription(Bool, args.uav_launched_topic, self.uav_launched_callback, 10)
            self.create_subscription(Bool, args.uav_landed_topic, self.uav_landed_callback, 10)
            self.create_subscription(String, args.motor_status_topic, self.motor_status_callback, 10)
            self.create_subscription(
                EncoderTicksStamped,
                args.encoder_stamped_topic,
                self.encoder_stamped_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(String, args.zed_status_topic, self.zed_status_callback, 10)
            self.timer = self.create_timer(max(0.01, float(args.control_period_s)), self.tick)
            self.get_logger().warn(
                f"Chassis controller started in {self.mode}; only velocity/STOP JSON is published."
            )
            self.get_logger().info(
                "Chassis sanity: "
                f"control_hz={1.0 / max(0.01, float(args.control_period_s)):.1f}, "
                f"imu_topic={args.imu_topic}, imu_qos={self.imu_qos_name}, "
                f"imu_timeout={self.config.imu_timeout_s:.3f}s, imu_min_rate={self.config.imu_min_rate_hz:.1f}Hz, "
                f"encoder_topic={args.encoder_stamped_topic}, "
                f"zed_status_topic={args.zed_status_topic}, "
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
            self.last_scan_angle_min = float(getattr(msg.scan, "angle_min", 0.0))
            self.last_scan_angle_increment = float(getattr(msg.scan, "angle_increment", 0.0))
            self.pivot_clearance_m = min_finite_range(self.last_scan_ranges)
            if bool(msg.encoder_available):
                self._update_encoder_feedback(
                    left_ticks=int(msg.left_encoder_ticks),
                    right_ticks=int(msg.right_encoder_ticks),
                    now_s=now_s,
                )

        def _queue_challenge2_target(self, x: float, y: float) -> bool:
            current = self.challenge2_target
            self.challenge2_target = (float(x), float(y))
            self.challenge2_target_bearing_rad = challenge2_target_bearing_rad(x, y)
            if current is not None and math.hypot(float(x) - current[0], float(y) - current[1]) < 0.05:
                return False
            self._reset_challenge2_state(keep_target=True, preserve_landed=True)
            return True

        def target_callback(self, msg: Any) -> None:
            x = float(msg.point.x)
            y = float(msg.point.y)
            if not math.isfinite(x) or not math.isfinite(y):
                self.get_logger().warn("Ignoring non-finite competition tracker target")
                return
            challenge2_new_target = self._queue_challenge2_target(x, y)
            current_target = self.tracker_pending_target
            if current_target is None and self.tracker.target is not None:
                current_target = (self.tracker.target.x, self.tracker.target.y)
            if current_target is not None and math.hypot(x - current_target[0], y - current_target[1]) < 0.05:
                self.tracker_pending_target = (x, y)
                return
            reset_tracker(self.tracker)
            self.tracker_pending_target = (x, y)
            self.get_logger().info(f"Competition tracker target queued: x={x:.3f}m y={y:.3f}m")
            if challenge2_new_target:
                self.get_logger().warn(f"Challenge 2 target queued: x={x:.3f}m y={y:.3f}m")

        def uav_launched_callback(self, msg: Any) -> None:
            if not bool(getattr(msg, "data", False)):
                return
            if not self.challenge1_uav_launched:
                self.get_logger().warn("Challenge 1 UAV launched signal received; UGV may start moving.")
            self.challenge1_uav_launched = True

        def uav_landed_callback(self, msg: Any) -> None:
            if not bool(getattr(msg, "data", False)):
                return
            now_s = time.monotonic()
            if not self.challenge1_uav_landed:
                self.challenge1_landed_s = now_s
                self.get_logger().warn("Challenge 1 UAV landed signal received; starting post-landing travel timer.")
            self.challenge1_uav_landed = True
            if not self.challenge2_uav_landed:
                self.challenge2_landed_s = now_s
                self.get_logger().warn("Challenge 2 UAV landed signal received; starting 10s rule timer.")
            self.challenge2_uav_landed = True

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
                ticks = encoder_ticks_from_motor_status(payload)
                if ticks is not None:
                    self._update_encoder_feedback(left_ticks=ticks[0], right_ticks=ticks[1], now_s=self.last_motor_status_s)

        def encoder_stamped_callback(self, msg: Any) -> None:
            self._update_encoder_feedback(
                left_ticks=int(msg.left_ticks),
                right_ticks=int(msg.right_ticks),
                now_s=time.monotonic(),
            )

        def zed_status_callback(self, msg: Any) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"Ignoring invalid ZED status JSON: {exc}")
                return
            if isinstance(payload, dict):
                self.zed_status = payload
                self.last_zed_status_s = time.monotonic()

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

        def _imu_fresh(self, now_s: float) -> bool:
            return self.last_imu_s is not None and now_s - self.last_imu_s <= self.config.imu_timeout_s

        def _imu_rate_ok(self) -> bool:
            min_rate_hz = max(0.0, float(self.config.imu_min_rate_hz))
            return min_rate_hz <= 0.0 or self._imu_rate_hz() >= min_rate_hz

        def _imu_health(self, now_s: float) -> Dict[str, Any]:
            age_s = None if self.last_imu_s is None else max(0.0, now_s - self.last_imu_s)
            rate_hz = self._imu_rate_hz()
            min_rate_hz = max(0.0, float(self.config.imu_min_rate_hz))
            return {
                "topic": str(args.imu_topic),
                "qos": self.imu_qos_name,
                "age_s": None if age_s is None else round(age_s, 3),
                "timeout_s": round(float(self.config.imu_timeout_s), 3),
                "fresh": age_s is not None and age_s <= float(self.config.imu_timeout_s),
                "rate_hz": round(rate_hz, 3),
                "min_rate_hz": round(min_rate_hz, 3),
                "rate_ok": min_rate_hz <= 0.0 or rate_hz >= min_rate_hz,
                "max_dt_s": round(self.imu_max_dt_s, 4),
                "skipped_integrations": self.imu_skipped_integrations,
                "arrival_samples": len(self.imu_arrival_times_s),
            }

        def _zed_imu_status(self, now_s: float) -> Dict[str, Any]:
            if self.zed_status is None:
                return {
                    "topic": str(args.zed_status_topic),
                    "age_s": None,
                    "available": False,
                    "imu_count": None,
                    "imu_rate_hz": None,
                    "imu_age_s": None,
                    "imu_publish_failures": None,
                    "last_imu_error": None,
                    "imu_busy_skips": None,
                    "last_imu_publish_s": None,
                }
            return {
                "topic": str(args.zed_status_topic),
                "age_s": None if self.last_zed_status_s is None else round(max(0.0, now_s - self.last_zed_status_s), 3),
                "available": True,
                "imu_count": self.zed_status.get("imu_count"),
                "imu_rate_hz": self.zed_status.get("imu_rate_hz"),
                "imu_age_s": self.zed_status.get("imu_age_s"),
                "imu_publish_failures": self.zed_status.get("imu_publish_failures"),
                "last_imu_error": self.zed_status.get("last_imu_error"),
                "imu_busy_skips": self.zed_status.get("imu_busy_skips"),
                "last_imu_publish_s": self.zed_status.get("last_imu_publish_s"),
            }

        def _encoder_health(self, now_s: float) -> Dict[str, Any]:
            age_s = None if self.last_encoder_feedback_s is None else max(0.0, now_s - self.last_encoder_feedback_s)
            return {
                "topic": str(args.encoder_stamped_topic),
                "available": self._encoder_feedback_available(),
                "fresh": self._encoder_feedback_fresh(now_s),
                "age_s": None if age_s is None else round(age_s, 3),
                "left_ticks": self.current_left_ticks,
                "right_ticks": self.current_right_ticks,
                "fallback_allowed": self._encoder_fallback_allowed(now_s),
            }

        def _encoder_feedback_available(self) -> bool:
            return self.encoder_available and self.current_left_ticks is not None and self.current_right_ticks is not None

        def _encoder_feedback_fresh(self, now_s: float) -> bool:
            return (
                self._encoder_feedback_available()
                and self.last_encoder_feedback_s is not None
                and now_s - self.last_encoder_feedback_s <= max(0.05, self.config.motor_status_timeout_s)
            )

        def _imu_control_ready(self, now_s: float) -> bool:
            return self._imu_fresh(now_s) and self._imu_rate_ok()

        def _encoder_fallback_allowed(self, now_s: float) -> bool:
            return bool(args.allow_encoder_heading_fallback) and self._encoder_feedback_fresh(now_s)

        def _heading_source(self, now_s: float) -> str:
            if self._imu_control_ready(now_s) and self.estimator.gyro_available:
                return "imu"
            if self._encoder_fallback_allowed(now_s):
                return "encoder_fallback"
            if self._encoder_feedback_fresh(now_s):
                return "encoder_observer"
            return "none"

        def _heading_rad(self, now_s: float) -> float:
            if self._imu_control_ready(now_s) and self.estimator.gyro_available:
                return self.estimator.gyro_heading_rad
            if self._encoder_fallback_allowed(now_s):
                return self.estimator.encoder_heading_rad
            return self.estimator.heading_rad

        def _update_encoder_feedback(self, *, left_ticks: int, right_ticks: int, now_s: Optional[float] = None) -> None:
            self.current_left_ticks = int(left_ticks)
            self.current_right_ticks = int(right_ticks)
            self.encoder_available = True
            self.last_encoder_feedback_s = time.monotonic() if now_s is None else float(now_s)
            self.estimator = update_encoder_heading(
                self.estimator,
                left_ticks=self.current_left_ticks,
                right_ticks=self.current_right_ticks,
                encoder_available=True,
                config=self.config,
            )

        def _start_straight_if_needed(self, now_s: float) -> None:
            if self.mode_start_s is not None:
                return
            self.mode_start_s = now_s
            self.target_heading_rad = self._heading_rad(now_s)
            self.previous_straight_omega_radps = 0.0
            self.last_mission_update_s = now_s

        def _start_pivot_if_needed(self, now_s: float) -> None:
            if self.pivot.state != "idle":
                return
            self.mode_start_s = now_s
            start_pivot(
                self.pivot,
                now_s=now_s,
                current_heading_rad=self._heading_rad(now_s),
                encoder_heading_rad=self.estimator.encoder_heading_rad,
                target_angle_rad=math.radians(self.config.pivot_angle_deg),
            )
            self.target_heading_rad = self.pivot.target_heading_rad

        def _start_curve_if_needed(self, now_s: float) -> None:
            if self.curve.state != "idle":
                return
            self.mode_start_s = now_s
            start_curve(
                self.curve,
                now_s=now_s,
                current_heading_rad=self._heading_rad(now_s),
                encoder_heading_rad=self.estimator.encoder_heading_rad,
                config=self.config,
            )
            self.target_heading_rad = self.curve.target_heading_rad

        def _reset_test_state(self) -> None:
            self.mode_start_s = None
            self.target_heading_rad = None
            reset_pivot(self.pivot)
            reset_curve(self.curve)
            self.encoder_gyro_disagreement_rad = 0.0
            self.slip_detected = False
            self.previous_straight_omega_radps = 0.0
            self.last_mission_update_s = None

        def _reset_tracker_state(self) -> None:
            reset_tracker(self.tracker)
            self.tracker_pending_target = None

        def _reset_challenge1_state(self) -> None:
            self.challenge1_state = "idle"
            self.challenge1_uav_launched = bool(args.challenge1_auto_start)
            self.challenge1_uav_landed = False
            self.challenge1_start_s = None
            self.challenge1_landed_s = None
            self.challenge1_complete_s = None
            self.challenge1_start_left_ticks = None
            self.challenge1_start_right_ticks = None
            self.challenge1_distance_m = 0.0
            self.challenge1_last_omega_radps = 0.0

        def _reset_challenge2_state(
            self,
            *,
            keep_target: bool = False,
            preserve_landed: bool = False,
        ) -> None:
            target = self.challenge2_target if keep_target else None
            landed = self.challenge2_uav_landed if preserve_landed else False
            landed_s = self.challenge2_landed_s if preserve_landed else None
            reset_tracker(self.challenge2_tracker)
            if self.mode == "challenge2_align_straight":
                reset_pivot(self.pivot)
            self.challenge2_target = target
            self.challenge2_state = "WAIT_TARGET"
            self.challenge2_start_s = None
            self.challenge2_landed_s = landed_s
            self.challenge2_uav_landed = landed
            self.challenge2_landing_requirement_met = False
            self.challenge2_target_distance_m = None
            self.challenge2_target_bearing_rad = (
                None if target is None else challenge2_target_bearing_rad(target[0], target[1])
            )
            self.challenge2_align_error_rad = 0.0
            self.challenge2_cross_track_error_m = 0.0
            self.challenge2_result_reason = "waiting_for_target"
            self.challenge2_last_omega_radps = 0.0
            self.challenge2_last_update_s = None

        def _tracker_safety_state(self, now_s: float) -> str:
            if not self.config.debug_ignore_nav_frame and (
                self.last_sensor_s is None or now_s - self.last_sensor_s > self.config.sensor_timeout_s
            ):
                return "sensor_stale"
            if not self._imu_fresh(now_s):
                return "imu_stale"
            if not self._imu_rate_ok():
                return "imu_rate_low"
            if last_motor_status_s := self.last_motor_status_s:
                if now_s - last_motor_status_s > self.config.motor_status_timeout_s:
                    return "motor_status_stale"
            else:
                return "motor_status_stale"
            motor = motor_status_ready(self.motor_status)
            if not motor.safe:
                return motor.reason
            if not self._encoder_feedback_available():
                return "encoder_unavailable"
            if not self._encoder_feedback_fresh(now_s):
                return "encoder_stale"
            return "ok"

        def _scan_sector_min(self, *, angle_min_deg: float, angle_max_deg: float) -> Optional[float]:
            if not self.last_scan_ranges:
                return None
            angle_min = math.radians(float(angle_min_deg))
            angle_max = math.radians(float(angle_max_deg))
            if angle_min > angle_max:
                angle_min, angle_max = angle_max, angle_min
            values: list[float] = []
            for index, value in enumerate(self.last_scan_ranges):
                distance = float(value)
                if not math.isfinite(distance) or distance <= 0.0:
                    continue
                angle = self.last_scan_angle_min + float(index) * self.last_scan_angle_increment
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if angle_min <= angle <= angle_max:
                    values.append(distance)
            return min(values) if values else None

        def _ensure_tracker_started(self, now_s: float) -> Optional[str]:
            if not bool(args.tracking_enabled):
                return "tracking_disabled"
            if self.tracker_pending_target is None:
                return "tracker_wait_target"
            if not self._encoder_feedback_available():
                return "encoder_unavailable"
            if not self._encoder_feedback_fresh(now_s):
                return "encoder_stale"
            if not self._imu_fresh(now_s):
                return "imu_stale"
            if self.tracker.target is None or self.tracker.state in {"WAIT_TARGET", "FAULT"}:
                x, y = self.tracker_pending_target
                start_tracker_goal(
                    self.tracker,
                    target_x_m=x,
                    target_y_m=y,
                    left_ticks=self.current_left_ticks,
                    right_ticks=self.current_right_ticks,
                    heading_rad=self._heading_rad(now_s),
                )
                self.mode_start_s = now_s
                self.get_logger().warn(
                    f"Competition tracker started: target=({x:.3f},{y:.3f})m "
                    f"stop_radius={self.tracker_config.target_stop_radius_m:.2f}m"
                )
            return None

        def _tick_competition_tracker(self, now_s: float) -> tuple[ControlCommand, float, float, str]:
            heading = self._heading_rad(now_s)
            safety_state = self._tracker_safety_state(now_s)
            if safety_state != "ok":
                return build_stop_command(safety_state), heading, 0.0, safety_state
            start_hold = self._ensure_tracker_started(now_s)
            if start_hold is not None:
                return build_stop_command(start_hold), heading, 0.0, start_hold
            assert self.current_left_ticks is not None
            assert self.current_right_ticks is not None
            update_tracker_odometry(
                self.tracker,
                left_ticks=self.current_left_ticks,
                right_ticks=self.current_right_ticks,
                heading_rad=heading,
                config=self.tracker_config,
            )
            left_clearance = self._scan_sector_min(angle_min_deg=35.0, angle_max_deg=110.0)
            right_clearance = self._scan_sector_min(angle_min_deg=-110.0, angle_max_deg=-35.0)
            step = step_tracker(
                self.tracker,
                config=self.tracker_config,
                front_clearance_m=self.front_clearance_m,
                left_clearance_m=left_clearance,
                right_clearance_m=right_clearance,
            )
            if step.state == "FAULT":
                self.tracker.state = "FAULT"
                return build_stop_command(step.reason), heading, self.tracker.heading_error_rad, step.reason
            if step.command_type == "velocity":
                return build_velocity_command(step.v_mps, step.omega_radps, step.reason), heading, self.tracker.heading_error_rad, "ok"
            return build_stop_command(step.reason), heading, self.tracker.heading_error_rad, step.reason

        def _challenge1_safety_state(self, now_s: float) -> str:
            if not self._imu_fresh(now_s):
                return "imu_stale"
            if not self._imu_rate_ok():
                return "imu_rate_low"
            if self.last_motor_status_s is None or now_s - self.last_motor_status_s > self.config.motor_status_timeout_s:
                return "motor_status_stale"
            motor = motor_status_ready(self.motor_status)
            if not motor.safe:
                return motor.reason
            if not self._encoder_feedback_available():
                return "encoder_unavailable"
            if not self._encoder_feedback_fresh(now_s):
                return "encoder_stale"
            if bool(args.challenge1_stop_on_obstacle) and not bool(args.debug_ignore_obstacles):
                front = self.front_clearance_m
                stop_m = max(0.0, float(args.obstacle_stop_m))
                if front is not None and math.isfinite(front) and front < stop_m:
                    return "challenge1_front_obstacle_stop"
                if self.near_obstacle:
                    return "challenge1_near_obstacle"
            return "ok"

        def _challenge1_speed_mps(self) -> float:
            requested = abs(float(args.challenge1_speed_mps))
            return max(
                requested,
                float(self.config.competition_min_speed_mps),
                float(self.config.competition_moving_target_speed_mps),
            )

        def _challenge1_distance_since_start_m(self) -> float:
            distance_m = encoder_average_distance_m(
                start_left_ticks=self.challenge1_start_left_ticks,
                start_right_ticks=self.challenge1_start_right_ticks,
                current_left_ticks=self.current_left_ticks,
                current_right_ticks=self.current_right_ticks,
                config=self.config,
            )
            self.challenge1_distance_m = abs(float(distance_m))
            return self.challenge1_distance_m

        def _start_challenge1_if_needed(self, now_s: float) -> None:
            if self.challenge1_start_s is not None:
                return
            self.challenge1_start_s = now_s
            self.challenge1_start_left_ticks = self.current_left_ticks
            self.challenge1_start_right_ticks = self.current_right_ticks
            self.target_heading_rad = self._heading_rad(now_s)
            self.challenge1_last_omega_radps = 0.0
            self.last_mission_update_s = now_s
            self.challenge1_state = "MOVING_WAIT_LANDING"
            if self.challenge1_uav_landed and self.challenge1_landed_s is None:
                self.challenge1_landed_s = now_s
            self.get_logger().warn(
                "Challenge 1 landing platform started: "
                f"speed={self._challenge1_speed_mps():.3f}m/s "
                f"post_landing={float(args.challenge1_post_landing_s):.1f}s "
                f"timeout={float(args.challenge1_timeout_s):.1f}s"
            )

        def _tick_challenge1_landing_platform(self, now_s: float) -> tuple[ControlCommand, float, float, str]:
            heading = self._heading_rad(now_s)
            safety_state = self._challenge1_safety_state(now_s)
            if safety_state != "ok":
                self.challenge1_state = "FAULT"
                return build_stop_command(safety_state), heading, 0.0, safety_state

            if self.challenge1_complete_s is not None:
                self.challenge1_state = "COMPLETE"
                return build_stop_command("challenge1_complete"), heading, 0.0, "challenge1_complete"

            if not self.challenge1_uav_launched:
                self.challenge1_state = "WAIT_UAV_LAUNCH"
                return build_stop_command("challenge1_wait_uav_launch"), heading, 0.0, "challenge1_wait_uav_launch"

            self._start_challenge1_if_needed(now_s)
            assert self.challenge1_start_s is not None
            self._challenge1_distance_since_start_m()

            timeout_s = max(0.0, float(args.challenge1_timeout_s))
            elapsed_s = now_s - self.challenge1_start_s
            if timeout_s > 0.0 and elapsed_s >= timeout_s:
                self.challenge1_state = "TIMEOUT"
                return build_stop_command("challenge1_timeout"), heading, 0.0, "challenge1_timeout"

            max_distance_m = max(0.0, float(args.challenge1_max_distance_m))
            if max_distance_m > 0.0 and self.challenge1_distance_m >= max_distance_m:
                self.challenge1_state = "DISTANCE_LIMIT"
                return build_stop_command("challenge1_distance_limit"), heading, 0.0, "challenge1_distance_limit"

            if self.challenge1_uav_landed:
                if self.challenge1_landed_s is None:
                    self.challenge1_landed_s = now_s
                post_elapsed_s = now_s - self.challenge1_landed_s
                if post_elapsed_s >= max(0.0, float(args.challenge1_post_landing_s)):
                    self.challenge1_complete_s = now_s
                    self.challenge1_state = "COMPLETE"
                    return build_stop_command("challenge1_complete"), heading, 0.0, "challenge1_complete"
                self.challenge1_state = "POST_LANDING_TRAVEL"
                reason = "challenge1_post_landing_travel"
            else:
                self.challenge1_state = "MOVING_WAIT_LANDING"
                reason = "challenge1_wait_landing"

            dt_s = 0.0 if self.last_mission_update_s is None else max(0.0, now_s - self.last_mission_update_s)
            self.last_mission_update_s = now_s
            target_heading = heading if self.target_heading_rad is None else float(self.target_heading_rad)
            heading_error = math.atan2(
                math.sin(target_heading - heading),
                math.cos(target_heading - heading),
            )
            self.straight_heading_error_samples.append(heading_error)
            omega = straight_omega_with_slew(
                heading_error_rad=heading_error,
                yaw_rate_radps=self.yaw_rate_radps,
                previous_omega_radps=self.challenge1_last_omega_radps,
                dt_s=dt_s,
                config=self.config,
            )
            self.challenge1_last_omega_radps = omega
            return build_velocity_command(self._challenge1_speed_mps(), omega, reason), heading, heading_error, "ok"

        def _challenge2_pivot_config(self) -> ChassisControllerConfig:
            max_omega = max(0.0, float(args.challenge2_pivot_max_omega_radps))
            return replace(
                self.config,
                max_omega_radps=max(max_omega, float(self.config.max_omega_radps)),
                pivot_max_omega_radps=max_omega,
                pivot_timeout_s=max(0.1, float(args.challenge2_pivot_timeout_s)),
                pivot_settle_error_rad=max(0.0, float(args.challenge2_pivot_settle_error_rad)),
                pivot_settle_time_s=max(0.0, float(args.challenge2_pivot_settle_time_s)),
            )

        def _challenge2_straight_config(self) -> ChassisControllerConfig:
            max_omega = max(0.0, float(args.challenge2_max_omega_radps))
            return replace(
                self.config,
                max_omega_radps=max(max_omega, float(self.config.max_omega_radps)),
                mission_straight_max_omega_radps=max_omega,
                heading_kp=float(args.challenge2_heading_kp),
                heading_kd=float(self.config.heading_kd),
            )

        def _challenge2_speed_mps(self, target_distance_m: float) -> float:
            base = max(
                abs(float(args.challenge2_speed_mps)),
                float(self.config.competition_min_speed_mps),
                float(self.config.competition_moving_target_speed_mps),
            )
            approach = max(
                abs(float(args.challenge2_approach_speed_mps)),
                float(self.config.competition_min_speed_mps),
                float(self.config.competition_moving_target_speed_mps),
            )
            stop_radius = max(0.05, float(args.challenge2_stop_radius_m))
            slowdown = max(stop_radius + 0.05, float(args.challenge2_slowdown_distance_m))
            if target_distance_m <= stop_radius:
                return 0.0
            if target_distance_m < slowdown:
                ratio = clamp((target_distance_m - stop_radius) / max(1e-6, slowdown - stop_radius), 0.0, 1.0)
                requested = approach + (base - approach) * ratio
            else:
                requested = base
            return apply_competition_speed_rule(
                requested,
                allow_stop=False,
                min_speed_mps=self.config.competition_min_speed_mps,
                moving_target_speed_mps=self.config.competition_moving_target_speed_mps,
            )

        def _update_challenge2_metrics(self, now_s: float) -> None:
            target = self.challenge2_target
            if target is None:
                self.challenge2_target_distance_m = None
                self.challenge2_target_bearing_rad = None
                self.challenge2_align_error_rad = 0.0
                self.challenge2_cross_track_error_m = 0.0
            else:
                pose = self.challenge2_tracker.pose
                bearing = challenge2_target_bearing_rad(target[0], target[1])
                self.challenge2_target_bearing_rad = bearing
                self.challenge2_target_distance_m = challenge2_target_distance_m(
                    target[0],
                    target[1],
                    pose_x_m=pose.x,
                    pose_y_m=pose.y,
                )
                self.challenge2_align_error_rad = wrap_pi(bearing - pose.yaw)
                self.challenge2_cross_track_error_m = challenge2_cross_track_error_m(
                    target[0],
                    target[1],
                    pose_x_m=pose.x,
                    pose_y_m=pose.y,
                )
            if self.challenge2_uav_landed and self.challenge2_landed_s is None:
                self.challenge2_landed_s = now_s
            self.challenge2_landing_requirement_met = challenge2_landing_requirement_met(
                self.challenge2_landed_s,
                now_s,
                float(args.challenge2_post_landing_s),
            )

        def _ensure_challenge2_started(self, now_s: float, heading: float) -> Optional[str]:
            if self.challenge2_target is None:
                self.challenge2_state = "WAIT_TARGET"
                self.challenge2_result_reason = "waiting_for_target"
                return "challenge2_wait_target"
            if not self._encoder_feedback_available():
                return "encoder_unavailable"
            if not self._encoder_feedback_fresh(now_s):
                return "encoder_stale"
            if self.challenge2_tracker.target is None or self.challenge2_tracker.state in {"WAIT_TARGET", "FAULT"}:
                assert self.current_left_ticks is not None
                assert self.current_right_ticks is not None
                x, y = self.challenge2_target
                start_tracker_goal(
                    self.challenge2_tracker,
                    target_x_m=x,
                    target_y_m=y,
                    left_ticks=self.current_left_ticks,
                    right_ticks=self.current_right_ticks,
                    heading_rad=heading,
                )
                self.challenge2_start_s = now_s
                self.challenge2_state = "ALIGN_TO_TARGET"
                self.challenge2_result_reason = "aligning_to_target"
                self.challenge2_last_omega_radps = 0.0
                self.challenge2_last_update_s = now_s
                reset_pivot(self.pivot)
                self.get_logger().warn(
                    "Challenge 2 align-then-straight started: "
                    f"target=({x:.3f},{y:.3f})m "
                    f"bearing={math.degrees(challenge2_target_bearing_rad(x, y)):.1f}deg "
                    f"stop_radius={float(args.challenge2_stop_radius_m):.2f}m"
                )
            return None

        def _challenge2_goal_stop_reason(self) -> str:
            return (
                "challenge2_goal_reached"
                if self.challenge2_landing_requirement_met
                else "challenge2_goal_reached_landing_requirement_unmet"
            )

        def _tick_challenge2_align_straight(self, now_s: float) -> tuple[ControlCommand, float, float, str]:
            heading = self._heading_rad(now_s)
            safety_state = self._tracker_safety_state(now_s)
            if safety_state != "ok":
                self.challenge2_state = "FAULT"
                self.challenge2_result_reason = safety_state
                reset_pivot(self.pivot)
                self.challenge2_last_omega_radps = 0.0
                return build_stop_command(safety_state), heading, 0.0, safety_state

            start_hold = self._ensure_challenge2_started(now_s, heading)
            self._update_challenge2_metrics(now_s)
            if start_hold is not None:
                return build_stop_command(start_hold), heading, 0.0, start_hold

            assert self.current_left_ticks is not None
            assert self.current_right_ticks is not None
            update_tracker_odometry(
                self.challenge2_tracker,
                left_ticks=self.current_left_ticks,
                right_ticks=self.current_right_ticks,
                heading_rad=heading,
                config=self.challenge2_tracker_config,
            )
            self._update_challenge2_metrics(now_s)
            target_distance = (
                math.inf if self.challenge2_target_distance_m is None else float(self.challenge2_target_distance_m)
            )
            stop_radius = max(0.05, float(args.challenge2_stop_radius_m))
            if target_distance <= stop_radius:
                self.challenge2_state = "GOAL_REACHED"
                self.challenge2_result_reason = self._challenge2_goal_stop_reason()
                self.challenge2_tracker.state = "GOAL_REACHED"
                self.challenge2_tracker.completion_reason = self.challenge2_result_reason
                reset_pivot(self.pivot)
                return build_stop_command(self.challenge2_result_reason), heading, self.challenge2_align_error_rad, "ok"

            if self.challenge2_state == "GOAL_REACHED":
                return build_stop_command(self.challenge2_result_reason), heading, self.challenge2_align_error_rad, "ok"

            if self.challenge2_state in {"WAIT_TARGET", "FAULT"}:
                self.challenge2_state = "ALIGN_TO_TARGET"

            if self.challenge2_state == "ALIGN_TO_TARGET":
                pivot_config = self._challenge2_pivot_config()
                align_error = self.challenge2_align_error_rad
                self.target_heading_rad = wrap_pi(self.challenge2_tracker.start_heading_rad + float(self.challenge2_target_bearing_rad or 0.0))
                if self.pivot.state == "idle" and abs(align_error) <= max(0.0, float(args.challenge2_pivot_settle_error_rad)):
                    self.challenge2_state = "DRIVE_STRAIGHT"
                    self.challenge2_result_reason = "align_already_within_tolerance"
                    self.challenge2_last_update_s = now_s
                    self.challenge2_last_omega_radps = 0.0
                    reset_pivot(self.pivot)
                else:
                    if self.pivot.state == "idle":
                        start_pivot(
                            self.pivot,
                            now_s=now_s,
                            current_heading_rad=heading,
                            encoder_heading_rad=self.estimator.encoder_heading_rad,
                            target_angle_rad=align_error,
                        )
                    step = step_profiled_pivot(
                        self.pivot,
                        now_s=now_s,
                        heading_rad=heading,
                        yaw_rate_radps=self.yaw_rate_radps,
                        encoder_heading_rad=self.estimator.encoder_heading_rad,
                        config=pivot_config,
                    )
                    self.challenge2_align_error_rad = step.heading_error_rad
                    self.target_heading_rad = self.pivot.target_heading_rad
                    self.encoder_gyro_disagreement_rad = pivot_encoder_gyro_disagreement(
                        pivot_start_gyro_heading_rad=self.pivot.start_heading_rad,
                        pivot_start_encoder_heading_rad=self.pivot.start_encoder_heading_rad,
                        current_gyro_heading_rad=self.estimator.gyro_heading_rad,
                        current_encoder_heading_rad=self.estimator.encoder_heading_rad,
                    )
                    self.slip_detected = self.encoder_gyro_disagreement_rad > self.config.slip_disagreement_rad
                    if step.state == "abort":
                        self.challenge2_state = "FAULT"
                        self.challenge2_result_reason = step.reason
                        return build_stop_command(step.reason), heading, step.heading_error_rad, step.reason
                    if step.complete:
                        self.challenge2_state = "DRIVE_STRAIGHT"
                        self.challenge2_result_reason = "align_complete"
                        self.challenge2_last_update_s = now_s
                        self.challenge2_last_omega_radps = 0.0
                    elif step.command_type == "velocity":
                        self.challenge2_result_reason = step.reason
                        return build_velocity_command(0.0, step.omega_radps, step.reason), heading, step.heading_error_rad, "ok"
                    else:
                        self.challenge2_result_reason = step.reason
                        return build_stop_command(step.reason), heading, step.heading_error_rad, "ok"

            if self.challenge2_state == "DRIVE_STRAIGHT":
                dt_s = 0.0 if self.challenge2_last_update_s is None else max(0.0, now_s - self.challenge2_last_update_s)
                self.challenge2_last_update_s = now_s
                straight_config = self._challenge2_straight_config()
                heading_error = self.challenge2_align_error_rad
                cross_track = self.challenge2_cross_track_error_m
                distance_scale = max(0.35, target_distance)
                cross_heading_correction = -float(args.challenge2_cross_track_kp) * math.atan2(cross_track, distance_scale)
                effective_heading_error = wrap_pi(heading_error + cross_heading_correction)
                self.straight_heading_error_samples.append(heading_error)
                omega = straight_omega_with_slew(
                    heading_error_rad=effective_heading_error,
                    yaw_rate_radps=self.yaw_rate_radps,
                    previous_omega_radps=self.challenge2_last_omega_radps,
                    dt_s=dt_s,
                    config=straight_config,
                )
                self.challenge2_last_omega_radps = omega
                v_cmd = self._challenge2_speed_mps(target_distance)
                self.sub_min_speed_command_blocked = (
                    0.0 < abs(float(args.challenge2_speed_mps)) < self.config.competition_min_speed_mps
                    and abs(v_cmd) >= self.config.competition_min_speed_mps
                )
                self.challenge2_result_reason = "driving_to_marker"
                return build_velocity_command(v_cmd, omega, "challenge2_drive_straight"), heading, heading_error, "ok"

            return build_stop_command(self.challenge2_result_reason), heading, self.challenge2_align_error_rad, "ok"

        def _reset_bias_and_heading(self) -> None:
            reset_gyro_bias_calibration(self.gyro_bias)
            self.estimator.gyro_heading_rad = 0.0
            self.estimator.last_stamp_s = None
            self.estimator.gyro_available = False

        def _calibrate_gyro_bias_if_needed(self, now_s: float) -> tuple[bool, str]:
            if not self._imu_fresh(now_s):
                if self._encoder_fallback_allowed(now_s):
                    self.estimator.gyro_available = False
                    return True, "encoder_heading_fallback"
                return False, "heading_feedback_stale"
            if not self._imu_rate_ok():
                if self._encoder_fallback_allowed(now_s):
                    self.estimator.gyro_available = False
                    return True, "encoder_heading_fallback"
                return False, "imu_rate_low"
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
            require_imu = not self._encoder_fallback_allowed(now_s)
            safety = evaluate_safety(
                now_s=now_s,
                last_sensor_s=self.last_sensor_s,
                last_imu_s=self.last_imu_s,
                last_motor_status_s=self.last_motor_status_s,
                motor_status=self.motor_status,
                near_obstacle=self.near_obstacle,
                front_clearance_m=self.front_clearance_m,
                config=self.config,
                require_imu=require_imu,
                imu_rate_hz=self._imu_rate_hz(),
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
                self.target_heading_rad = self._heading_rad(now_s)
                self.target_distance_m = segment.distance_m
                self.mission_state = "straight_active"
            elif segment.segment_type == "pivot":
                reset_pivot(self.pivot)
                start_pivot(
                    self.pivot,
                    now_s=now_s,
                    current_heading_rad=self._heading_rad(now_s),
                    encoder_heading_rad=self.estimator.encoder_heading_rad,
                    target_angle_rad=math.radians(segment.angle_deg),
                )
                self.target_heading_rad = self.pivot.target_heading_rad
                self.target_distance_m = None
                self.mission_state = "pivot_active"
            else:
                self.target_heading_rad = self._heading_rad(now_s)
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
            if self.config.competition_continuous_motion_enabled:
                return build_velocity_command(
                    self.config.competition_moving_target_speed_mps,
                    0.0,
                    f"continuous_segment_transition:{reason}",
                )
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
            require_imu = not self._encoder_fallback_allowed(now_s)
            decision = classify_mission_safety(
                now_s=now_s,
                last_sensor_s=self.last_sensor_s,
                last_imu_s=self.last_imu_s,
                last_motor_status_s=self.last_motor_status_s,
                motor_status=self.motor_status,
                near_obstacle=self.near_obstacle,
                front_clearance_m=self.front_clearance_m,
                config=self.config,
                require_imu=require_imu,
                imu_rate_hz=self._imu_rate_hz(),
            )
            self.mission_safety_level = decision.level
            self.mission_safety_reason = decision.reason

        def _segment_key(self) -> str:
            segment_type = None if self._current_segment() is None else self._current_segment().segment_type
            return f"{self.segment_index}:{segment_type}:{self.segment_start_s}"

        def _measured_speed_mps(self) -> float:
            return average_abs_measured_speed_mps(self.motor_status)

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
            heading = self._heading_rad(now_s)
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
                if self.config.competition_continuous_motion_enabled and segment.segment_type in {"pivot", "wait"}:
                    reason = f"active_{segment.segment_type}_segment_invalid"
                    self.mission_state = "abort"
                    self.mission_safety_level = "critical"
                    self.mission_safety_reason = reason
                    self._log_safety_stop(reason, now_s)
                    return build_stop_command(reason), heading, heading_error, reason
                self._start_mission_segment(now_s)
                if not self.config.competition_continuous_motion_enabled:
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
                        moving_target_speed_mps=self.config.competition_moving_target_speed_mps,
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
                if step.state == "abort":
                    self.mission_state = "abort"
                    self.mission_safety_level = "critical"
                    self.mission_safety_reason = step.reason
                    self._log_safety_stop(step.reason, now_s)
                    return build_stop_command(step.reason), heading, heading_error, step.reason
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
            heading = self._heading_rad(now_s)
            heading_error = 0.0
            safety_state = "idle"
            cmd = build_stop_command("idle")

            if self.mode == "idle":
                self._reset_test_state()
                self._reset_mission_state()
                reset_tracker(self.tracker)
                self._reset_challenge1_state()
                self._reset_challenge2_state()
            elif self.mode == "challenge1_landing_platform":
                calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
                if not calibrated:
                    self.challenge1_state = "GYRO_BIAS_CALIBRATION"
                    cmd = build_stop_command(calibration_reason)
                    safety_state = calibration_reason
                else:
                    cmd, heading, heading_error, safety_state = self._tick_challenge1_landing_platform(now_s)
            elif self.mode == "challenge2_align_straight":
                calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
                if not calibrated:
                    self.challenge2_state = "GYRO_BIAS_CALIBRATION"
                    self.challenge2_result_reason = calibration_reason
                    cmd = build_stop_command(calibration_reason)
                    safety_state = calibration_reason
                else:
                    cmd, heading, heading_error, safety_state = self._tick_challenge2_align_straight(now_s)
            elif self.mode == "competition_tracker":
                calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
                if not calibrated:
                    cmd = build_stop_command(calibration_reason)
                    safety_state = calibration_reason
                else:
                    cmd, heading, heading_error, safety_state = self._tick_competition_tracker(now_s)
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
                        heading = self._heading_rad(now_s)
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
                                dt_s = (
                                    0.0
                                    if self.last_mission_update_s is None
                                    else max(0.0, now_s - self.last_mission_update_s)
                                )
                                self.last_mission_update_s = now_s
                                heading_error = math.atan2(
                                    math.sin(float(self.target_heading_rad) - heading),
                                    math.cos(float(self.target_heading_rad) - heading),
                                )
                                self.straight_heading_error_samples.append(heading_error)
                                omega = straight_omega_with_slew(
                                    heading_error_rad=heading_error,
                                    yaw_rate_radps=self.yaw_rate_radps,
                                    previous_omega_radps=self.previous_straight_omega_radps,
                                    dt_s=dt_s,
                                    config=self.config,
                                )
                                self.previous_straight_omega_radps = omega
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
                        elif self.mode == "curve_test":
                            self._start_curve_if_needed(now_s)
                            step = step_curve(
                                self.curve,
                                now_s=now_s,
                                heading_rad=heading,
                                yaw_rate_radps=self.yaw_rate_radps,
                                config=self.config,
                            )
                            heading_error = step.heading_error_rad
                            self.target_heading_rad = self.curve.target_heading_rad
                            self.encoder_gyro_disagreement_rad = pivot_encoder_gyro_disagreement(
                                pivot_start_gyro_heading_rad=self.curve.start_heading_rad,
                                pivot_start_encoder_heading_rad=self.curve.start_encoder_heading_rad,
                                current_gyro_heading_rad=self.estimator.gyro_heading_rad,
                                current_encoder_heading_rad=self.estimator.encoder_heading_rad,
                            )
                            self.slip_detected = (
                                self.encoder_gyro_disagreement_rad > self.config.slip_disagreement_rad
                            )
                            if step.command_type == "velocity":
                                cmd = build_velocity_command(step.v_mps, step.omega_radps, step.reason)
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
            self.get_logger().warn(f"Chassis test holding STOP: {self._safety_log_detail(reason, now_s)}")
            self.last_safety_reason = reason
            self.last_safety_log_s = now_s

        @staticmethod
        def _status_value_text(value: Any, suffix: str = "") -> str:
            if value is None:
                return "-"
            if isinstance(value, float):
                return f"{value:.3f}{suffix}"
            return f"{value}{suffix}"

        def _safety_log_detail(self, reason: str, now_s: float) -> str:
            if reason not in {"imu_stale", "imu_rate_low", "heading_feedback_stale"}:
                return reason
            imu = self._imu_health(now_s)
            zed = self._zed_imu_status(now_s)
            encoder = self._encoder_health(now_s)
            return (
                f"{reason} "
                f"(imu_topic={imu['topic']} qos={imu['qos']} "
                f"age={self._status_value_text(imu['age_s'], 's')} "
                f"rate={self._status_value_text(imu['rate_hz'], 'Hz')} "
                f"min={self._status_value_text(imu['min_rate_hz'], 'Hz')} "
                f"zed_age={self._status_value_text(zed.get('age_s'), 's')} "
                f"zed_rate={self._status_value_text(zed.get('imu_rate_hz'), 'Hz')} "
                f"zed_count={self._status_value_text(zed.get('imu_count'))} "
                f"zed_failures={self._status_value_text(zed.get('imu_publish_failures'))} "
                f"zed_error={zed.get('last_imu_error') or '-'} "
                f"zed_busy_skips={self._status_value_text(zed.get('imu_busy_skips'))} "
                f"encoder_age={self._status_value_text(encoder.get('age_s'), 's')} "
                f"encoder_fresh={encoder.get('fresh')} "
                f"encoder_fallback={encoder.get('fallback_allowed')})"
            )

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
            self._update_challenge2_metrics(now_s)
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
            if self.mode == "challenge2_align_straight":
                straight_limit = max(0.0, float(args.challenge2_max_omega_radps))
                pivot_limit = max(0.0, float(args.challenge2_pivot_max_omega_radps))
            _, _, curve_limit = curve_speed_radius_omega(self.config)
            if self.mode == "curve_test" and self.curve.state not in {"idle", "complete"}:
                omega_limit = curve_limit
            elif pivot_state not in {"idle", "complete"}:
                omega_limit = pivot_limit
            else:
                omega_limit = straight_limit
            pivot_state_elapsed_s = (
                None if self.pivot.state_start_s is None else round(max(0.0, now_s - self.pivot.state_start_s), 3)
            )
            pivot_clearance_known = self.last_scan_ranges is not None and self.pivot_clearance_m is not None
            curve_state = self.curve.state
            if self.mode == "curve_test" and safety_state in {
                "gyro_bias_calibration",
                "gyro_bias_unstable",
                "gyro_bias_encoder_motion",
            }:
                curve_state = "gyro_bias_calibration"
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
                "challenge1_state": self.challenge1_state,
                "challenge1_uav_launched": self.challenge1_uav_launched,
                "challenge1_uav_landed": self.challenge1_uav_landed,
                "challenge1_speed_mps": self._challenge1_speed_mps(),
                "challenge1_distance_m": self.challenge1_distance_m,
                "challenge1_elapsed_s": (
                    None if self.challenge1_start_s is None else round(max(0.0, now_s - self.challenge1_start_s), 3)
                ),
                "challenge1_post_landing_elapsed_s": (
                    None if self.challenge1_landed_s is None else round(max(0.0, now_s - self.challenge1_landed_s), 3)
                ),
                "challenge1_post_landing_required_s": max(0.0, float(args.challenge1_post_landing_s)),
                "challenge1_timeout_s": max(0.0, float(args.challenge1_timeout_s)),
                "challenge1_max_distance_m": max(0.0, float(args.challenge1_max_distance_m)),
                "challenge2_state": self.challenge2_state,
                "challenge2_target_m": (
                    None
                    if self.challenge2_target is None
                    else [round(float(self.challenge2_target[0]), 4), round(float(self.challenge2_target[1]), 4)]
                ),
                "challenge2_pose_m": [
                    round(float(self.challenge2_tracker.pose.x), 4),
                    round(float(self.challenge2_tracker.pose.y), 4),
                    round(float(self.challenge2_tracker.pose.yaw), 5),
                ],
                "challenge2_target_distance_m": (
                    None
                    if self.challenge2_target_distance_m is None
                    else round(float(self.challenge2_target_distance_m), 4)
                ),
                "challenge2_target_bearing_rad": (
                    None
                    if self.challenge2_target_bearing_rad is None
                    else round(float(self.challenge2_target_bearing_rad), 5)
                ),
                "challenge2_align_error_rad": round(float(self.challenge2_align_error_rad), 5),
                "challenge2_cross_track_error_m": round(float(self.challenge2_cross_track_error_m), 4),
                "challenge2_uav_landed": self.challenge2_uav_landed,
                "challenge2_post_landing_elapsed_s": (
                    None if self.challenge2_landed_s is None else round(max(0.0, now_s - self.challenge2_landed_s), 3)
                ),
                "challenge2_post_landing_required_s": max(0.0, float(args.challenge2_post_landing_s)),
                "challenge2_landing_requirement_met": self.challenge2_landing_requirement_met,
                "challenge2_result_reason": self.challenge2_result_reason,
                "competition_min_speed_mps": self.config.competition_min_speed_mps,
                "moving_target_speed_mps": self.config.competition_moving_target_speed_mps,
                "competition_continuous_motion_enabled": self.config.competition_continuous_motion_enabled,
                "commanded_speed_mps": abs(last_v),
                "measured_speed_mps": self._measured_speed_mps(),
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
                "heading_source": self._heading_source(now_s),
                "gyro_heading_rad": self.estimator.gyro_heading_rad,
                "encoder_heading_rad": self.estimator.encoder_heading_rad,
                "encoder_health": self._encoder_health(now_s),
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
                "curve_state": curve_state,
                "curve_target_angle_rad": self.curve.target_angle_rad,
                "curve_heading_error_rad": self.curve.last_error_rad,
                "curve_actual_angle_rad": self.curve.target_angle_rad - self.curve.last_error_rad,
                "curve_elapsed_s": (
                    None if self.curve.motion_start_s is None else round(max(0.0, now_s - self.curve.motion_start_s), 3)
                ),
                "curve_progress_age_s": (
                    None if self.curve.last_progress_s is None else round(max(0.0, now_s - self.curve.last_progress_s), 3)
                ),
                "curve_abort_reason": self.curve.abort_reason,
                "curve_min_abs_error_rad": (
                    self.curve.min_abs_error_rad if math.isfinite(self.curve.min_abs_error_rad) else None
                ),
                "curve_radius_m": self.curve.radius_m,
                "curve_arc_length_m": self.curve.arc_length_m,
                "curve_direction": self.curve.direction,
                "curve_side_reverse_blocked": self.curve.side_reverse_blocked,
                "tracking_enabled": bool(args.tracking_enabled),
                **tracker_status(self.tracker),
                "v_mps": last_v,
                "omega_radps": last_omega,
                "omega_saturated": abs(last_omega) >= max(0.0, omega_limit - 1e-6) and abs(last_omega) > 1e-6,
                "straight_omega_limit_radps": straight_limit,
                "straight_omega_slew_radps2": self.config.mission_straight_omega_slew_radps2,
                "heading_kp": self.config.heading_kp,
                "heading_kd": self.config.heading_kd,
                "heading_deadband_rad": self.config.heading_deadband_rad,
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
                "imu_min_rate_hz": round(max(0.0, float(self.config.imu_min_rate_hz)), 3),
                "imu_health": self._imu_health(now_s),
                "zed_imu_status": self._zed_imu_status(now_s),
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
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        if node is not None:
            try:
                node._publish_command(build_stop_command("shutdown"))
            except Exception:
                pass
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
