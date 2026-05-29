"""High-level chassis helpers for safe Jetson heading tests."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple


@dataclass(frozen=True)
class ChassisControllerConfig:
    straight_speed_mps: float = 0.20
    straight_duration_s: float = 2.0
    pivot_angle_deg: float = 90.0
    max_omega_radps: float = 0.45
    heading_kp: float = 1.2
    heading_kd: float = 0.15
    pivot_kp: float = 1.0
    heading_deadband_rad: float = 0.025
    stop_clearance_m: float = 0.45
    sensor_timeout_s: float = 0.30
    motor_status_timeout_s: float = 0.50
    imu_timeout_s: float = 0.12
    max_test_duration_s: float = 3.0
    gyro_bias_calibration_s: float = 1.5
    gyro_bias_max_std_radps: float = 0.03
    gyro_bias_warn_abs_radps: float = 0.10
    gyro_bias_max_encoder_delta_ticks: int = 2
    pivot_max_omega_radps: float = 0.35
    pivot_min_omega_radps: float = 0.16
    pivot_breakaway_omega_radps: float = 0.18
    pivot_breakaway_s: float = 0.20
    pivot_accel_limit_radps2: float = 0.80
    pivot_decel_limit_radps2: float = 0.60
    pivot_approach_error_rad: float = 0.25
    pivot_min_omega_disable_error_rad: float = 0.10
    pivot_kp_approach: float = 0.75
    pivot_kd_yaw_rate: float = 0.10
    pivot_settle_error_rad: float = 0.035
    pivot_settle_yaw_rate_radps: float = 0.05
    pivot_settle_time_s: float = 0.35
    pivot_brake_s: float = 0.15
    pivot_timeout_s: float = 4.0
    pivot_max_correction_retries: int = 1
    pivot_clearance_m: float = 0.35
    slip_disagreement_rad: float = 0.35
    competition_min_speed_mps: float = 0.089408
    mission_default_speed_mps: float = 0.15
    mission_reliable_speed_mps: float = 0.15
    mission_slow_speed_mps: float = 0.09
    mission_emergency_stop_clearance_m: float = 0.18
    mission_critical_sensor_timeout_s: float = 1.0
    mission_straight_max_omega_radps: float = 0.20
    mission_straight_omega_slew_radps2: float = 0.80
    debug_allow_sub_min_crawl: bool = False
    track_width_m: float = 0.425
    wheel_radius_m: float = 0.0825
    ticks_per_rev: float = 3200.0


@dataclass(frozen=True)
class SafetyDecision:
    safe: bool
    reason: str


@dataclass
class ChassisEstimatorState:
    gyro_heading_rad: float = 0.0
    encoder_heading_rad: float = 0.0
    last_stamp_s: Optional[float] = None
    last_left_ticks: Optional[int] = None
    last_right_ticks: Optional[int] = None
    gyro_available: bool = False
    encoder_available: bool = False

    @property
    def heading_rad(self) -> float:
        return self.gyro_heading_rad if self.gyro_available else self.encoder_heading_rad


@dataclass
class GyroBiasCalibrationState:
    start_s: Optional[float] = None
    samples: list[float] = field(default_factory=list)
    start_left_ticks: Optional[int] = None
    start_right_ticks: Optional[int] = None
    bias_radps: float = 0.0
    std_radps: float = 0.0
    ready: bool = False
    unstable_reason: Optional[str] = None


@dataclass(frozen=True)
class GyroBiasCalibrationResult:
    ready: bool
    unstable: bool
    reason: str
    bias_radps: float
    std_radps: float


@dataclass
class PivotControllerState:
    state: str = "idle"
    state_start_s: Optional[float] = None
    motion_start_s: Optional[float] = None
    target_heading_rad: Optional[float] = None
    start_heading_rad: float = 0.0
    start_encoder_heading_rad: float = 0.0
    last_update_s: Optional[float] = None
    previous_omega_radps: float = 0.0
    settle_start_s: Optional[float] = None
    retry_count: int = 0
    complete: bool = False
    abort_reason: Optional[str] = None
    last_error_rad: float = 0.0
    last_command_reason: str = "idle"


@dataclass(frozen=True)
class PivotStepResult:
    command_type: str
    omega_radps: float
    reason: str
    state: str
    heading_error_rad: float
    complete: bool = False


def clamp(value: float, lower: float, upper: float) -> float:
    return max(float(lower), min(float(upper), float(value)))


def wrap_pi(angle_rad: float) -> float:
    wrapped = (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi and angle_rad > 0.0:
        return math.pi
    return wrapped


def integrate_gyro_yaw(current_heading_rad: float, yaw_rate_radps: float, dt_s: float) -> float:
    if dt_s <= 0.0 or not math.isfinite(dt_s):
        return wrap_pi(current_heading_rad)
    return wrap_pi(current_heading_rad + yaw_rate_radps * dt_s)


def integrate_gyro_yaw_clamped(
    current_heading_rad: float,
    yaw_rate_radps: float,
    dt_s: float,
    *,
    min_dt_s: float = 0.001,
    max_dt_s: float = 0.05,
) -> Tuple[float, bool]:
    if dt_s < min_dt_s or dt_s > max_dt_s or not math.isfinite(dt_s):
        return wrap_pi(current_heading_rad), False
    return integrate_gyro_yaw(current_heading_rad, yaw_rate_radps, dt_s), True


def encoder_yaw_delta(
    left_delta_ticks: float,
    right_delta_ticks: float,
    *,
    wheel_radius_m: float,
    ticks_per_rev: float,
    track_width_m: float,
) -> float:
    ticks = max(1e-6, float(ticks_per_rev))
    track = max(1e-6, float(track_width_m))
    circumference_m = 2.0 * math.pi * max(1e-6, float(wheel_radius_m))
    left_m = float(left_delta_ticks) / ticks * circumference_m
    right_m = float(right_delta_ticks) / ticks * circumference_m
    return (right_m - left_m) / track


def encoder_ticks_to_distance_m(
    delta_ticks: float,
    *,
    wheel_radius_m: float,
    ticks_per_rev: float,
) -> float:
    ticks = max(1e-6, float(ticks_per_rev))
    circumference_m = 2.0 * math.pi * max(1e-6, float(wheel_radius_m))
    return float(delta_ticks) / ticks * circumference_m


def update_gyro_heading(
    state: ChassisEstimatorState,
    *,
    stamp_s: float,
    raw_yaw_rate_radps: float,
    gyro_bias_radps: float,
    min_dt_s: float = 0.001,
    max_dt_s: float = 0.05,
) -> Tuple[ChassisEstimatorState, float, bool]:
    yaw_rate = float(raw_yaw_rate_radps) - float(gyro_bias_radps)
    if state.last_stamp_s is None:
        state.last_stamp_s = float(stamp_s)
        state.gyro_available = True
        return state, yaw_rate, False
    dt_s = float(stamp_s) - float(state.last_stamp_s)
    state.gyro_heading_rad, integrated = integrate_gyro_yaw_clamped(
        state.gyro_heading_rad,
        yaw_rate,
        dt_s,
        min_dt_s=min_dt_s,
        max_dt_s=max_dt_s,
    )
    state.last_stamp_s = float(stamp_s)
    state.gyro_available = True
    return state, yaw_rate, integrated


def update_estimator(
    state: ChassisEstimatorState,
    *,
    stamp_s: float,
    yaw_rate_radps: float,
    left_ticks: Optional[int],
    right_ticks: Optional[int],
    encoder_available: bool,
    config: ChassisControllerConfig,
) -> ChassisEstimatorState:
    dt_s = 0.0 if state.last_stamp_s is None else clamp(stamp_s - state.last_stamp_s, 0.0, 0.2)
    state.gyro_heading_rad = integrate_gyro_yaw(state.gyro_heading_rad, yaw_rate_radps, dt_s)
    state.gyro_available = True

    if encoder_available and left_ticks is not None and right_ticks is not None:
        if state.last_left_ticks is not None and state.last_right_ticks is not None:
            state.encoder_heading_rad = wrap_pi(
                state.encoder_heading_rad
                + encoder_yaw_delta(
                    left_ticks - state.last_left_ticks,
                    right_ticks - state.last_right_ticks,
                    wheel_radius_m=config.wheel_radius_m,
                    ticks_per_rev=config.ticks_per_rev,
                    track_width_m=config.track_width_m,
                )
            )
        state.last_left_ticks = int(left_ticks)
        state.last_right_ticks = int(right_ticks)
        state.encoder_available = True

    state.last_stamp_s = float(stamp_s)
    return state


def update_encoder_heading(
    state: ChassisEstimatorState,
    *,
    left_ticks: Optional[int],
    right_ticks: Optional[int],
    encoder_available: bool,
    config: ChassisControllerConfig,
) -> ChassisEstimatorState:
    if encoder_available and left_ticks is not None and right_ticks is not None:
        if state.last_left_ticks is not None and state.last_right_ticks is not None:
            state.encoder_heading_rad = wrap_pi(
                state.encoder_heading_rad
                + encoder_yaw_delta(
                    left_ticks - state.last_left_ticks,
                    right_ticks - state.last_right_ticks,
                    wheel_radius_m=config.wheel_radius_m,
                    ticks_per_rev=config.ticks_per_rev,
                    track_width_m=config.track_width_m,
                )
            )
        state.last_left_ticks = int(left_ticks)
        state.last_right_ticks = int(right_ticks)
        state.encoder_available = True
    return state


def gyro_bias_sample_std(samples: Sequence[float]) -> float:
    if not samples:
        return 0.0
    mean = sum(float(sample) for sample in samples) / float(len(samples))
    variance = sum((float(sample) - mean) ** 2 for sample in samples) / float(len(samples))
    return math.sqrt(max(0.0, variance))


def update_gyro_bias_calibration(
    state: GyroBiasCalibrationState,
    *,
    now_s: float,
    raw_yaw_rate_radps: float,
    left_ticks: Optional[int],
    right_ticks: Optional[int],
    config: ChassisControllerConfig,
) -> GyroBiasCalibrationResult:
    duration_s = max(0.0, float(config.gyro_bias_calibration_s))
    if duration_s <= 0.0:
        state.ready = True
        return GyroBiasCalibrationResult(True, False, "gyro_bias_disabled", 0.0, 0.0)
    if state.ready:
        return GyroBiasCalibrationResult(True, False, "gyro_bias_ready", state.bias_radps, state.std_radps)
    if state.start_s is None:
        state.start_s = float(now_s)
        state.samples.clear()
        state.start_left_ticks = None if left_ticks is None else int(left_ticks)
        state.start_right_ticks = None if right_ticks is None else int(right_ticks)
        state.unstable_reason = None

    state.samples.append(float(raw_yaw_rate_radps))
    if now_s - state.start_s < duration_s:
        return GyroBiasCalibrationResult(False, False, "gyro_bias_calibration", 0.0, 0.0)

    bias = sum(state.samples) / float(max(1, len(state.samples)))
    std = gyro_bias_sample_std(state.samples)
    state.bias_radps = bias
    state.std_radps = std

    moved = False
    if (
        left_ticks is not None
        and right_ticks is not None
        and state.start_left_ticks is not None
        and state.start_right_ticks is not None
    ):
        max_delta = max(
            abs(int(left_ticks) - state.start_left_ticks),
            abs(int(right_ticks) - state.start_right_ticks),
        )
        moved = max_delta > int(config.gyro_bias_max_encoder_delta_ticks)
    if moved:
        unstable_reason = "gyro_bias_encoder_motion"
    elif std > max(0.0, float(config.gyro_bias_max_std_radps)):
        unstable_reason = "gyro_bias_unstable"
    else:
        state.ready = True
        warn_abs = abs(bias) > max(0.0, float(config.gyro_bias_warn_abs_radps))
        reason = "gyro_bias_large" if warn_abs else "gyro_bias_ready"
        return GyroBiasCalibrationResult(True, False, reason, bias, std)

    reset_gyro_bias_calibration(state)
    return GyroBiasCalibrationResult(False, True, unstable_reason, bias, std)


def reset_gyro_bias_calibration(state: GyroBiasCalibrationState) -> None:
    state.start_s = None
    state.samples.clear()
    state.start_left_ticks = None
    state.start_right_ticks = None
    state.bias_radps = 0.0
    state.std_radps = 0.0
    state.ready = False
    state.unstable_reason = None


def compute_straight_omega(
    *,
    target_heading_rad: float,
    heading_rad: float,
    yaw_rate_radps: float,
    config: ChassisControllerConfig,
) -> Tuple[float, float]:
    error = wrap_pi(target_heading_rad - heading_rad)
    control_error = 0.0 if abs(error) < config.heading_deadband_rad else error
    omega = config.heading_kp * control_error - config.heading_kd * yaw_rate_radps
    return clamp(omega, -config.max_omega_radps, config.max_omega_radps), error


def compute_pivot_omega(
    *,
    target_heading_rad: float,
    heading_rad: float,
    config: ChassisControllerConfig,
) -> Tuple[float, float]:
    error = wrap_pi(target_heading_rad - heading_rad)
    control_error = 0.0 if abs(error) < config.heading_deadband_rad else error
    omega = config.pivot_kp * control_error
    return clamp(omega, -config.max_omega_radps, config.max_omega_radps), error


def slew_limit(current_value: float, target_value: float, max_delta: float) -> float:
    delta = clamp(float(target_value) - float(current_value), -abs(float(max_delta)), abs(float(max_delta)))
    return float(current_value) + delta


def compute_profiled_pivot_omega(
    *,
    heading_error_rad: float,
    yaw_rate_radps: float,
    previous_omega_radps: float,
    dt_s: float,
    state: str,
    config: ChassisControllerConfig,
) -> float:
    error = float(heading_error_rad)
    sign = 1.0 if error >= 0.0 else -1.0
    abs_error = abs(error)
    max_omega = max(0.0, min(float(config.pivot_max_omega_radps), float(config.max_omega_radps)))

    if state == "breakaway":
        omega_target = sign * min(max_omega, max(0.0, float(config.pivot_breakaway_omega_radps)))
    elif state == "approach":
        omega_target = (
            float(config.pivot_kp_approach) * error
            - float(config.pivot_kd_yaw_rate) * float(yaw_rate_radps)
        )
        omega_target = clamp(omega_target, -max_omega, max_omega)
    else:
        profile_abs = math.sqrt(
            max(
                0.0,
                2.0
                * max(0.0, float(config.pivot_decel_limit_radps2))
                * max(abs_error - max(0.0, float(config.pivot_approach_error_rad)), 0.0),
            )
        )
        profile_abs = min(max_omega, profile_abs)
        if abs_error > max(0.0, float(config.pivot_min_omega_disable_error_rad)):
            profile_abs = max(profile_abs, min(max_omega, max(0.0, float(config.pivot_min_omega_radps))))
        omega_target = sign * profile_abs

    max_delta = max(0.0, float(config.pivot_accel_limit_radps2)) * max(0.0, float(dt_s))
    return slew_limit(previous_omega_radps, omega_target, max_delta)


def start_pivot(
    state: PivotControllerState,
    *,
    now_s: float,
    current_heading_rad: float,
    encoder_heading_rad: float,
    target_angle_rad: float,
) -> None:
    state.state = "breakaway"
    state.state_start_s = float(now_s)
    state.motion_start_s = float(now_s)
    state.target_heading_rad = wrap_pi(float(current_heading_rad) + float(target_angle_rad))
    state.start_heading_rad = float(current_heading_rad)
    state.start_encoder_heading_rad = float(encoder_heading_rad)
    state.last_update_s = float(now_s)
    state.previous_omega_radps = 0.0
    state.settle_start_s = None
    state.retry_count = 0
    state.complete = False
    state.abort_reason = None
    state.last_error_rad = wrap_pi(state.target_heading_rad - float(current_heading_rad))
    state.last_command_reason = "pivot_breakaway"


def reset_pivot(state: PivotControllerState) -> None:
    state.state = "idle"
    state.state_start_s = None
    state.motion_start_s = None
    state.target_heading_rad = None
    state.start_heading_rad = 0.0
    state.start_encoder_heading_rad = 0.0
    state.last_update_s = None
    state.previous_omega_radps = 0.0
    state.settle_start_s = None
    state.retry_count = 0
    state.complete = False
    state.abort_reason = None
    state.last_error_rad = 0.0
    state.last_command_reason = "idle"


def pivot_encoder_gyro_disagreement(
    *,
    pivot_start_gyro_heading_rad: float,
    pivot_start_encoder_heading_rad: float,
    current_gyro_heading_rad: float,
    current_encoder_heading_rad: float,
) -> float:
    gyro_delta = wrap_pi(float(current_gyro_heading_rad) - float(pivot_start_gyro_heading_rad))
    encoder_delta = wrap_pi(float(current_encoder_heading_rad) - float(pivot_start_encoder_heading_rad))
    return abs(wrap_pi(encoder_delta - gyro_delta))


def step_profiled_pivot(
    state: PivotControllerState,
    *,
    now_s: float,
    heading_rad: float,
    yaw_rate_radps: float,
    encoder_heading_rad: float,
    config: ChassisControllerConfig,
) -> PivotStepResult:
    if state.target_heading_rad is None or state.state in {"idle", "complete", "abort"}:
        return PivotStepResult("stop", 0.0, state.last_command_reason, state.state, state.last_error_rad, state.complete)

    error = wrap_pi(float(state.target_heading_rad) - float(heading_rad))
    state.last_error_rad = error
    dt_s = 0.0 if state.last_update_s is None else clamp(float(now_s) - state.last_update_s, 0.0, 0.1)
    state.last_update_s = float(now_s)
    motion_start_s = float(state.motion_start_s) if state.motion_start_s is not None else float(now_s)
    elapsed_total_s = float(now_s) - motion_start_s
    if elapsed_total_s > max(0.0, float(config.pivot_timeout_s)):
        state.state = "abort"
        state.abort_reason = "pivot_timeout"
        state.last_command_reason = "pivot_timeout"
        return PivotStepResult("stop", 0.0, "pivot_timeout", state.state, error)

    state_start_s = float(state.state_start_s) if state.state_start_s is not None else float(now_s)
    state_elapsed_s = float(now_s) - state_start_s
    abs_error = abs(error)
    if state.state == "breakaway" and state_elapsed_s >= max(0.0, float(config.pivot_breakaway_s)):
        state.state = "approach" if abs_error <= float(config.pivot_approach_error_rad) else "rotate"
        state.state_start_s = float(now_s)
    elif state.state == "rotate" and abs_error <= float(config.pivot_approach_error_rad):
        state.state = "approach"
        state.state_start_s = float(now_s)

    settled = (
        abs_error <= max(0.0, float(config.pivot_settle_error_rad))
        and abs(float(yaw_rate_radps)) <= max(0.0, float(config.pivot_settle_yaw_rate_radps))
    )

    if state.state == "approach" and settled:
        state.state = "stop_brake"
        state.state_start_s = float(now_s)
        state.previous_omega_radps = 0.0
        state.last_command_reason = "pivot_stop_brake"
        return PivotStepResult("stop", 0.0, "pivot_stop_brake", state.state, error)

    if state.state == "stop_brake":
        state.previous_omega_radps = 0.0
        if state_elapsed_s >= max(0.0, float(config.pivot_brake_s)):
            state.state = "settle"
            state.state_start_s = float(now_s)
            state.settle_start_s = float(now_s) if settled else None
        state.last_command_reason = "pivot_stop_brake"
        return PivotStepResult("stop", 0.0, "pivot_stop_brake", state.state, error)

    if state.state == "settle":
        state.previous_omega_radps = 0.0
        if settled:
            if state.settle_start_s is None:
                state.settle_start_s = float(now_s)
            if now_s - state.settle_start_s >= max(0.0, float(config.pivot_settle_time_s)):
                state.state = "complete"
                state.complete = True
                state.last_command_reason = "pivot_test_complete"
                return PivotStepResult("stop", 0.0, "pivot_test_complete", state.state, error, True)
        else:
            state.settle_start_s = None
            if state.retry_count < max(0, int(config.pivot_max_correction_retries)):
                state.retry_count += 1
                state.state = "correction_retry"
                state.state_start_s = float(now_s)
            else:
                state.state = "complete"
                state.complete = True
                state.last_command_reason = "pivot_test_complete"
                return PivotStepResult("stop", 0.0, "pivot_test_complete", state.state, error, True)
        state.last_command_reason = "pivot_settling"
        return PivotStepResult("stop", 0.0, "pivot_settling", state.state, error)

    if state.state == "correction_retry":
        if settled:
            state.state = "stop_brake"
            state.state_start_s = float(now_s)
            state.previous_omega_radps = 0.0
            state.last_command_reason = "pivot_stop_brake"
            return PivotStepResult("stop", 0.0, "pivot_stop_brake", state.state, error)
        if abs_error > float(config.pivot_approach_error_rad):
            state.state = "rotate"
            state.state_start_s = float(now_s)
        else:
            state.state = "approach"
            state.state_start_s = float(now_s)

    omega = compute_profiled_pivot_omega(
        heading_error_rad=error,
        yaw_rate_radps=yaw_rate_radps,
        previous_omega_radps=state.previous_omega_radps,
        dt_s=max(0.0, dt_s),
        state=state.state,
        config=config,
    )
    state.previous_omega_radps = omega
    reason = f"pivot_{state.state}"
    state.last_command_reason = reason
    return PivotStepResult("velocity", omega, reason, state.state, error)


def min_finite_range(ranges: Iterable[float]) -> Optional[float]:
    finite_values = [float(value) for value in ranges if math.isfinite(float(value)) and float(value) > 0.0]
    if not finite_values:
        return None
    return min(finite_values)


def evaluate_pivot_clearance(ranges: Optional[Iterable[float]], config: ChassisControllerConfig) -> SafetyDecision:
    if ranges is None:
        return SafetyDecision(True, "ok")
    min_range = min_finite_range(ranges)
    if min_range is not None and min_range < max(0.0, float(config.pivot_clearance_m)):
        return SafetyDecision(False, "pivot_clearance_low")
    return SafetyDecision(True, "ok")


def motor_status_ready(status: Optional[Dict[str, Any]]) -> SafetyDecision:
    if not status:
        return SafetyDecision(False, "motor_status_missing")
    if not bool(status.get("connected", False)):
        return SafetyDecision(False, "motor_disconnected")
    if not bool(status.get("teensy_pid_params_synced", False)):
        return SafetyDecision(False, "motor_params_not_synced")
    for key in ("fault_reason", "fault"):
        value = status.get(key)
        if value is None:
            continue
        text = str(value).strip().lower()
        if text and text not in {"none", "0", "false"}:
            return SafetyDecision(False, f"motor_fault:{value}")
    return SafetyDecision(True, "ok")


def evaluate_safety(
    *,
    now_s: float,
    last_sensor_s: Optional[float],
    last_motor_status_s: Optional[float],
    motor_status: Optional[Dict[str, Any]],
    near_obstacle: bool,
    front_clearance_m: Optional[float],
    config: ChassisControllerConfig,
    last_imu_s: Optional[float] = None,
    require_imu: bool = False,
) -> SafetyDecision:
    if last_sensor_s is None or now_s - last_sensor_s > config.sensor_timeout_s:
        return SafetyDecision(False, "sensor_stale")
    if require_imu and (last_imu_s is None or now_s - last_imu_s > config.imu_timeout_s):
        return SafetyDecision(False, "imu_stale")
    if last_motor_status_s is None or now_s - last_motor_status_s > config.motor_status_timeout_s:
        return SafetyDecision(False, "motor_status_stale")

    motor = motor_status_ready(motor_status)
    if not motor.safe:
        return motor
    if near_obstacle:
        return SafetyDecision(False, "near_obstacle")
    if front_clearance_m is None or math.isnan(front_clearance_m):
        return SafetyDecision(False, "front_clearance_invalid")
    if math.isfinite(front_clearance_m) and front_clearance_m < config.stop_clearance_m:
        return SafetyDecision(False, "front_clearance_low")
    return SafetyDecision(True, "ok")


def bounded_duration(requested_s: float, config: ChassisControllerConfig) -> float:
    return clamp(requested_s, 0.0, config.max_test_duration_s)
