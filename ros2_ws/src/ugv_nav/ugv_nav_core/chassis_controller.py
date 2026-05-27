"""High-level chassis helpers for safe Jetson heading tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


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
    max_test_duration_s: float = 3.0
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
) -> SafetyDecision:
    if last_sensor_s is None or now_s - last_sensor_s > config.sensor_timeout_s:
        return SafetyDecision(False, "sensor_stale")
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
