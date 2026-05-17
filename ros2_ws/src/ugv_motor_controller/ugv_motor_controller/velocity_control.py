#!/usr/bin/env python3
"""ROS-free helpers for raw and velocity motor command handling."""

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


RAW_FALLBACK_BY_MODE = {
    "FORWARD": (0.35, 0.35),
    "BACKWARD": (-0.35, -0.35),
    "TURN_LEFT": (-0.30, 0.30),
    "TURN_RIGHT": (0.30, -0.30),
    "STOP": (0.0, 0.0),
}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def finite_float(value: Any, *, name: str) -> float:
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite")
    return out


def is_stop_command(obj: Dict[str, Any]) -> bool:
    mode = str(obj.get("mode", "")).upper()
    command_type = str(obj.get("command_type", "")).lower()
    return mode == "STOP" or command_type == "stop"


def extract_raw_drive(obj: Dict[str, Any]) -> Tuple[float, float]:
    if "raw_left" in obj and "raw_right" in obj:
        return (
            finite_float(obj["raw_left"], name="raw_left"),
            finite_float(obj["raw_right"], name="raw_right"),
        )

    mode = str(obj.get("mode", "STOP")).upper()
    return RAW_FALLBACK_BY_MODE.get(mode, (0.0, 0.0))


def extract_velocity_command(
    obj: Dict[str, Any],
    *,
    prefer_velocity_fields: bool = True,
) -> Optional[Tuple[float, float]]:
    command_type = str(obj.get("command_type", "")).lower()
    controller = str(obj.get("controller", "")).lower()
    mode = str(obj.get("mode", "")).upper()
    if command_type == "raw":
        return None

    explicit_velocity = (
        command_type == "velocity"
        or controller == "velocity"
        or mode == "VELOCITY"
    )
    has_velocity_fields = "v_mps" in obj and "omega_radps" in obj
    if not has_velocity_fields:
        return None
    if not explicit_velocity and not prefer_velocity_fields:
        return None
    if not explicit_velocity and ("raw_left" in obj or "raw_right" in obj):
        return None

    return (
        finite_float(obj["v_mps"], name="v_mps"),
        finite_float(obj["omega_radps"], name="omega_radps"),
    )


def select_drive_command(
    obj: Dict[str, Any],
    *,
    velocity_control_enabled: bool,
    prefer_velocity_fields: bool = True,
) -> Tuple[str, Optional[Tuple[float, float]], Optional[Tuple[float, float]]]:
    velocity_cmd = extract_velocity_command(
        obj,
        prefer_velocity_fields=prefer_velocity_fields,
    )
    if bool(velocity_control_enabled) and velocity_cmd is not None:
        return "velocity", velocity_cmd, None
    return "raw", None, extract_raw_drive(obj)


def velocity_to_wheel_speeds(
    v_mps: float,
    omega_radps: float,
    track_width_m: float,
) -> Tuple[float, float]:
    track = max(1e-6, float(track_width_m))
    return (
        float(v_mps) - 0.5 * float(omega_radps) * track,
        float(v_mps) + 0.5 * float(omega_radps) * track,
    )


@dataclass(frozen=True)
class RawFallbackFloorResult:
    left_raw: float
    right_raw: float
    applied_left: bool
    applied_right: bool


def apply_velocity_raw_fallback_floor(
    left_raw: float,
    right_raw: float,
    *,
    enabled: bool,
    command_type: str,
    mode: str,
    min_wheel_raw: float,
    min_target_raw: float,
) -> RawFallbackFloorResult:
    if (
        not bool(enabled)
        or str(command_type).lower() != "velocity"
        or str(mode).upper() == "STOP"
    ):
        return RawFallbackFloorResult(float(left_raw), float(right_raw), False, False)

    floor = abs(float(min_wheel_raw))
    threshold = abs(float(min_target_raw))
    if floor <= 0.0:
        return RawFallbackFloorResult(float(left_raw), float(right_raw), False, False)

    def floor_one(raw: float) -> Tuple[float, bool]:
        value = finite_float(raw, name="raw")
        mag = abs(value)
        if mag <= 0.0:
            return value, False
        if mag >= threshold and mag < floor:
            return math.copysign(floor, value), True
        return value, False

    left, applied_left = floor_one(left_raw)
    right, applied_right = floor_one(right_raw)
    return RawFallbackFloorResult(left, right, applied_left, applied_right)


def encoder_delta_to_wheel_speed_mps(
    delta_ticks: int,
    dt_s: float,
    wheel_radius_m: float,
    ticks_per_rev: int,
) -> float:
    dt = max(1e-6, float(dt_s))
    ticks = max(1, int(ticks_per_rev))
    revs = float(delta_ticks) / float(ticks)
    return (revs * 2.0 * math.pi * float(wheel_radius_m)) / dt


@dataclass(frozen=True)
class EncoderSpeedSample:
    left_ticks: int
    right_ticks: int
    host_time_s: float
    controller_millis: Optional[int] = None


@dataclass(frozen=True)
class EncoderSpeedEstimate:
    left_mps: float
    right_mps: float
    dt_s: float
    dt_source: str
    anomaly: Optional[str] = None


@dataclass(frozen=True)
class EncoderSpeedUpdate:
    estimate: Optional[EncoderSpeedEstimate]
    baseline_sample: EncoderSpeedSample
    skipped: bool
    accumulated_dt_s: float
    dt_source: Optional[str] = None
    anomaly: Optional[str] = None


def append_anomaly(existing: Optional[str], token: str) -> str:
    if not existing:
        return token
    parts = [part for part in str(existing).split(";") if part]
    if token not in parts:
        parts.append(token)
    return ";".join(parts)


def _encoder_dt_s(previous: EncoderSpeedSample, current: EncoderSpeedSample) -> Tuple[float, str, Optional[str]]:
    host_dt_s = float(current.host_time_s) - float(previous.host_time_s)
    if previous.controller_millis is not None and current.controller_millis is not None:
        controller_dt_ms = int(current.controller_millis) - int(previous.controller_millis)
        if controller_dt_ms > 0:
            return controller_dt_ms / 1000.0, "controller_millis", None
        anomaly = "controller_millis_nonpositive_dt"
        if host_dt_s > 0.0:
            return host_dt_s, "host_time_fallback", anomaly
        return 1e-6, "host_time_fallback", anomaly
    if host_dt_s > 0.0:
        return host_dt_s, "host_time", None
    return 1e-6, "host_time", "host_time_nonpositive_dt"


def estimate_encoder_wheel_speeds(
    previous: EncoderSpeedSample,
    current: EncoderSpeedSample,
    *,
    wheel_radius_m: float,
    ticks_per_rev: int,
    previous_left_mps: float = 0.0,
    previous_right_mps: float = 0.0,
    filter_alpha: float = 1.0,
    max_abs_speed_mps: float = 3.0,
) -> EncoderSpeedEstimate:
    dt_s, dt_source, anomaly = _encoder_dt_s(previous, current)
    left = encoder_delta_to_wheel_speed_mps(
        int(current.left_ticks) - int(previous.left_ticks),
        dt_s,
        wheel_radius_m,
        ticks_per_rev,
    )
    right = encoder_delta_to_wheel_speed_mps(
        int(current.right_ticks) - int(previous.right_ticks),
        dt_s,
        wheel_radius_m,
        ticks_per_rev,
    )

    max_speed = abs(float(max_abs_speed_mps))
    if max_speed > 0.0:
        clamped_left = clamp(left, -max_speed, max_speed)
        clamped_right = clamp(right, -max_speed, max_speed)
        if clamped_left != left or clamped_right != right:
            anomaly = append_anomaly(anomaly, "wheel_speed_sanity_clamped")
        left, right = clamped_left, clamped_right

    alpha = clamp(float(filter_alpha), 0.0, 1.0)
    left = float(previous_left_mps) + alpha * (left - float(previous_left_mps))
    right = float(previous_right_mps) + alpha * (right - float(previous_right_mps))
    return EncoderSpeedEstimate(
        left_mps=left,
        right_mps=right,
        dt_s=dt_s,
        dt_source=dt_source,
        anomaly=anomaly,
    )


def update_encoder_wheel_speed_estimate(
    baseline: Optional[EncoderSpeedSample],
    current: EncoderSpeedSample,
    *,
    wheel_radius_m: float,
    ticks_per_rev: int,
    previous_left_mps: float = 0.0,
    previous_right_mps: float = 0.0,
    filter_alpha: float = 1.0,
    max_abs_speed_mps: float = 3.0,
    min_host_dt_s: float = 0.015,
) -> EncoderSpeedUpdate:
    if baseline is None:
        return EncoderSpeedUpdate(
            estimate=None,
            baseline_sample=current,
            skipped=False,
            accumulated_dt_s=0.0,
            dt_source=None,
            anomaly=None,
        )

    dt_s, dt_source, anomaly = _encoder_dt_s(baseline, current)
    accumulated_dt_s = max(0.0, float(current.host_time_s) - float(baseline.host_time_s))
    if dt_source in {"host_time", "host_time_fallback"} and dt_s < max(0.0, float(min_host_dt_s)):
        return EncoderSpeedUpdate(
            estimate=None,
            baseline_sample=baseline,
            skipped=True,
            accumulated_dt_s=accumulated_dt_s,
            dt_source=dt_source,
            anomaly=append_anomaly(anomaly, "host_dt_too_small_skipped"),
        )

    estimate = estimate_encoder_wheel_speeds(
        baseline,
        current,
        wheel_radius_m=wheel_radius_m,
        ticks_per_rev=ticks_per_rev,
        previous_left_mps=previous_left_mps,
        previous_right_mps=previous_right_mps,
        filter_alpha=filter_alpha,
        max_abs_speed_mps=max_abs_speed_mps,
    )
    return EncoderSpeedUpdate(
        estimate=estimate,
        baseline_sample=current,
        skipped=False,
        accumulated_dt_s=dt_s,
        dt_source=estimate.dt_source,
        anomaly=estimate.anomaly,
    )


def encoder_speed_is_fresh(last_encoder_speed_time: float, now: float, timeout_s: float) -> bool:
    if float(last_encoder_speed_time) <= 0.0:
        return False
    return float(now) - float(last_encoder_speed_time) <= max(0.0, float(timeout_s))


def stale_encoder_control_mode(*, fallback_to_raw_without_encoder: bool) -> str:
    return "velocity_raw_fallback" if bool(fallback_to_raw_without_encoder) else "velocity_safe_neutral"


def command_age_s(last_command_time_s: float, now_s: float) -> Optional[float]:
    if float(last_command_time_s) <= 0.0:
        return None
    return max(0.0, float(now_s) - float(last_command_time_s))


def command_is_timed_out(last_command_time_s: float, now_s: float, timeout_s: float) -> bool:
    age = command_age_s(last_command_time_s, now_s)
    if age is None:
        return False
    return age > max(0.0, float(timeout_s))


def active_command_refresh_due(
    last_command_time_s: float,
    last_motor_send_time_s: float,
    now_s: float,
    *,
    timeout_s: float,
    refresh_period_s: float,
) -> bool:
    if float(last_command_time_s) <= 0.0:
        return False
    if command_is_timed_out(last_command_time_s, now_s, timeout_s):
        return False
    if float(last_motor_send_time_s) <= 0.0:
        return True
    return float(now_s) - float(last_motor_send_time_s) >= max(0.0, float(refresh_period_s))


@dataclass
class VelocityPidConfig:
    kp: float = 0.80
    ki: float = 0.0
    kd: float = 0.02
    integral_limit: float = 0.30
    feedforward_raw_per_mps: float = 1.35
    min_target_mps: float = 0.02
    max_target_mps: float = 0.60
    max_raw: float = 1.0


@dataclass
class VelocityPidResult:
    target_mps: float
    measured_mps: float
    error_mps: float
    p_raw: float
    i_raw: float
    d_raw: float
    feedforward_raw: float
    output_raw: float

    def as_dict(self) -> Dict[str, float]:
        return {
            "target_mps": round(self.target_mps, 4),
            "measured_mps": round(self.measured_mps, 4),
            "error_mps": round(self.error_mps, 4),
            "p_raw": round(self.p_raw, 4),
            "i_raw": round(self.i_raw, 4),
            "d_raw": round(self.d_raw, 4),
            "feedforward_raw": round(self.feedforward_raw, 4),
            "output_raw": round(self.output_raw, 4),
        }


class WheelVelocityPid:
    def __init__(self, config: VelocityPidConfig):
        self.config = config
        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, target_mps: float, measured_mps: float, dt_s: float) -> VelocityPidResult:
        cfg = self.config
        target = clamp(
            float(target_mps),
            -abs(float(cfg.max_target_mps)),
            abs(float(cfg.max_target_mps)),
        )
        if abs(target) < abs(float(cfg.min_target_mps)):
            target = 0.0

        measured = float(measured_mps)
        error = target - measured
        dt = max(1e-6, float(dt_s))
        self.integral = clamp(
            self.integral + error * dt,
            -abs(float(cfg.integral_limit)),
            abs(float(cfg.integral_limit)),
        )
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error

        p_raw = float(cfg.kp) * error
        i_raw = float(cfg.ki) * self.integral
        d_raw = float(cfg.kd) * derivative
        feedforward_raw = float(cfg.feedforward_raw_per_mps) * target
        output_raw = clamp(
            feedforward_raw + p_raw + i_raw + d_raw,
            -abs(float(cfg.max_raw)),
            abs(float(cfg.max_raw)),
        )
        if target == 0.0 and abs(measured) < abs(float(cfg.min_target_mps)):
            output_raw = 0.0
        return VelocityPidResult(
            target_mps=target,
            measured_mps=measured,
            error_mps=error,
            p_raw=p_raw,
            i_raw=i_raw,
            d_raw=d_raw,
            feedforward_raw=feedforward_raw,
            output_raw=output_raw,
        )


def reset_velocity_pid_pair(left_pid: WheelVelocityPid, right_pid: WheelVelocityPid) -> None:
    left_pid.reset()
    right_pid.reset()
