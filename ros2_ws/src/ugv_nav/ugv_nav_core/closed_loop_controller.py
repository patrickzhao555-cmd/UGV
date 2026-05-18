from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from ugv_nav_core.row_transition_controller import TRANSITION_SEGMENTS, RowTransitionController, RowTransitionRequest


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def finite_optional(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _cfg_float(nav_cfg: Any, name: str, default: float) -> float:
    return float(getattr(nav_cfg, name, default))


def _cfg_bool(nav_cfg: Any, name: str, default: bool = False) -> bool:
    return bool(getattr(nav_cfg, name, default))


def compute_heading_hold_correction(
    target_yaw_rad: float,
    estimated_yaw_rad: float,
    *,
    yaw_rate_radps: Optional[float] = None,
    kp: float = 1.10,
    kd: float = 0.05,
    deadband_deg: float = 3.0,
    max_omega_radps: float = 0.55,
    heading_error_sign: float = 1.0,
) -> Tuple[float, float]:
    sign = 1.0 if float(heading_error_sign) >= 0.0 else -1.0
    heading_error = sign * wrap_to_pi(float(target_yaw_rad) - float(estimated_yaw_rad))
    if abs(heading_error) <= math.radians(max(0.0, float(deadband_deg))):
        return heading_error, 0.0
    yaw_rate_error = 0.0
    if yaw_rate_radps is not None and math.isfinite(float(yaw_rate_radps)):
        yaw_rate_error = -float(yaw_rate_radps)
    omega = float(kp) * heading_error + float(kd) * yaw_rate_error
    return heading_error, clamp(omega, -abs(float(max_omega_radps)), abs(float(max_omega_radps)))


def compute_lane_follow_correction(
    lane_y_m: float,
    estimated_y_m: float,
    row_direction: float,
    *,
    kp_heading: float = 1.10,
    kp_omega: float = 1.00,
    deadband_m: float = 0.03,
    max_heading_deg: float = 18.0,
    max_omega_radps: float = 0.35,
    lane_error_sign: float = 1.0,
    lane_correction_sign: float = 1.0,
) -> Tuple[float, float, float]:
    error_sign = 1.0 if float(lane_error_sign) >= 0.0 else -1.0
    correction_sign = 1.0 if float(lane_correction_sign) >= 0.0 else -1.0
    cross_track_error = error_sign * (float(lane_y_m) - float(estimated_y_m))
    if abs(cross_track_error) <= max(0.0, float(deadband_m)):
        return cross_track_error, 0.0, 0.0
    direction = 1.0 if float(row_direction) >= 0.0 else -1.0
    max_heading = math.radians(max(0.0, float(max_heading_deg)))
    desired_heading_offset = correction_sign * direction * clamp(
        math.atan(float(kp_heading) * cross_track_error),
        -max_heading,
        max_heading,
    )
    omega = clamp(
        float(kp_omega) * desired_heading_offset,
        -abs(float(max_omega_radps)),
        abs(float(max_omega_radps)),
    )
    return cross_track_error, desired_heading_offset, omega


def wheel_targets_from_v_omega(v_mps: float, omega_radps: float, track_width_m: float) -> Tuple[float, float]:
    half_track = 0.5 * max(1e-6, float(track_width_m))
    left_target_mps = float(v_mps) - float(omega_radps) * half_track
    right_target_mps = float(v_mps) + float(omega_radps) * half_track
    return left_target_mps, right_target_mps


def interpolate_by_speed(
    speed_mps: float,
    low_speed_mps: float,
    high_speed_mps: float,
    low_value: float,
    high_value: float,
) -> float:
    low_speed = float(low_speed_mps)
    high_speed = float(high_speed_mps)
    if high_speed <= low_speed + 1e-9:
        return float(high_value)
    t = clamp((abs(float(speed_mps)) - low_speed) / (high_speed - low_speed), 0.0, 1.0)
    return float(low_value) + t * (float(high_value) - float(low_value))


def scheduled_row_follower_gains(nav_cfg: Any, speed_mps: float) -> Tuple[float, float, bool]:
    enabled = _cfg_bool(nav_cfg, "row_follower_speed_schedule_enabled", True)
    if not enabled:
        return (
            _cfg_float(nav_cfg, "lane_follow_kp_heading", 1.10),
            _cfg_float(nav_cfg, "heading_hold_kp", 1.10),
            False,
        )
    low_speed = _cfg_float(nav_cfg, "row_follower_low_speed_mps", 0.09)
    high_speed = _cfg_float(nav_cfg, "row_follower_high_speed_mps", 0.22)
    lane_kp = interpolate_by_speed(
        speed_mps,
        low_speed,
        high_speed,
        _cfg_float(nav_cfg, "row_follower_low_speed_lane_kp", _cfg_float(nav_cfg, "lane_follow_kp_heading", 1.10)),
        _cfg_float(nav_cfg, "row_follower_high_speed_lane_kp", _cfg_float(nav_cfg, "lane_follow_kp_heading", 1.10)),
    )
    heading_kp = interpolate_by_speed(
        speed_mps,
        low_speed,
        high_speed,
        _cfg_float(nav_cfg, "row_follower_low_speed_heading_kp", _cfg_float(nav_cfg, "heading_hold_kp", 1.10)),
        _cfg_float(nav_cfg, "row_follower_high_speed_heading_kp", _cfg_float(nav_cfg, "heading_hold_kp", 1.10)),
    )
    return lane_kp, heading_kp, True


@dataclass
class RowFollowerOmegaState:
    last_omega_radps: float = 0.0
    last_timestamp_s: Optional[float] = None

    def reset(self) -> None:
        self.last_omega_radps = 0.0
        self.last_timestamp_s = None


def smooth_row_follower_omega(
    raw_omega_radps: float,
    timestamp_s: float,
    state: RowFollowerOmegaState,
    *,
    low_pass_alpha: float = 0.35,
    rate_limit_rps2: float = 0.6,
    min_interval_s: float = 0.0,
) -> Tuple[float, bool, float]:
    alpha = clamp(float(low_pass_alpha), 0.0, 1.0)
    rate_limit = max(0.0, float(rate_limit_rps2))
    timestamp = float(timestamp_s)
    raw_omega = float(raw_omega_radps)
    if state.last_timestamp_s is None:
        state.last_timestamp_s = timestamp
        state.last_omega_radps = raw_omega
        return raw_omega, False, 0.0

    dt = max(0.0, timestamp - state.last_timestamp_s)
    min_interval = max(0.0, float(min_interval_s))
    if min_interval > 0.0 and dt < min_interval:
        return state.last_omega_radps, True, dt

    low_passed = (1.0 - alpha) * state.last_omega_radps + alpha * raw_omega
    clamped = False
    if rate_limit > 0.0 and dt > 1e-6:
        max_delta = rate_limit * dt
        limited = clamp(low_passed, state.last_omega_radps - max_delta, state.last_omega_radps + max_delta)
        clamped = abs(limited - low_passed) > 1e-9
        low_passed = limited
    state.last_timestamp_s = timestamp
    state.last_omega_radps = low_passed
    return low_passed, clamped, dt


def apply_forward_arc_only_limit(
    v_mps: float,
    omega_radps: float,
    track_width_m: float,
    *,
    enabled: bool = True,
    margin: float = 0.75,
    min_v_mps: float = 0.08,
) -> Tuple[float, float, Optional[float], bool, float, float]:
    final_v = float(v_mps)
    final_omega = float(omega_radps)
    if not enabled:
        left, right = wheel_targets_from_v_omega(final_v, final_omega, track_width_m)
        return final_v, final_omega, None, False, left, right

    final_v = max(final_v, max(0.0, float(min_v_mps)))
    track_width = max(1e-6, float(track_width_m))
    forward_margin = clamp(float(margin), 0.0, 1.0)
    omega_limit = 2.0 * final_v / track_width * forward_margin
    clamped_omega = clamp(final_omega, -omega_limit, omega_limit)
    clamped = abs(clamped_omega - final_omega) > 1e-9
    left, right = wheel_targets_from_v_omega(final_v, clamped_omega, track_width)
    return final_v, clamped_omega, omega_limit, clamped, left, right


@dataclass
class ClosedLoopHealthSample:
    timestamp_s: float
    cross_track_error_m: float
    heading_error_deg: float
    final_omega_radps: float
    estimated_yaw_deg: float
    pose_x_m: float
    pose_y_m: float
    odom_ds_m: float
    odom_dtheta_deg: float
    command_v_mps: float
    command_omega_radps: float


@dataclass
class ClosedLoopHealthState:
    samples: list[ClosedLoopHealthSample] = field(default_factory=list)
    cross_track_growth_count: int = 0
    heading_growth_count: int = 0
    no_motion_count: int = 0
    divergence_persist_count: int = 0
    diverging: bool = False
    divergence_reason: str = ""
    cross_track_error_trend: float = 0.0
    heading_error_trend: float = 0.0
    last_error_abs: Optional[float] = None
    current_error_abs: Optional[float] = None

    def reset(self) -> None:
        self.samples.clear()
        self.cross_track_growth_count = 0
        self.heading_growth_count = 0
        self.no_motion_count = 0
        self.divergence_persist_count = 0
        self.diverging = False
        self.divergence_reason = ""
        self.cross_track_error_trend = 0.0
        self.heading_error_trend = 0.0
        self.last_error_abs = None
        self.current_error_abs = None


def _sign_value(value: float) -> int:
    if value > 1e-9:
        return 1
    if value < -1e-9:
        return -1
    return 0


def closed_loop_correction_sign_ok(
    *,
    cross_track_error_m: float,
    omega_lane_radps: float,
    heading_error_rad: float,
    omega_heading_radps: float,
    row_direction: float,
    lane_active: bool,
    heading_active: bool,
    min_error_m: float,
    heading_deadband_deg: float,
) -> bool:
    checks = []
    if lane_active and abs(cross_track_error_m) >= max(0.0, float(min_error_m)):
        expected_lane_sign = _sign_value(float(row_direction) * float(cross_track_error_m))
        lane_sign = _sign_value(float(omega_lane_radps))
        if expected_lane_sign and lane_sign:
            checks.append(expected_lane_sign == lane_sign)
    if heading_active and abs(math.degrees(float(heading_error_rad))) >= max(0.0, float(heading_deadband_deg)):
        heading_sign = _sign_value(float(heading_error_rad))
        omega_sign = _sign_value(float(omega_heading_radps))
        if heading_sign and omega_sign:
            checks.append(heading_sign == omega_sign)
    return all(checks) if checks else True


def update_closed_loop_health(
    state: ClosedLoopHealthState,
    *,
    enabled: bool,
    sample: ClosedLoopHealthSample,
    lane_active: bool,
    heading_active: bool,
    correction_sign_ok: bool,
    window: int = 5,
    min_error_m: float = 0.08,
    max_growth_m: float = 0.05,
) -> Dict[str, Any]:
    if not enabled:
        state.reset()
        return {
            "closed_loop_diverging": False,
            "divergence_reason": "health_disabled",
            "cross_track_error_trend": 0.0,
            "heading_error_trend": 0.0,
            "last_error_abs": None,
            "current_error_abs": None,
        }

    window = max(2, int(window))
    min_error_m = max(0.0, float(min_error_m))
    max_growth_m = max(0.0, float(max_growth_m))
    abs_cte = abs(float(sample.cross_track_error_m))
    abs_heading = abs(float(sample.heading_error_deg))
    prev = state.samples[-1] if state.samples else None
    state.last_error_abs = None if prev is None else abs(prev.cross_track_error_m)
    state.current_error_abs = abs_cte

    if prev is not None:
        if lane_active and abs_cte > abs(prev.cross_track_error_m) + 1e-4:
            state.cross_track_growth_count += 1
        else:
            state.cross_track_growth_count = 0
        if heading_active and abs_heading > abs(prev.heading_error_deg) + 0.25:
            state.heading_growth_count += 1
        else:
            state.heading_growth_count = 0
        pose_delta = math.hypot(sample.pose_x_m - prev.pose_x_m, sample.pose_y_m - prev.pose_y_m)
        commanded_motion = abs(sample.command_v_mps) >= 0.02 or abs(sample.command_omega_radps) >= 0.08
        odom_motion = abs(sample.odom_ds_m) >= 0.003 or abs(sample.odom_dtheta_deg) >= 0.5 or pose_delta >= 0.003
        if commanded_motion and not odom_motion:
            state.no_motion_count += 1
        else:
            state.no_motion_count = 0
    else:
        state.cross_track_growth_count = 0
        state.heading_growth_count = 0
        state.no_motion_count = 0

    state.samples.append(sample)
    if len(state.samples) > window:
        state.samples = state.samples[-window:]
    oldest = state.samples[0]
    state.cross_track_error_trend = abs_cte - abs(oldest.cross_track_error_m)
    state.heading_error_trend = abs_heading - abs(oldest.heading_error_deg)

    heading_growth_limit_deg = max(2.0, max_growth_m * 50.0)
    cross_track_diverging = (
        lane_active
        and abs_cte >= min_error_m
        and state.cross_track_growth_count >= window - 1
        and state.cross_track_error_trend >= max_growth_m
    )
    heading_diverging = (
        heading_active
        and state.heading_growth_count >= window - 1
        and state.heading_error_trend >= heading_growth_limit_deg
    )
    no_motion_diverging = state.no_motion_count >= window - 1
    sign_diverging = not correction_sign_ok and (abs_cte >= min_error_m or abs_heading >= heading_growth_limit_deg)

    reasons = []
    if cross_track_diverging:
        reasons.append("cross_track_error_growing")
    if heading_diverging:
        reasons.append("heading_error_growing")
    if sign_diverging:
        reasons.append("correction_sign_mismatch")
    if no_motion_diverging:
        reasons.append("commanded_motion_zero_odom")
    state.diverging = bool(reasons)
    state.divergence_reason = "_".join(reasons)
    if state.diverging:
        state.divergence_persist_count += 1
    else:
        state.divergence_persist_count = 0

    return {
        "closed_loop_diverging": state.diverging,
        "divergence_reason": state.divergence_reason,
        "cross_track_error_trend": round(state.cross_track_error_trend, 4),
        "heading_error_trend": round(state.heading_error_trend, 3),
        "last_error_abs": None if state.last_error_abs is None else round(state.last_error_abs, 4),
        "current_error_abs": round(state.current_error_abs, 4),
    }


def default_closed_loop_debug(nav_cfg: Optional[Any] = None) -> Dict[str, Any]:
    return {
        "closed_loop_enabled": _cfg_bool(nav_cfg, "competition_closed_loop_enabled") if nav_cfg is not None else False,
        "closed_loop_active": False,
        "heading_hold_enabled": _cfg_bool(nav_cfg, "heading_hold_enabled") if nav_cfg is not None else False,
        "lane_follow_enabled": _cfg_bool(nav_cfg, "lane_follow_enabled") if nav_cfg is not None else False,
        "heading_hold_kp": _cfg_float(nav_cfg, "heading_hold_kp", 1.10) if nav_cfg is not None else 1.10,
        "heading_hold_kd": _cfg_float(nav_cfg, "heading_hold_kd", 0.05) if nav_cfg is not None else 0.05,
        "heading_hold_deadband_deg": _cfg_float(nav_cfg, "heading_hold_deadband_deg", 3.0) if nav_cfg is not None else 3.0,
        "heading_hold_max_omega_rps": _cfg_float(nav_cfg, "heading_hold_max_omega_rps", 0.55) if nav_cfg is not None else 0.55,
        "lane_follow_kp_heading": _cfg_float(nav_cfg, "lane_follow_kp_heading", 1.10) if nav_cfg is not None else 1.10,
        "lane_follow_kp_omega": _cfg_float(nav_cfg, "lane_follow_kp_omega", 1.00) if nav_cfg is not None else 1.00,
        "lane_follow_deadband_m": _cfg_float(nav_cfg, "lane_follow_deadband_m", 0.03) if nav_cfg is not None else 0.03,
        "lane_follow_max_heading_deg": _cfg_float(nav_cfg, "lane_follow_max_heading_deg", 18.0) if nav_cfg is not None else 18.0,
        "lane_follow_max_omega_rps": _cfg_float(nav_cfg, "lane_follow_max_omega_rps", 0.35) if nav_cfg is not None else 0.35,
        "row_follower_speed_schedule_enabled": _cfg_bool(nav_cfg, "row_follower_speed_schedule_enabled", True) if nav_cfg is not None else True,
        "row_follower_low_speed_mps": _cfg_float(nav_cfg, "row_follower_low_speed_mps", 0.09) if nav_cfg is not None else 0.09,
        "row_follower_high_speed_mps": _cfg_float(nav_cfg, "row_follower_high_speed_mps", 0.22) if nav_cfg is not None else 0.22,
        "row_follower_low_speed_lane_kp": _cfg_float(nav_cfg, "row_follower_low_speed_lane_kp", 0.85) if nav_cfg is not None else 0.85,
        "row_follower_high_speed_lane_kp": _cfg_float(nav_cfg, "row_follower_high_speed_lane_kp", 1.00) if nav_cfg is not None else 1.00,
        "row_follower_low_speed_heading_kp": _cfg_float(nav_cfg, "row_follower_low_speed_heading_kp", 0.95) if nav_cfg is not None else 0.95,
        "row_follower_high_speed_heading_kp": _cfg_float(nav_cfg, "row_follower_high_speed_heading_kp", 1.10) if nav_cfg is not None else 1.10,
        "row_follower_omega_low_pass_alpha": _cfg_float(nav_cfg, "row_follower_omega_low_pass_alpha", 0.35) if nav_cfg is not None else 0.35,
        "row_follower_omega_rate_limit_rps2": _cfg_float(nav_cfg, "row_follower_omega_rate_limit_rps2", 0.6) if nav_cfg is not None else 0.6,
        "row_follower_min_correction_interval_s": _cfg_float(nav_cfg, "row_follower_min_correction_interval_s", 0.0) if nav_cfg is not None else 0.0,
        "row_follower_scheduled_lane_kp": _cfg_float(nav_cfg, "lane_follow_kp_heading", 1.10) if nav_cfg is not None else 1.10,
        "row_follower_scheduled_heading_kp": _cfg_float(nav_cfg, "heading_hold_kp", 1.10) if nav_cfg is not None else 1.10,
        "row_follower_raw_omega_radps": 0.0,
        "row_follower_smoothed_omega_radps": 0.0,
        "row_follower_rate_limited": False,
        "row_follower_update_dt_s": None,
        "sweep_planner_omega_weight": 0.0,
        "planner_omega_radps": 0.0,
        "planner_omega_contribution_radps": 0.0,
        "planner_gap_ignored": False,
        "planner_gap_reason": "",
        "sweep_subphase": "",
        "row_transition_active": False,
        "row_transition_style": "",
        "row_transition_segment": "",
        "row_transition_progress": 0.0,
        "row_transition_done": False,
        "row_transition_reason": "",
        "row_transition_health": "",
        "row_transition_diverging": False,
        "row_transition_timeout": False,
        "row_transition_geometry_warning": "",
        "row_transition_target_m": None,
        "turn_capture_zone": None,
        "turn_settle_count": None,
        "turn_yaw_error_sign_changed": False,
        "turn_overshoot_deg": None,
        "turn_effective_target_yaw_deg": None,
        "turn_command_phase": None,
        "turn_radius_m": None,
        "turn_side": "",
        "target_side_yaw_deg": None,
        "target_next_row_yaw_deg": None,
        "target_row_yaw_deg": None,
        "segment_yaw_error_deg": None,
        "segment_lane_error_m": None,
        "yaw_capture_error_deg": None,
        "lane_capture_error_m": None,
        "transition_emitted_v_mps": None,
        "transition_emitted_omega_radps": None,
        "transition_left_target_mps": None,
        "transition_right_target_mps": None,
        "active_robot_track_width_m": None,
        "forward_arc_only_enabled": _cfg_bool(nav_cfg, "forward_arc_only_enabled", True) if nav_cfg is not None else True,
        "forward_arc_margin": _cfg_float(nav_cfg, "forward_arc_margin", 0.75) if nav_cfg is not None else 0.75,
        "min_sweep_v_mps": _cfg_float(nav_cfg, "min_sweep_v_mps", 0.08) if nav_cfg is not None else 0.08,
        "forward_arc_omega_limit_radps": None,
        "forward_arc_clamped": False,
        "final_left_target_mps": 0.0,
        "final_right_target_mps": 0.0,
        "closed_loop_health_enabled": _cfg_bool(nav_cfg, "closed_loop_health_enabled", True) if nav_cfg is not None else True,
        "closed_loop_diverging": False,
        "divergence_reason": "",
        "cross_track_error_trend": 0.0,
        "heading_error_trend": 0.0,
        "correction_sign_ok": True,
        "omega_command_sign": _cfg_float(nav_cfg, "omega_command_sign", 1.0) if nav_cfg is not None else 1.0,
        "heading_error_sign": _cfg_float(nav_cfg, "heading_error_sign", 1.0) if nav_cfg is not None else 1.0,
        "lane_error_sign": _cfg_float(nav_cfg, "lane_error_sign", 1.0) if nav_cfg is not None else 1.0,
        "lane_correction_sign": _cfg_float(nav_cfg, "lane_correction_sign", 1.0) if nav_cfg is not None else 1.0,
        "last_error_abs": None,
        "current_error_abs": None,
        "divergence_action": str(getattr(nav_cfg, "closed_loop_divergence_action", "slow_then_stop")) if nav_cfg is not None else "slow_then_stop",
        "target_yaw_deg": None,
        "estimated_yaw_deg": None,
        "heading_error_deg": None,
        "cross_track_error_m": None,
        "omega_heading_radps": 0.0,
        "omega_lane_radps": 0.0,
        "lane_heading_offset_deg": 0.0,
        "final_v_mps": 0.0,
        "final_omega_radps": 0.0,
        "heading_source": "unavailable",
        "reason": "inactive",
    }


def competition_sweep_row_direction(v2: Dict[str, Any]) -> Optional[float]:
    direction = finite_optional(v2.get("sweep_row_direction"))
    if direction is not None:
        return 1.0 if direction >= 0.0 else -1.0
    row = v2.get("active_cell_row")
    try:
        return 1.0 if int(row) % 2 == 0 else -1.0
    except (TypeError, ValueError):
        return None


def competition_heading_source_and_rate(navigator: Any, frame: Any) -> Tuple[str, Optional[float]]:
    nav_cfg = navigator.nav_cfg
    if not _cfg_bool(nav_cfg, "use_imu_yaw") or getattr(frame, "imu", None) is None:
        return "odom_yaw", None
    yaw_rate = frame.imu.yaw_rate(nav_cfg.imu_yaw_axis, nav_cfg.imu_yaw_sign)
    if math.isfinite(yaw_rate) and abs(yaw_rate) <= nav_cfg.imu_yaw_max_rate_rps:
        return "odom_yaw_imu_rate", yaw_rate
    return "odom_yaw_imu_unhealthy", None


def apply_competition_closed_loop_command(
    navigator: Any,
    cmd: Any,
    frame: Any,
    mission_status: dict,
) -> Any:
    nav_cfg = navigator.nav_cfg
    debug = default_closed_loop_debug(nav_cfg)
    v2 = mission_status.get("competition_v2") if isinstance(mission_status, dict) else None
    if not isinstance(v2, dict):
        debug["reason"] = "competition_v2_status_missing"
        navigator.closed_loop_debug = debug
        return cmd

    phase = str(v2.get("phase") or "")
    debug["closed_loop_enabled"] = bool(nav_cfg.competition_closed_loop_enabled and v2.get("enabled", False))
    if not debug["closed_loop_enabled"]:
        debug["reason"] = "disabled"
        navigator.closed_loop_debug = debug
        return cmd
    if phase != "sweep_search":
        debug["reason"] = f"phase_{phase or 'unknown'}"
        navigator.closed_loop_debug = debug
        return cmd
    if cmd.mode == "STOP":
        debug["reason"] = "stop_command"
        navigator.closed_loop_debug = debug
        return cmd
    if cmd.controller == "active_scan":
        debug["reason"] = "active_scan_command"
        navigator.closed_loop_debug = debug
        return cmd
    velocity_debug = navigator.velocity_debug or {}
    safety_state = str(velocity_debug.get("safety_state") or "")
    sweep_subphase = str(v2.get("sweep_subphase") or "")
    transition_active = bool(v2.get("row_transition_active", False)) or sweep_subphase in TRANSITION_SEGMENTS
    debug["sweep_subphase"] = sweep_subphase
    debug["row_transition_active"] = bool(transition_active)
    debug["active_robot_track_width_m"] = round(float(getattr(navigator.robot_cfg, "track_width_m", 0.0)), 4)
    if navigator.physical_stall.detected:
        if transition_active:
            debug.update(
                {
                    "closed_loop_active": True,
                    "row_transition_reason": "row_transition_physical_stall",
                    "row_transition_health": "physical_stall",
                    "reason": "row_transition_physical_stall",
                }
            )
            navigator.closed_loop_debug = debug
            return cmd.__class__(
                "STOP",
                reason="row transition physical stall",
                controller="velocity",
                command_type="stop",
            )
        debug["reason"] = "physical_stall_recovery"
        navigator.closed_loop_debug = debug
        return cmd
    if safety_state in {"stop", "no_safe_trajectory"}:
        if transition_active:
            debug.update(
                {
                    "row_transition_reason": f"safety_{safety_state}",
                    "reason": f"row_transition_safety_{safety_state}",
                }
            )
            navigator.closed_loop_debug = debug
            return cmd.__class__(
                "STOP",
                reason=f"row transition safety stop: {safety_state}",
                controller="velocity",
                command_type="stop",
            )
        debug["reason"] = f"safety_{safety_state}"
        navigator.closed_loop_debug = debug
        return cmd

    if transition_active:
        current_lane_y = finite_optional(v2.get("current_lane_y_m"))
        next_lane_y = finite_optional(v2.get("next_lane_y_m"))
        row_end_x = finite_optional(v2.get("row_end_x_m"))
        row_direction = competition_sweep_row_direction(v2)
        if current_lane_y is None or next_lane_y is None or row_end_x is None or row_direction is None:
            debug["reason"] = "row_transition_context_missing"
            navigator.closed_loop_debug = debug
            return cmd
        pose = navigator.state.estimated_pose
        heading_source, yaw_rate = competition_heading_source_and_rate(navigator, frame)
        sectors = navigator.state.sectors
        stop_clearance = max(
            _cfg_float(nav_cfg, "continuous_stop_clearance_m", 0.48),
            0.5 * float(getattr(navigator.robot_cfg, "length_m", 0.762)) + _cfg_float(nav_cfg, "front_safety_margin_m", 0.08),
        )
        forward_corridor_safe = not math.isfinite(float(sectors.front_m)) or float(sectors.front_m) > stop_clearance
        controller = getattr(navigator, "row_transition_controller", None)
        if controller is None:
            controller = RowTransitionController()
            navigator.row_transition_controller = controller
        drive_speed_factor = finite_optional(mission_status.get("drive_speed_factor")) or finite_optional(v2.get("drive_speed_factor")) or 1.0
        request = RowTransitionRequest(
            segment=sweep_subphase,
            pose_x_m=pose.x,
            pose_y_m=pose.y,
            pose_yaw_rad=pose.yaw,
            row_direction=row_direction,
            current_lane_y_m=current_lane_y,
            next_lane_y_m=next_lane_y,
            row_end_x_m=row_end_x,
            track_width_m=navigator.robot_cfg.track_width_m,
            drive_speed_factor=drive_speed_factor,
            yaw_rate_radps=yaw_rate,
            min_emitted_v_mps=finite_optional(v2.get("sweep_transition_min_emitted_v_mps")) or 0.12,
            min_emitted_omega_radps=finite_optional(v2.get("sweep_transition_min_emitted_omega_rps")) or 0.20,
            max_emitted_omega_radps=finite_optional(v2.get("sweep_transition_max_emitted_omega_rps")) or 0.40,
            inner_wheel_min_mps=finite_optional(v2.get("sweep_transition_inner_wheel_min_mps")) or 0.04,
            max_v_mps=nav_cfg.continuous_max_speed_mps,
            turn_90_yaw_tolerance_rad=math.radians(finite_optional(v2.get("sweep_turn_90_yaw_tolerance_deg")) or 7.0),
            turn_strong_error_rad=math.radians(finite_optional(v2.get("sweep_turn_strong_error_deg")) or 30.0),
            turn_capture_error_rad=math.radians(finite_optional(v2.get("sweep_turn_capture_error_deg")) or 12.0),
            turn_settle_error_rad=math.radians(finite_optional(v2.get("sweep_turn_settle_error_deg")) or 7.0),
            turn_settle_frames=int(finite_optional(v2.get("sweep_turn_settle_frames")) or 3),
            lane_capture_tolerance_m=finite_optional(v2.get("sweep_lane_capture_tolerance_m")) or 0.20,
            yaw_capture_tolerance_rad=math.radians(finite_optional(v2.get("sweep_yaw_capture_tolerance_deg")) or 8.0),
            forward_corridor_safe=forward_corridor_safe,
        )
        transition_cmd = controller.compute(request)
        final_v = transition_cmd.desired_v_mps
        final_omega = transition_cmd.desired_omega_radps
        left_target = finite_optional(transition_cmd.debug.get("desired_left_target_mps"))
        right_target = finite_optional(transition_cmd.debug.get("desired_right_target_mps"))
        if left_target is None or right_target is None:
            left_target, right_target = wheel_targets_from_v_omega(final_v, final_omega, navigator.robot_cfg.track_width_m)
        trajectory_collision = False
        try:
            local_map = navigator._local_safety_costmap()
            _end, local_states = navigator.velocity_planner._simulate(
                pose,
                final_v,
                final_omega,
                nav_cfg.continuous_horizon_s,
                nav_cfg.continuous_dt_s,
            )
            trajectory_collision = bool(navigator.velocity_planner._trajectory_in_collision(local_map, pose, local_states))
        except Exception:
            trajectory_collision = False
        if trajectory_collision or not forward_corridor_safe:
            stop_reason = "local_costmap_collision" if trajectory_collision else "forward_corridor_unsafe"
            debug.update(
                {
                    "closed_loop_active": True,
                    "row_transition_reason": stop_reason,
                    "reason": f"row_transition_safety_{stop_reason}",
                    **transition_cmd.debug,
                }
            )
            navigator.closed_loop_debug = debug
            return cmd.__class__(
                "STOP",
                reason=f"row transition safety stop: {stop_reason}",
                controller="velocity",
                command_type="stop",
            )
        adjusted = navigator.velocity_planner._velocity_to_command(
            final_v,
            final_omega,
            f"{cmd.reason}; competition closed-loop row transition",
            nav_cfg.continuous_horizon_s,
        )
        navigator.velocity_planner.last_v = final_v
        navigator.velocity_planner.last_omega = final_omega
        navigator.velocity_planner.last_timestamp = frame.encoder.timestamp
        navigator.state.latest_cmd = adjusted
        transition_target_yaw_deg = transition_cmd.debug.get("target_next_row_yaw_deg")
        if sweep_subphase in {"turn_out_90", "cross_lane"}:
            transition_target_yaw_deg = transition_cmd.debug.get("target_side_yaw_deg", transition_target_yaw_deg)
        debug.update(
            {
                "closed_loop_active": True,
                "heading_source": heading_source,
                "target_yaw_deg": transition_target_yaw_deg,
                "target_row_yaw_deg": transition_cmd.debug.get("target_row_yaw_deg"),
                "estimated_yaw_deg": round(math.degrees(pose.yaw), 2),
                "heading_error_deg": transition_cmd.debug.get("segment_yaw_error_deg"),
                "cross_track_error_m": transition_cmd.debug.get("segment_lane_error_m"),
                "omega_heading_radps": 0.0,
                "omega_lane_radps": 0.0,
                "lane_heading_offset_deg": 0.0,
                "planner_omega_radps": float(cmd.omega_radps) if math.isfinite(float(cmd.omega_radps)) else 0.0,
                "planner_omega_contribution_radps": 0.0,
                "planner_gap_ignored": True,
                "planner_gap_reason": "row_transition_controller_authority",
                "final_v_mps": round(final_v, 4),
                "final_omega_radps": round(final_omega, 4),
                "forward_arc_omega_limit_radps": None,
                "forward_arc_clamped": False,
                "final_left_target_mps": round(left_target, 4),
                "final_right_target_mps": round(right_target, 4),
                "reason": "row_transition_closed_loop",
                **transition_cmd.debug,
            }
        )
        navigator.closed_loop_debug = debug
        return adjusted

    lane_y = finite_optional(v2.get("active_lane_y_m"))
    row_direction = competition_sweep_row_direction(v2)
    if lane_y is None or row_direction is None:
        debug["reason"] = "lane_context_missing"
        navigator.closed_loop_debug = debug
        return cmd

    pose = navigator.state.estimated_pose
    target_yaw = 0.0 if row_direction > 0.0 else math.pi
    status_target_yaw_deg = finite_optional(v2.get("sweep_target_yaw_deg"))
    if status_target_yaw_deg is not None:
        target_yaw = math.radians(status_target_yaw_deg)
    heading_source, yaw_rate = competition_heading_source_and_rate(navigator, frame)

    min_v = max(
        nav_cfg.continuous_min_speed_mps,
        finite_optional(v2.get("minimum_speed_mps")) or 0.0,
        0.02,
    )
    base_v = abs(float(cmd.v_mps))
    if base_v <= 1e-6 and cmd.mode in {"FORWARD", "BACKWARD"}:
        base_v = abs(float(cmd.move_m)) / max(0.05, nav_cfg.continuous_dt_s)
    final_v = clamp(max(base_v, min_v, _cfg_float(nav_cfg, "min_sweep_v_mps", 0.08)), 0.0, nav_cfg.continuous_max_speed_mps)
    scheduled_lane_kp, scheduled_heading_kp, speed_schedule_active = scheduled_row_follower_gains(nav_cfg, final_v)

    heading_error, omega_heading = compute_heading_hold_correction(
        target_yaw,
        pose.yaw,
        yaw_rate_radps=yaw_rate,
        kp=scheduled_heading_kp,
        kd=nav_cfg.heading_hold_kd,
        deadband_deg=nav_cfg.heading_hold_deadband_deg,
        max_omega_radps=nav_cfg.heading_hold_max_omega_rps,
        heading_error_sign=nav_cfg.heading_error_sign,
    )
    if not nav_cfg.heading_hold_enabled:
        omega_heading = 0.0

    cross_track_error, lane_heading_offset, omega_lane = compute_lane_follow_correction(
        lane_y,
        pose.y,
        row_direction,
        kp_heading=scheduled_lane_kp,
        kp_omega=nav_cfg.lane_follow_kp_omega,
        deadband_m=nav_cfg.lane_follow_deadband_m,
        max_heading_deg=nav_cfg.lane_follow_max_heading_deg,
        max_omega_radps=nav_cfg.lane_follow_max_omega_rps,
        lane_error_sign=nav_cfg.lane_error_sign,
        lane_correction_sign=nav_cfg.lane_correction_sign,
    )
    if not nav_cfg.lane_follow_enabled:
        lane_heading_offset = 0.0
        omega_lane = 0.0
    correction_sign_ok = closed_loop_correction_sign_ok(
        cross_track_error_m=cross_track_error,
        omega_lane_radps=omega_lane,
        heading_error_rad=heading_error,
        omega_heading_radps=omega_heading,
        row_direction=row_direction,
        lane_active=bool(nav_cfg.lane_follow_enabled),
        heading_active=bool(nav_cfg.heading_hold_enabled),
        min_error_m=nav_cfg.closed_loop_divergence_min_error_m,
        heading_deadband_deg=nav_cfg.heading_hold_deadband_deg,
    )

    row_raw_omega = omega_heading + omega_lane
    row_state = getattr(navigator, "row_follower_omega_state", None)
    if row_state is None:
        row_state = RowFollowerOmegaState()
        navigator.row_follower_omega_state = row_state
    row_omega, row_rate_limited, row_update_dt_s = smooth_row_follower_omega(
        row_raw_omega,
        float(frame.encoder.timestamp),
        row_state,
        low_pass_alpha=_cfg_float(nav_cfg, "row_follower_omega_low_pass_alpha", 0.35),
        rate_limit_rps2=_cfg_float(nav_cfg, "row_follower_omega_rate_limit_rps2", 0.6),
        min_interval_s=_cfg_float(nav_cfg, "row_follower_min_correction_interval_s", 0.0),
    )
    round_name = str(v2.get("round") or v2.get("mode") or mission_status.get("mode") or "").strip().lower()
    if round_name == "round3":
        planner_omega_weight = _cfg_float(nav_cfg, "sweep_planner_omega_weight_round3", 0.2)
    else:
        planner_omega_weight = _cfg_float(nav_cfg, "sweep_planner_omega_weight_round2", 0.0)
    planner_omega_weight = clamp(planner_omega_weight, 0.0, 1.0)
    planner_omega = float(cmd.omega_radps) if math.isfinite(float(cmd.omega_radps)) else 0.0
    planner_omega_contribution = planner_omega_weight * planner_omega
    planner_gap_ignored = bool(round_name == "round2" and planner_omega_weight <= 1e-9)
    planner_gap_reason = (
        "local_planner_gap_omega_ignored_round2_row_follower_authority"
        if planner_gap_ignored
        else ""
    )
    final_omega = clamp(
        planner_omega_contribution + row_omega,
        -nav_cfg.continuous_max_omega_rps,
        nav_cfg.continuous_max_omega_rps,
    )
    final_omega *= 1.0 if nav_cfg.omega_command_sign >= 0.0 else -1.0
    final_v, final_omega, forward_arc_limit, forward_arc_clamped, left_target, right_target = apply_forward_arc_only_limit(
        final_v,
        final_omega,
        navigator.robot_cfg.track_width_m,
        enabled=bool(nav_cfg.forward_arc_only_enabled),
        margin=nav_cfg.forward_arc_margin,
        min_v_mps=min(nav_cfg.continuous_max_speed_mps, max(nav_cfg.min_sweep_v_mps, min_v)),
    )
    health_state = getattr(navigator, "closed_loop_health", None)
    if health_state is None:
        health_state = ClosedLoopHealthState()
        navigator.closed_loop_health = health_state
    odom_delta = getattr(navigator, "last_odom_delta", {}) or {}
    health = update_closed_loop_health(
        health_state,
        enabled=bool(nav_cfg.closed_loop_health_enabled),
        sample=ClosedLoopHealthSample(
            timestamp_s=float(frame.encoder.timestamp),
            cross_track_error_m=float(cross_track_error),
            heading_error_deg=math.degrees(float(heading_error)),
            final_omega_radps=float(final_omega),
            estimated_yaw_deg=math.degrees(float(pose.yaw)),
            pose_x_m=float(pose.x),
            pose_y_m=float(pose.y),
            odom_ds_m=float(odom_delta.get("ds_used_m", odom_delta.get("ds_m", 0.0)) or 0.0),
            odom_dtheta_deg=float(odom_delta.get("dtheta_deg") or 0.0),
            command_v_mps=float(final_v),
            command_omega_radps=float(final_omega),
        ),
        lane_active=bool(nav_cfg.lane_follow_enabled),
        heading_active=bool(nav_cfg.heading_hold_enabled),
        correction_sign_ok=bool(correction_sign_ok),
        window=nav_cfg.closed_loop_divergence_window,
        min_error_m=nav_cfg.closed_loop_divergence_min_error_m,
        max_growth_m=nav_cfg.closed_loop_divergence_max_growth_m,
    )
    divergence_action = str(nav_cfg.closed_loop_divergence_action or "slow_then_stop").strip().lower()
    if divergence_action not in {"warn", "slow", "slow_then_stop", "stop"}:
        divergence_action = "slow_then_stop"
    if health["closed_loop_diverging"]:
        if divergence_action in {"slow", "slow_then_stop"}:
            final_v = max(0.0, min(final_v, max(nav_cfg.min_sweep_v_mps, min_v) * 0.45))
            final_omega = clamp(final_omega, -0.18, 0.18)
            left_target, right_target = wheel_targets_from_v_omega(final_v, final_omega, navigator.robot_cfg.track_width_m)
        if divergence_action == "stop" or (
            divergence_action == "slow_then_stop"
            and health_state.divergence_persist_count >= 2
        ):
            debug.update(
                {
                    "closed_loop_active": True,
                    "target_yaw_deg": round(math.degrees(target_yaw), 2),
                    "estimated_yaw_deg": round(math.degrees(pose.yaw), 2),
                    "heading_error_deg": round(math.degrees(heading_error), 2),
                    "cross_track_error_m": round(cross_track_error, 4),
                    "omega_heading_radps": round(omega_heading, 4),
                    "omega_lane_radps": round(omega_lane, 4),
                    "lane_heading_offset_deg": round(math.degrees(lane_heading_offset), 2),
                    "row_follower_speed_schedule_enabled": bool(speed_schedule_active),
                    "row_follower_scheduled_lane_kp": round(scheduled_lane_kp, 4),
                    "row_follower_scheduled_heading_kp": round(scheduled_heading_kp, 4),
                    "row_follower_raw_omega_radps": round(row_raw_omega, 4),
                    "row_follower_smoothed_omega_radps": round(row_omega, 4),
                    "row_follower_rate_limited": bool(row_rate_limited),
                    "row_follower_update_dt_s": None if row_update_dt_s is None else round(row_update_dt_s, 4),
                    "sweep_planner_omega_weight": round(planner_omega_weight, 3),
                    "planner_omega_radps": round(planner_omega, 4),
                    "planner_omega_contribution_radps": round(planner_omega_contribution, 4),
                    "planner_gap_ignored": bool(planner_gap_ignored),
                    "planner_gap_reason": planner_gap_reason,
                    "final_v_mps": 0.0,
                    "final_omega_radps": 0.0,
                    "forward_arc_omega_limit_radps": None if forward_arc_limit is None else round(forward_arc_limit, 4),
                    "forward_arc_clamped": bool(forward_arc_clamped),
                    "final_left_target_mps": 0.0,
                    "final_right_target_mps": 0.0,
                    "heading_source": heading_source,
                    "reason": f"closed_loop_divergence_{divergence_action}",
                    "correction_sign_ok": bool(correction_sign_ok),
                    "divergence_action": divergence_action,
                    **health,
                }
            )
            navigator.closed_loop_debug = debug
            return cmd.__class__(
                "STOP",
                reason=f"closed-loop divergence: {health['divergence_reason']}",
                controller="velocity",
                command_type="stop",
            )

    adjusted = navigator.velocity_planner._velocity_to_command(
        final_v,
        final_omega,
        f"{cmd.reason}; competition closed-loop sweep lane/heading",
        nav_cfg.continuous_horizon_s,
    )
    navigator.velocity_planner.last_v = final_v
    navigator.velocity_planner.last_omega = final_omega
    navigator.velocity_planner.last_timestamp = frame.encoder.timestamp
    navigator.state.latest_cmd = adjusted

    debug.update(
        {
            "closed_loop_active": True,
            "target_yaw_deg": round(math.degrees(target_yaw), 2),
            "estimated_yaw_deg": round(math.degrees(pose.yaw), 2),
            "heading_error_deg": round(math.degrees(heading_error), 2),
            "cross_track_error_m": round(cross_track_error, 4),
            "omega_heading_radps": round(omega_heading, 4),
            "omega_lane_radps": round(omega_lane, 4),
            "lane_heading_offset_deg": round(math.degrees(lane_heading_offset), 2),
            "row_follower_speed_schedule_enabled": bool(speed_schedule_active),
            "row_follower_scheduled_lane_kp": round(scheduled_lane_kp, 4),
            "row_follower_scheduled_heading_kp": round(scheduled_heading_kp, 4),
            "row_follower_raw_omega_radps": round(row_raw_omega, 4),
            "row_follower_smoothed_omega_radps": round(row_omega, 4),
            "row_follower_rate_limited": bool(row_rate_limited),
            "row_follower_update_dt_s": None if row_update_dt_s is None else round(row_update_dt_s, 4),
            "sweep_planner_omega_weight": round(planner_omega_weight, 3),
            "planner_omega_radps": round(planner_omega, 4),
            "planner_omega_contribution_radps": round(planner_omega_contribution, 4),
            "planner_gap_ignored": bool(planner_gap_ignored),
            "planner_gap_reason": planner_gap_reason,
            "final_v_mps": round(final_v, 4),
            "final_omega_radps": round(final_omega, 4),
            "forward_arc_omega_limit_radps": None if forward_arc_limit is None else round(forward_arc_limit, 4),
            "forward_arc_clamped": bool(forward_arc_clamped),
            "final_left_target_mps": round(left_target, 4),
            "final_right_target_mps": round(right_target, 4),
            "heading_source": heading_source,
            "reason": "closed_loop_divergence_slow" if health["closed_loop_diverging"] else "sweep_lane_heading_closed_loop",
            "correction_sign_ok": bool(correction_sign_ok),
            "divergence_action": divergence_action,
            **health,
        }
    )
    navigator.closed_loop_debug = debug
    return adjusted
