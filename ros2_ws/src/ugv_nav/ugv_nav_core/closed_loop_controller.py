from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple


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
) -> Tuple[float, float]:
    heading_error = wrap_to_pi(float(target_yaw_rad) - float(estimated_yaw_rad))
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
) -> Tuple[float, float, float]:
    cross_track_error = float(lane_y_m) - float(estimated_y_m)
    if abs(cross_track_error) <= max(0.0, float(deadband_m)):
        return cross_track_error, 0.0, 0.0
    direction = 1.0 if float(row_direction) >= 0.0 else -1.0
    max_heading = math.radians(max(0.0, float(max_heading_deg)))
    desired_heading_offset = direction * clamp(
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
    if navigator.physical_stall.detected:
        debug["reason"] = "physical_stall_recovery"
        navigator.closed_loop_debug = debug
        return cmd

    velocity_debug = navigator.velocity_debug or {}
    safety_state = str(velocity_debug.get("safety_state") or "")
    if safety_state in {"stop", "no_safe_trajectory"}:
        debug["reason"] = f"safety_{safety_state}"
        navigator.closed_loop_debug = debug
        return cmd

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

    heading_error, omega_heading = compute_heading_hold_correction(
        target_yaw,
        pose.yaw,
        yaw_rate_radps=yaw_rate,
        kp=nav_cfg.heading_hold_kp,
        kd=nav_cfg.heading_hold_kd,
        deadband_deg=nav_cfg.heading_hold_deadband_deg,
        max_omega_radps=nav_cfg.heading_hold_max_omega_rps,
    )
    if not nav_cfg.heading_hold_enabled:
        omega_heading = 0.0

    cross_track_error, lane_heading_offset, omega_lane = compute_lane_follow_correction(
        lane_y,
        pose.y,
        row_direction,
        kp_heading=nav_cfg.lane_follow_kp_heading,
        kp_omega=nav_cfg.lane_follow_kp_omega,
        deadband_m=nav_cfg.lane_follow_deadband_m,
        max_heading_deg=nav_cfg.lane_follow_max_heading_deg,
        max_omega_radps=nav_cfg.lane_follow_max_omega_rps,
    )
    if not nav_cfg.lane_follow_enabled:
        lane_heading_offset = 0.0
        omega_lane = 0.0

    min_v = max(
        nav_cfg.continuous_min_speed_mps,
        finite_optional(v2.get("minimum_speed_mps")) or 0.0,
        0.02,
    )
    base_v = abs(float(cmd.v_mps))
    if base_v <= 1e-6 and cmd.mode in {"FORWARD", "BACKWARD"}:
        base_v = abs(float(cmd.move_m)) / max(0.05, nav_cfg.continuous_dt_s)
    final_v = clamp(max(base_v, min_v), 0.0, nav_cfg.continuous_max_speed_mps)
    planner_omega = float(cmd.omega_radps) if math.isfinite(float(cmd.omega_radps)) else 0.0
    final_omega = clamp(
        0.20 * planner_omega + omega_heading + omega_lane,
        -nav_cfg.continuous_max_omega_rps,
        nav_cfg.continuous_max_omega_rps,
    )
    if final_v > 0.005:
        max_arc_omega = max(0.02, 1.75 * final_v / max(1e-6, navigator.robot_cfg.track_width_m))
        final_omega = clamp(final_omega, -max_arc_omega, max_arc_omega)

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
            "final_v_mps": round(final_v, 4),
            "final_omega_radps": round(final_omega, 4),
            "heading_source": heading_source,
            "reason": "sweep_lane_heading_closed_loop",
        }
    )
    navigator.closed_loop_debug = debug
    return adjusted
