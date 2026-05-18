from __future__ import annotations

from typing import Any

from ugv_nav_core.competition_mission import NavigationFeedback


def competition_v2_status_for_update(update: Any) -> dict:
    v2 = dict(update.status)
    return {
        "enabled": update.enabled,
        "mode": update.round,
        "phase": update.phase,
        "stop_requested": update.stop_requested,
        "speed_scale": round(update.speed_scale, 3),
        "speed_reason": update.reason,
        "min_speed_mps": round(update.minimum_speed_mps, 6),
        "active_goal_m": list(v2.get("active_goal_m", [])),
        "final_goal_m": v2.get("target_goal_m") if update.target_known else None,
        "final_goal_source": update.target_source,
        "competition_v2": v2,
    }


def apply_competition_v2_motion_policy(navigator: Any, mission_status: dict) -> None:
    v2 = mission_status.get("competition_v2") if isinstance(mission_status, dict) else None
    if not isinstance(v2, dict):
        navigator.nav_cfg.competition_sweep_active = False
        return
    navigator.nav_cfg.competition_sweep_active = bool(v2.get("enabled", False) and v2.get("phase") == "sweep_search")
    navigator.nav_cfg.sweep_allow_pure_turn = bool(v2.get("sweep_allow_pure_turn", False))
    try:
        navigator.nav_cfg.sweep_heading_tolerance_deg = max(0.0, float(v2.get("sweep_heading_tolerance_deg", 25.0)))
    except (TypeError, ValueError):
        navigator.nav_cfg.sweep_heading_tolerance_deg = 25.0


def navigation_feedback_for_competition_v2(
    navigator: Any,
    cmd: Any,
    frame: Any,
) -> NavigationFeedback:
    velocity_debug = navigator.velocity_debug or {}
    closed_loop_debug = navigator.closed_loop_debug or {}
    finish_reason = str(navigator.state.finish_reason or "")
    planner_failed = navigator.state.planner_name == "failed" and not navigator.state.path
    no_safe = velocity_debug.get("safety_state") == "no_safe_trajectory"
    stuck_steps = int(getattr(navigator, "_stuck_counter", 0))
    stuck = (
        stuck_steps >= max(1, int(navigator.nav_cfg.stuck_trigger_steps))
        or "stuck" in str(cmd.reason).lower()
        or "stuck" in finish_reason.lower()
    )
    progress_m = float(navigator.last_odom_delta.get("ds_m") or 0.0)
    return NavigationFeedback(
        now_s=frame.encoder.timestamp,
        no_safe_trajectory=bool(no_safe),
        local_planner_failed=bool(planner_failed),
        stuck=bool(stuck),
        stuck_steps=stuck_steps,
        physical_stall_detected=bool(navigator.physical_stall.detected),
        physical_stall_steps=int(navigator.physical_stall.steps),
        physical_stall_reason=navigator.physical_stall.reason,
        closed_loop_active=bool(closed_loop_debug.get("closed_loop_active", False)),
        closed_loop_diverging=bool(closed_loop_debug.get("closed_loop_diverging", False)),
        closed_loop_reason=str(closed_loop_debug.get("divergence_reason") or closed_loop_debug.get("reason") or ""),
        finish_reason=finish_reason,
        progress_m=progress_m,
    )


__all__ = [
    "apply_competition_v2_motion_policy",
    "competition_v2_status_for_update",
    "navigation_feedback_for_competition_v2",
]
