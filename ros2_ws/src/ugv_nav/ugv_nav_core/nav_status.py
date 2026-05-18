from __future__ import annotations

import math
from typing import Any, Dict


def _finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def build_nav_status(navigator: Any, frame: Any, cmd: Any, mission_status: dict) -> Dict[str, Any]:
    pose = navigator.state.estimated_pose
    goal = navigator.state.goal_pose
    field_map = frame.field_map
    cmd_dict = cmd.as_dict()
    closed_loop_status = dict(navigator.closed_loop_debug)
    pre_scale_final_v = closed_loop_status.get("final_v_mps")
    pre_scale_final_omega = closed_loop_status.get("final_omega_radps")
    emitted_v = _finite_float(cmd_dict.get("v_mps"), 0.0)
    emitted_omega = _finite_float(cmd_dict.get("omega_radps"), 0.0)
    track_width = max(1e-6, _finite_float(getattr(navigator.robot_cfg, "track_width_m", 0.0), 0.0))
    emitted_left_target = emitted_v - 0.5 * emitted_omega * track_width
    emitted_right_target = emitted_v + 0.5 * emitted_omega * track_width
    command_scaling_status = {
        "pre_scale_final_v_mps": pre_scale_final_v,
        "pre_scale_final_omega_radps": pre_scale_final_omega,
        "emitted_v_mps": round(emitted_v, 4),
        "emitted_omega_radps": round(emitted_omega, 4),
        "emitted_left_target_mps": round(emitted_left_target, 4),
        "emitted_right_target_mps": round(emitted_right_target, 4),
        "final_v_mps_meaning": "pre_drive_scale_closed_loop_command",
        "final_omega_radps_meaning": "pre_drive_scale_closed_loop_command",
        "emitted_command_meaning": "post_drive_scale_ugv_nav_cmd",
    }
    closed_loop_status.update(command_scaling_status)
    status = {
        "stamp": round(frame.encoder.timestamp, 3),
        "mission": mission_status,
        "pose_m": [round(pose.x, 3), round(pose.y, 3), round(math.degrees(pose.yaw), 1)],
        "goal_m": [round(goal.x, 3), round(goal.y, 3)],
        "distance_to_goal_m": round(math.hypot(goal.x - pose.x, goal.y - pose.y), 3),
        "cmd": cmd_dict,
        "planner": navigator.state.planner_name,
        "imu_yaw_fusion": {
            "enabled": navigator.nav_cfg.use_imu_yaw,
            "axis": navigator.nav_cfg.imu_yaw_axis,
            "sign": navigator.nav_cfg.imu_yaw_sign,
            "blend": round(navigator.nav_cfg.imu_yaw_blend, 3),
        },
        "replans": navigator.state.replans,
        "plan_time_ms": round(navigator.state.plan_time_ms, 2),
        "known_obstacle_updates": navigator.state.discovered_points,
        "path_points": len(navigator.state.path),
        "path_idx": navigator.state.path_idx,
        "active_scan": navigator.active_scan_status(),
        "recovery_state": navigator.recovery_state.value,
        **navigator.physical_stall_status(),
        "local_costmap": navigator.local_costmap_debug,
        "velocity_control": navigator.velocity_debug,
        "closed_loop_enabled": bool(closed_loop_status.get("closed_loop_enabled", False)),
        "heading_hold_enabled": bool(closed_loop_status.get("heading_hold_enabled", False)),
        "lane_follow_enabled": bool(closed_loop_status.get("lane_follow_enabled", False)),
        "target_yaw_deg": closed_loop_status.get("target_yaw_deg"),
        "estimated_yaw_deg": closed_loop_status.get("estimated_yaw_deg"),
        "heading_error_deg": closed_loop_status.get("heading_error_deg"),
        "cross_track_error_m": closed_loop_status.get("cross_track_error_m"),
        "omega_heading_radps": closed_loop_status.get("omega_heading_radps"),
        "omega_lane_radps": closed_loop_status.get("omega_lane_radps"),
        "final_v_mps": closed_loop_status.get("final_v_mps"),
        "final_omega_radps": closed_loop_status.get("final_omega_radps"),
        **command_scaling_status,
        "heading_source": closed_loop_status.get("heading_source"),
        "competition_closed_loop": closed_loop_status,
        "sectors_m": {
            "front": None if math.isinf(navigator.state.sectors.front_m) else round(navigator.state.sectors.front_m, 3),
            "front_left": None if math.isinf(navigator.state.sectors.front_left_m) else round(navigator.state.sectors.front_left_m, 3),
            "front_right": None if math.isinf(navigator.state.sectors.front_right_m) else round(navigator.state.sectors.front_right_m, 3),
            "left": None if math.isinf(navigator.state.sectors.left_m) else round(navigator.state.sectors.left_m, 3),
            "right": None if math.isinf(navigator.state.sectors.right_m) else round(navigator.state.sectors.right_m, 3),
            "rear": None if math.isinf(navigator.state.sectors.rear_m) else round(navigator.state.sectors.rear_m, 3),
        },
        "encoder_ticks": [frame.encoder.left_total, frame.encoder.right_total],
        "odom_delta": navigator.last_odom_delta,
        "lidar_hit_count": len(frame.lidar.hit_points_local),
        "zed_hit_count": len(frame.zed.hit_points_local),
        "field_map": {
            "version": field_map.version,
            "source": field_map.source,
            "obstacle_cells": len(field_map.obstacle_cells),
            "goal_m": [round(field_map.goal_xy[0], 3), round(field_map.goal_xy[1], 3)] if field_map and field_map.goal_xy else None,
        } if field_map else None,
        "mission_flag": {
            "state": frame.mission_flag.state,
            "source": frame.mission_flag.source,
            "age_s": round(max(0.0, frame.encoder.timestamp - frame.mission_flag.timestamp), 3),
        } if frame.mission_flag else None,
        "finish_reason": navigator.state.finish_reason,
    }
    if mission_status.get("competition_v2") is not None:
        status["competition_v2"] = mission_status["competition_v2"]
        v2 = mission_status["competition_v2"]
        for key in [
            "sweep_subphase",
            "row_index",
            "row_direction",
            "current_lane_y_m",
            "next_lane_y_m",
            "row_end_x_m",
            "row_transition_active",
            "row_transition_style",
            "row_transition_progress",
            "row_transition_done",
            "row_transition_reason",
            "target_row_yaw_deg",
            "yaw_capture_error_deg",
            "lane_capture_error_m",
            "turn_radius_m",
            "turn_side",
        ]:
            status[key] = v2.get(key)
    return status


__all__ = ["build_nav_status"]
