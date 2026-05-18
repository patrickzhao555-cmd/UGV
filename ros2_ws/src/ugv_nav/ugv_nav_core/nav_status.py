from __future__ import annotations

import math
from typing import Any, Dict


def build_nav_status(navigator: Any, frame: Any, cmd: Any, mission_status: dict) -> Dict[str, Any]:
    pose = navigator.state.estimated_pose
    goal = navigator.state.goal_pose
    field_map = frame.field_map
    status = {
        "stamp": round(frame.encoder.timestamp, 3),
        "mission": mission_status,
        "pose_m": [round(pose.x, 3), round(pose.y, 3), round(math.degrees(pose.yaw), 1)],
        "goal_m": [round(goal.x, 3), round(goal.y, 3)],
        "distance_to_goal_m": round(math.hypot(goal.x - pose.x, goal.y - pose.y), 3),
        "cmd": cmd.as_dict(),
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
        "closed_loop_enabled": bool(navigator.closed_loop_debug.get("closed_loop_enabled", False)),
        "heading_hold_enabled": bool(navigator.closed_loop_debug.get("heading_hold_enabled", False)),
        "lane_follow_enabled": bool(navigator.closed_loop_debug.get("lane_follow_enabled", False)),
        "target_yaw_deg": navigator.closed_loop_debug.get("target_yaw_deg"),
        "estimated_yaw_deg": navigator.closed_loop_debug.get("estimated_yaw_deg"),
        "heading_error_deg": navigator.closed_loop_debug.get("heading_error_deg"),
        "cross_track_error_m": navigator.closed_loop_debug.get("cross_track_error_m"),
        "omega_heading_radps": navigator.closed_loop_debug.get("omega_heading_radps"),
        "omega_lane_radps": navigator.closed_loop_debug.get("omega_lane_radps"),
        "final_v_mps": navigator.closed_loop_debug.get("final_v_mps"),
        "final_omega_radps": navigator.closed_loop_debug.get("final_omega_radps"),
        "heading_source": navigator.closed_loop_debug.get("heading_source"),
        "competition_closed_loop": navigator.closed_loop_debug,
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
    return status


__all__ = ["build_nav_status"]
