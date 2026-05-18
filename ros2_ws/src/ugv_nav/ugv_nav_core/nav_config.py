from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Tuple


FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M
CM_TO_M = 0.01
YARD_TO_M = 0.9144
FIELD_CELLS_DEFAULT = 15


def ft(x: float) -> float:
    return x * FT_TO_M


def yd(x: float) -> float:
    return x * YARD_TO_M


def normalize_drive_speed_level(value: int) -> int:
    return int(max(1, min(4, int(value))))


def drive_speed_factor(level: int) -> float:
    return normalize_drive_speed_level(level) / 4.0


def drive_speed_level_arg(value: str) -> int:
    try:
        level = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("drive speed level must be an integer from 1 to 4") from exc
    if level < 1 or level > 4:
        raise argparse.ArgumentTypeError("drive speed level must be from 1 to 4")
    return level


@dataclass
class RobotConfig:
    length_m: float = ft(30.0 / 12.0)
    width_m: float = ft(30.0 / 12.0)
    track_width_m: float = ft(2.0)
    wheel_radius_m: float = 0.06
    ticks_per_rev: int = 1000
    obstacle_buffer_m: float = 0.025
    lidar_offset_x_m: float = 0.30
    lidar_offset_y_m: float = 0.0


@dataclass
class SensorConfig:
    lidar_range_m: float = 3.8
    lidar_num_beams: int = 181
    zed_fov_deg: float = 110.0
    zed_range_m: float = 3.2
    zed_los_step_m: float = 0.04


@dataclass
class NavConfig:
    map_resolution_m: float = 0.08
    planner_step_m: float = 0.22
    goal_tol_m: float = 0.18
    path_reach_tol_m: float = 0.12
    local_lookahead_m: float = 0.55
    turn_threshold_deg: float = 16.0
    max_turn_cmd_deg: float = 28.0
    turn_step_choices_deg: Tuple[float, ...] = (10.0, 18.0, 28.0, 38.0)
    forward_step_choices_m: Tuple[float, ...] = (0.08, 0.12, 0.16)
    backward_step_choices_m: Tuple[float, ...] = (0.08, 0.12)
    replan_cooldown_s: float = 0.22
    replan_lookahead_poses: int = 28
    stuck_pose_epsilon_m: float = 0.018
    stuck_trigger_steps: int = 3
    blocked_patch_radius_m: float = 0.22
    blocked_patch_distance_m: float = 0.30
    blocked_patch_ttl_steps: int = 28
    front_safety_margin_m: float = 0.08
    rear_safety_margin_m: float = 0.08
    global_plan_inflation_m: float = 0.18
    local_plan_inflation_m: float = 0.0
    local_turn_switch_penalty: float = 0.18
    local_reverse_penalty: float = 0.28
    local_turn_penalty: float = 0.10
    local_goal_progress_weight: float = 4.5
    local_heading_weight: float = 0.95
    local_corridor_forward_bonus: float = 0.80
    local_corridor_turn_penalty: float = 0.35
    local_corridor_front_clear_m: float = 0.62
    local_corridor_side_clear_m: float = 0.45
    active_scan_enabled: bool = True
    active_scan_front_clear_m: float = 1.25
    active_scan_release_clear_m: float = 1.12
    active_scan_corridor_extra_width_m: float = 0.03
    active_scan_depth_points_threshold: int = 12
    active_scan_confirm_steps: int = 3
    active_scan_plan_fail_confirm_steps: int = 4
    active_scan_allow_probe_clear_m: float = 0.95
    active_scan_turn_deg: float = 28.0
    active_scan_steps: int = 7
    active_scan_panoramic_steps: int = 20
    active_scan_cooldown_steps: int = 4
    active_scan_probe_steps: int = 5
    continuous_control_enabled: bool = True
    continuous_horizon_s: float = 1.35
    continuous_dt_s: float = 0.10
    continuous_v_samples: int = 5
    continuous_omega_samples: int = 13
    continuous_max_speed_mps: float = 0.36
    continuous_min_speed_mps: float = 0.05
    continuous_max_omega_rps: float = 1.15
    continuous_accel_limit_mps2: float = 0.35
    continuous_omega_accel_limit_rps2: float = 1.80
    continuous_lowpass_alpha: float = 0.55
    continuous_raw_per_mps: float = 1.35
    continuous_max_raw: float = 0.55
    continuous_slowdown_clearance_m: float = 1.20
    continuous_stop_clearance_m: float = 0.48
    continuous_gap_lookahead_m: float = 1.80
    continuous_gap_buffer_m: float = 0.025
    continuous_latency_buffer_s: float = 0.25
    continuous_allow_costmap_soft_penalty: bool = False
    emit_velocity_commands: bool = True
    competition_closed_loop_enabled: bool = True
    heading_hold_enabled: bool = True
    lane_follow_enabled: bool = True
    heading_hold_kp: float = 1.10
    heading_hold_kd: float = 0.05
    heading_hold_deadband_deg: float = 3.0
    heading_hold_max_omega_rps: float = 0.55
    lane_follow_kp_heading: float = 1.10
    lane_follow_kp_omega: float = 1.00
    lane_follow_deadband_m: float = 0.03
    lane_follow_max_heading_deg: float = 18.0
    lane_follow_max_omega_rps: float = 0.35
    row_follower_speed_schedule_enabled: bool = True
    row_follower_low_speed_mps: float = 0.09
    row_follower_high_speed_mps: float = 0.22
    row_follower_low_speed_lane_kp: float = 0.85
    row_follower_high_speed_lane_kp: float = 1.00
    row_follower_low_speed_heading_kp: float = 0.95
    row_follower_high_speed_heading_kp: float = 1.10
    row_follower_omega_low_pass_alpha: float = 0.35
    row_follower_omega_rate_limit_rps2: float = 0.60
    row_follower_min_correction_interval_s: float = 0.0
    sweep_planner_omega_weight_round2: float = 0.0
    sweep_planner_omega_weight_round3: float = 0.2
    sweep_metrics_log_enabled: bool = False
    sweep_metrics_log_dir: str = "~/.ros/ugv_sweep_metrics"
    forward_arc_only_enabled: bool = True
    forward_arc_margin: float = 0.75
    min_sweep_v_mps: float = 0.08
    omega_command_sign: float = 1.0
    heading_error_sign: float = 1.0
    lane_error_sign: float = 1.0
    lane_correction_sign: float = 1.0
    closed_loop_health_enabled: bool = True
    closed_loop_divergence_window: int = 5
    closed_loop_divergence_min_error_m: float = 0.08
    closed_loop_divergence_max_growth_m: float = 0.05
    closed_loop_divergence_action: str = "slow_then_stop"
    local_costmap_enabled: bool = True
    local_costmap_width_m: float = 4.0
    local_costmap_height_m: float = 4.0
    local_costmap_resolution_m: float = 0.06
    local_costmap_dynamic_decay_s: float = 1.00
    local_costmap_obstacle_radius_m: float = 0.06
    local_costmap_inflation_m: float = 0.08
    local_costmap_lidar_clear_radius_m: float = 0.05
    local_costmap_max_raytrace_m: float = 4.0
    allow_stop_at_goal: bool = True
    nonstop_when_blocked: bool = False
    allow_reverse: bool = False
    lidar_used_fov_deg: float = 180.0
    lidar_map_stride: int = 4
    use_imu_yaw: bool = False
    imu_yaw_blend: float = 0.25
    imu_yaw_axis: str = "z"
    imu_yaw_sign: float = 1.0
    imu_yaw_max_rate_rps: float = 4.0
    min_motion_raw: float = 0.22
    recovery_turn_raw: float = 0.32
    competition_sweep_active: bool = False
    sweep_allow_pure_turn: bool = False
    sweep_heading_tolerance_deg: float = 25.0
    physical_stall_timeout_s: float = 1.5
    physical_stall_min_raw: float = 0.12
    physical_stall_min_v_mps: float = 0.04


@dataclass
class SimConfig:
    field_w_m: float = yd(15.0)
    field_h_m: float = yd(15.0)
    dt_s: float = 0.10
    max_steps: int = 900
    show_gui: bool = True


__all__ = [
    "CM_TO_M",
    "FIELD_CELLS_DEFAULT",
    "FT_TO_M",
    "M_TO_FT",
    "YARD_TO_M",
    "NavConfig",
    "RobotConfig",
    "SensorConfig",
    "SimConfig",
    "drive_speed_factor",
    "drive_speed_level_arg",
    "ft",
    "normalize_drive_speed_level",
    "yd",
]
