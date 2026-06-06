from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration


def _workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").exists():
            return parent
    raise RuntimeError("Could not find workspace root containing src/ugv_nav/ugv_nav_dual_mode.py")


def generate_launch_description():
    workspace_root = _workspace_root()
    motor_launch = workspace_root / "src" / "ugv_motor_controller" / "launch" / "motor_controller.launch.py"
    sensor_launch = workspace_root / "src" / "ugv_sensor_sync" / "launch" / "sensor_sync_launch.py"
    nav_script = workspace_root / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"
    challenge3_script = workspace_root / "src" / "ugv_nav" / "ugv_challenge3_corridor.py"
    target_receiver = workspace_root / "src" / "ugv_sensor_sync" / "ugv_sensor_sync_nodes" / "uwb_node.py"

    start_motor_controller = LaunchConfiguration("start_motor_controller")
    start_sensor_sync = LaunchConfiguration("start_sensor_sync")
    start_nav = LaunchConfiguration("start_nav")
    start_zed = LaunchConfiguration("start_zed")
    start_lidar = LaunchConfiguration("start_lidar")
    start_lidar_filter = LaunchConfiguration("start_lidar_filter")
    start_fusion = LaunchConfiguration("start_fusion")
    start_debug_status = LaunchConfiguration("start_debug_status")
    start_uav_target_receiver = LaunchConfiguration("start_uav_target_receiver")
    start_challenge3_corridor = LaunchConfiguration("start_challenge3_corridor")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    lidar_filter_forward_fov_deg = LaunchConfiguration("lidar_filter_forward_fov_deg")
    fusion_lidar_front_min_cluster_points = LaunchConfiguration("fusion_lidar_front_min_cluster_points")
    fusion_lidar_front_cluster_max_gap_m = LaunchConfiguration("fusion_lidar_front_cluster_max_gap_m")
    zed_publish_rate_hz = LaunchConfiguration("zed_publish_rate_hz")
    zed_imu_publish_rate_hz = LaunchConfiguration("zed_imu_publish_rate_hz")
    zed_imu_rate_window_s = LaunchConfiguration("zed_imu_rate_window_s")
    zed_depth_downsample_factor = LaunchConfiguration("zed_depth_downsample_factor")
    zed_image_downsample_factor = LaunchConfiguration("zed_image_downsample_factor")
    zed_publish_image = LaunchConfiguration("zed_publish_image")
    motor_port = LaunchConfiguration("motor_port")
    motor_baud = LaunchConfiguration("motor_baud")
    motor_dry_run = LaunchConfiguration("motor_dry_run")
    motor_command_timeout_s = LaunchConfiguration("motor_command_timeout_s")
    motor_command_refresh_period_s = LaunchConfiguration("motor_command_refresh_period_s")
    motor_track_width_m = LaunchConfiguration("motor_track_width_m")
    motor_left_forward_speed_scale = LaunchConfiguration("motor_left_forward_speed_scale")
    motor_right_forward_speed_scale = LaunchConfiguration("motor_right_forward_speed_scale")
    motor_left_reverse_speed_scale = LaunchConfiguration("motor_left_reverse_speed_scale")
    motor_right_reverse_speed_scale = LaunchConfiguration("motor_right_reverse_speed_scale")
    motor_wheel_radius_m = LaunchConfiguration("motor_wheel_radius_m")
    motor_ticks_per_rev = LaunchConfiguration("motor_ticks_per_rev")
    motor_pwm_min_us = LaunchConfiguration("motor_pwm_min_us")
    motor_pwm_neutral_us = LaunchConfiguration("motor_pwm_neutral_us")
    motor_pwm_max_us = LaunchConfiguration("motor_pwm_max_us")
    motor_pwm_slew_rate_us_per_s = LaunchConfiguration("motor_pwm_slew_rate_us_per_s")
    motor_teensy_control_hz = LaunchConfiguration("motor_teensy_control_hz")
    motor_teensy_pid_kp = LaunchConfiguration("motor_teensy_pid_kp")
    motor_teensy_pid_ki = LaunchConfiguration("motor_teensy_pid_ki")
    motor_teensy_pid_kd = LaunchConfiguration("motor_teensy_pid_kd")
    motor_teensy_pid_feedforward_us_per_tps = LaunchConfiguration("motor_teensy_pid_feedforward_us_per_tps")
    motor_enable_teensy_side_specific_pid_params = LaunchConfiguration("motor_enable_teensy_side_specific_pid_params")
    motor_teensy_left_pid_feedforward_us_per_tps = LaunchConfiguration("motor_teensy_left_pid_feedforward_us_per_tps")
    motor_teensy_right_pid_feedforward_us_per_tps = LaunchConfiguration("motor_teensy_right_pid_feedforward_us_per_tps")
    motor_teensy_right_reverse_pid_feedforward_us_per_tps = LaunchConfiguration(
        "motor_teensy_right_reverse_pid_feedforward_us_per_tps"
    )
    motor_teensy_pid_static_ff_us = LaunchConfiguration("motor_teensy_pid_static_ff_us")
    motor_teensy_pid_static_ff_full_target_tps = LaunchConfiguration("motor_teensy_pid_static_ff_full_target_tps")
    motor_teensy_pid_static_ff_fade_start_ratio = LaunchConfiguration(
        "motor_teensy_pid_static_ff_fade_start_ratio"
    )
    motor_teensy_pid_static_ff_fade_end_ratio = LaunchConfiguration(
        "motor_teensy_pid_static_ff_fade_end_ratio"
    )
    motor_teensy_left_pid_static_ff_us = LaunchConfiguration("motor_teensy_left_pid_static_ff_us")
    motor_teensy_right_pid_static_ff_us = LaunchConfiguration("motor_teensy_right_pid_static_ff_us")
    motor_teensy_right_reverse_pid_static_ff_us = LaunchConfiguration("motor_teensy_right_reverse_pid_static_ff_us")
    motor_teensy_right_reverse_pwm_floor_us = LaunchConfiguration("motor_teensy_right_reverse_pwm_floor_us")
    motor_teensy_pid_output_limit_us = LaunchConfiguration("motor_teensy_pid_output_limit_us")
    motor_teensy_left_pid_output_limit_us = LaunchConfiguration("motor_teensy_left_pid_output_limit_us")
    motor_teensy_right_pid_output_limit_us = LaunchConfiguration("motor_teensy_right_pid_output_limit_us")
    motor_teensy_pid_min_target_tps = LaunchConfiguration("motor_teensy_pid_min_target_tps")
    motor_teensy_left_motor_sign = LaunchConfiguration("motor_teensy_left_motor_sign")
    motor_teensy_right_motor_sign = LaunchConfiguration("motor_teensy_right_motor_sign")
    motor_teensy_fl_encoder_sign = LaunchConfiguration("motor_teensy_fl_encoder_sign")
    motor_teensy_fr_encoder_sign = LaunchConfiguration("motor_teensy_fr_encoder_sign")
    motor_teensy_rl_encoder_sign = LaunchConfiguration("motor_teensy_rl_encoder_sign")
    motor_teensy_rr_encoder_sign = LaunchConfiguration("motor_teensy_rr_encoder_sign")
    motor_teensy_side_mismatch_warn_tps = LaunchConfiguration("motor_teensy_side_mismatch_warn_tps")
    motor_teensy_side_mismatch_fault_tps = LaunchConfiguration("motor_teensy_side_mismatch_fault_tps")
    motor_teensy_encoder_jump_tps = LaunchConfiguration("motor_teensy_encoder_jump_tps")
    motor_teensy_pid_param_ack_timeout_s = LaunchConfiguration("motor_teensy_pid_param_ack_timeout_s")
    motor_teensy_pid_param_write_interval_s = LaunchConfiguration("motor_teensy_pid_param_write_interval_s")
    nav_status_period_s = LaunchConfiguration("nav_status_period_s")
    nav_control_period_s = LaunchConfiguration("nav_control_period_s")
    nav_controller_mode = LaunchConfiguration("nav_controller_mode")
    nav_allow_legacy_controller = LaunchConfiguration("nav_allow_legacy_controller")
    nav_frame_topic = LaunchConfiguration("nav_frame_topic")
    nav_imu_topic = LaunchConfiguration("nav_imu_topic")
    nav_imu_qos = LaunchConfiguration("nav_imu_qos")
    nav_imu_yaw_axis = LaunchConfiguration("nav_imu_yaw_axis")
    nav_imu_yaw_sign = LaunchConfiguration("nav_imu_yaw_sign")
    nav_imu_timeout_s = LaunchConfiguration("nav_imu_timeout_s")
    nav_imu_min_rate_hz = LaunchConfiguration("nav_imu_min_rate_hz")
    nav_motor_status_topic = LaunchConfiguration("nav_motor_status_topic")
    nav_encoder_stamped_topic = LaunchConfiguration("nav_encoder_stamped_topic")
    nav_zed_status_topic = LaunchConfiguration("nav_zed_status_topic")
    nav_allow_encoder_heading_fallback = LaunchConfiguration("nav_allow_encoder_heading_fallback")
    nav_straight_speed_mps = LaunchConfiguration("nav_straight_speed_mps")
    nav_straight_duration_s = LaunchConfiguration("nav_straight_duration_s")
    nav_pivot_angle_deg = LaunchConfiguration("nav_pivot_angle_deg")
    nav_curve_angle_deg = LaunchConfiguration("nav_curve_angle_deg")
    nav_curve_speed_mps = LaunchConfiguration("nav_curve_speed_mps")
    nav_curve_radius_m = LaunchConfiguration("nav_curve_radius_m")
    nav_curve_omega_radps = LaunchConfiguration("nav_curve_omega_radps")
    nav_curve_direction = LaunchConfiguration("nav_curve_direction")
    nav_arc_min_turn_radius_m = LaunchConfiguration("nav_arc_min_turn_radius_m")
    nav_arc_max_omega_radps = LaunchConfiguration("nav_arc_max_omega_radps")
    nav_curve_omega_slew_radps2 = LaunchConfiguration("nav_curve_omega_slew_radps2")
    nav_curve_timeout_s = LaunchConfiguration("nav_curve_timeout_s")
    nav_curve_no_progress_timeout_s = LaunchConfiguration("nav_curve_no_progress_timeout_s")
    nav_curve_min_progress_rad = LaunchConfiguration("nav_curve_min_progress_rad")
    nav_curve_approach_error_rad = LaunchConfiguration("nav_curve_approach_error_rad")
    nav_curve_kp_approach = LaunchConfiguration("nav_curve_kp_approach")
    nav_curve_kd_yaw_rate = LaunchConfiguration("nav_curve_kd_yaw_rate")
    nav_curve_min_omega_radps = LaunchConfiguration("nav_curve_min_omega_radps")
    nav_curve_min_omega_disable_error_rad = LaunchConfiguration("nav_curve_min_omega_disable_error_rad")
    nav_allow_side_reverse = LaunchConfiguration("nav_allow_side_reverse")
    nav_target_topic = LaunchConfiguration("nav_target_topic")
    uav_target_input_mode = LaunchConfiguration("uav_target_input_mode")
    uav_target_units = LaunchConfiguration("uav_target_units")
    uav_target_frame_id = LaunchConfiguration("uav_target_frame_id")
    uav_esp_serial_port = LaunchConfiguration("uav_esp_serial_port")
    uav_esp_serial_baud = LaunchConfiguration("uav_esp_serial_baud")
    uav_esp_serial_protocol = LaunchConfiguration("uav_esp_serial_protocol")
    uav_esp_require_checksum = LaunchConfiguration("uav_esp_require_checksum")
    nav_uav_launched_topic = LaunchConfiguration("nav_uav_launched_topic")
    nav_uav_landed_topic = LaunchConfiguration("nav_uav_landed_topic")
    nav_manual_target_x_m = LaunchConfiguration("nav_manual_target_x_m")
    nav_manual_target_y_m = LaunchConfiguration("nav_manual_target_y_m")
    nav_tracking_enabled = LaunchConfiguration("nav_tracking_enabled")
    nav_target_stop_radius_m = LaunchConfiguration("nav_target_stop_radius_m")
    nav_tracking_lookahead_min_m = LaunchConfiguration("nav_tracking_lookahead_min_m")
    nav_tracking_lookahead_max_m = LaunchConfiguration("nav_tracking_lookahead_max_m")
    nav_tracking_lookahead_speed_gain = LaunchConfiguration("nav_tracking_lookahead_speed_gain")
    nav_tracking_nominal_speed_mps = LaunchConfiguration("nav_tracking_nominal_speed_mps")
    nav_tracking_max_speed_mps = LaunchConfiguration("nav_tracking_max_speed_mps")
    nav_tracking_max_omega_radps = LaunchConfiguration("nav_tracking_max_omega_radps")
    nav_tracking_heading_kp = LaunchConfiguration("nav_tracking_heading_kp")
    nav_tracking_cross_track_kp = LaunchConfiguration("nav_tracking_cross_track_kp")
    nav_tracking_slowdown_distance_m = LaunchConfiguration("nav_tracking_slowdown_distance_m")
    nav_obstacle_warn_m = LaunchConfiguration("nav_obstacle_warn_m")
    nav_obstacle_stop_m = LaunchConfiguration("nav_obstacle_stop_m")
    nav_bypass_offset_m = LaunchConfiguration("nav_bypass_offset_m")
    nav_bypass_forward_m = LaunchConfiguration("nav_bypass_forward_m")
    nav_bypass_rejoin_ahead_m = LaunchConfiguration("nav_bypass_rejoin_ahead_m")
    nav_challenge1_auto_start = LaunchConfiguration("nav_challenge1_auto_start")
    nav_challenge1_speed_mps = LaunchConfiguration("nav_challenge1_speed_mps")
    nav_challenge1_post_landing_s = LaunchConfiguration("nav_challenge1_post_landing_s")
    nav_challenge1_timeout_s = LaunchConfiguration("nav_challenge1_timeout_s")
    nav_challenge1_max_distance_m = LaunchConfiguration("nav_challenge1_max_distance_m")
    nav_challenge1_stop_on_obstacle = LaunchConfiguration("nav_challenge1_stop_on_obstacle")
    nav_challenge2_speed_mps = LaunchConfiguration("nav_challenge2_speed_mps")
    nav_challenge2_approach_speed_mps = LaunchConfiguration("nav_challenge2_approach_speed_mps")
    nav_challenge2_slowdown_distance_m = LaunchConfiguration("nav_challenge2_slowdown_distance_m")
    nav_challenge2_stop_radius_m = LaunchConfiguration("nav_challenge2_stop_radius_m")
    nav_challenge2_post_landing_s = LaunchConfiguration("nav_challenge2_post_landing_s")
    nav_challenge2_start_pose_set = LaunchConfiguration("nav_challenge2_start_pose_set")
    nav_challenge2_require_start_pose = LaunchConfiguration("nav_challenge2_require_start_pose")
    nav_challenge2_start_x_m = LaunchConfiguration("nav_challenge2_start_x_m")
    nav_challenge2_start_y_m = LaunchConfiguration("nav_challenge2_start_y_m")
    nav_challenge2_start_yaw_deg = LaunchConfiguration("nav_challenge2_start_yaw_deg")
    nav_challenge2_pivot_max_omega_radps = LaunchConfiguration("nav_challenge2_pivot_max_omega_radps")
    nav_challenge2_pivot_timeout_s = LaunchConfiguration("nav_challenge2_pivot_timeout_s")
    nav_challenge2_pivot_settle_error_rad = LaunchConfiguration("nav_challenge2_pivot_settle_error_rad")
    nav_challenge2_pivot_settle_time_s = LaunchConfiguration("nav_challenge2_pivot_settle_time_s")
    nav_challenge2_align_arc_speed_mps = LaunchConfiguration("nav_challenge2_align_arc_speed_mps")
    nav_challenge2_align_max_omega_radps = LaunchConfiguration("nav_challenge2_align_max_omega_radps")
    nav_challenge2_align_min_turn_radius_m = LaunchConfiguration("nav_challenge2_align_min_turn_radius_m")
    nav_challenge2_align_heading_kp = LaunchConfiguration("nav_challenge2_align_heading_kp")
    nav_challenge2_align_no_progress_timeout_s = LaunchConfiguration("nav_challenge2_align_no_progress_timeout_s")
    nav_challenge2_align_close_guard_m = LaunchConfiguration("nav_challenge2_align_close_guard_m")
    nav_challenge2_heading_kp = LaunchConfiguration("nav_challenge2_heading_kp")
    nav_challenge2_cross_track_kp = LaunchConfiguration("nav_challenge2_cross_track_kp")
    nav_challenge2_max_omega_radps = LaunchConfiguration("nav_challenge2_max_omega_radps")
    challenge3_start_pose_set = LaunchConfiguration("challenge3_start_pose_set")
    challenge3_start_x_m = LaunchConfiguration("challenge3_start_x_m")
    challenge3_start_y_m = LaunchConfiguration("challenge3_start_y_m")
    challenge3_start_yaw_deg = LaunchConfiguration("challenge3_start_yaw_deg")
    challenge3_scan_topic = LaunchConfiguration("challenge3_scan_topic")
    challenge3_require_scan = LaunchConfiguration("challenge3_require_scan")
    challenge3_lidar_min_cluster_points = LaunchConfiguration("challenge3_lidar_min_cluster_points")
    challenge3_lidar_cluster_max_gap_m = LaunchConfiguration("challenge3_lidar_cluster_max_gap_m")
    challenge3_field_width_m = LaunchConfiguration("challenge3_field_width_m")
    challenge3_field_height_m = LaunchConfiguration("challenge3_field_height_m")
    challenge3_field_margin_m = LaunchConfiguration("challenge3_field_margin_m")
    challenge3_lane_offsets_m = LaunchConfiguration("challenge3_lane_offsets_m")
    challenge3_obstacle_lookahead_m = LaunchConfiguration("challenge3_obstacle_lookahead_m")
    challenge3_route_corridor_half_width_m = LaunchConfiguration("challenge3_route_corridor_half_width_m")
    challenge3_emergency_stop_m = LaunchConfiguration("challenge3_emergency_stop_m")
    challenge3_obstacle_memory_ttl_s = LaunchConfiguration("challenge3_obstacle_memory_ttl_s")
    challenge3_obstacle_passed_behind_m = LaunchConfiguration("challenge3_obstacle_passed_behind_m")
    challenge3_lane_change_distance_m = LaunchConfiguration("challenge3_lane_change_distance_m")
    challenge3_rejoin_distance_m = LaunchConfiguration("challenge3_rejoin_distance_m")
    challenge3_lookahead_m = LaunchConfiguration("challenge3_lookahead_m")
    challenge3_cruise_speed_mps = LaunchConfiguration("challenge3_cruise_speed_mps")
    challenge3_hard_turn_speed_mps = LaunchConfiguration("challenge3_hard_turn_speed_mps")
    challenge3_hard_turn_max_omega_radps = LaunchConfiguration("challenge3_hard_turn_max_omega_radps")
    challenge3_hard_turn_error_rad = LaunchConfiguration("challenge3_hard_turn_error_rad")
    challenge3_cruise_heading_kp = LaunchConfiguration("challenge3_cruise_heading_kp")
    challenge3_cruise_max_omega_radps = LaunchConfiguration("challenge3_cruise_max_omega_radps")
    nav_max_omega_radps = LaunchConfiguration("nav_max_omega_radps")
    nav_heading_kp = LaunchConfiguration("nav_heading_kp")
    nav_heading_kd = LaunchConfiguration("nav_heading_kd")
    nav_pivot_kp = LaunchConfiguration("nav_pivot_kp")
    nav_heading_deadband_rad = LaunchConfiguration("nav_heading_deadband_rad")
    nav_stop_clearance_m = LaunchConfiguration("nav_stop_clearance_m")
    nav_sensor_timeout_s = LaunchConfiguration("nav_sensor_timeout_s")
    nav_motor_status_timeout_s = LaunchConfiguration("nav_motor_status_timeout_s")
    nav_max_test_duration_s = LaunchConfiguration("nav_max_test_duration_s")
    nav_gyro_bias_calibration_s = LaunchConfiguration("nav_gyro_bias_calibration_s")
    nav_gyro_bias_max_std_radps = LaunchConfiguration("nav_gyro_bias_max_std_radps")
    nav_gyro_bias_warn_abs_radps = LaunchConfiguration("nav_gyro_bias_warn_abs_radps")
    nav_gyro_bias_max_encoder_delta_ticks = LaunchConfiguration("nav_gyro_bias_max_encoder_delta_ticks")
    nav_pivot_max_omega_radps = LaunchConfiguration("nav_pivot_max_omega_radps")
    nav_pivot_min_omega_radps = LaunchConfiguration("nav_pivot_min_omega_radps")
    nav_pivot_breakaway_omega_radps = LaunchConfiguration("nav_pivot_breakaway_omega_radps")
    nav_pivot_breakaway_s = LaunchConfiguration("nav_pivot_breakaway_s")
    nav_pivot_accel_limit_radps2 = LaunchConfiguration("nav_pivot_accel_limit_radps2")
    nav_pivot_decel_limit_radps2 = LaunchConfiguration("nav_pivot_decel_limit_radps2")
    nav_pivot_approach_error_rad = LaunchConfiguration("nav_pivot_approach_error_rad")
    nav_pivot_min_omega_disable_error_rad = LaunchConfiguration("nav_pivot_min_omega_disable_error_rad")
    nav_pivot_kp_approach = LaunchConfiguration("nav_pivot_kp_approach")
    nav_pivot_kd_yaw_rate = LaunchConfiguration("nav_pivot_kd_yaw_rate")
    nav_pivot_settle_error_rad = LaunchConfiguration("nav_pivot_settle_error_rad")
    nav_pivot_settle_yaw_rate_radps = LaunchConfiguration("nav_pivot_settle_yaw_rate_radps")
    nav_pivot_settle_time_s = LaunchConfiguration("nav_pivot_settle_time_s")
    nav_pivot_brake_s = LaunchConfiguration("nav_pivot_brake_s")
    nav_pivot_timeout_s = LaunchConfiguration("nav_pivot_timeout_s")
    nav_pivot_max_correction_retries = LaunchConfiguration("nav_pivot_max_correction_retries")
    nav_pivot_clearance_m = LaunchConfiguration("nav_pivot_clearance_m")
    nav_slip_disagreement_rad = LaunchConfiguration("nav_slip_disagreement_rad")
    nav_mission_file = LaunchConfiguration("nav_mission_file")
    nav_competition_min_speed_mps = LaunchConfiguration("nav_competition_min_speed_mps")
    nav_competition_moving_target_speed_mps = LaunchConfiguration("nav_competition_moving_target_speed_mps")
    nav_competition_continuous_motion_enabled = LaunchConfiguration("nav_competition_continuous_motion_enabled")
    nav_mission_default_speed_mps = LaunchConfiguration("nav_mission_default_speed_mps")
    nav_mission_reliable_speed_mps = LaunchConfiguration("nav_mission_reliable_speed_mps")
    nav_mission_slow_speed_mps = LaunchConfiguration("nav_mission_slow_speed_mps")
    nav_mission_emergency_stop_clearance_m = LaunchConfiguration("nav_mission_emergency_stop_clearance_m")
    nav_mission_critical_sensor_timeout_s = LaunchConfiguration("nav_mission_critical_sensor_timeout_s")
    nav_mission_straight_max_omega_radps = LaunchConfiguration("nav_mission_straight_max_omega_radps")
    nav_mission_straight_omega_slew_radps2 = LaunchConfiguration("nav_mission_straight_omega_slew_radps2")
    nav_debug_allow_sub_min_crawl = LaunchConfiguration("nav_debug_allow_sub_min_crawl")
    nav_debug_allow_unknown_args = LaunchConfiguration("nav_debug_allow_unknown_args")
    nav_debug_allow_unknown_pivot_clearance = LaunchConfiguration("nav_debug_allow_unknown_pivot_clearance")
    nav_debug_ignore_nav_frame = LaunchConfiguration("nav_debug_ignore_nav_frame")
    nav_debug_ignore_obstacles = LaunchConfiguration("nav_debug_ignore_obstacles")
    nav_mission_stop_on_degraded_obstacle = LaunchConfiguration("nav_mission_stop_on_degraded_obstacle")
    nav_mission_telemetry_active_hz = LaunchConfiguration("nav_mission_telemetry_active_hz")
    nav_mission_telemetry_flush_period_s = LaunchConfiguration("nav_mission_telemetry_flush_period_s")
    nav_mission_telemetry_flush_max_records = LaunchConfiguration("nav_mission_telemetry_flush_max_records")
    nav_imu_rate_window_s = LaunchConfiguration("nav_imu_rate_window_s")
    nav_stuck_detection_enabled = LaunchConfiguration("nav_stuck_detection_enabled")
    nav_straight_stuck_timeout_s = LaunchConfiguration("nav_straight_stuck_timeout_s")
    nav_pivot_stuck_timeout_s = LaunchConfiguration("nav_pivot_stuck_timeout_s")
    nav_pivot_breakaway_retry_scale = LaunchConfiguration("nav_pivot_breakaway_retry_scale")
    nav_telemetry_enabled = LaunchConfiguration("nav_telemetry_enabled")
    nav_telemetry_dir = LaunchConfiguration("nav_telemetry_dir")

    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(motor_launch)),
        launch_arguments={
            "port": motor_port,
            "baud": motor_baud,
            "dry_run": motor_dry_run,
            "command_timeout_s": motor_command_timeout_s,
            "command_refresh_period_s": motor_command_refresh_period_s,
            "track_width_m": motor_track_width_m,
            "left_forward_speed_scale": motor_left_forward_speed_scale,
            "right_forward_speed_scale": motor_right_forward_speed_scale,
            "left_reverse_speed_scale": motor_left_reverse_speed_scale,
            "right_reverse_speed_scale": motor_right_reverse_speed_scale,
            "wheel_radius_m": motor_wheel_radius_m,
            "ticks_per_rev": motor_ticks_per_rev,
            "pwm_min_us": motor_pwm_min_us,
            "pwm_neutral_us": motor_pwm_neutral_us,
            "pwm_max_us": motor_pwm_max_us,
            "pwm_slew_rate_us_per_s": motor_pwm_slew_rate_us_per_s,
            "teensy_control_hz": motor_teensy_control_hz,
            "teensy_pid_kp": motor_teensy_pid_kp,
            "teensy_pid_ki": motor_teensy_pid_ki,
            "teensy_pid_kd": motor_teensy_pid_kd,
            "teensy_pid_feedforward_us_per_tps": motor_teensy_pid_feedforward_us_per_tps,
            "enable_teensy_side_specific_pid_params": motor_enable_teensy_side_specific_pid_params,
            "teensy_left_pid_feedforward_us_per_tps": motor_teensy_left_pid_feedforward_us_per_tps,
            "teensy_right_pid_feedforward_us_per_tps": motor_teensy_right_pid_feedforward_us_per_tps,
            "teensy_right_reverse_pid_feedforward_us_per_tps": (
                motor_teensy_right_reverse_pid_feedforward_us_per_tps
            ),
            "teensy_pid_static_ff_us": motor_teensy_pid_static_ff_us,
            "teensy_pid_static_ff_full_target_tps": motor_teensy_pid_static_ff_full_target_tps,
            "teensy_pid_static_ff_fade_start_ratio": motor_teensy_pid_static_ff_fade_start_ratio,
            "teensy_pid_static_ff_fade_end_ratio": motor_teensy_pid_static_ff_fade_end_ratio,
            "teensy_left_pid_static_ff_us": motor_teensy_left_pid_static_ff_us,
            "teensy_right_pid_static_ff_us": motor_teensy_right_pid_static_ff_us,
            "teensy_right_reverse_pid_static_ff_us": motor_teensy_right_reverse_pid_static_ff_us,
            "teensy_right_reverse_pwm_floor_us": motor_teensy_right_reverse_pwm_floor_us,
            "teensy_pid_output_limit_us": motor_teensy_pid_output_limit_us,
            "teensy_left_pid_output_limit_us": motor_teensy_left_pid_output_limit_us,
            "teensy_right_pid_output_limit_us": motor_teensy_right_pid_output_limit_us,
            "teensy_pid_min_target_tps": motor_teensy_pid_min_target_tps,
            "teensy_left_motor_sign": motor_teensy_left_motor_sign,
            "teensy_right_motor_sign": motor_teensy_right_motor_sign,
            "teensy_fl_encoder_sign": motor_teensy_fl_encoder_sign,
            "teensy_fr_encoder_sign": motor_teensy_fr_encoder_sign,
            "teensy_rl_encoder_sign": motor_teensy_rl_encoder_sign,
            "teensy_rr_encoder_sign": motor_teensy_rr_encoder_sign,
            "teensy_side_mismatch_warn_tps": motor_teensy_side_mismatch_warn_tps,
            "teensy_side_mismatch_fault_tps": motor_teensy_side_mismatch_fault_tps,
            "teensy_encoder_jump_tps": motor_teensy_encoder_jump_tps,
            "teensy_pid_param_ack_timeout_s": motor_teensy_pid_param_ack_timeout_s,
            "teensy_pid_param_write_interval_s": motor_teensy_pid_param_write_interval_s,
        }.items(),
        condition=IfCondition(start_motor_controller),
    )

    sensor_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sensor_launch)),
        launch_arguments={
            "start_zed": start_zed,
            "start_lidar": start_lidar,
            "start_lidar_filter": start_lidar_filter,
            "start_fusion": start_fusion,
            "start_debug_status": start_debug_status,
            "start_uwb": "false",
            "lidar_port": lidar_port,
            "lidar_baud": lidar_baud,
            "lidar_filter_forward_fov_deg": lidar_filter_forward_fov_deg,
            "fusion_lidar_front_min_cluster_points": fusion_lidar_front_min_cluster_points,
            "fusion_lidar_front_cluster_max_gap_m": fusion_lidar_front_cluster_max_gap_m,
            "zed_publish_rate_hz": zed_publish_rate_hz,
            "zed_imu_publish_rate_hz": zed_imu_publish_rate_hz,
            "zed_imu_rate_window_s": zed_imu_rate_window_s,
            "zed_depth_downsample_factor": zed_depth_downsample_factor,
            "zed_image_downsample_factor": zed_image_downsample_factor,
            "zed_publish_image": zed_publish_image,
        }.items(),
        condition=IfCondition(start_sensor_sync),
    )

    target_receiver_node = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(target_receiver),
            "--ros-args",
            "-p",
            ["input_mode:=", uav_target_input_mode],
            "-p",
            ["output_topic:=", nav_target_topic],
            "-p",
            ["frame_id:=", uav_target_frame_id],
            "-p",
            ["target_units:=", uav_target_units],
            "-p",
            ["serial_port:=", uav_esp_serial_port],
            "-p",
            ["serial_baud:=", uav_esp_serial_baud],
            "-p",
            ["serial_protocol:=", uav_esp_serial_protocol],
            "-p",
            ["require_checksum:=", uav_esp_require_checksum],
        ],
        output="screen",
        condition=IfCondition(start_uav_target_receiver),
    )

    nav_controller = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(nav_script),
            "--mode",
            "real",
            "--nav-status-period-s",
            nav_status_period_s,
            "--control-period-s",
            nav_control_period_s,
            "--controller-mode",
            nav_controller_mode,
            "--allow-legacy-controller",
            nav_allow_legacy_controller,
            "--nav-frame-topic",
            nav_frame_topic,
            "--imu-topic",
            nav_imu_topic,
            "--imu-qos",
            nav_imu_qos,
            "--imu-yaw-axis",
            nav_imu_yaw_axis,
            "--imu-yaw-sign",
            nav_imu_yaw_sign,
            "--imu-min-rate-hz",
            nav_imu_min_rate_hz,
            "--imu-timeout-s",
            nav_imu_timeout_s,
            "--motor-status-topic",
            nav_motor_status_topic,
            "--encoder-stamped-topic",
            nav_encoder_stamped_topic,
            "--zed-status-topic",
            nav_zed_status_topic,
            "--allow-encoder-heading-fallback",
            nav_allow_encoder_heading_fallback,
            "--straight-speed-mps",
            nav_straight_speed_mps,
            "--straight-duration-s",
            nav_straight_duration_s,
            "--pivot-angle-deg",
            nav_pivot_angle_deg,
            "--curve-angle-deg",
            nav_curve_angle_deg,
            "--curve-speed-mps",
            nav_curve_speed_mps,
            "--curve-radius-m",
            nav_curve_radius_m,
            "--curve-omega-radps",
            nav_curve_omega_radps,
            "--curve-direction",
            nav_curve_direction,
            "--arc-min-turn-radius-m",
            nav_arc_min_turn_radius_m,
            "--arc-max-omega-radps",
            nav_arc_max_omega_radps,
            "--curve-omega-slew-radps2",
            nav_curve_omega_slew_radps2,
            "--curve-timeout-s",
            nav_curve_timeout_s,
            "--curve-no-progress-timeout-s",
            nav_curve_no_progress_timeout_s,
            "--curve-min-progress-rad",
            nav_curve_min_progress_rad,
            "--curve-approach-error-rad",
            nav_curve_approach_error_rad,
            "--curve-kp-approach",
            nav_curve_kp_approach,
            "--curve-kd-yaw-rate",
            nav_curve_kd_yaw_rate,
            "--curve-min-omega-radps",
            nav_curve_min_omega_radps,
            "--curve-min-omega-disable-error-rad",
            nav_curve_min_omega_disable_error_rad,
            "--allow-side-reverse",
            nav_allow_side_reverse,
            "--target-topic",
            nav_target_topic,
            "--uav-launched-topic",
            nav_uav_launched_topic,
            "--uav-landed-topic",
            nav_uav_landed_topic,
            "--manual-target-x-m",
            nav_manual_target_x_m,
            "--manual-target-y-m",
            nav_manual_target_y_m,
            "--tracking-enabled",
            nav_tracking_enabled,
            "--target-stop-radius-m",
            nav_target_stop_radius_m,
            "--tracking-lookahead-min-m",
            nav_tracking_lookahead_min_m,
            "--tracking-lookahead-max-m",
            nav_tracking_lookahead_max_m,
            "--tracking-lookahead-speed-gain",
            nav_tracking_lookahead_speed_gain,
            "--tracking-nominal-speed-mps",
            nav_tracking_nominal_speed_mps,
            "--tracking-max-speed-mps",
            nav_tracking_max_speed_mps,
            "--tracking-max-omega-radps",
            nav_tracking_max_omega_radps,
            "--tracking-heading-kp",
            nav_tracking_heading_kp,
            "--tracking-cross-track-kp",
            nav_tracking_cross_track_kp,
            "--tracking-slowdown-distance-m",
            nav_tracking_slowdown_distance_m,
            "--obstacle-warn-m",
            nav_obstacle_warn_m,
            "--obstacle-stop-m",
            nav_obstacle_stop_m,
            "--bypass-offset-m",
            nav_bypass_offset_m,
            "--bypass-forward-m",
            nav_bypass_forward_m,
            "--bypass-rejoin-ahead-m",
            nav_bypass_rejoin_ahead_m,
            "--challenge1-auto-start",
            nav_challenge1_auto_start,
            "--challenge1-speed-mps",
            nav_challenge1_speed_mps,
            "--challenge1-post-landing-s",
            nav_challenge1_post_landing_s,
            "--challenge1-timeout-s",
            nav_challenge1_timeout_s,
            "--challenge1-max-distance-m",
            nav_challenge1_max_distance_m,
            "--challenge1-stop-on-obstacle",
            nav_challenge1_stop_on_obstacle,
            "--challenge2-speed-mps",
            nav_challenge2_speed_mps,
            "--challenge2-approach-speed-mps",
            nav_challenge2_approach_speed_mps,
            "--challenge2-slowdown-distance-m",
            nav_challenge2_slowdown_distance_m,
            "--challenge2-stop-radius-m",
            nav_challenge2_stop_radius_m,
            "--challenge2-post-landing-s",
            nav_challenge2_post_landing_s,
            "--challenge2-start-pose-set",
            nav_challenge2_start_pose_set,
            "--challenge2-require-start-pose",
            nav_challenge2_require_start_pose,
            "--challenge2-start-x-m",
            nav_challenge2_start_x_m,
            "--challenge2-start-y-m",
            nav_challenge2_start_y_m,
            "--challenge2-start-yaw-deg",
            nav_challenge2_start_yaw_deg,
            "--challenge2-pivot-max-omega-radps",
            nav_challenge2_pivot_max_omega_radps,
            "--challenge2-pivot-timeout-s",
            nav_challenge2_pivot_timeout_s,
            "--challenge2-pivot-settle-error-rad",
            nav_challenge2_pivot_settle_error_rad,
            "--challenge2-pivot-settle-time-s",
            nav_challenge2_pivot_settle_time_s,
            "--challenge2-align-arc-speed-mps",
            nav_challenge2_align_arc_speed_mps,
            "--challenge2-align-max-omega-radps",
            nav_challenge2_align_max_omega_radps,
            "--challenge2-align-min-turn-radius-m",
            nav_challenge2_align_min_turn_radius_m,
            "--challenge2-align-heading-kp",
            nav_challenge2_align_heading_kp,
            "--challenge2-align-no-progress-timeout-s",
            nav_challenge2_align_no_progress_timeout_s,
            "--challenge2-align-close-guard-m",
            nav_challenge2_align_close_guard_m,
            "--challenge2-heading-kp",
            nav_challenge2_heading_kp,
            "--challenge2-cross-track-kp",
            nav_challenge2_cross_track_kp,
            "--challenge2-max-omega-radps",
            nav_challenge2_max_omega_radps,
            "--max-omega-radps",
            nav_max_omega_radps,
            "--heading-kp",
            nav_heading_kp,
            "--heading-kd",
            nav_heading_kd,
            "--pivot-kp",
            nav_pivot_kp,
            "--heading-deadband-rad",
            nav_heading_deadband_rad,
            "--stop-clearance-m",
            nav_stop_clearance_m,
            "--sensor-timeout-s",
            nav_sensor_timeout_s,
            "--motor-status-timeout-s",
            nav_motor_status_timeout_s,
            "--max-test-duration-s",
            nav_max_test_duration_s,
            "--gyro-bias-calibration-s",
            nav_gyro_bias_calibration_s,
            "--gyro-bias-max-std-radps",
            nav_gyro_bias_max_std_radps,
            "--gyro-bias-warn-abs-radps",
            nav_gyro_bias_warn_abs_radps,
            "--gyro-bias-max-encoder-delta-ticks",
            nav_gyro_bias_max_encoder_delta_ticks,
            "--pivot-max-omega-radps",
            nav_pivot_max_omega_radps,
            "--pivot-min-omega-radps",
            nav_pivot_min_omega_radps,
            "--pivot-breakaway-omega-radps",
            nav_pivot_breakaway_omega_radps,
            "--pivot-breakaway-s",
            nav_pivot_breakaway_s,
            "--pivot-accel-limit-radps2",
            nav_pivot_accel_limit_radps2,
            "--pivot-decel-limit-radps2",
            nav_pivot_decel_limit_radps2,
            "--pivot-approach-error-rad",
            nav_pivot_approach_error_rad,
            "--pivot-min-omega-disable-error-rad",
            nav_pivot_min_omega_disable_error_rad,
            "--pivot-kp-approach",
            nav_pivot_kp_approach,
            "--pivot-kd-yaw-rate",
            nav_pivot_kd_yaw_rate,
            "--pivot-settle-error-rad",
            nav_pivot_settle_error_rad,
            "--pivot-settle-yaw-rate-radps",
            nav_pivot_settle_yaw_rate_radps,
            "--pivot-settle-time-s",
            nav_pivot_settle_time_s,
            "--pivot-brake-s",
            nav_pivot_brake_s,
            "--pivot-timeout-s",
            nav_pivot_timeout_s,
            "--pivot-max-correction-retries",
            nav_pivot_max_correction_retries,
            "--pivot-clearance-m",
            nav_pivot_clearance_m,
            "--slip-disagreement-rad",
            nav_slip_disagreement_rad,
            "--mission-file",
            nav_mission_file,
            "--competition-min-speed-mps",
            nav_competition_min_speed_mps,
            "--competition-moving-target-speed-mps",
            nav_competition_moving_target_speed_mps,
            "--competition-continuous-motion-enabled",
            nav_competition_continuous_motion_enabled,
            "--mission-default-speed-mps",
            nav_mission_default_speed_mps,
            "--mission-reliable-speed-mps",
            nav_mission_reliable_speed_mps,
            "--mission-slow-speed-mps",
            nav_mission_slow_speed_mps,
            "--mission-emergency-stop-clearance-m",
            nav_mission_emergency_stop_clearance_m,
            "--mission-critical-sensor-timeout-s",
            nav_mission_critical_sensor_timeout_s,
            "--mission-straight-max-omega-radps",
            nav_mission_straight_max_omega_radps,
            "--mission-straight-omega-slew-radps2",
            nav_mission_straight_omega_slew_radps2,
            "--debug-allow-sub-min-crawl",
            nav_debug_allow_sub_min_crawl,
            "--debug-allow-unknown-args",
            nav_debug_allow_unknown_args,
            "--debug-allow-unknown-pivot-clearance",
            nav_debug_allow_unknown_pivot_clearance,
            "--debug-ignore-nav-frame",
            nav_debug_ignore_nav_frame,
            "--debug-ignore-obstacles",
            nav_debug_ignore_obstacles,
            "--mission-stop-on-degraded-obstacle",
            nav_mission_stop_on_degraded_obstacle,
            "--mission-telemetry-active-hz",
            nav_mission_telemetry_active_hz,
            "--mission-telemetry-flush-period-s",
            nav_mission_telemetry_flush_period_s,
            "--mission-telemetry-flush-max-records",
            nav_mission_telemetry_flush_max_records,
            "--imu-rate-window-s",
            nav_imu_rate_window_s,
            "--stuck-detection-enabled",
            nav_stuck_detection_enabled,
            "--straight-stuck-timeout-s",
            nav_straight_stuck_timeout_s,
            "--pivot-stuck-timeout-s",
            nav_pivot_stuck_timeout_s,
            "--pivot-breakaway-retry-scale",
            nav_pivot_breakaway_retry_scale,
            "--telemetry-enabled",
            nav_telemetry_enabled,
            "--telemetry-dir",
            nav_telemetry_dir,
            "--track-width-m",
            motor_track_width_m,
            "--wheel-radius-m",
            motor_wheel_radius_m,
            "--ticks-per-rev",
            motor_ticks_per_rev,
        ],
        output="screen",
        condition=IfCondition(start_nav),
    )

    challenge3_controller = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(challenge3_script),
            "--ros-args",
            "-p",
            ["command_topic:=", "/ugv_nav_cmd"],
            "-p",
            ["status_topic:=", "/ugv_nav_status"],
            "-p",
            ["target_topic:=", nav_target_topic],
            "-p",
            ["encoder_topic:=", nav_encoder_stamped_topic],
            "-p",
            ["imu_topic:=", nav_imu_topic],
            "-p",
            ["scan_topic:=", challenge3_scan_topic],
            "-p",
            ["motor_status_topic:=", nav_motor_status_topic],
            "-p",
            ["uav_landed_topic:=", nav_uav_landed_topic],
            "-p",
            ["control_period_s:=", nav_control_period_s],
            "-p",
            ["status_period_s:=", nav_status_period_s],
            "-p",
            ["start_pose_set:=", challenge3_start_pose_set],
            "-p",
            ["start_x_m:=", challenge3_start_x_m],
            "-p",
            ["start_y_m:=", challenge3_start_y_m],
            "-p",
            ["start_yaw_deg:=", challenge3_start_yaw_deg],
            "-p",
            ["imu_yaw_axis:=", nav_imu_yaw_axis],
            "-p",
            ["imu_yaw_sign:=", nav_imu_yaw_sign],
            "-p",
            ["imu_timeout_s:=", nav_imu_timeout_s],
            "-p",
            ["imu_min_rate_hz:=", nav_imu_min_rate_hz],
            "-p",
            ["imu_rate_window_s:=", nav_imu_rate_window_s],
            "-p",
            ["encoder_timeout_s:=", nav_motor_status_timeout_s],
            "-p",
            ["scan_timeout_s:=", nav_sensor_timeout_s],
            "-p",
            ["motor_status_timeout_s:=", nav_motor_status_timeout_s],
            "-p",
            ["require_scan:=", challenge3_require_scan],
            "-p",
            ["lidar_min_cluster_points:=", challenge3_lidar_min_cluster_points],
            "-p",
            ["lidar_cluster_max_gap_m:=", challenge3_lidar_cluster_max_gap_m],
            "-p",
            ["gyro_bias_calibration_s:=", nav_gyro_bias_calibration_s],
            "-p",
            ["gyro_bias_max_std_radps:=", nav_gyro_bias_max_std_radps],
            "-p",
            ["gyro_bias_max_encoder_delta_ticks:=", nav_gyro_bias_max_encoder_delta_ticks],
            "-p",
            ["field_width_m:=", challenge3_field_width_m],
            "-p",
            ["field_height_m:=", challenge3_field_height_m],
            "-p",
            ["field_margin_m:=", challenge3_field_margin_m],
            "-p",
            ["lane_offsets_m:=", challenge3_lane_offsets_m],
            "-p",
            ["obstacle_lookahead_m:=", challenge3_obstacle_lookahead_m],
            "-p",
            ["route_corridor_half_width_m:=", challenge3_route_corridor_half_width_m],
            "-p",
            ["emergency_stop_m:=", challenge3_emergency_stop_m],
            "-p",
            ["obstacle_memory_ttl_s:=", challenge3_obstacle_memory_ttl_s],
            "-p",
            ["obstacle_passed_behind_m:=", challenge3_obstacle_passed_behind_m],
            "-p",
            ["lane_change_distance_m:=", challenge3_lane_change_distance_m],
            "-p",
            ["rejoin_distance_m:=", challenge3_rejoin_distance_m],
            "-p",
            ["lookahead_m:=", challenge3_lookahead_m],
            "-p",
            ["cruise_speed_mps:=", challenge3_cruise_speed_mps],
            "-p",
            ["hard_turn_speed_mps:=", challenge3_hard_turn_speed_mps],
            "-p",
            ["hard_turn_max_omega_radps:=", challenge3_hard_turn_max_omega_radps],
            "-p",
            ["hard_turn_error_rad:=", challenge3_hard_turn_error_rad],
            "-p",
            ["cruise_heading_kp:=", challenge3_cruise_heading_kp],
            "-p",
            ["cruise_max_omega_radps:=", challenge3_cruise_max_omega_radps],
            "-p",
            ["track_width_m:=", motor_track_width_m],
            "-p",
            ["wheel_radius_m:=", motor_wheel_radius_m],
            "-p",
            ["ticks_per_rev:=", motor_ticks_per_rev],
        ],
        output="screen",
        condition=IfCondition(start_challenge3_corridor),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_motor_controller", default_value="true"),
            DeclareLaunchArgument("start_sensor_sync", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("start_zed", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_lidar_filter", default_value="true"),
            DeclareLaunchArgument("start_fusion", default_value="true"),
            DeclareLaunchArgument("start_debug_status", default_value="true"),
            DeclareLaunchArgument("start_uav_target_receiver", default_value="false"),
            DeclareLaunchArgument("start_challenge3_corridor", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_baud", default_value="115200"),
            DeclareLaunchArgument("lidar_filter_forward_fov_deg", default_value="230.0"),
            DeclareLaunchArgument("fusion_lidar_front_min_cluster_points", default_value="3"),
            DeclareLaunchArgument("fusion_lidar_front_cluster_max_gap_m", default_value="0.35"),
            DeclareLaunchArgument("zed_publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("zed_imu_publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("zed_imu_rate_window_s", default_value="2.0"),
            DeclareLaunchArgument("zed_depth_downsample_factor", default_value="2"),
            DeclareLaunchArgument("zed_image_downsample_factor", default_value="1"),
            DeclareLaunchArgument("zed_publish_image", default_value="false"),
            DeclareLaunchArgument("motor_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("motor_baud", default_value="115200"),
            DeclareLaunchArgument("motor_dry_run", default_value="false"),
            DeclareLaunchArgument("motor_command_timeout_s", default_value="0.75"),
            DeclareLaunchArgument("motor_command_refresh_period_s", default_value="0.10"),
            DeclareLaunchArgument("motor_track_width_m", default_value="0.416"),
            DeclareLaunchArgument("motor_left_forward_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("motor_right_forward_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("motor_left_reverse_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("motor_right_reverse_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("motor_wheel_radius_m", default_value="0.0825"),
            DeclareLaunchArgument("motor_ticks_per_rev", default_value="3200"),
            DeclareLaunchArgument("motor_pwm_min_us", default_value="1000"),
            DeclareLaunchArgument("motor_pwm_neutral_us", default_value="1500"),
            DeclareLaunchArgument("motor_pwm_max_us", default_value="2000"),
            DeclareLaunchArgument("motor_pwm_slew_rate_us_per_s", default_value="2400.0"),
            DeclareLaunchArgument("motor_teensy_control_hz", default_value="50.0"),
            DeclareLaunchArgument("motor_teensy_pid_kp", default_value="0.04"),
            DeclareLaunchArgument("motor_teensy_pid_ki", default_value="0.0"),
            DeclareLaunchArgument("motor_teensy_pid_kd", default_value="0.0"),
            DeclareLaunchArgument("motor_teensy_pid_feedforward_us_per_tps", default_value="0.08"),
            DeclareLaunchArgument("motor_enable_teensy_side_specific_pid_params", default_value="false"),
            DeclareLaunchArgument("motor_teensy_left_pid_feedforward_us_per_tps", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_pid_feedforward_us_per_tps", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_reverse_pid_feedforward_us_per_tps", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_pid_static_ff_us", default_value="410.0"),
            DeclareLaunchArgument("motor_teensy_pid_static_ff_full_target_tps", default_value="1500.0"),
            DeclareLaunchArgument("motor_teensy_pid_static_ff_fade_start_ratio", default_value="0.20"),
            DeclareLaunchArgument("motor_teensy_pid_static_ff_fade_end_ratio", default_value="0.85"),
            DeclareLaunchArgument("motor_teensy_left_pid_static_ff_us", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_pid_static_ff_us", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_reverse_pid_static_ff_us", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_reverse_pwm_floor_us", default_value="0.0"),
            DeclareLaunchArgument("motor_teensy_pid_output_limit_us", default_value="500.0"),
            DeclareLaunchArgument("motor_teensy_left_pid_output_limit_us", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_right_pid_output_limit_us", default_value="-1.0"),
            DeclareLaunchArgument("motor_teensy_pid_min_target_tps", default_value="2.0"),
            DeclareLaunchArgument("motor_teensy_left_motor_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_right_motor_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_fl_encoder_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_fr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_rl_encoder_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_rr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_side_mismatch_warn_tps", default_value="80.0"),
            DeclareLaunchArgument("motor_teensy_side_mismatch_fault_tps", default_value="180.0"),
            DeclareLaunchArgument("motor_teensy_encoder_jump_tps", default_value="12000.0"),
            DeclareLaunchArgument("motor_teensy_pid_param_ack_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("motor_teensy_pid_param_write_interval_s", default_value="0.03"),
            DeclareLaunchArgument("nav_status_period_s", default_value="0.25"),
            DeclareLaunchArgument("nav_control_period_s", default_value="0.02"),
            DeclareLaunchArgument("nav_controller_mode", default_value="competition_tracker"),
            DeclareLaunchArgument("nav_allow_legacy_controller", default_value="false"),
            DeclareLaunchArgument("nav_frame_topic", default_value="/sensors/nav_frame"),
            DeclareLaunchArgument("nav_imu_topic", default_value="/zed/imu"),
            DeclareLaunchArgument("nav_imu_qos", default_value="sensor_data"),
            DeclareLaunchArgument("nav_imu_yaw_axis", default_value="y"),
            DeclareLaunchArgument("nav_imu_yaw_sign", default_value="-1.0"),
            DeclareLaunchArgument("nav_imu_timeout_s", default_value="0.30"),
            DeclareLaunchArgument("nav_imu_min_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("nav_motor_status_topic", default_value="/motor_controller/status"),
            DeclareLaunchArgument("nav_encoder_stamped_topic", default_value="/encoder_ticks_stamped"),
            DeclareLaunchArgument("nav_zed_status_topic", default_value="/zed/status"),
            DeclareLaunchArgument("nav_allow_encoder_heading_fallback", default_value="false"),
            DeclareLaunchArgument("nav_straight_speed_mps", default_value="0.20"),
            DeclareLaunchArgument("nav_straight_duration_s", default_value="2.0"),
            DeclareLaunchArgument("nav_pivot_angle_deg", default_value="90.0"),
            DeclareLaunchArgument("nav_curve_angle_deg", default_value="90.0"),
            DeclareLaunchArgument("nav_curve_speed_mps", default_value="0.15"),
            DeclareLaunchArgument("nav_curve_radius_m", default_value="1.0"),
            DeclareLaunchArgument("nav_curve_omega_radps", default_value="0.0"),
            DeclareLaunchArgument("nav_curve_direction", default_value="left"),
            DeclareLaunchArgument("nav_arc_min_turn_radius_m", default_value="0.75"),
            DeclareLaunchArgument("nav_arc_max_omega_radps", default_value="0.45"),
            DeclareLaunchArgument("nav_curve_omega_slew_radps2", default_value="0.80"),
            DeclareLaunchArgument("nav_curve_timeout_s", default_value="0.0"),
            DeclareLaunchArgument("nav_curve_no_progress_timeout_s", default_value="1.5"),
            DeclareLaunchArgument("nav_curve_min_progress_rad", default_value="0.025"),
            DeclareLaunchArgument("nav_curve_approach_error_rad", default_value="0.25"),
            DeclareLaunchArgument("nav_curve_kp_approach", default_value="0.90"),
            DeclareLaunchArgument("nav_curve_kd_yaw_rate", default_value="0.08"),
            DeclareLaunchArgument("nav_curve_min_omega_radps", default_value="0.14"),
            DeclareLaunchArgument("nav_curve_min_omega_disable_error_rad", default_value="0.08"),
            DeclareLaunchArgument("nav_allow_side_reverse", default_value="false"),
            DeclareLaunchArgument("nav_target_topic", default_value="/ugv/uav_target"),
            DeclareLaunchArgument("uav_target_input_mode", default_value="serial"),
            DeclareLaunchArgument("uav_target_units", default_value="meters"),
            DeclareLaunchArgument("uav_target_frame_id", default_value="map"),
            DeclareLaunchArgument("uav_esp_serial_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("uav_esp_serial_baud", default_value="115200"),
            DeclareLaunchArgument("uav_esp_serial_protocol", default_value="binary14"),
            DeclareLaunchArgument("uav_esp_require_checksum", default_value="false"),
            DeclareLaunchArgument("nav_uav_launched_topic", default_value="/ugv/uav_launched"),
            DeclareLaunchArgument("nav_uav_landed_topic", default_value="/ugv/uav_landed"),
            DeclareLaunchArgument("nav_manual_target_x_m", default_value="0.0"),
            DeclareLaunchArgument("nav_manual_target_y_m", default_value="0.0"),
            DeclareLaunchArgument("nav_tracking_enabled", default_value="true"),
            DeclareLaunchArgument("nav_target_stop_radius_m", default_value="0.75"),
            DeclareLaunchArgument("nav_tracking_lookahead_min_m", default_value="0.45"),
            DeclareLaunchArgument("nav_tracking_lookahead_max_m", default_value="1.20"),
            DeclareLaunchArgument("nav_tracking_lookahead_speed_gain", default_value="1.8"),
            DeclareLaunchArgument("nav_tracking_nominal_speed_mps", default_value="0.25"),
            DeclareLaunchArgument("nav_tracking_max_speed_mps", default_value="0.42"),
            DeclareLaunchArgument("nav_tracking_max_omega_radps", default_value="0.85"),
            DeclareLaunchArgument("nav_tracking_heading_kp", default_value="0.85"),
            DeclareLaunchArgument("nav_tracking_cross_track_kp", default_value="0.75"),
            DeclareLaunchArgument("nav_tracking_slowdown_distance_m", default_value="1.20"),
            DeclareLaunchArgument("nav_obstacle_warn_m", default_value="2.0"),
            DeclareLaunchArgument("nav_obstacle_stop_m", default_value="1.0"),
            DeclareLaunchArgument("nav_bypass_offset_m", default_value="1.1"),
            DeclareLaunchArgument("nav_bypass_forward_m", default_value="2.0"),
            DeclareLaunchArgument("nav_bypass_rejoin_ahead_m", default_value="2.0"),
            DeclareLaunchArgument("nav_challenge1_auto_start", default_value="true"),
            DeclareLaunchArgument("nav_challenge1_speed_mps", default_value="0.24"),
            DeclareLaunchArgument("nav_challenge1_post_landing_s", default_value="40.0"),
            DeclareLaunchArgument("nav_challenge1_timeout_s", default_value="420.0"),
            DeclareLaunchArgument("nav_challenge1_max_distance_m", default_value="0.0"),
            DeclareLaunchArgument("nav_challenge1_stop_on_obstacle", default_value="true"),
            DeclareLaunchArgument("nav_challenge2_speed_mps", default_value="0.24"),
            DeclareLaunchArgument("nav_challenge2_approach_speed_mps", default_value="0.12"),
            DeclareLaunchArgument("nav_challenge2_slowdown_distance_m", default_value="1.5"),
            DeclareLaunchArgument("nav_challenge2_stop_radius_m", default_value="0.75"),
            DeclareLaunchArgument("nav_challenge2_post_landing_s", default_value="10.0"),
            DeclareLaunchArgument("nav_challenge2_start_pose_set", default_value="false"),
            DeclareLaunchArgument("nav_challenge2_require_start_pose", default_value="true"),
            DeclareLaunchArgument("nav_challenge2_start_x_m", default_value="0.0"),
            DeclareLaunchArgument("nav_challenge2_start_y_m", default_value="0.0"),
            DeclareLaunchArgument("nav_challenge2_start_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("nav_challenge2_pivot_max_omega_radps", default_value="0.85"),
            DeclareLaunchArgument("nav_challenge2_pivot_timeout_s", default_value="25.0"),
            DeclareLaunchArgument("nav_challenge2_pivot_settle_error_rad", default_value="0.035"),
            DeclareLaunchArgument("nav_challenge2_pivot_settle_time_s", default_value="0.35"),
            DeclareLaunchArgument("nav_challenge2_align_arc_speed_mps", default_value="1.70"),
            DeclareLaunchArgument("nav_challenge2_align_max_omega_radps", default_value="7.80"),
            DeclareLaunchArgument("nav_challenge2_align_min_turn_radius_m", default_value="0.21795"),
            DeclareLaunchArgument("nav_challenge2_align_heading_kp", default_value="22.345354"),
            DeclareLaunchArgument("nav_challenge2_align_no_progress_timeout_s", default_value="4.0"),
            DeclareLaunchArgument("nav_challenge2_align_close_guard_m", default_value="0.40"),
            DeclareLaunchArgument("nav_challenge2_heading_kp", default_value="0.85"),
            DeclareLaunchArgument("nav_challenge2_cross_track_kp", default_value="0.75"),
            DeclareLaunchArgument("nav_challenge2_max_omega_radps", default_value="0.85"),
            DeclareLaunchArgument("challenge3_start_pose_set", default_value="false"),
            DeclareLaunchArgument("challenge3_start_x_m", default_value="0.0"),
            DeclareLaunchArgument("challenge3_start_y_m", default_value="0.0"),
            DeclareLaunchArgument("challenge3_start_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("challenge3_scan_topic", default_value="/scan/filtered"),
            DeclareLaunchArgument("challenge3_require_scan", default_value="true"),
            DeclareLaunchArgument("challenge3_lidar_min_cluster_points", default_value="3"),
            DeclareLaunchArgument("challenge3_lidar_cluster_max_gap_m", default_value="0.35"),
            DeclareLaunchArgument("challenge3_field_width_m", default_value="13.716"),
            DeclareLaunchArgument("challenge3_field_height_m", default_value="13.716"),
            DeclareLaunchArgument("challenge3_field_margin_m", default_value="0.45"),
            DeclareLaunchArgument("challenge3_lane_offsets_m", default_value="0.0,1.6,-1.6,2.2,-2.2"),
            DeclareLaunchArgument("challenge3_obstacle_lookahead_m", default_value="5.5"),
            DeclareLaunchArgument("challenge3_route_corridor_half_width_m", default_value="0.70"),
            DeclareLaunchArgument("challenge3_emergency_stop_m", default_value="0.65"),
            DeclareLaunchArgument("challenge3_obstacle_memory_ttl_s", default_value="8.0"),
            DeclareLaunchArgument("challenge3_obstacle_passed_behind_m", default_value="1.0"),
            DeclareLaunchArgument("challenge3_lane_change_distance_m", default_value="5.0"),
            DeclareLaunchArgument("challenge3_rejoin_distance_m", default_value="6.0"),
            DeclareLaunchArgument("challenge3_lookahead_m", default_value="2.4"),
            DeclareLaunchArgument("challenge3_cruise_speed_mps", default_value="0.24"),
            DeclareLaunchArgument("challenge3_hard_turn_speed_mps", default_value="1.70"),
            DeclareLaunchArgument("challenge3_hard_turn_max_omega_radps", default_value="7.80"),
            DeclareLaunchArgument("challenge3_hard_turn_error_rad", default_value="0.35"),
            DeclareLaunchArgument("challenge3_cruise_heading_kp", default_value="0.85"),
            DeclareLaunchArgument("challenge3_cruise_max_omega_radps", default_value="0.85"),
            DeclareLaunchArgument("nav_max_omega_radps", default_value="0.45"),
            DeclareLaunchArgument("nav_heading_kp", default_value="0.6"),
            DeclareLaunchArgument("nav_heading_kd", default_value="0.08"),
            DeclareLaunchArgument("nav_pivot_kp", default_value="1.0"),
            DeclareLaunchArgument("nav_heading_deadband_rad", default_value="0.025"),
            DeclareLaunchArgument("nav_stop_clearance_m", default_value="0.45"),
            DeclareLaunchArgument("nav_sensor_timeout_s", default_value="0.30"),
            DeclareLaunchArgument("nav_motor_status_timeout_s", default_value="0.50"),
            DeclareLaunchArgument("nav_max_test_duration_s", default_value="3.0"),
            DeclareLaunchArgument("nav_gyro_bias_calibration_s", default_value="1.5"),
            DeclareLaunchArgument("nav_gyro_bias_max_std_radps", default_value="0.03"),
            DeclareLaunchArgument("nav_gyro_bias_warn_abs_radps", default_value="0.10"),
            DeclareLaunchArgument("nav_gyro_bias_max_encoder_delta_ticks", default_value="2"),
            DeclareLaunchArgument("nav_pivot_max_omega_radps", default_value="0.35"),
            DeclareLaunchArgument("nav_pivot_min_omega_radps", default_value="0.16"),
            DeclareLaunchArgument("nav_pivot_breakaway_omega_radps", default_value="0.18"),
            DeclareLaunchArgument("nav_pivot_breakaway_s", default_value="0.20"),
            DeclareLaunchArgument("nav_pivot_accel_limit_radps2", default_value="0.80"),
            DeclareLaunchArgument("nav_pivot_decel_limit_radps2", default_value="0.60"),
            DeclareLaunchArgument("nav_pivot_approach_error_rad", default_value="0.25"),
            DeclareLaunchArgument("nav_pivot_min_omega_disable_error_rad", default_value="0.10"),
            DeclareLaunchArgument("nav_pivot_kp_approach", default_value="0.75"),
            DeclareLaunchArgument("nav_pivot_kd_yaw_rate", default_value="0.10"),
            DeclareLaunchArgument("nav_pivot_settle_error_rad", default_value="0.035"),
            DeclareLaunchArgument("nav_pivot_settle_yaw_rate_radps", default_value="0.05"),
            DeclareLaunchArgument("nav_pivot_settle_time_s", default_value="0.35"),
            DeclareLaunchArgument("nav_pivot_brake_s", default_value="0.15"),
            DeclareLaunchArgument("nav_pivot_timeout_s", default_value="4.0"),
            DeclareLaunchArgument("nav_pivot_max_correction_retries", default_value="1"),
            DeclareLaunchArgument("nav_pivot_clearance_m", default_value="0.35"),
            DeclareLaunchArgument("nav_slip_disagreement_rad", default_value="0.35"),
            DeclareLaunchArgument("nav_mission_file", default_value=""),
            DeclareLaunchArgument("nav_competition_min_speed_mps", default_value="0.0894"),
            DeclareLaunchArgument("nav_competition_moving_target_speed_mps", default_value="0.12"),
            DeclareLaunchArgument("nav_competition_continuous_motion_enabled", default_value="true"),
            DeclareLaunchArgument("nav_mission_default_speed_mps", default_value="0.15"),
            DeclareLaunchArgument("nav_mission_reliable_speed_mps", default_value="0.15"),
            DeclareLaunchArgument("nav_mission_slow_speed_mps", default_value="0.09"),
            DeclareLaunchArgument("nav_mission_emergency_stop_clearance_m", default_value="0.18"),
            DeclareLaunchArgument("nav_mission_critical_sensor_timeout_s", default_value="1.0"),
            DeclareLaunchArgument("nav_mission_straight_max_omega_radps", default_value="0.20"),
            DeclareLaunchArgument("nav_mission_straight_omega_slew_radps2", default_value="0.80"),
            DeclareLaunchArgument("nav_debug_allow_sub_min_crawl", default_value="false"),
            DeclareLaunchArgument("nav_debug_allow_unknown_args", default_value="false"),
            DeclareLaunchArgument("nav_debug_allow_unknown_pivot_clearance", default_value="false"),
            DeclareLaunchArgument("nav_debug_ignore_nav_frame", default_value="false"),
            DeclareLaunchArgument("nav_debug_ignore_obstacles", default_value="false"),
            DeclareLaunchArgument("nav_mission_stop_on_degraded_obstacle", default_value="true"),
            DeclareLaunchArgument("nav_mission_telemetry_active_hz", default_value="50.0"),
            DeclareLaunchArgument("nav_mission_telemetry_flush_period_s", default_value="0.50"),
            DeclareLaunchArgument("nav_mission_telemetry_flush_max_records", default_value="25"),
            DeclareLaunchArgument("nav_imu_rate_window_s", default_value="2.0"),
            DeclareLaunchArgument("nav_stuck_detection_enabled", default_value="true"),
            DeclareLaunchArgument("nav_straight_stuck_timeout_s", default_value="0.50"),
            DeclareLaunchArgument("nav_pivot_stuck_timeout_s", default_value="0.45"),
            DeclareLaunchArgument("nav_pivot_breakaway_retry_scale", default_value="1.25"),
            DeclareLaunchArgument("nav_telemetry_enabled", default_value="true"),
            DeclareLaunchArgument("nav_telemetry_dir", default_value="~/.ros/ugv_mission_logs"),
            motor_controller_launch,
            sensor_sync_launch,
            target_receiver_node,
            challenge3_controller,
            nav_controller,
        ]
    )
