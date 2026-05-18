import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_workspace_root() -> Path:
    env_ws = os.environ.get('UGV_WS')
    candidates = []
    if env_ws:
        candidates.append(Path(env_ws))

    here = Path(__file__).resolve()
    candidates.extend(here.parents)

    for base in candidates:
        nav_script = base / 'src' / 'ugv_nav' / 'ugv_nav_dual_mode.py'
        if nav_script.exists():
            return base

    raise RuntimeError(
        'Could not find workspace root containing src/ugv_nav/ugv_nav_dual_mode.py. '
        'Set UGV_WS before running this bring-up launch.'
    )


def generate_launch_description():
    workspace_root = _resolve_workspace_root()
    nav_script = str(workspace_root / 'src' / 'ugv_nav' / 'ugv_nav_dual_mode.py')
    marker_vision_script = str(
        workspace_root / 'src' / 'ugv_perception' / 'ugv_perception' / 'marker_vision_node.py'
    )
    marker_vision_test_script = str(
        workspace_root / 'src' / 'ugv_perception' / 'ugv_perception' / 'marker_vision_test_node.py'
    )
    yolo_semantic_obstacle_script = str(
        workspace_root / 'src' / 'ugv_perception' / 'ugv_perception' / 'yolo_semantic_obstacle_node.py'
    )
    debug_dashboard_script = str(
        workspace_root / 'src' / 'ugv_perception' / 'ugv_perception' / 'ugv_debug_dashboard.py'
    )
    sensor_sync_launch_file = str(workspace_root / 'src' / 'ugv_sensor_sync' / 'launch' / 'sensor_sync_launch.py')
    sensor_nodes_root = workspace_root / 'src' / 'ugv_sensor_sync' / 'ugv_sensor_sync_nodes'
    mock_field_map_script = str(sensor_nodes_root / 'mock_field_map_node.py')
    bench_goal_script = str(sensor_nodes_root / 'bench_goal_node.py')
    debug_status_script = str(sensor_nodes_root / 'debug_status_node.py')

    start_uwb = LaunchConfiguration('start_uwb')
    start_zed = LaunchConfiguration('start_zed')
    start_lidar = LaunchConfiguration('start_lidar')
    start_fusion = LaunchConfiguration('start_fusion')
    start_motor_controller = LaunchConfiguration('start_motor_controller')
    start_nav = LaunchConfiguration('start_nav')
    start_debug_status = LaunchConfiguration('start_debug_status')
    start_bench_goal = LaunchConfiguration('start_bench_goal')
    start_mock_field_map = LaunchConfiguration('start_mock_field_map')
    competition_mode = LaunchConfiguration('competition_mode')
    mission_mode = LaunchConfiguration('mission_mode')
    start_corner = LaunchConfiguration('start_corner')
    ugv_start_x_m = LaunchConfiguration('ugv_start_x_m')
    ugv_start_y_m = LaunchConfiguration('ugv_start_y_m')
    ugv_start_yaw_deg = LaunchConfiguration('ugv_start_yaw_deg')
    center_loiter_radius_m = LaunchConfiguration('center_loiter_radius_m')
    target_accept_radius_m = LaunchConfiguration('target_accept_radius_m')
    min_speed_mps = LaunchConfiguration('min_speed_mps')
    drive_speed_level = LaunchConfiguration('drive_speed_level')
    straight_distance_m = LaunchConfiguration('straight_distance_m')
    competition_mission_v2_enabled = LaunchConfiguration('competition_mission_v2_enabled')
    sweep_cell_size_m = LaunchConfiguration('sweep_cell_size_m')
    sweep_lane_spacing_m = LaunchConfiguration('sweep_lane_spacing_m')
    sweep_coverage_radius_m = LaunchConfiguration('sweep_coverage_radius_m')
    sweep_coverage_threshold = LaunchConfiguration('sweep_coverage_threshold')
    sweep_goal_timeout_s = LaunchConfiguration('sweep_goal_timeout_s')
    sweep_fail_limit = LaunchConfiguration('sweep_fail_limit')
    sweep_lane_tolerance_m = LaunchConfiguration('sweep_lane_tolerance_m')
    sweep_heading_tolerance_deg = LaunchConfiguration('sweep_heading_tolerance_deg')
    sweep_allow_pure_turn = LaunchConfiguration('sweep_allow_pure_turn')
    sweep_stall_action = LaunchConfiguration('sweep_stall_action')
    min_competition_speed_mps = LaunchConfiguration('min_competition_speed_mps')
    bench_goal_x_m = LaunchConfiguration('bench_goal_x_m')
    bench_goal_y_m = LaunchConfiguration('bench_goal_y_m')
    bench_goal_period_s = LaunchConfiguration('bench_goal_period_s')
    mock_marker_cell = LaunchConfiguration('mock_marker_cell')
    mock_obstacles_json = LaunchConfiguration('mock_obstacles_json')
    target_topic = LaunchConfiguration('target_topic')
    start_marker_vision = LaunchConfiguration('start_marker_vision')
    start_marker_vision_test = LaunchConfiguration('start_marker_vision_test')
    start_yolo_obstacles = LaunchConfiguration('start_yolo_obstacles')
    start_debug_dashboard = LaunchConfiguration('start_debug_dashboard')
    marker_model_path = LaunchConfiguration('marker_model_path')
    marker_model_max_descriptors = LaunchConfiguration('marker_model_max_descriptors')
    marker_min_good_matches = LaunchConfiguration('marker_min_good_matches')
    marker_confirmation_frames = LaunchConfiguration('marker_confirmation_frames')
    marker_confirmation_radius_m = LaunchConfiguration('marker_confirmation_radius_m')
    marker_size_m = LaunchConfiguration('marker_size_m')
    marker_min_projected_size_m = LaunchConfiguration('marker_min_projected_size_m')
    marker_max_projected_size_m = LaunchConfiguration('marker_max_projected_size_m')
    marker_enable_generic_detector = LaunchConfiguration('marker_enable_generic_detector')
    marker_generic_min_area_frac = LaunchConfiguration('marker_generic_min_area_frac')
    marker_generic_min_contrast = LaunchConfiguration('marker_generic_min_contrast')
    marker_generic_min_grid_score = LaunchConfiguration('marker_generic_min_grid_score')
    marker_generic_min_border_light_ratio = LaunchConfiguration('marker_generic_min_border_light_ratio')
    marker_vision_test_period_s = LaunchConfiguration('marker_vision_test_period_s')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_device = LaunchConfiguration('yolo_device')
    yolo_imgsz = LaunchConfiguration('yolo_imgsz')
    yolo_confidence = LaunchConfiguration('yolo_confidence')
    yolo_max_hz = LaunchConfiguration('yolo_max_hz')
    yolo_obstacle_classes = LaunchConfiguration('yolo_obstacle_classes')
    dashboard_update_hz = LaunchConfiguration('dashboard_update_hz')
    dashboard_camera_search_depth_m = LaunchConfiguration('dashboard_camera_search_depth_m')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_freq_hz = LaunchConfiguration('lidar_scan_freq_hz')
    zed_publish_rate_hz = LaunchConfiguration('zed_publish_rate_hz')
    zed_depth_downsample_factor = LaunchConfiguration('zed_depth_downsample_factor')
    zed_publish_image = LaunchConfiguration('zed_publish_image')
    fusion_zed_fresh_timeout_s = LaunchConfiguration('fusion_zed_fresh_timeout_s')
    fusion_allow_lidar_only = LaunchConfiguration('fusion_allow_lidar_only')
    fusion_depth_invalid_warn_frames = LaunchConfiguration('fusion_depth_invalid_warn_frames')
    fusion_lidar_front_fov_deg = LaunchConfiguration('fusion_lidar_front_fov_deg')
    fusion_depth_projection_stride_px = LaunchConfiguration('fusion_depth_projection_stride_px')
    fusion_depth_ground_filter_enabled = LaunchConfiguration('fusion_depth_ground_filter_enabled')
    fusion_depth_ground_min_delta_m = LaunchConfiguration('fusion_depth_ground_min_delta_m')
    fusion_depth_ground_ratio = LaunchConfiguration('fusion_depth_ground_ratio')
    fusion_depth_obstacle_min_component_height_px = LaunchConfiguration('fusion_depth_obstacle_min_component_height_px')
    fusion_depth_front_corridor_half_width_m = LaunchConfiguration('fusion_depth_front_corridor_half_width_m')
    fusion_imu_smoothing_alpha = LaunchConfiguration('fusion_imu_smoothing_alpha')
    motor_port = LaunchConfiguration('motor_port')
    motor_baud = LaunchConfiguration('motor_baud')
    motor_raw_command_scale_us = LaunchConfiguration('motor_raw_command_scale_us')
    motor_pwm_slew_rate_us_per_s = LaunchConfiguration('motor_pwm_slew_rate_us_per_s')
    motor_command_timeout_s = LaunchConfiguration('motor_command_timeout_s')
    motor_command_refresh_period_s = LaunchConfiguration('motor_command_refresh_period_s')
    motor_dry_run = LaunchConfiguration('motor_dry_run')
    motor_velocity_control_enabled = LaunchConfiguration('motor_velocity_control_enabled')
    motor_prefer_velocity_fields = LaunchConfiguration('motor_prefer_velocity_fields')
    motor_wheel_radius_m = LaunchConfiguration('motor_wheel_radius_m')
    motor_ticks_per_rev = LaunchConfiguration('motor_ticks_per_rev')
    motor_velocity_kp = LaunchConfiguration('motor_velocity_kp')
    motor_velocity_ki = LaunchConfiguration('motor_velocity_ki')
    motor_velocity_kd = LaunchConfiguration('motor_velocity_kd')
    motor_velocity_integral_limit = LaunchConfiguration('motor_velocity_integral_limit')
    motor_velocity_feedforward_raw_per_mps = LaunchConfiguration('motor_velocity_feedforward_raw_per_mps')
    motor_velocity_min_target_mps = LaunchConfiguration('motor_velocity_min_target_mps')
    motor_velocity_max_target_mps = LaunchConfiguration('motor_velocity_max_target_mps')
    motor_velocity_control_period_s = LaunchConfiguration('motor_velocity_control_period_s')
    motor_velocity_stale_encoder_timeout_s = LaunchConfiguration('motor_velocity_stale_encoder_timeout_s')
    motor_velocity_fallback_to_raw_without_encoder = LaunchConfiguration('motor_velocity_fallback_to_raw_without_encoder')
    motor_velocity_encoder_speed_filter_alpha = LaunchConfiguration('motor_velocity_encoder_speed_filter_alpha')
    motor_velocity_encoder_speed_max_mps = LaunchConfiguration('motor_velocity_encoder_speed_max_mps')
    motor_velocity_encoder_speed_min_dt_s = LaunchConfiguration('motor_velocity_encoder_speed_min_dt_s')
    motor_velocity_raw_fallback_floor_enabled = LaunchConfiguration('motor_velocity_raw_fallback_floor_enabled')
    motor_velocity_raw_fallback_min_wheel_raw = LaunchConfiguration('motor_velocity_raw_fallback_min_wheel_raw')
    motor_velocity_raw_fallback_min_turn_raw = LaunchConfiguration('motor_velocity_raw_fallback_min_turn_raw')
    motor_velocity_raw_fallback_min_target_raw = LaunchConfiguration('motor_velocity_raw_fallback_min_target_raw')
    min_motion_raw = LaunchConfiguration('min_motion_raw')
    recovery_turn_raw = LaunchConfiguration('recovery_turn_raw')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    mission_flag_topic = LaunchConfiguration('mission_flag_topic')
    use_imu_yaw = LaunchConfiguration('use_imu_yaw')
    imu_yaw_blend = LaunchConfiguration('imu_yaw_blend')
    imu_yaw_axis = LaunchConfiguration('imu_yaw_axis')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')
    robot_length_m = LaunchConfiguration('robot_length_m')
    robot_width_m = LaunchConfiguration('robot_width_m')
    robot_track_width_m = LaunchConfiguration('robot_track_width_m')
    robot_wheel_radius_m = LaunchConfiguration('robot_wheel_radius_m')
    robot_ticks_per_rev = LaunchConfiguration('robot_ticks_per_rev')
    robot_obstacle_buffer_m = LaunchConfiguration('robot_obstacle_buffer_m')
    lidar_offset_x_m = LaunchConfiguration('lidar_offset_x_m')
    lidar_offset_y_m = LaunchConfiguration('lidar_offset_y_m')
    lidar_used_fov_deg = LaunchConfiguration('lidar_used_fov_deg')
    allow_reverse = LaunchConfiguration('allow_reverse')
    front_safety_margin_m = LaunchConfiguration('front_safety_margin_m')
    rear_safety_margin_m = LaunchConfiguration('rear_safety_margin_m')
    local_plan_inflation_m = LaunchConfiguration('local_plan_inflation_m')
    active_scan_enabled = LaunchConfiguration('active_scan_enabled')
    active_scan_confirm_steps = LaunchConfiguration('active_scan_confirm_steps')
    active_scan_steps = LaunchConfiguration('active_scan_steps')
    active_scan_panoramic_steps = LaunchConfiguration('active_scan_panoramic_steps')
    active_scan_cooldown_steps = LaunchConfiguration('active_scan_cooldown_steps')
    active_scan_probe_steps = LaunchConfiguration('active_scan_probe_steps')
    active_scan_front_clear_m = LaunchConfiguration('active_scan_front_clear_m')
    active_scan_corridor_extra_width_m = LaunchConfiguration('active_scan_corridor_extra_width_m')
    continuous_control_enabled = LaunchConfiguration('continuous_control_enabled')
    continuous_max_speed_mps = LaunchConfiguration('continuous_max_speed_mps')
    continuous_max_omega_rps = LaunchConfiguration('continuous_max_omega_rps')
    continuous_horizon_s = LaunchConfiguration('continuous_horizon_s')
    continuous_accel_limit_mps2 = LaunchConfiguration('continuous_accel_limit_mps2')
    continuous_omega_accel_limit_rps2 = LaunchConfiguration('continuous_omega_accel_limit_rps2')
    continuous_lowpass_alpha = LaunchConfiguration('continuous_lowpass_alpha')
    continuous_raw_per_mps = LaunchConfiguration('continuous_raw_per_mps')
    continuous_slowdown_clearance_m = LaunchConfiguration('continuous_slowdown_clearance_m')
    continuous_stop_clearance_m = LaunchConfiguration('continuous_stop_clearance_m')
    continuous_gap_buffer_m = LaunchConfiguration('continuous_gap_buffer_m')
    continuous_latency_buffer_s = LaunchConfiguration('continuous_latency_buffer_s')
    continuous_allow_costmap_soft_penalty = LaunchConfiguration('continuous_allow_costmap_soft_penalty')
    emit_velocity_commands = LaunchConfiguration('emit_velocity_commands')
    local_costmap_enabled = LaunchConfiguration('local_costmap_enabled')
    local_costmap_width_m = LaunchConfiguration('local_costmap_width_m')
    local_costmap_height_m = LaunchConfiguration('local_costmap_height_m')
    local_costmap_resolution_m = LaunchConfiguration('local_costmap_resolution_m')
    local_costmap_dynamic_decay_s = LaunchConfiguration('local_costmap_dynamic_decay_s')
    local_costmap_obstacle_radius_m = LaunchConfiguration('local_costmap_obstacle_radius_m')
    local_costmap_inflation_m = LaunchConfiguration('local_costmap_inflation_m')
    local_costmap_lidar_clear_radius_m = LaunchConfiguration('local_costmap_lidar_clear_radius_m')
    local_costmap_max_raytrace_m = LaunchConfiguration('local_costmap_max_raytrace_m')

    sensor_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            sensor_sync_launch_file
        ),
        launch_arguments={
            'start_uwb': start_uwb,
            'start_zed': start_zed,
            'start_lidar': start_lidar,
            'start_fusion': start_fusion,
            'lidar_port': lidar_port,
            'lidar_baud': lidar_baud,
            'lidar_scan_freq_hz': lidar_scan_freq_hz,
            'zed_publish_rate_hz': zed_publish_rate_hz,
            'zed_depth_downsample_factor': zed_depth_downsample_factor,
            'zed_publish_image': zed_publish_image,
            'fusion_zed_fresh_timeout_s': fusion_zed_fresh_timeout_s,
            'fusion_allow_lidar_only': fusion_allow_lidar_only,
            'fusion_depth_invalid_warn_frames': fusion_depth_invalid_warn_frames,
            'fusion_lidar_front_fov_deg': fusion_lidar_front_fov_deg,
            'fusion_depth_projection_stride_px': fusion_depth_projection_stride_px,
            'fusion_depth_ground_filter_enabled': fusion_depth_ground_filter_enabled,
            'fusion_depth_ground_min_delta_m': fusion_depth_ground_min_delta_m,
            'fusion_depth_ground_ratio': fusion_depth_ground_ratio,
            'fusion_depth_obstacle_min_component_height_px': fusion_depth_obstacle_min_component_height_px,
            'fusion_depth_front_corridor_half_width_m': fusion_depth_front_corridor_half_width_m,
            'fusion_imu_smoothing_alpha': fusion_imu_smoothing_alpha,
        }.items(),
    )

    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ugv_motor_controller'),
                'launch',
                'motor_controller.launch.py',
            ])
        ),
        condition=IfCondition(start_motor_controller),
        launch_arguments={
            'port': motor_port,
            'baud': motor_baud,
            'raw_command_scale_us': motor_raw_command_scale_us,
            'pwm_slew_rate_us_per_s': motor_pwm_slew_rate_us_per_s,
            'command_timeout_s': motor_command_timeout_s,
            'command_refresh_period_s': motor_command_refresh_period_s,
            'dry_run': motor_dry_run,
            'velocity_control_enabled': motor_velocity_control_enabled,
            'prefer_velocity_fields': motor_prefer_velocity_fields,
            'track_width_m': robot_track_width_m,
            'wheel_radius_m': motor_wheel_radius_m,
            'ticks_per_rev': motor_ticks_per_rev,
            'velocity_kp': motor_velocity_kp,
            'velocity_ki': motor_velocity_ki,
            'velocity_kd': motor_velocity_kd,
            'velocity_integral_limit': motor_velocity_integral_limit,
            'velocity_feedforward_raw_per_mps': motor_velocity_feedforward_raw_per_mps,
            'velocity_min_target_mps': motor_velocity_min_target_mps,
            'velocity_max_target_mps': motor_velocity_max_target_mps,
            'velocity_control_period_s': motor_velocity_control_period_s,
            'velocity_stale_encoder_timeout_s': motor_velocity_stale_encoder_timeout_s,
            'velocity_fallback_to_raw_without_encoder': motor_velocity_fallback_to_raw_without_encoder,
            'velocity_encoder_speed_filter_alpha': motor_velocity_encoder_speed_filter_alpha,
            'velocity_encoder_speed_max_mps': motor_velocity_encoder_speed_max_mps,
            'velocity_encoder_speed_min_dt_s': motor_velocity_encoder_speed_min_dt_s,
            'velocity_raw_fallback_floor_enabled': motor_velocity_raw_fallback_floor_enabled,
            'velocity_raw_fallback_min_wheel_raw': motor_velocity_raw_fallback_min_wheel_raw,
            'velocity_raw_fallback_min_turn_raw': motor_velocity_raw_fallback_min_turn_raw,
            'velocity_raw_fallback_min_target_raw': motor_velocity_raw_fallback_min_target_raw,
            'invert_left_command': invert_left_command,
            'invert_right_command': invert_right_command,
            'invert_left_encoder': invert_left_encoder,
            'invert_right_encoder': invert_right_encoder,
        }.items(),
    )

    nav_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            nav_script,
            '--mode',
            'real',
            '--competition-mode',
            competition_mode,
            '--mission-mode',
            mission_mode,
            '--start-corner',
            start_corner,
            '--start-x-m',
            ugv_start_x_m,
            '--start-y-m',
            ugv_start_y_m,
            '--start-yaw-deg',
            ugv_start_yaw_deg,
            '--center-loiter-radius-m',
            center_loiter_radius_m,
            '--target-accept-radius-m',
            target_accept_radius_m,
            '--straight-distance-m',
            straight_distance_m,
            '--competition-mission-v2-enabled',
            competition_mission_v2_enabled,
            '--sweep-cell-size-m',
            sweep_cell_size_m,
            '--sweep-lane-spacing-m',
            sweep_lane_spacing_m,
            '--sweep-coverage-radius-m',
            sweep_coverage_radius_m,
            '--sweep-coverage-threshold',
            sweep_coverage_threshold,
            '--sweep-goal-timeout-s',
            sweep_goal_timeout_s,
            '--sweep-fail-limit',
            sweep_fail_limit,
            '--sweep-lane-tolerance-m',
            sweep_lane_tolerance_m,
            '--sweep-heading-tolerance-deg',
            sweep_heading_tolerance_deg,
            '--sweep-allow-pure-turn',
            sweep_allow_pure_turn,
            '--sweep-stall-action',
            sweep_stall_action,
            '--min-competition-speed-mps',
            min_competition_speed_mps,
            '--target-topic',
            target_topic,
            '--mission-flag-topic',
            mission_flag_topic,
            '--use-imu-yaw',
            use_imu_yaw,
            '--imu-yaw-blend',
            imu_yaw_blend,
            '--imu-yaw-axis',
            imu_yaw_axis,
            '--imu-yaw-sign',
            imu_yaw_sign,
            '--robot-length-m',
            robot_length_m,
            '--robot-width-m',
            robot_width_m,
            '--robot-track-width-m',
            robot_track_width_m,
            '--robot-wheel-radius-m',
            robot_wheel_radius_m,
            '--robot-ticks-per-rev',
            robot_ticks_per_rev,
            '--robot-obstacle-buffer-m',
            robot_obstacle_buffer_m,
            '--lidar-offset-x-m',
            lidar_offset_x_m,
            '--lidar-offset-y-m',
            lidar_offset_y_m,
            '--lidar-used-fov-deg',
            lidar_used_fov_deg,
            '--allow-reverse',
            allow_reverse,
            '--min-motion-raw',
            min_motion_raw,
            '--recovery-turn-raw',
            recovery_turn_raw,
            '--min-speed-mps',
            min_speed_mps,
            '--drive-speed-level',
            drive_speed_level,
            '--front-safety-margin-m',
            front_safety_margin_m,
            '--rear-safety-margin-m',
            rear_safety_margin_m,
            '--local-plan-inflation-m',
            local_plan_inflation_m,
            '--active-scan-enabled',
            active_scan_enabled,
            '--active-scan-confirm-steps',
            active_scan_confirm_steps,
            '--active-scan-steps',
            active_scan_steps,
            '--active-scan-panoramic-steps',
            active_scan_panoramic_steps,
            '--active-scan-cooldown-steps',
            active_scan_cooldown_steps,
            '--active-scan-probe-steps',
            active_scan_probe_steps,
            '--active-scan-front-clear-m',
            active_scan_front_clear_m,
            '--active-scan-corridor-extra-width-m',
            active_scan_corridor_extra_width_m,
            '--continuous-control-enabled',
            continuous_control_enabled,
            '--continuous-max-speed-mps',
            continuous_max_speed_mps,
            '--continuous-max-omega-rps',
            continuous_max_omega_rps,
            '--continuous-horizon-s',
            continuous_horizon_s,
            '--continuous-accel-limit-mps2',
            continuous_accel_limit_mps2,
            '--continuous-omega-accel-limit-rps2',
            continuous_omega_accel_limit_rps2,
            '--continuous-lowpass-alpha',
            continuous_lowpass_alpha,
            '--continuous-raw-per-mps',
            continuous_raw_per_mps,
            '--continuous-slowdown-clearance-m',
            continuous_slowdown_clearance_m,
            '--continuous-stop-clearance-m',
            continuous_stop_clearance_m,
            '--continuous-gap-buffer-m',
            continuous_gap_buffer_m,
            '--continuous-latency-buffer-s',
            continuous_latency_buffer_s,
            '--continuous-allow-costmap-soft-penalty',
            continuous_allow_costmap_soft_penalty,
            '--emit-velocity-commands',
            emit_velocity_commands,
            '--local-costmap-enabled',
            local_costmap_enabled,
            '--local-costmap-width-m',
            local_costmap_width_m,
            '--local-costmap-height-m',
            local_costmap_height_m,
            '--local-costmap-resolution-m',
            local_costmap_resolution_m,
            '--local-costmap-dynamic-decay-s',
            local_costmap_dynamic_decay_s,
            '--local-costmap-obstacle-radius-m',
            local_costmap_obstacle_radius_m,
            '--local-costmap-inflation-m',
            local_costmap_inflation_m,
            '--local-costmap-lidar-clear-radius-m',
            local_costmap_lidar_clear_radius_m,
            '--local-costmap-max-raytrace-m',
            local_costmap_max_raytrace_m,
        ],
        output='screen',
        condition=IfCondition(start_nav),
    )

    def ros_param_arg(name: str, value):
        return [TextSubstitution(text=f'{name}:='), value]

    marker_vision_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            marker_vision_script,
            '--ros-args',
            '-p',
            ros_param_arg('model_path', marker_model_path),
            '-p',
            ros_param_arg('model_max_descriptors', marker_model_max_descriptors),
            '-p',
            ros_param_arg('min_good_matches', marker_min_good_matches),
            '-p',
            ros_param_arg('confirmation_frames', marker_confirmation_frames),
            '-p',
            ros_param_arg('confirmation_radius_m', marker_confirmation_radius_m),
            '-p',
            ros_param_arg('target_reached_radius_m', target_accept_radius_m),
            '-p',
            ros_param_arg('marker_size_m', marker_size_m),
            '-p',
            ros_param_arg('min_projected_size_m', marker_min_projected_size_m),
            '-p',
            ros_param_arg('max_projected_size_m', marker_max_projected_size_m),
            '-p',
            ros_param_arg('enable_generic_detector', marker_enable_generic_detector),
            '-p',
            ros_param_arg('generic_min_area_frac', marker_generic_min_area_frac),
            '-p',
            ros_param_arg('generic_min_contrast', marker_generic_min_contrast),
            '-p',
            ros_param_arg('generic_min_grid_score', marker_generic_min_grid_score),
            '-p',
            ros_param_arg('generic_min_border_light_ratio', marker_generic_min_border_light_ratio),
        ],
        output='screen',
        condition=IfCondition(start_marker_vision),
    )

    marker_vision_test_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            marker_vision_test_script,
            '--ros-args',
            '-p',
            ros_param_arg('searching_period_s', marker_vision_test_period_s),
        ],
        output='screen',
        condition=IfCondition(start_marker_vision_test),
    )

    yolo_semantic_obstacle_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            yolo_semantic_obstacle_script,
            '--ros-args',
            '-p',
            ros_param_arg('model_path', yolo_model_path),
            '-p',
            ros_param_arg('device', yolo_device),
            '-p',
            ros_param_arg('imgsz', yolo_imgsz),
            '-p',
            ros_param_arg('confidence', yolo_confidence),
            '-p',
            ros_param_arg('max_hz', yolo_max_hz),
            '-p',
            ros_param_arg('obstacle_classes', yolo_obstacle_classes),
        ],
        output='screen',
        condition=IfCondition(start_yolo_obstacles),
    )

    debug_dashboard_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='python3'),
            debug_dashboard_script,
            '--ros-args',
            '-p',
            ros_param_arg('update_hz', dashboard_update_hz),
            '-p',
            ros_param_arg('camera_search_depth_m', dashboard_camera_search_depth_m),
            '-p',
            ros_param_arg('lidar_used_fov_deg', lidar_used_fov_deg),
            '-p',
            ros_param_arg('lidar_offset_x_m', lidar_offset_x_m),
            '-p',
            ros_param_arg('lidar_offset_y_m', lidar_offset_y_m),
            '-p',
            ros_param_arg('robot_length_m', robot_length_m),
            '-p',
            ros_param_arg('robot_width_m', robot_width_m),
        ],
        output='screen',
        condition=IfCondition(start_debug_dashboard),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_uwb', default_value='false'),
        DeclareLaunchArgument('start_zed', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_fusion', default_value='true'),
        DeclareLaunchArgument('start_motor_controller', default_value='true'),
        DeclareLaunchArgument('start_nav', default_value='true'),
        DeclareLaunchArgument('start_debug_status', default_value='true'),
        DeclareLaunchArgument('start_bench_goal', default_value='false'),
        DeclareLaunchArgument('start_mock_field_map', default_value='false'),
        DeclareLaunchArgument('start_marker_vision', default_value='true'),
        DeclareLaunchArgument('start_marker_vision_test', default_value='false'),
        DeclareLaunchArgument('start_yolo_obstacles', default_value='false'),
        DeclareLaunchArgument('start_debug_dashboard', default_value='false'),
        DeclareLaunchArgument('competition_mode', default_value='false'),
        DeclareLaunchArgument('mission_mode', default_value='manual'),
        DeclareLaunchArgument('start_corner', default_value='lower_left'),
        DeclareLaunchArgument('ugv_start_x_m', default_value='nan'),
        DeclareLaunchArgument('ugv_start_y_m', default_value='nan'),
        DeclareLaunchArgument('ugv_start_yaw_deg', default_value='nan'),
        DeclareLaunchArgument('center_loiter_radius_m', default_value='0.75'),
        DeclareLaunchArgument('target_accept_radius_m', default_value='0.9144'),
        DeclareLaunchArgument('min_speed_mps', default_value='0.178816'),
        DeclareLaunchArgument('drive_speed_level', default_value='4'),
        DeclareLaunchArgument('straight_distance_m', default_value='11.8872'),
        DeclareLaunchArgument('competition_mission_v2_enabled', default_value='true'),
        DeclareLaunchArgument('sweep_cell_size_m', default_value='0.75'),
        DeclareLaunchArgument('sweep_lane_spacing_m', default_value='0.75'),
        DeclareLaunchArgument('sweep_coverage_radius_m', default_value='0.55'),
        DeclareLaunchArgument('sweep_coverage_threshold', default_value='0.85'),
        DeclareLaunchArgument('sweep_goal_timeout_s', default_value='8.0'),
        DeclareLaunchArgument('sweep_fail_limit', default_value='3'),
        DeclareLaunchArgument('sweep_lane_tolerance_m', default_value='0.30'),
        DeclareLaunchArgument('sweep_heading_tolerance_deg', default_value='25.0'),
        DeclareLaunchArgument('sweep_allow_pure_turn', default_value='false'),
        DeclareLaunchArgument('sweep_stall_action', default_value='skip'),
        DeclareLaunchArgument('min_competition_speed_mps', default_value='0.0894'),
        DeclareLaunchArgument('bench_goal_x_m', default_value='12.2'),
        DeclareLaunchArgument('bench_goal_y_m', default_value='12.0'),
        DeclareLaunchArgument('bench_goal_period_s', default_value='1.0'),
        DeclareLaunchArgument('mock_marker_cell', default_value='7,7'),
        DeclareLaunchArgument('mock_obstacles_json', default_value=''),
        DeclareLaunchArgument('target_topic', default_value='/ugv/target'),
        DeclareLaunchArgument(
            'marker_model_path',
            default_value=str(workspace_root / 'src' / 'ugv_perception' / 'models' / 'marker_orb_model.npz'),
        ),
        DeclareLaunchArgument('marker_model_max_descriptors', default_value='65000'),
        DeclareLaunchArgument('marker_min_good_matches', default_value='18'),
        DeclareLaunchArgument('marker_confirmation_frames', default_value='2'),
        DeclareLaunchArgument('marker_confirmation_radius_m', default_value='0.75'),
        DeclareLaunchArgument('marker_size_m', default_value='0.3048'),
        DeclareLaunchArgument('marker_min_projected_size_m', default_value='0.10'),
        DeclareLaunchArgument('marker_max_projected_size_m', default_value='0.75'),
        DeclareLaunchArgument('marker_enable_generic_detector', default_value='false'),
        DeclareLaunchArgument('marker_generic_min_area_frac', default_value='0.002'),
        DeclareLaunchArgument('marker_generic_min_contrast', default_value='55.0'),
        DeclareLaunchArgument('marker_generic_min_grid_score', default_value='0.18'),
        DeclareLaunchArgument('marker_generic_min_border_light_ratio', default_value='0.12'),
        DeclareLaunchArgument('marker_vision_test_period_s', default_value='3.0'),
        DeclareLaunchArgument('yolo_model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('yolo_device', default_value='auto'),
        DeclareLaunchArgument('yolo_imgsz', default_value='416'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.35'),
        DeclareLaunchArgument('yolo_max_hz', default_value='2.0'),
        DeclareLaunchArgument('yolo_obstacle_classes', default_value='person,chair,couch,dining table,bench,potted plant,backpack,suitcase'),
        DeclareLaunchArgument('dashboard_update_hz', default_value='8.0'),
        DeclareLaunchArgument('dashboard_camera_search_depth_m', default_value='0.30'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_scan_freq_hz', default_value='10.0'),
        DeclareLaunchArgument('zed_publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('zed_depth_downsample_factor', default_value='2'),
        DeclareLaunchArgument('zed_publish_image', default_value='false'),
        DeclareLaunchArgument('fusion_zed_fresh_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('fusion_allow_lidar_only', default_value='true'),
        DeclareLaunchArgument('fusion_depth_invalid_warn_frames', default_value='2'),
        DeclareLaunchArgument('fusion_lidar_front_fov_deg', default_value='70.0'),
        DeclareLaunchArgument('fusion_depth_projection_stride_px', default_value='8'),
        DeclareLaunchArgument('fusion_depth_ground_filter_enabled', default_value='true'),
        DeclareLaunchArgument('fusion_depth_ground_min_delta_m', default_value='0.18'),
        DeclareLaunchArgument('fusion_depth_ground_ratio', default_value='0.88'),
        DeclareLaunchArgument('fusion_depth_obstacle_min_component_height_px', default_value='14'),
        DeclareLaunchArgument('fusion_depth_front_corridor_half_width_m', default_value='0.42'),
        DeclareLaunchArgument('fusion_imu_smoothing_alpha', default_value='0.25'),
        DeclareLaunchArgument('mission_flag_topic', default_value='/ugv/mission_flag'),
        DeclareLaunchArgument('use_imu_yaw', default_value='false'),
        DeclareLaunchArgument('imu_yaw_blend', default_value='0.25'),
        DeclareLaunchArgument('imu_yaw_axis', default_value='z'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('robot_length_m', default_value='0.762'),
        DeclareLaunchArgument('robot_width_m', default_value='0.762'),
        DeclareLaunchArgument('robot_track_width_m', default_value='0.6096'),
        DeclareLaunchArgument('robot_wheel_radius_m', default_value='0.06'),
        DeclareLaunchArgument('robot_ticks_per_rev', default_value='1000'),
        DeclareLaunchArgument('robot_obstacle_buffer_m', default_value='0.025'),
        DeclareLaunchArgument('lidar_offset_x_m', default_value='0.30'),
        DeclareLaunchArgument('lidar_offset_y_m', default_value='0.0'),
        DeclareLaunchArgument('lidar_used_fov_deg', default_value='180.0'),
        DeclareLaunchArgument('allow_reverse', default_value='false'),
        DeclareLaunchArgument('front_safety_margin_m', default_value='0.08'),
        DeclareLaunchArgument('rear_safety_margin_m', default_value='0.08'),
        DeclareLaunchArgument('local_plan_inflation_m', default_value='0.0'),
        DeclareLaunchArgument('active_scan_enabled', default_value='true'),
        DeclareLaunchArgument('active_scan_confirm_steps', default_value='3'),
        DeclareLaunchArgument('active_scan_steps', default_value='7'),
        DeclareLaunchArgument('active_scan_panoramic_steps', default_value='20'),
        DeclareLaunchArgument('active_scan_cooldown_steps', default_value='4'),
        DeclareLaunchArgument('active_scan_probe_steps', default_value='5'),
        DeclareLaunchArgument('active_scan_front_clear_m', default_value='1.25'),
        DeclareLaunchArgument('active_scan_corridor_extra_width_m', default_value='0.03'),
        DeclareLaunchArgument('continuous_control_enabled', default_value='true'),
        DeclareLaunchArgument('continuous_max_speed_mps', default_value='0.36'),
        DeclareLaunchArgument('continuous_max_omega_rps', default_value='1.15'),
        DeclareLaunchArgument('continuous_horizon_s', default_value='1.35'),
        DeclareLaunchArgument('continuous_accel_limit_mps2', default_value='0.35'),
        DeclareLaunchArgument('continuous_omega_accel_limit_rps2', default_value='1.80'),
        DeclareLaunchArgument('continuous_lowpass_alpha', default_value='0.55'),
        DeclareLaunchArgument('continuous_raw_per_mps', default_value='1.35'),
        DeclareLaunchArgument('continuous_slowdown_clearance_m', default_value='1.20'),
        DeclareLaunchArgument('continuous_stop_clearance_m', default_value='0.48'),
        DeclareLaunchArgument('continuous_gap_buffer_m', default_value='0.025'),
        DeclareLaunchArgument('continuous_latency_buffer_s', default_value='0.25'),
        DeclareLaunchArgument('continuous_allow_costmap_soft_penalty', default_value='false'),
        DeclareLaunchArgument('emit_velocity_commands', default_value='true'),
        DeclareLaunchArgument('local_costmap_enabled', default_value='true'),
        DeclareLaunchArgument('local_costmap_width_m', default_value='4.0'),
        DeclareLaunchArgument('local_costmap_height_m', default_value='4.0'),
        DeclareLaunchArgument('local_costmap_resolution_m', default_value='0.06'),
        DeclareLaunchArgument('local_costmap_dynamic_decay_s', default_value='1.0'),
        DeclareLaunchArgument('local_costmap_obstacle_radius_m', default_value='0.06'),
        DeclareLaunchArgument('local_costmap_inflation_m', default_value='0.08'),
        DeclareLaunchArgument('local_costmap_lidar_clear_radius_m', default_value='0.05'),
        DeclareLaunchArgument('local_costmap_max_raytrace_m', default_value='4.0'),
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('motor_baud', default_value='115200'),
        DeclareLaunchArgument('motor_raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('motor_pwm_slew_rate_us_per_s', default_value='2400.0'),
        DeclareLaunchArgument('motor_command_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('motor_command_refresh_period_s', default_value='0.25'),
        DeclareLaunchArgument('motor_dry_run', default_value='false'),
        DeclareLaunchArgument('motor_velocity_control_enabled', default_value='false'),
        DeclareLaunchArgument('motor_prefer_velocity_fields', default_value='true'),
        DeclareLaunchArgument('motor_wheel_radius_m', default_value='0.06'),
        DeclareLaunchArgument('motor_ticks_per_rev', default_value='1000'),
        DeclareLaunchArgument('motor_velocity_kp', default_value='0.80'),
        DeclareLaunchArgument('motor_velocity_ki', default_value='0.0'),
        DeclareLaunchArgument('motor_velocity_kd', default_value='0.02'),
        DeclareLaunchArgument('motor_velocity_integral_limit', default_value='0.30'),
        DeclareLaunchArgument('motor_velocity_feedforward_raw_per_mps', default_value='1.35'),
        DeclareLaunchArgument('motor_velocity_min_target_mps', default_value='0.02'),
        DeclareLaunchArgument('motor_velocity_max_target_mps', default_value='0.60'),
        DeclareLaunchArgument('motor_velocity_control_period_s', default_value='0.05'),
        DeclareLaunchArgument('motor_velocity_stale_encoder_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('motor_velocity_fallback_to_raw_without_encoder', default_value='false'),
        DeclareLaunchArgument('motor_velocity_encoder_speed_filter_alpha', default_value='0.65'),
        DeclareLaunchArgument('motor_velocity_encoder_speed_max_mps', default_value='2.0'),
        DeclareLaunchArgument('motor_velocity_encoder_speed_min_dt_s', default_value='0.015'),
        DeclareLaunchArgument('motor_velocity_raw_fallback_floor_enabled', default_value='false'),
        DeclareLaunchArgument('motor_velocity_raw_fallback_min_wheel_raw', default_value='0.14'),
        DeclareLaunchArgument('motor_velocity_raw_fallback_min_turn_raw', default_value='0.28'),
        DeclareLaunchArgument('motor_velocity_raw_fallback_min_target_raw', default_value='0.001'),
        DeclareLaunchArgument('min_motion_raw', default_value='0.22'),
        DeclareLaunchArgument('recovery_turn_raw', default_value='0.32'),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='true'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        sensor_sync_launch,
        motor_controller_launch,
        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                mock_field_map_script,
                '--ros-args',
                '-r',
                '__node:=mock_field_map_node',
                '-p',
                ros_param_arg('topic', target_topic),
                '-p',
                ros_param_arg('start_corner', start_corner),
                '-p',
                ros_param_arg('marker_cell', mock_marker_cell),
                '-p',
                'publish_format:=target_xy',
                '-p',
                ros_param_arg('obstacles_json', mock_obstacles_json),
            ],
            output='screen',
            condition=IfCondition(start_mock_field_map),
        ),
        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                bench_goal_script,
                '--ros-args',
                '-r',
                '__node:=bench_goal_node',
                '-p',
                ros_param_arg('goal_x_m', bench_goal_x_m),
                '-p',
                ros_param_arg('goal_y_m', bench_goal_y_m),
                '-p',
                ros_param_arg('publish_period_s', bench_goal_period_s),
            ],
            output='screen',
            condition=IfCondition(start_bench_goal),
        ),
        marker_vision_process,
        marker_vision_test_process,
        yolo_semantic_obstacle_process,
        debug_dashboard_process,
        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                debug_status_script,
                '--ros-args',
                '-r',
                '__node:=ugv_debug_status_node',
            ],
            output='screen',
            condition=IfCondition(start_debug_status),
        ),
        nav_process,
    ])
