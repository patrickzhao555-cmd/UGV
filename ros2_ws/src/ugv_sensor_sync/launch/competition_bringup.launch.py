import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    bench_goal_x_m = LaunchConfiguration('bench_goal_x_m')
    bench_goal_y_m = LaunchConfiguration('bench_goal_y_m')
    bench_goal_period_s = LaunchConfiguration('bench_goal_period_s')
    mock_marker_cell = LaunchConfiguration('mock_marker_cell')
    mock_obstacles_json = LaunchConfiguration('mock_obstacles_json')
    target_topic = LaunchConfiguration('target_topic')
    start_marker_vision = LaunchConfiguration('start_marker_vision')
    start_marker_vision_test = LaunchConfiguration('start_marker_vision_test')
    start_yolo_obstacles = LaunchConfiguration('start_yolo_obstacles')
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
    fusion_imu_smoothing_alpha = LaunchConfiguration('fusion_imu_smoothing_alpha')
    motor_port = LaunchConfiguration('motor_port')
    motor_baud = LaunchConfiguration('motor_baud')
    motor_raw_command_scale_us = LaunchConfiguration('motor_raw_command_scale_us')
    motor_pwm_slew_rate_us_per_s = LaunchConfiguration('motor_pwm_slew_rate_us_per_s')
    motor_dry_run = LaunchConfiguration('motor_dry_run')
    min_motion_raw = LaunchConfiguration('min_motion_raw')
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
    lidar_offset_x_m = LaunchConfiguration('lidar_offset_x_m')
    lidar_offset_y_m = LaunchConfiguration('lidar_offset_y_m')
    lidar_used_fov_deg = LaunchConfiguration('lidar_used_fov_deg')
    allow_reverse = LaunchConfiguration('allow_reverse')
    front_safety_margin_m = LaunchConfiguration('front_safety_margin_m')
    rear_safety_margin_m = LaunchConfiguration('rear_safety_margin_m')
    local_plan_inflation_m = LaunchConfiguration('local_plan_inflation_m')

    sensor_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ugv_sensor_sync'),
                'launch',
                'sensor_sync_launch.py',
            ])
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
            'dry_run': motor_dry_run,
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
        DeclareLaunchArgument('yolo_device', default_value=''),
        DeclareLaunchArgument('yolo_imgsz', default_value='416'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.35'),
        DeclareLaunchArgument('yolo_max_hz', default_value='2.0'),
        DeclareLaunchArgument('yolo_obstacle_classes', default_value='person,chair,couch,dining table,bench,potted plant,backpack,suitcase'),
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
        DeclareLaunchArgument('fusion_imu_smoothing_alpha', default_value='0.25'),
        DeclareLaunchArgument('mission_flag_topic', default_value='/ugv/mission_flag'),
        DeclareLaunchArgument('use_imu_yaw', default_value='false'),
        DeclareLaunchArgument('imu_yaw_blend', default_value='0.25'),
        DeclareLaunchArgument('imu_yaw_axis', default_value='z'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('robot_length_m', default_value='0.762'),
        DeclareLaunchArgument('robot_width_m', default_value='0.762'),
        DeclareLaunchArgument('robot_track_width_m', default_value='0.6096'),
        DeclareLaunchArgument('lidar_offset_x_m', default_value='0.30'),
        DeclareLaunchArgument('lidar_offset_y_m', default_value='0.0'),
        DeclareLaunchArgument('lidar_used_fov_deg', default_value='180.0'),
        DeclareLaunchArgument('allow_reverse', default_value='false'),
        DeclareLaunchArgument('front_safety_margin_m', default_value='0.10'),
        DeclareLaunchArgument('rear_safety_margin_m', default_value='0.08'),
        DeclareLaunchArgument('local_plan_inflation_m', default_value='0.08'),
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('motor_baud', default_value='115200'),
        DeclareLaunchArgument('motor_raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('motor_pwm_slew_rate_us_per_s', default_value='2400.0'),
        DeclareLaunchArgument('motor_dry_run', default_value='false'),
        DeclareLaunchArgument('min_motion_raw', default_value='0.22'),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='false'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        sensor_sync_launch,
        motor_controller_launch,
        Node(
            package='ugv_sensor_sync',
            executable='mock_field_map_node',
            name='mock_field_map_node',
            output='screen',
            condition=IfCondition(start_mock_field_map),
            parameters=[{
                'topic': target_topic,
                'start_corner': start_corner,
                'marker_cell': mock_marker_cell,
                'publish_format': 'target_xy',
                'obstacles_json': mock_obstacles_json,
            }],
        ),
        Node(
            package='ugv_sensor_sync',
            executable='bench_goal_node',
            name='bench_goal_node',
            output='screen',
            condition=IfCondition(start_bench_goal),
            parameters=[{
                'goal_x_m': bench_goal_x_m,
                'goal_y_m': bench_goal_y_m,
                'publish_period_s': bench_goal_period_s,
            }],
        ),
        marker_vision_process,
        marker_vision_test_process,
        yolo_semantic_obstacle_process,
        Node(
            package='ugv_sensor_sync',
            executable='debug_status_node',
            name='ugv_debug_status_node',
            output='screen',
            condition=IfCondition(start_debug_status),
        ),
        nav_process,
    ])
