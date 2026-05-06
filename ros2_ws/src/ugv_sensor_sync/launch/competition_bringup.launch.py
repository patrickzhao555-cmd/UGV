import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
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

    start_uwb = LaunchConfiguration('start_uwb')
    start_motor_controller = LaunchConfiguration('start_motor_controller')
    start_nav = LaunchConfiguration('start_nav')
    start_debug_status = LaunchConfiguration('start_debug_status')
    start_bench_goal = LaunchConfiguration('start_bench_goal')
    start_mock_field_map = LaunchConfiguration('start_mock_field_map')
    competition_mode = LaunchConfiguration('competition_mode')
    start_corner = LaunchConfiguration('start_corner')
    center_loiter_radius_m = LaunchConfiguration('center_loiter_radius_m')
    target_accept_radius_m = LaunchConfiguration('target_accept_radius_m')
    bench_goal_x_m = LaunchConfiguration('bench_goal_x_m')
    bench_goal_y_m = LaunchConfiguration('bench_goal_y_m')
    bench_goal_period_s = LaunchConfiguration('bench_goal_period_s')
    mock_marker_cell = LaunchConfiguration('mock_marker_cell')
    mock_obstacles_json = LaunchConfiguration('mock_obstacles_json')
    start_marker_vision = LaunchConfiguration('start_marker_vision')
    marker_model_path = LaunchConfiguration('marker_model_path')
    marker_min_good_matches = LaunchConfiguration('marker_min_good_matches')
    marker_confirmation_frames = LaunchConfiguration('marker_confirmation_frames')
    marker_confirmation_radius_m = LaunchConfiguration('marker_confirmation_radius_m')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_freq_hz = LaunchConfiguration('lidar_scan_freq_hz')
    zed_publish_rate_hz = LaunchConfiguration('zed_publish_rate_hz')
    zed_depth_downsample_factor = LaunchConfiguration('zed_depth_downsample_factor')
    zed_publish_image = LaunchConfiguration('zed_publish_image')
    fusion_zed_fresh_timeout_s = LaunchConfiguration('fusion_zed_fresh_timeout_s')
    fusion_depth_invalid_warn_frames = LaunchConfiguration('fusion_depth_invalid_warn_frames')
    fusion_lidar_front_fov_deg = LaunchConfiguration('fusion_lidar_front_fov_deg')
    fusion_imu_smoothing_alpha = LaunchConfiguration('fusion_imu_smoothing_alpha')
    motor_port = LaunchConfiguration('motor_port')
    motor_baud = LaunchConfiguration('motor_baud')
    motor_raw_command_scale_us = LaunchConfiguration('motor_raw_command_scale_us')
    motor_pwm_slew_rate_us_per_s = LaunchConfiguration('motor_pwm_slew_rate_us_per_s')
    motor_dry_run = LaunchConfiguration('motor_dry_run')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    mission_flag_topic = LaunchConfiguration('mission_flag_topic')
    use_imu_yaw = LaunchConfiguration('use_imu_yaw')
    imu_yaw_blend = LaunchConfiguration('imu_yaw_blend')
    imu_yaw_axis = LaunchConfiguration('imu_yaw_axis')
    imu_yaw_sign = LaunchConfiguration('imu_yaw_sign')

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
            'lidar_port': lidar_port,
            'lidar_baud': lidar_baud,
            'lidar_scan_freq_hz': lidar_scan_freq_hz,
            'zed_publish_rate_hz': zed_publish_rate_hz,
            'zed_depth_downsample_factor': zed_depth_downsample_factor,
            'zed_publish_image': zed_publish_image,
            'fusion_zed_fresh_timeout_s': fusion_zed_fresh_timeout_s,
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
            '--start-corner',
            start_corner,
            '--center-loiter-radius-m',
            center_loiter_radius_m,
            '--target-accept-radius-m',
            target_accept_radius_m,
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
        ],
        output='screen',
        condition=IfCondition(start_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_uwb', default_value='false'),
        DeclareLaunchArgument('start_motor_controller', default_value='true'),
        DeclareLaunchArgument('start_nav', default_value='true'),
        DeclareLaunchArgument('start_debug_status', default_value='true'),
        DeclareLaunchArgument('start_bench_goal', default_value='false'),
        DeclareLaunchArgument('start_mock_field_map', default_value='false'),
        DeclareLaunchArgument('start_marker_vision', default_value='false'),
        DeclareLaunchArgument('competition_mode', default_value='false'),
        DeclareLaunchArgument('start_corner', default_value='lower_left'),
        DeclareLaunchArgument('center_loiter_radius_m', default_value='0.75'),
        DeclareLaunchArgument('target_accept_radius_m', default_value='0.9144'),
        DeclareLaunchArgument('bench_goal_x_m', default_value='12.2'),
        DeclareLaunchArgument('bench_goal_y_m', default_value='12.0'),
        DeclareLaunchArgument('bench_goal_period_s', default_value='1.0'),
        DeclareLaunchArgument('mock_marker_cell', default_value='7,7'),
        DeclareLaunchArgument('mock_obstacles_json', default_value=''),
        DeclareLaunchArgument(
            'marker_model_path',
            default_value=str(workspace_root / 'src' / 'ugv_perception' / 'models' / 'marker_orb_model.npz'),
        ),
        DeclareLaunchArgument('marker_min_good_matches', default_value='18'),
        DeclareLaunchArgument('marker_confirmation_frames', default_value='2'),
        DeclareLaunchArgument('marker_confirmation_radius_m', default_value='0.75'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_scan_freq_hz', default_value='10.0'),
        DeclareLaunchArgument('zed_publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('zed_depth_downsample_factor', default_value='2'),
        DeclareLaunchArgument('zed_publish_image', default_value='false'),
        DeclareLaunchArgument('fusion_zed_fresh_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('fusion_depth_invalid_warn_frames', default_value='2'),
        DeclareLaunchArgument('fusion_lidar_front_fov_deg', default_value='70.0'),
        DeclareLaunchArgument('fusion_imu_smoothing_alpha', default_value='0.25'),
        DeclareLaunchArgument('mission_flag_topic', default_value='/ugv/mission_flag'),
        DeclareLaunchArgument('use_imu_yaw', default_value='false'),
        DeclareLaunchArgument('imu_yaw_blend', default_value='0.25'),
        DeclareLaunchArgument('imu_yaw_axis', default_value='z'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('motor_baud', default_value='115200'),
        DeclareLaunchArgument('motor_raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('motor_pwm_slew_rate_us_per_s', default_value='2400.0'),
        DeclareLaunchArgument('motor_dry_run', default_value='false'),
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
                'start_corner': start_corner,
                'marker_cell': mock_marker_cell,
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
        Node(
            package='ugv_perception',
            executable='marker_vision_node',
            name='marker_vision_node',
            output='screen',
            condition=IfCondition(start_marker_vision),
            parameters=[{
                'model_path': ParameterValue(marker_model_path, value_type=str),
                'min_good_matches': ParameterValue(marker_min_good_matches, value_type=int),
                'confirmation_frames': ParameterValue(marker_confirmation_frames, value_type=int),
                'confirmation_radius_m': ParameterValue(marker_confirmation_radius_m, value_type=float),
            }],
        ),
        Node(
            package='ugv_sensor_sync',
            executable='debug_status_node',
            name='ugv_debug_status_node',
            output='screen',
            condition=IfCondition(start_debug_status),
        ),
        nav_process,
    ])
