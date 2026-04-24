import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_freq_hz = LaunchConfiguration('lidar_scan_freq_hz')
    motor_port = LaunchConfiguration('motor_port')
    motor_baud = LaunchConfiguration('motor_baud')
    motor_raw_command_scale_us = LaunchConfiguration('motor_raw_command_scale_us')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')

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
        ],
        output='screen',
        condition=IfCondition(start_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_uwb', default_value='false'),
        DeclareLaunchArgument('start_motor_controller', default_value='true'),
        DeclareLaunchArgument('start_nav', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_scan_freq_hz', default_value='5.5'),
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('motor_baud', default_value='115200'),
        DeclareLaunchArgument('motor_raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='false'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        sensor_sync_launch,
        motor_controller_launch,
        nav_process,
    ])
