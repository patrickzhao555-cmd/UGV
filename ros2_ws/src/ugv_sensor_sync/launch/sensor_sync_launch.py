import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable, LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def _resolve_workspace_root() -> Path:
    env_ws = os.environ.get('UGV_WS')
    candidates = []
    if env_ws:
        candidates.append(Path(env_ws))

    here = Path(__file__).resolve()
    candidates.extend(here.parents)

    for base in candidates:
        node_file = base / 'src' / 'ugv_sensor_sync' / 'ugv_sensor_sync_nodes' / 'fusion_node.py'
        if node_file.exists():
            return base

    raise RuntimeError(
        'Could not find workspace root containing src/ugv_sensor_sync/ugv_sensor_sync_nodes. '
        'Set UGV_WS before running this launch.'
    )


def generate_launch_description():
    workspace_root = _resolve_workspace_root()
    sensor_nodes_root = workspace_root / 'src' / 'ugv_sensor_sync' / 'ugv_sensor_sync_nodes'
    zed_sync_script = str(sensor_nodes_root / 'zed_sync_node.py')
    lidar_sync_script = str(sensor_nodes_root / 'lidar_sync_node.py')
    lidar_filter_script = str(sensor_nodes_root / 'lidar_scan_filter_node.py')
    uwb_script = str(sensor_nodes_root / 'uwb_node.py')
    fusion_script = str(sensor_nodes_root / 'fusion_node.py')

    start_uwb = LaunchConfiguration('start_uwb')
    start_zed = LaunchConfiguration('start_zed')
    start_lidar = LaunchConfiguration('start_lidar')
    start_lidar_filter = LaunchConfiguration('start_lidar_filter')
    start_fusion = LaunchConfiguration('start_fusion')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_freq_hz = LaunchConfiguration('lidar_scan_freq_hz')
    lidar_filter_forward_fov_deg = LaunchConfiguration('lidar_filter_forward_fov_deg')
    lidar_filtered_topic = LaunchConfiguration('lidar_filtered_topic')
    zed_publish_rate_hz = LaunchConfiguration('zed_publish_rate_hz')
    zed_imu_publish_rate_hz = LaunchConfiguration('zed_imu_publish_rate_hz')
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

    def ros_param_arg(name: str, value):
        return [TextSubstitution(text=f'{name}:='), value]

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_uwb',
            default_value='false',
            description='Launch the UWB serial bridge as part of the sensor sync stack.',
        ),
        DeclareLaunchArgument(
            'start_zed',
            default_value='true',
            description='Launch the ZED depth/IMU sync node.',
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Launch the lidar driver and lidar timestamp sync node.',
        ),
        DeclareLaunchArgument(
            'start_lidar_filter',
            default_value='true',
            description='Filter the rear LiDAR sector blocked by the UGV body before fusion/Nav2.',
        ),
        DeclareLaunchArgument(
            'start_fusion',
            default_value='true',
            description='Launch the fused sensor frame node.',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port used by the lidar driver.',
        ),
        DeclareLaunchArgument(
            'lidar_baud',
            default_value='115200',
            description='Baud rate used by the lidar driver.',
        ),
        DeclareLaunchArgument(
            'lidar_scan_freq_hz',
            default_value='10.0',
            description='Expected lidar scan frequency for midpoint timestamp correction.',
        ),
        DeclareLaunchArgument(
            'lidar_filter_forward_fov_deg',
            default_value='250.0',
            description='Forward LiDAR sector kept for navigation. The remaining rear sector is invalidated.',
        ),
        DeclareLaunchArgument(
            'lidar_filtered_topic',
            default_value='/scan/filtered',
            description='Filtered LiDAR scan topic consumed by fusion, Nav2 costmaps, and collision monitor.',
        ),
        DeclareLaunchArgument(
            'zed_publish_rate_hz',
            default_value='10.0',
            description='Target ZED publish rate.',
        ),
        DeclareLaunchArgument(
            'zed_imu_publish_rate_hz',
            default_value='100.0',
            description='Independent ZED IMU publish rate for chassis heading control.',
        ),
        DeclareLaunchArgument(
            'zed_depth_downsample_factor',
            default_value='2',
            description='Integer decimation factor applied before publishing ZED depth frames.',
        ),
        DeclareLaunchArgument(
            'zed_publish_image',
            default_value='false',
            description='Publish ZED left image on /zed/image. Enable when marker vision is running.',
        ),
        DeclareLaunchArgument(
            'fusion_zed_fresh_timeout_s',
            default_value='0.75',
            description='Maximum receive age allowed when fusion falls back to the freshest ZED frame.',
        ),
        DeclareLaunchArgument(
            'fusion_allow_lidar_only',
            default_value='true',
            description='Allow nav-frame fusion from LiDAR and encoders when no complete ZED frame is available.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_invalid_warn_frames',
            default_value='2',
            description='Number of consecutive invalid ZED depth ROIs before fusion marks the forward view unsafe.',
        ),
        DeclareLaunchArgument(
            'fusion_lidar_front_fov_deg',
            default_value='70.0',
            description='LiDAR angle window centered on 0 rad used for front clearance.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_projection_stride_px',
            default_value='8',
            description='Depth obstacle cell stride. Lower values detect thinner chair/table legs at higher CPU cost.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_ground_filter_enabled',
            default_value='true',
            description='Filter ZED depth by rejecting the floor/background row model before projecting obstacles.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_ground_min_delta_m',
            default_value='0.18',
            description='Minimum depth difference from per-row floor/background before a depth pixel is an obstacle candidate.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_ground_ratio',
            default_value='0.88',
            description='Obstacle candidate depth must be this ratio or closer than the row floor/background estimate.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_obstacle_min_component_height_px',
            default_value='14',
            description='Minimum vertical image height for a ground-filtered ZED depth obstacle component.',
        ),
        DeclareLaunchArgument(
            'fusion_depth_front_corridor_half_width_m',
            default_value='0.42',
            description='Half width of the forward depth corridor used for front clearance.',
        ),
        DeclareLaunchArgument(
            'fusion_imu_smoothing_alpha',
            default_value='0.25',
            description='Low-pass alpha for IMU values copied into NavSensorFrame and debug summaries.',
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            condition=IfCondition(start_lidar),
            parameters=[{
                'serial_port': ParameterValue(lidar_port, value_type=str),
                'serial_baudrate': ParameterValue(lidar_baud, value_type=int),
                'frame_id': 'lidar',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                zed_sync_script,
                '--ros-args',
                '-r',
                '__node:=zed_sync_node',
                '-p',
                ros_param_arg('publish_rate_hz', zed_publish_rate_hz),
                '-p',
                ros_param_arg('imu_publish_rate_hz', zed_imu_publish_rate_hz),
                '-p',
                ros_param_arg('depth_downsample_factor', zed_depth_downsample_factor),
                '-p',
                ros_param_arg('publish_image', zed_publish_image),
            ],
            output='screen',
            condition=IfCondition(start_zed),
        ),

        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='python3'),
                    lidar_sync_script,
                    '--ros-args',
                    '-r',
                    '__node:=lidar_sync_node',
                    '-p',
                    ros_param_arg('scan_freq_hz', lidar_scan_freq_hz),
                ],
                output='screen',
                condition=IfCondition(start_lidar),
            ),
        ]),

        TimerAction(period=2.5, actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='python3'),
                    lidar_filter_script,
                    '--ros-args',
                    '-r',
                    '__node:=lidar_scan_filter_node',
                    '-p',
                    ros_param_arg('input_topic', '/scan/synced'),
                    '-p',
                    ros_param_arg('output_topic', lidar_filtered_topic),
                    '-p',
                    ros_param_arg('forward_fov_deg', lidar_filter_forward_fov_deg),
                ],
                output='screen',
                condition=IfCondition(start_lidar_filter),
            ),
        ]),

        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                uwb_script,
                '--ros-args',
                '-r',
                '__node:=uwb_node',
                '-p',
                'port:=/dev/ttyUSB1',
                '-p',
                'baud:=115200',
                '-p',
                'use_esp32_timestamp:=false',
            ],
            output='screen',
            condition=IfCondition(start_uwb),
        ),

        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='python3'),
                    fusion_script,
                    '--ros-args',
                    '-r',
                    '__node:=fusion_node',
                    '-p',
                    ros_param_arg('scan_topic', lidar_filtered_topic),
                    '-p',
                    ros_param_arg('zed_fresh_timeout_s', fusion_zed_fresh_timeout_s),
                    '-p',
                    ros_param_arg('allow_lidar_only_fallback', fusion_allow_lidar_only),
                    '-p',
                    ros_param_arg('depth_invalid_warn_frames', fusion_depth_invalid_warn_frames),
                    '-p',
                    ros_param_arg('lidar_front_fov_deg', fusion_lidar_front_fov_deg),
                    '-p',
                    ros_param_arg('depth_projection_stride_px', fusion_depth_projection_stride_px),
                    '-p',
                    ros_param_arg('depth_ground_filter_enabled', fusion_depth_ground_filter_enabled),
                    '-p',
                    ros_param_arg('depth_ground_min_delta_m', fusion_depth_ground_min_delta_m),
                    '-p',
                    ros_param_arg('depth_ground_ratio', fusion_depth_ground_ratio),
                    '-p',
                    ros_param_arg('depth_obstacle_min_component_height_px', fusion_depth_obstacle_min_component_height_px),
                    '-p',
                    ros_param_arg('depth_front_corridor_half_width_m', fusion_depth_front_corridor_half_width_m),
                    '-p',
                    ros_param_arg('imu_smoothing_alpha', fusion_imu_smoothing_alpha),
                ],
                output='screen',
                condition=IfCondition(start_fusion),
            ),
        ]),
    ])
