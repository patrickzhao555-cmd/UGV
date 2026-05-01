from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    start_uwb = LaunchConfiguration('start_uwb')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    lidar_scan_freq_hz = LaunchConfiguration('lidar_scan_freq_hz')
    zed_publish_rate_hz = LaunchConfiguration('zed_publish_rate_hz')
    zed_depth_downsample_factor = LaunchConfiguration('zed_depth_downsample_factor')
    fusion_zed_fresh_timeout_s = LaunchConfiguration('fusion_zed_fresh_timeout_s')
    fusion_depth_invalid_warn_frames = LaunchConfiguration('fusion_depth_invalid_warn_frames')
    fusion_lidar_front_fov_deg = LaunchConfiguration('fusion_lidar_front_fov_deg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_uwb',
            default_value='false',
            description='Launch the UWB serial bridge as part of the sensor sync stack.',
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
            'zed_publish_rate_hz',
            default_value='10.0',
            description='Target ZED publish rate.',
        ),
        DeclareLaunchArgument(
            'zed_depth_downsample_factor',
            default_value='2',
            description='Integer decimation factor applied before publishing ZED depth frames.',
        ),
        DeclareLaunchArgument(
            'fusion_zed_fresh_timeout_s',
            default_value='0.75',
            description='Maximum receive age allowed when fusion falls back to the freshest ZED frame.',
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

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': ParameterValue(lidar_port, value_type=str),
                'serial_baudrate': ParameterValue(lidar_baud, value_type=int),
                'frame_id': 'lidar',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        Node(
            package='ugv_sensor_sync',
            executable='zed_sync_node',
            name='zed_sync_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': ParameterValue(zed_publish_rate_hz, value_type=float),
                'depth_downsample_factor': ParameterValue(zed_depth_downsample_factor, value_type=int),
            }],
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='ugv_sensor_sync',
                executable='lidar_sync_node',
                name='lidar_sync_node',
                output='screen',
                parameters=[{
                    'scan_freq_hz': ParameterValue(lidar_scan_freq_hz, value_type=float),
                }],
            ),
        ]),

        Node(
            package='ugv_sensor_sync',
            executable='uwb_node',
            name='uwb_node',
            output='screen',
            condition=IfCondition(start_uwb),
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 115200,
                'use_esp32_timestamp': False,
            }],
        ),

        TimerAction(period=3.0, actions=[
            Node(
                package='ugv_sensor_sync',
                executable='fusion_node',
                name='fusion_node',
                output='screen',
                parameters=[{
                    'zed_fresh_timeout_s': ParameterValue(fusion_zed_fresh_timeout_s, value_type=float),
                    'depth_invalid_warn_frames': ParameterValue(fusion_depth_invalid_warn_frames, value_type=int),
                    'lidar_front_fov_deg': ParameterValue(fusion_lidar_front_fov_deg, value_type=float),
                }],
            ),
        ]),
    ])
