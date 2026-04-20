from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription([

        # --- RPLiDAR A1M8 driver (publishes /scan) ---
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port':      '/dev/ttyUSB0',  # update if different
                'serial_baudrate':  115200,
                'frame_id':         'lidar',
                'inverted':         False,
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }],
        ),

        # --- ZED camera + IMU ---
        Node(
            package='ugv_sensor_sync',
            executable='zed_sync_node',
            name='zed_sync_node',
            output='screen',
        ),

        # --- Lidar re-stamper (waits 2s for /scan to appear) ---
        TimerAction(period=2.0, actions=[
            Node(
                package='ugv_sensor_sync',
                executable='lidar_sync_node',
                name='lidar_sync_node',
                output='screen',
                parameters=[{'scan_freq_hz': 5.5}],
            ),
        ]),

        # --- UWB (starts even if ESP32 not plugged in) ---
        Node(
            package='ugv_sensor_sync',
            executable='uwb_node',
            name='uwb_node',
            output='screen',
            parameters=[{
                'port':                '/dev/ttyUSB1',
                'baud':                115200,
                'use_esp32_timestamp': False,
            }],
        ),

        # --- Fusion (waits 4s for all sensors to be ready) ---
        TimerAction(period=4.0, actions=[
            Node(
                package='ugv_sensor_sync',
                executable='fusion_node',
                name='fusion_node',
                output='screen',
            ),
        ]),

    ])