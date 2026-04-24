from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    raw_command_scale_us = LaunchConfiguration('raw_command_scale_us')
    command_timeout_s = LaunchConfiguration('command_timeout_s')
    status_period_s = LaunchConfiguration('status_period_s')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('command_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('status_period_s', default_value='0.5'),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='false'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        Node(
            package='ugv_motor_controller',
            executable='motor_controller_bridge',
            name='motor_controller_bridge',
            output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=str),
                'baud': ParameterValue(baud, value_type=int),
                'raw_command_scale_us': ParameterValue(raw_command_scale_us, value_type=float),
                'command_timeout_s': ParameterValue(command_timeout_s, value_type=float),
                'status_period_s': ParameterValue(status_period_s, value_type=float),
                'invert_left_command': ParameterValue(invert_left_command, value_type=bool),
                'invert_right_command': ParameterValue(invert_right_command, value_type=bool),
                'invert_left_encoder': ParameterValue(invert_left_encoder, value_type=bool),
                'invert_right_encoder': ParameterValue(invert_right_encoder, value_type=bool),
            }],
        ),
    ])
