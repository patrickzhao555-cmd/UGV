from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    raw_command_scale_us = LaunchConfiguration('raw_command_scale_us')
    pwm_slew_rate_us_per_s = LaunchConfiguration('pwm_slew_rate_us_per_s')
    command_timeout_s = LaunchConfiguration('command_timeout_s')
    status_period_s = LaunchConfiguration('status_period_s')
    dry_run = LaunchConfiguration('dry_run')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    track_width_m = LaunchConfiguration('track_width_m')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')

    motion = LaunchConfiguration('motion')
    raw = LaunchConfiguration('raw')
    raw_left = LaunchConfiguration('raw_left')
    raw_right = LaunchConfiguration('raw_right')
    duration_s = LaunchConfiguration('duration_s')
    startup_wait_s = LaunchConfiguration('startup_wait_s')
    yes = LaunchConfiguration('yes')

    motor_bridge = Node(
        package='ugv_motor_controller',
        executable='motor_controller_bridge',
        name='motor_controller_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(port, value_type=str),
            'baud': ParameterValue(baud, value_type=int),
            'raw_command_scale_us': ParameterValue(raw_command_scale_us, value_type=float),
            'pwm_slew_rate_us_per_s': ParameterValue(pwm_slew_rate_us_per_s, value_type=float),
            'command_timeout_s': ParameterValue(command_timeout_s, value_type=float),
            'status_period_s': ParameterValue(status_period_s, value_type=float),
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'invert_left_command': ParameterValue(invert_left_command, value_type=bool),
            'invert_right_command': ParameterValue(invert_right_command, value_type=bool),
            'invert_left_encoder': ParameterValue(invert_left_encoder, value_type=bool),
            'invert_right_encoder': ParameterValue(invert_right_encoder, value_type=bool),
        }],
    )

    motor_test = Node(
        package='ugv_motor_controller',
        executable='motor_direct_test',
        name='motor_direct_test',
        output='screen',
        parameters=[{
            'motion': ParameterValue(motion, value_type=str),
            'raw': ParameterValue(raw, value_type=float),
            'raw_left': ParameterValue(raw_left, value_type=float),
            'raw_right': ParameterValue(raw_right, value_type=float),
            'duration_s': ParameterValue(duration_s, value_type=float),
            'startup_wait_s': ParameterValue(startup_wait_s, value_type=float),
            'invert_left_command': ParameterValue(invert_left_command, value_type=bool),
            'invert_right_command': ParameterValue(invert_right_command, value_type=bool),
            'invert_left_encoder': ParameterValue(invert_left_encoder, value_type=bool),
            'invert_right_encoder': ParameterValue(invert_right_encoder, value_type=bool),
            'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
            'track_width_m': ParameterValue(track_width_m, value_type=float),
            'ticks_per_rev': ParameterValue(ticks_per_rev, value_type=int),
            'yes': ParameterValue(yes, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('pwm_slew_rate_us_per_s', default_value='2400.0'),
        DeclareLaunchArgument('command_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('status_period_s', default_value='0.5'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='true'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.06'),
        DeclareLaunchArgument('track_width_m', default_value='0.6096'),
        DeclareLaunchArgument('ticks_per_rev', default_value='1000'),
        DeclareLaunchArgument('motion', default_value='forward'),
        DeclareLaunchArgument('raw', default_value='0.22'),
        DeclareLaunchArgument('raw_left', default_value='0.0'),
        DeclareLaunchArgument('raw_right', default_value='0.0'),
        DeclareLaunchArgument('duration_s', default_value='0.8'),
        DeclareLaunchArgument('startup_wait_s', default_value='1.5'),
        DeclareLaunchArgument('yes', default_value='false'),
        motor_bridge,
        TimerAction(period=0.5, actions=[motor_test]),
    ])
