from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    raw_command_scale_us = LaunchConfiguration('raw_command_scale_us')
    pwm_slew_rate_us_per_s = LaunchConfiguration('pwm_slew_rate_us_per_s')
    command_timeout_s = LaunchConfiguration('command_timeout_s')
    command_refresh_period_s = LaunchConfiguration('command_refresh_period_s')
    status_period_s = LaunchConfiguration('status_period_s')
    dry_run = LaunchConfiguration('dry_run')
    motor_control_location = LaunchConfiguration('motor_control_location')
    invert_left_command = LaunchConfiguration('invert_left_command')
    invert_right_command = LaunchConfiguration('invert_right_command')
    invert_left_encoder = LaunchConfiguration('invert_left_encoder')
    invert_right_encoder = LaunchConfiguration('invert_right_encoder')
    velocity_control_enabled = LaunchConfiguration('velocity_control_enabled')
    prefer_velocity_fields = LaunchConfiguration('prefer_velocity_fields')
    track_width_m = LaunchConfiguration('track_width_m')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    teensy_left_motor_sign = LaunchConfiguration('teensy_left_motor_sign')
    teensy_right_motor_sign = LaunchConfiguration('teensy_right_motor_sign')
    teensy_fl_encoder_sign = LaunchConfiguration('teensy_fl_encoder_sign')
    teensy_fr_encoder_sign = LaunchConfiguration('teensy_fr_encoder_sign')
    teensy_rl_encoder_sign = LaunchConfiguration('teensy_rl_encoder_sign')
    teensy_rr_encoder_sign = LaunchConfiguration('teensy_rr_encoder_sign')
    velocity_kp = LaunchConfiguration('velocity_kp')
    velocity_ki = LaunchConfiguration('velocity_ki')
    velocity_kd = LaunchConfiguration('velocity_kd')
    velocity_integral_limit = LaunchConfiguration('velocity_integral_limit')
    velocity_feedforward_raw_per_mps = LaunchConfiguration('velocity_feedforward_raw_per_mps')
    velocity_min_target_mps = LaunchConfiguration('velocity_min_target_mps')
    velocity_max_target_mps = LaunchConfiguration('velocity_max_target_mps')
    velocity_control_period_s = LaunchConfiguration('velocity_control_period_s')
    velocity_stale_encoder_timeout_s = LaunchConfiguration('velocity_stale_encoder_timeout_s')
    velocity_fallback_to_raw_without_encoder = LaunchConfiguration('velocity_fallback_to_raw_without_encoder')
    velocity_encoder_speed_filter_alpha = LaunchConfiguration('velocity_encoder_speed_filter_alpha')
    velocity_encoder_speed_max_mps = LaunchConfiguration('velocity_encoder_speed_max_mps')
    velocity_encoder_speed_min_dt_s = LaunchConfiguration('velocity_encoder_speed_min_dt_s')
    velocity_raw_fallback_floor_enabled = LaunchConfiguration('velocity_raw_fallback_floor_enabled')
    velocity_raw_fallback_min_wheel_raw = LaunchConfiguration('velocity_raw_fallback_min_wheel_raw')
    velocity_raw_fallback_min_turn_raw = LaunchConfiguration('velocity_raw_fallback_min_turn_raw')
    velocity_raw_fallback_min_target_raw = LaunchConfiguration('velocity_raw_fallback_min_target_raw')
    teensy_pid_param_ack_timeout_s = LaunchConfiguration('teensy_pid_param_ack_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('raw_command_scale_us', default_value='900.0'),
        DeclareLaunchArgument('pwm_slew_rate_us_per_s', default_value='2400.0'),
        DeclareLaunchArgument('command_timeout_s', default_value='0.75'),
        DeclareLaunchArgument('command_refresh_period_s', default_value='0.1'),
        DeclareLaunchArgument('status_period_s', default_value='0.5'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument(
            'motor_control_location',
            default_value=EnvironmentVariable('MOTOR_CONTROL_LOCATION', default_value='ros'),
        ),
        DeclareLaunchArgument('invert_left_command', default_value='false'),
        DeclareLaunchArgument('invert_right_command', default_value='true'),
        DeclareLaunchArgument('invert_left_encoder', default_value='false'),
        DeclareLaunchArgument('invert_right_encoder', default_value='false'),
        DeclareLaunchArgument('velocity_control_enabled', default_value='false'),
        DeclareLaunchArgument('prefer_velocity_fields', default_value='true'),
        DeclareLaunchArgument('track_width_m', default_value='0.6096'),
        DeclareLaunchArgument(
            'wheel_radius_m',
            default_value=EnvironmentVariable('MOTOR_WHEEL_RADIUS_M', default_value='0.0889'),
        ),
        DeclareLaunchArgument(
            'ticks_per_rev',
            default_value=EnvironmentVariable('MOTOR_TICKS_PER_REV', default_value='3200'),
        ),
        DeclareLaunchArgument(
            'teensy_left_motor_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_LEFT_MOTOR_SIGN', default_value='1'),
        ),
        DeclareLaunchArgument(
            'teensy_right_motor_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_RIGHT_MOTOR_SIGN', default_value='-1'),
        ),
        DeclareLaunchArgument(
            'teensy_fl_encoder_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_FL_ENCODER_SIGN', default_value='1'),
        ),
        DeclareLaunchArgument(
            'teensy_fr_encoder_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_FR_ENCODER_SIGN', default_value='1'),
        ),
        DeclareLaunchArgument(
            'teensy_rl_encoder_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_RL_ENCODER_SIGN', default_value='1'),
        ),
        DeclareLaunchArgument(
            'teensy_rr_encoder_sign',
            default_value=EnvironmentVariable('MOTOR_TEENSY_RR_ENCODER_SIGN', default_value='1'),
        ),
        DeclareLaunchArgument('velocity_kp', default_value='0.80'),
        DeclareLaunchArgument('velocity_ki', default_value='0.0'),
        DeclareLaunchArgument('velocity_kd', default_value='0.02'),
        DeclareLaunchArgument('velocity_integral_limit', default_value='0.30'),
        DeclareLaunchArgument('velocity_feedforward_raw_per_mps', default_value='1.35'),
        DeclareLaunchArgument('velocity_min_target_mps', default_value='0.02'),
        DeclareLaunchArgument('velocity_max_target_mps', default_value='0.60'),
        DeclareLaunchArgument('velocity_control_period_s', default_value='0.05'),
        DeclareLaunchArgument('velocity_stale_encoder_timeout_s', default_value='0.25'),
        DeclareLaunchArgument('velocity_fallback_to_raw_without_encoder', default_value='false'),
        DeclareLaunchArgument('velocity_encoder_speed_filter_alpha', default_value='0.65'),
        DeclareLaunchArgument('velocity_encoder_speed_max_mps', default_value='2.0'),
        DeclareLaunchArgument('velocity_encoder_speed_min_dt_s', default_value='0.015'),
        DeclareLaunchArgument('velocity_raw_fallback_floor_enabled', default_value='false'),
        DeclareLaunchArgument('velocity_raw_fallback_min_wheel_raw', default_value='0.0'),
        DeclareLaunchArgument('velocity_raw_fallback_min_turn_raw', default_value='0.28'),
        DeclareLaunchArgument('velocity_raw_fallback_min_target_raw', default_value='0.001'),
        DeclareLaunchArgument('teensy_pid_param_ack_timeout_s', default_value='1.0'),
        Node(
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
                'command_refresh_period_s': ParameterValue(command_refresh_period_s, value_type=float),
                'status_period_s': ParameterValue(status_period_s, value_type=float),
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'motor_control_location': ParameterValue(motor_control_location, value_type=str),
                'invert_left_command': ParameterValue(invert_left_command, value_type=bool),
                'invert_right_command': ParameterValue(invert_right_command, value_type=bool),
                'invert_left_encoder': ParameterValue(invert_left_encoder, value_type=bool),
                'invert_right_encoder': ParameterValue(invert_right_encoder, value_type=bool),
                'velocity_control_enabled': ParameterValue(velocity_control_enabled, value_type=bool),
                'prefer_velocity_fields': ParameterValue(prefer_velocity_fields, value_type=bool),
                'track_width_m': ParameterValue(track_width_m, value_type=float),
                'wheel_radius_m': ParameterValue(wheel_radius_m, value_type=float),
                'ticks_per_rev': ParameterValue(ticks_per_rev, value_type=int),
                'teensy_left_motor_sign': ParameterValue(teensy_left_motor_sign, value_type=int),
                'teensy_right_motor_sign': ParameterValue(teensy_right_motor_sign, value_type=int),
                'teensy_fl_encoder_sign': ParameterValue(teensy_fl_encoder_sign, value_type=int),
                'teensy_fr_encoder_sign': ParameterValue(teensy_fr_encoder_sign, value_type=int),
                'teensy_rl_encoder_sign': ParameterValue(teensy_rl_encoder_sign, value_type=int),
                'teensy_rr_encoder_sign': ParameterValue(teensy_rr_encoder_sign, value_type=int),
                'velocity_kp': ParameterValue(velocity_kp, value_type=float),
                'velocity_ki': ParameterValue(velocity_ki, value_type=float),
                'velocity_kd': ParameterValue(velocity_kd, value_type=float),
                'velocity_integral_limit': ParameterValue(velocity_integral_limit, value_type=float),
                'velocity_feedforward_raw_per_mps': ParameterValue(velocity_feedforward_raw_per_mps, value_type=float),
                'velocity_min_target_mps': ParameterValue(velocity_min_target_mps, value_type=float),
                'velocity_max_target_mps': ParameterValue(velocity_max_target_mps, value_type=float),
                'velocity_control_period_s': ParameterValue(velocity_control_period_s, value_type=float),
                'velocity_stale_encoder_timeout_s': ParameterValue(velocity_stale_encoder_timeout_s, value_type=float),
                'velocity_fallback_to_raw_without_encoder': ParameterValue(velocity_fallback_to_raw_without_encoder, value_type=bool),
                'velocity_encoder_speed_filter_alpha': ParameterValue(velocity_encoder_speed_filter_alpha, value_type=float),
                'velocity_encoder_speed_max_mps': ParameterValue(velocity_encoder_speed_max_mps, value_type=float),
                'velocity_encoder_speed_min_dt_s': ParameterValue(velocity_encoder_speed_min_dt_s, value_type=float),
                'velocity_raw_fallback_floor_enabled': ParameterValue(velocity_raw_fallback_floor_enabled, value_type=bool),
                'velocity_raw_fallback_min_wheel_raw': ParameterValue(velocity_raw_fallback_min_wheel_raw, value_type=float),
                'velocity_raw_fallback_min_turn_raw': ParameterValue(velocity_raw_fallback_min_turn_raw, value_type=float),
                'velocity_raw_fallback_min_target_raw': ParameterValue(velocity_raw_fallback_min_target_raw, value_type=float),
                'teensy_pid_param_ack_timeout_s': ParameterValue(teensy_pid_param_ack_timeout_s, value_type=float),
            }],
        ),
    ])
