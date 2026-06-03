from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    dry_run = LaunchConfiguration("dry_run")
    command_timeout_s = LaunchConfiguration("command_timeout_s")
    command_refresh_period_s = LaunchConfiguration("command_refresh_period_s")
    track_width_m = LaunchConfiguration("track_width_m")
    left_forward_speed_scale = LaunchConfiguration("left_forward_speed_scale")
    right_forward_speed_scale = LaunchConfiguration("right_forward_speed_scale")
    left_reverse_speed_scale = LaunchConfiguration("left_reverse_speed_scale")
    right_reverse_speed_scale = LaunchConfiguration("right_reverse_speed_scale")
    wheel_radius_m = LaunchConfiguration("wheel_radius_m")
    ticks_per_rev = LaunchConfiguration("ticks_per_rev")
    pwm_min_us = LaunchConfiguration("pwm_min_us")
    pwm_neutral_us = LaunchConfiguration("pwm_neutral_us")
    pwm_max_us = LaunchConfiguration("pwm_max_us")
    pwm_slew_rate_us_per_s = LaunchConfiguration("pwm_slew_rate_us_per_s")
    teensy_control_hz = LaunchConfiguration("teensy_control_hz")
    teensy_pid_kp = LaunchConfiguration("teensy_pid_kp")
    teensy_pid_ki = LaunchConfiguration("teensy_pid_ki")
    teensy_pid_kd = LaunchConfiguration("teensy_pid_kd")
    teensy_pid_feedforward_us_per_tps = LaunchConfiguration("teensy_pid_feedforward_us_per_tps")
    teensy_left_pid_feedforward_us_per_tps = LaunchConfiguration("teensy_left_pid_feedforward_us_per_tps")
    teensy_right_pid_feedforward_us_per_tps = LaunchConfiguration("teensy_right_pid_feedforward_us_per_tps")
    teensy_pid_static_ff_us = LaunchConfiguration("teensy_pid_static_ff_us")
    teensy_left_pid_static_ff_us = LaunchConfiguration("teensy_left_pid_static_ff_us")
    teensy_right_pid_static_ff_us = LaunchConfiguration("teensy_right_pid_static_ff_us")
    teensy_pid_output_limit_us = LaunchConfiguration("teensy_pid_output_limit_us")
    teensy_left_pid_output_limit_us = LaunchConfiguration("teensy_left_pid_output_limit_us")
    teensy_right_pid_output_limit_us = LaunchConfiguration("teensy_right_pid_output_limit_us")
    teensy_pid_min_target_tps = LaunchConfiguration("teensy_pid_min_target_tps")
    teensy_left_motor_sign = LaunchConfiguration("teensy_left_motor_sign")
    teensy_right_motor_sign = LaunchConfiguration("teensy_right_motor_sign")
    teensy_fl_encoder_sign = LaunchConfiguration("teensy_fl_encoder_sign")
    teensy_fr_encoder_sign = LaunchConfiguration("teensy_fr_encoder_sign")
    teensy_rl_encoder_sign = LaunchConfiguration("teensy_rl_encoder_sign")
    teensy_rr_encoder_sign = LaunchConfiguration("teensy_rr_encoder_sign")
    teensy_stall_fault_enabled = LaunchConfiguration("teensy_stall_fault_enabled")
    teensy_stall_target_tps = LaunchConfiguration("teensy_stall_target_tps")
    teensy_stall_near_zero_tps = LaunchConfiguration("teensy_stall_near_zero_tps")
    teensy_stall_moving_peer_tps = LaunchConfiguration("teensy_stall_moving_peer_tps")
    teensy_stall_pwm_delta_us = LaunchConfiguration("teensy_stall_pwm_delta_us")
    teensy_stall_timeout_ms = LaunchConfiguration("teensy_stall_timeout_ms")
    teensy_sign_mismatch_tps = LaunchConfiguration("teensy_sign_mismatch_tps")
    teensy_sign_mismatch_target_tps = LaunchConfiguration("teensy_sign_mismatch_target_tps")
    teensy_sign_mismatch_timeout_ms = LaunchConfiguration("teensy_sign_mismatch_timeout_ms")
    teensy_side_mismatch_fault_enabled = LaunchConfiguration("teensy_side_mismatch_fault_enabled")
    teensy_side_mismatch_warn_tps = LaunchConfiguration("teensy_side_mismatch_warn_tps")
    teensy_side_mismatch_fault_tps = LaunchConfiguration("teensy_side_mismatch_fault_tps")
    teensy_encoder_jump_fault_enabled = LaunchConfiguration("teensy_encoder_jump_fault_enabled")
    teensy_encoder_jump_tps = LaunchConfiguration("teensy_encoder_jump_tps")
    teensy_pid_param_ack_timeout_s = LaunchConfiguration("teensy_pid_param_ack_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.75"),
            DeclareLaunchArgument("command_refresh_period_s", default_value="0.10"),
            DeclareLaunchArgument(
                "track_width_m",
                default_value=EnvironmentVariable("MOTOR_TRACK_WIDTH_M", default_value="0.416"),
            ),
            DeclareLaunchArgument(
                "left_forward_speed_scale",
                default_value=EnvironmentVariable("MOTOR_LEFT_FORWARD_SPEED_SCALE", default_value="1.0"),
            ),
            DeclareLaunchArgument(
                "right_forward_speed_scale",
                default_value=EnvironmentVariable("MOTOR_RIGHT_FORWARD_SPEED_SCALE", default_value="1.0"),
            ),
            DeclareLaunchArgument(
                "left_reverse_speed_scale",
                default_value=EnvironmentVariable("MOTOR_LEFT_REVERSE_SPEED_SCALE", default_value="1.0"),
            ),
            DeclareLaunchArgument(
                "right_reverse_speed_scale",
                default_value=EnvironmentVariable("MOTOR_RIGHT_REVERSE_SPEED_SCALE", default_value="1.0"),
            ),
            DeclareLaunchArgument(
                "wheel_radius_m",
                default_value=EnvironmentVariable("MOTOR_WHEEL_RADIUS_M", default_value="0.0825"),
            ),
            DeclareLaunchArgument(
                "ticks_per_rev",
                default_value=EnvironmentVariable("MOTOR_TICKS_PER_REV", default_value="3200"),
            ),
            DeclareLaunchArgument(
                "pwm_min_us",
                default_value=EnvironmentVariable("MOTOR_PWM_MIN_US", default_value="1100"),
            ),
            DeclareLaunchArgument(
                "pwm_neutral_us",
                default_value=EnvironmentVariable("MOTOR_PWM_NEUTRAL_US", default_value="1500"),
            ),
            DeclareLaunchArgument(
                "pwm_max_us",
                default_value=EnvironmentVariable("MOTOR_PWM_MAX_US", default_value="1900"),
            ),
            DeclareLaunchArgument(
                "pwm_slew_rate_us_per_s",
                default_value=EnvironmentVariable("MOTOR_PWM_SLEW_RATE_US_PER_S", default_value="2400.0"),
            ),
            DeclareLaunchArgument(
                "teensy_control_hz",
                default_value=EnvironmentVariable("MOTOR_TEENSY_CONTROL_HZ", default_value="50.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_kp",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_KP", default_value="0.05"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_ki",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_KI", default_value="0.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_kd",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_KD", default_value="0.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_feedforward_us_per_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_FF_US_PER_TPS", default_value="0.04"),
            ),
            DeclareLaunchArgument(
                "teensy_left_pid_feedforward_us_per_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_LEFT_PID_FF_US_PER_TPS", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_right_pid_feedforward_us_per_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RIGHT_PID_FF_US_PER_TPS", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_static_ff_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_STATIC_FF_US", default_value="170.0"),
            ),
            DeclareLaunchArgument(
                "teensy_left_pid_static_ff_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_LEFT_PID_STATIC_FF_US", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_right_pid_static_ff_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RIGHT_PID_STATIC_FF_US", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_output_limit_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_OUTPUT_LIMIT_US", default_value="350.0"),
            ),
            DeclareLaunchArgument(
                "teensy_left_pid_output_limit_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_LEFT_PID_OUTPUT_LIMIT_US", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_right_pid_output_limit_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RIGHT_PID_OUTPUT_LIMIT_US", default_value="-1.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_min_target_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PID_MIN_TARGET_TPS", default_value="2.0"),
            ),
            DeclareLaunchArgument(
                "teensy_left_motor_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_LEFT_MOTOR_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_right_motor_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RIGHT_MOTOR_SIGN", default_value="-1"),
            ),
            DeclareLaunchArgument(
                "teensy_fl_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_FL_ENCODER_SIGN", default_value="-1"),
            ),
            DeclareLaunchArgument(
                "teensy_fr_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_FR_ENCODER_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_rl_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RL_ENCODER_SIGN", default_value="-1"),
            ),
            DeclareLaunchArgument(
                "teensy_rr_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RR_ENCODER_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_fault_enabled",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_FAULT_ENABLED", default_value="true"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_target_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_TARGET_TPS", default_value="15.0"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_near_zero_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_NEAR_ZERO_TPS", default_value="2.0"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_moving_peer_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_MOVING_PEER_TPS", default_value="12.0"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_pwm_delta_us",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_PWM_DELTA_US", default_value="120.0"),
            ),
            DeclareLaunchArgument(
                "teensy_stall_timeout_ms",
                default_value=EnvironmentVariable("MOTOR_TEENSY_STALL_TIMEOUT_MS", default_value="300"),
            ),
            DeclareLaunchArgument(
                "teensy_sign_mismatch_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIGN_MISMATCH_TPS", default_value="10.0"),
            ),
            DeclareLaunchArgument(
                "teensy_sign_mismatch_target_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIGN_MISMATCH_TARGET_TPS", default_value="100.0"),
            ),
            DeclareLaunchArgument(
                "teensy_sign_mismatch_timeout_ms",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIGN_MISMATCH_TIMEOUT_MS", default_value="250"),
            ),
            DeclareLaunchArgument(
                "teensy_side_mismatch_fault_enabled",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIDE_MISMATCH_FAULT_ENABLED", default_value="true"),
            ),
            DeclareLaunchArgument(
                "teensy_side_mismatch_warn_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIDE_MISMATCH_WARN_TPS", default_value="80.0"),
            ),
            DeclareLaunchArgument(
                "teensy_side_mismatch_fault_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_SIDE_MISMATCH_FAULT_TPS", default_value="180.0"),
            ),
            DeclareLaunchArgument(
                "teensy_encoder_jump_fault_enabled",
                default_value=EnvironmentVariable("MOTOR_TEENSY_ENCODER_JUMP_FAULT_ENABLED", default_value="true"),
            ),
            DeclareLaunchArgument(
                "teensy_encoder_jump_tps",
                default_value=EnvironmentVariable("MOTOR_TEENSY_ENCODER_JUMP_TPS", default_value="12000.0"),
            ),
            DeclareLaunchArgument(
                "teensy_pid_param_ack_timeout_s",
                default_value=EnvironmentVariable("MOTOR_TEENSY_PARAM_ACK_TIMEOUT_S", default_value="1.0"),
            ),
            Node(
                package="ugv_motor_controller",
                executable="motor_controller_bridge",
                name="motor_controller_bridge",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(port, value_type=str),
                        "baud": ParameterValue(baud, value_type=int),
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "command_timeout_s": ParameterValue(command_timeout_s, value_type=float),
                        "command_refresh_period_s": ParameterValue(command_refresh_period_s, value_type=float),
                        "track_width_m": ParameterValue(track_width_m, value_type=float),
                        "left_forward_speed_scale": ParameterValue(left_forward_speed_scale, value_type=float),
                        "right_forward_speed_scale": ParameterValue(right_forward_speed_scale, value_type=float),
                        "left_reverse_speed_scale": ParameterValue(left_reverse_speed_scale, value_type=float),
                        "right_reverse_speed_scale": ParameterValue(right_reverse_speed_scale, value_type=float),
                        "wheel_radius_m": ParameterValue(wheel_radius_m, value_type=float),
                        "ticks_per_rev": ParameterValue(ticks_per_rev, value_type=int),
                        "pwm_min_us": ParameterValue(pwm_min_us, value_type=int),
                        "pwm_neutral_us": ParameterValue(pwm_neutral_us, value_type=int),
                        "pwm_max_us": ParameterValue(pwm_max_us, value_type=int),
                        "pwm_slew_rate_us_per_s": ParameterValue(pwm_slew_rate_us_per_s, value_type=float),
                        "teensy_control_hz": ParameterValue(teensy_control_hz, value_type=float),
                        "teensy_pid_kp": ParameterValue(teensy_pid_kp, value_type=float),
                        "teensy_pid_ki": ParameterValue(teensy_pid_ki, value_type=float),
                        "teensy_pid_kd": ParameterValue(teensy_pid_kd, value_type=float),
                        "teensy_pid_feedforward_us_per_tps": ParameterValue(
                            teensy_pid_feedforward_us_per_tps,
                            value_type=float,
                        ),
                        "teensy_left_pid_feedforward_us_per_tps": ParameterValue(
                            teensy_left_pid_feedforward_us_per_tps,
                            value_type=float,
                        ),
                        "teensy_right_pid_feedforward_us_per_tps": ParameterValue(
                            teensy_right_pid_feedforward_us_per_tps,
                            value_type=float,
                        ),
                        "teensy_pid_static_ff_us": ParameterValue(teensy_pid_static_ff_us, value_type=float),
                        "teensy_left_pid_static_ff_us": ParameterValue(teensy_left_pid_static_ff_us, value_type=float),
                        "teensy_right_pid_static_ff_us": ParameterValue(teensy_right_pid_static_ff_us, value_type=float),
                        "teensy_pid_output_limit_us": ParameterValue(teensy_pid_output_limit_us, value_type=float),
                        "teensy_left_pid_output_limit_us": ParameterValue(teensy_left_pid_output_limit_us, value_type=float),
                        "teensy_right_pid_output_limit_us": ParameterValue(teensy_right_pid_output_limit_us, value_type=float),
                        "teensy_pid_min_target_tps": ParameterValue(teensy_pid_min_target_tps, value_type=float),
                        "teensy_left_motor_sign": ParameterValue(teensy_left_motor_sign, value_type=int),
                        "teensy_right_motor_sign": ParameterValue(teensy_right_motor_sign, value_type=int),
                        "teensy_fl_encoder_sign": ParameterValue(teensy_fl_encoder_sign, value_type=int),
                        "teensy_fr_encoder_sign": ParameterValue(teensy_fr_encoder_sign, value_type=int),
                        "teensy_rl_encoder_sign": ParameterValue(teensy_rl_encoder_sign, value_type=int),
                        "teensy_rr_encoder_sign": ParameterValue(teensy_rr_encoder_sign, value_type=int),
                        "teensy_stall_fault_enabled": ParameterValue(teensy_stall_fault_enabled, value_type=bool),
                        "teensy_stall_target_tps": ParameterValue(teensy_stall_target_tps, value_type=float),
                        "teensy_stall_near_zero_tps": ParameterValue(teensy_stall_near_zero_tps, value_type=float),
                        "teensy_stall_moving_peer_tps": ParameterValue(teensy_stall_moving_peer_tps, value_type=float),
                        "teensy_stall_pwm_delta_us": ParameterValue(teensy_stall_pwm_delta_us, value_type=float),
                        "teensy_stall_timeout_ms": ParameterValue(teensy_stall_timeout_ms, value_type=int),
                        "teensy_sign_mismatch_tps": ParameterValue(teensy_sign_mismatch_tps, value_type=float),
                        "teensy_sign_mismatch_target_tps": ParameterValue(
                            teensy_sign_mismatch_target_tps,
                            value_type=float,
                        ),
                        "teensy_sign_mismatch_timeout_ms": ParameterValue(
                            teensy_sign_mismatch_timeout_ms,
                            value_type=int,
                        ),
                        "teensy_side_mismatch_fault_enabled": ParameterValue(
                            teensy_side_mismatch_fault_enabled,
                            value_type=bool,
                        ),
                        "teensy_side_mismatch_warn_tps": ParameterValue(
                            teensy_side_mismatch_warn_tps,
                            value_type=float,
                        ),
                        "teensy_side_mismatch_fault_tps": ParameterValue(
                            teensy_side_mismatch_fault_tps,
                            value_type=float,
                        ),
                        "teensy_encoder_jump_fault_enabled": ParameterValue(
                            teensy_encoder_jump_fault_enabled,
                            value_type=bool,
                        ),
                        "teensy_encoder_jump_tps": ParameterValue(teensy_encoder_jump_tps, value_type=float),
                        "teensy_pid_param_ack_timeout_s": ParameterValue(
                            teensy_pid_param_ack_timeout_s,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
