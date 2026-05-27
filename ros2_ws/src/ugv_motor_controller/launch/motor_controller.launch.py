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
    wheel_radius_m = LaunchConfiguration("wheel_radius_m")
    ticks_per_rev = LaunchConfiguration("ticks_per_rev")
    teensy_pid_kp = LaunchConfiguration("teensy_pid_kp")
    teensy_pid_ki = LaunchConfiguration("teensy_pid_ki")
    teensy_pid_kd = LaunchConfiguration("teensy_pid_kd")
    teensy_left_motor_sign = LaunchConfiguration("teensy_left_motor_sign")
    teensy_right_motor_sign = LaunchConfiguration("teensy_right_motor_sign")
    teensy_fl_encoder_sign = LaunchConfiguration("teensy_fl_encoder_sign")
    teensy_fr_encoder_sign = LaunchConfiguration("teensy_fr_encoder_sign")
    teensy_rl_encoder_sign = LaunchConfiguration("teensy_rl_encoder_sign")
    teensy_rr_encoder_sign = LaunchConfiguration("teensy_rr_encoder_sign")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("command_timeout_s", default_value="0.75"),
            DeclareLaunchArgument("command_refresh_period_s", default_value="0.10"),
            DeclareLaunchArgument("track_width_m", default_value="0.6096"),
            DeclareLaunchArgument(
                "wheel_radius_m",
                default_value=EnvironmentVariable("MOTOR_WHEEL_RADIUS_M", default_value="0.0889"),
            ),
            DeclareLaunchArgument(
                "ticks_per_rev",
                default_value=EnvironmentVariable("MOTOR_TICKS_PER_REV", default_value="3200"),
            ),
            DeclareLaunchArgument("teensy_pid_kp", default_value="0.80"),
            DeclareLaunchArgument("teensy_pid_ki", default_value="0.0"),
            DeclareLaunchArgument("teensy_pid_kd", default_value="0.02"),
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
                default_value=EnvironmentVariable("MOTOR_TEENSY_FL_ENCODER_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_fr_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_FR_ENCODER_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_rl_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RL_ENCODER_SIGN", default_value="1"),
            ),
            DeclareLaunchArgument(
                "teensy_rr_encoder_sign",
                default_value=EnvironmentVariable("MOTOR_TEENSY_RR_ENCODER_SIGN", default_value="1"),
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
                        "wheel_radius_m": ParameterValue(wheel_radius_m, value_type=float),
                        "ticks_per_rev": ParameterValue(ticks_per_rev, value_type=int),
                        "teensy_pid_kp": ParameterValue(teensy_pid_kp, value_type=float),
                        "teensy_pid_ki": ParameterValue(teensy_pid_ki, value_type=float),
                        "teensy_pid_kd": ParameterValue(teensy_pid_kd, value_type=float),
                        "teensy_left_motor_sign": ParameterValue(teensy_left_motor_sign, value_type=int),
                        "teensy_right_motor_sign": ParameterValue(teensy_right_motor_sign, value_type=int),
                        "teensy_fl_encoder_sign": ParameterValue(teensy_fl_encoder_sign, value_type=int),
                        "teensy_fr_encoder_sign": ParameterValue(teensy_fr_encoder_sign, value_type=int),
                        "teensy_rl_encoder_sign": ParameterValue(teensy_rl_encoder_sign, value_type=int),
                        "teensy_rr_encoder_sign": ParameterValue(teensy_rr_encoder_sign, value_type=int),
                    }
                ],
            ),
        ]
    )
