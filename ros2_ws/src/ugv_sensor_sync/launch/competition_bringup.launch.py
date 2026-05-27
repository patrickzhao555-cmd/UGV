from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration


def _workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").exists():
            return parent
    raise RuntimeError("Could not find workspace root containing src/ugv_nav/ugv_nav_dual_mode.py")


def generate_launch_description():
    workspace_root = _workspace_root()
    motor_launch = workspace_root / "src" / "ugv_motor_controller" / "launch" / "motor_controller.launch.py"
    nav_script = workspace_root / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"

    start_motor_controller = LaunchConfiguration("start_motor_controller")
    start_nav = LaunchConfiguration("start_nav")
    motor_port = LaunchConfiguration("motor_port")
    motor_baud = LaunchConfiguration("motor_baud")
    motor_dry_run = LaunchConfiguration("motor_dry_run")
    motor_command_timeout_s = LaunchConfiguration("motor_command_timeout_s")
    motor_command_refresh_period_s = LaunchConfiguration("motor_command_refresh_period_s")
    motor_wheel_radius_m = LaunchConfiguration("motor_wheel_radius_m")
    motor_ticks_per_rev = LaunchConfiguration("motor_ticks_per_rev")
    motor_teensy_left_motor_sign = LaunchConfiguration("motor_teensy_left_motor_sign")
    motor_teensy_right_motor_sign = LaunchConfiguration("motor_teensy_right_motor_sign")
    motor_teensy_fl_encoder_sign = LaunchConfiguration("motor_teensy_fl_encoder_sign")
    motor_teensy_fr_encoder_sign = LaunchConfiguration("motor_teensy_fr_encoder_sign")
    motor_teensy_rl_encoder_sign = LaunchConfiguration("motor_teensy_rl_encoder_sign")
    motor_teensy_rr_encoder_sign = LaunchConfiguration("motor_teensy_rr_encoder_sign")
    nav_status_period_s = LaunchConfiguration("nav_status_period_s")

    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(motor_launch)),
        launch_arguments={
            "port": motor_port,
            "baud": motor_baud,
            "dry_run": motor_dry_run,
            "command_timeout_s": motor_command_timeout_s,
            "command_refresh_period_s": motor_command_refresh_period_s,
            "wheel_radius_m": motor_wheel_radius_m,
            "ticks_per_rev": motor_ticks_per_rev,
            "teensy_left_motor_sign": motor_teensy_left_motor_sign,
            "teensy_right_motor_sign": motor_teensy_right_motor_sign,
            "teensy_fl_encoder_sign": motor_teensy_fl_encoder_sign,
            "teensy_fr_encoder_sign": motor_teensy_fr_encoder_sign,
            "teensy_rl_encoder_sign": motor_teensy_rl_encoder_sign,
            "teensy_rr_encoder_sign": motor_teensy_rr_encoder_sign,
        }.items(),
        condition=IfCondition(start_motor_controller),
    )

    nav_placeholder = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(nav_script),
            "--mode",
            "real",
            "--nav-status-period-s",
            nav_status_period_s,
        ],
        output="screen",
        condition=IfCondition(start_nav),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_motor_controller", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("motor_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("motor_baud", default_value="115200"),
            DeclareLaunchArgument("motor_dry_run", default_value="false"),
            DeclareLaunchArgument("motor_command_timeout_s", default_value="0.75"),
            DeclareLaunchArgument("motor_command_refresh_period_s", default_value="0.10"),
            DeclareLaunchArgument("motor_wheel_radius_m", default_value="0.0889"),
            DeclareLaunchArgument("motor_ticks_per_rev", default_value="3200"),
            DeclareLaunchArgument("motor_teensy_left_motor_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_right_motor_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_fl_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_fr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_rl_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_rr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("nav_status_period_s", default_value="1.0"),
            motor_controller_launch,
            nav_placeholder,
        ]
    )
