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
    motor_track_width_m = LaunchConfiguration("motor_track_width_m")
    motor_wheel_radius_m = LaunchConfiguration("motor_wheel_radius_m")
    motor_ticks_per_rev = LaunchConfiguration("motor_ticks_per_rev")
    motor_teensy_control_hz = LaunchConfiguration("motor_teensy_control_hz")
    motor_teensy_pid_kp = LaunchConfiguration("motor_teensy_pid_kp")
    motor_teensy_pid_ki = LaunchConfiguration("motor_teensy_pid_ki")
    motor_teensy_pid_kd = LaunchConfiguration("motor_teensy_pid_kd")
    motor_teensy_pid_feedforward_us_per_tps = LaunchConfiguration("motor_teensy_pid_feedforward_us_per_tps")
    motor_teensy_pid_static_ff_us = LaunchConfiguration("motor_teensy_pid_static_ff_us")
    motor_teensy_left_motor_sign = LaunchConfiguration("motor_teensy_left_motor_sign")
    motor_teensy_right_motor_sign = LaunchConfiguration("motor_teensy_right_motor_sign")
    motor_teensy_fl_encoder_sign = LaunchConfiguration("motor_teensy_fl_encoder_sign")
    motor_teensy_fr_encoder_sign = LaunchConfiguration("motor_teensy_fr_encoder_sign")
    motor_teensy_rl_encoder_sign = LaunchConfiguration("motor_teensy_rl_encoder_sign")
    motor_teensy_rr_encoder_sign = LaunchConfiguration("motor_teensy_rr_encoder_sign")
    motor_teensy_side_mismatch_warn_tps = LaunchConfiguration("motor_teensy_side_mismatch_warn_tps")
    motor_teensy_side_mismatch_fault_tps = LaunchConfiguration("motor_teensy_side_mismatch_fault_tps")
    motor_teensy_encoder_jump_tps = LaunchConfiguration("motor_teensy_encoder_jump_tps")
    nav_status_period_s = LaunchConfiguration("nav_status_period_s")
    nav_controller_mode = LaunchConfiguration("nav_controller_mode")
    nav_frame_topic = LaunchConfiguration("nav_frame_topic")
    nav_motor_status_topic = LaunchConfiguration("nav_motor_status_topic")
    nav_straight_speed_mps = LaunchConfiguration("nav_straight_speed_mps")
    nav_straight_duration_s = LaunchConfiguration("nav_straight_duration_s")
    nav_pivot_angle_deg = LaunchConfiguration("nav_pivot_angle_deg")
    nav_max_omega_radps = LaunchConfiguration("nav_max_omega_radps")
    nav_heading_kp = LaunchConfiguration("nav_heading_kp")
    nav_heading_kd = LaunchConfiguration("nav_heading_kd")
    nav_pivot_kp = LaunchConfiguration("nav_pivot_kp")
    nav_heading_deadband_rad = LaunchConfiguration("nav_heading_deadband_rad")
    nav_stop_clearance_m = LaunchConfiguration("nav_stop_clearance_m")
    nav_sensor_timeout_s = LaunchConfiguration("nav_sensor_timeout_s")
    nav_motor_status_timeout_s = LaunchConfiguration("nav_motor_status_timeout_s")

    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(motor_launch)),
        launch_arguments={
            "port": motor_port,
            "baud": motor_baud,
            "dry_run": motor_dry_run,
            "command_timeout_s": motor_command_timeout_s,
            "command_refresh_period_s": motor_command_refresh_period_s,
            "track_width_m": motor_track_width_m,
            "wheel_radius_m": motor_wheel_radius_m,
            "ticks_per_rev": motor_ticks_per_rev,
            "teensy_control_hz": motor_teensy_control_hz,
            "teensy_pid_kp": motor_teensy_pid_kp,
            "teensy_pid_ki": motor_teensy_pid_ki,
            "teensy_pid_kd": motor_teensy_pid_kd,
            "teensy_pid_feedforward_us_per_tps": motor_teensy_pid_feedforward_us_per_tps,
            "teensy_pid_static_ff_us": motor_teensy_pid_static_ff_us,
            "teensy_left_motor_sign": motor_teensy_left_motor_sign,
            "teensy_right_motor_sign": motor_teensy_right_motor_sign,
            "teensy_fl_encoder_sign": motor_teensy_fl_encoder_sign,
            "teensy_fr_encoder_sign": motor_teensy_fr_encoder_sign,
            "teensy_rl_encoder_sign": motor_teensy_rl_encoder_sign,
            "teensy_rr_encoder_sign": motor_teensy_rr_encoder_sign,
            "teensy_side_mismatch_warn_tps": motor_teensy_side_mismatch_warn_tps,
            "teensy_side_mismatch_fault_tps": motor_teensy_side_mismatch_fault_tps,
            "teensy_encoder_jump_tps": motor_teensy_encoder_jump_tps,
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
            "--controller-mode",
            nav_controller_mode,
            "--nav-frame-topic",
            nav_frame_topic,
            "--motor-status-topic",
            nav_motor_status_topic,
            "--straight-speed-mps",
            nav_straight_speed_mps,
            "--straight-duration-s",
            nav_straight_duration_s,
            "--pivot-angle-deg",
            nav_pivot_angle_deg,
            "--max-omega-radps",
            nav_max_omega_radps,
            "--heading-kp",
            nav_heading_kp,
            "--heading-kd",
            nav_heading_kd,
            "--pivot-kp",
            nav_pivot_kp,
            "--heading-deadband-rad",
            nav_heading_deadband_rad,
            "--stop-clearance-m",
            nav_stop_clearance_m,
            "--sensor-timeout-s",
            nav_sensor_timeout_s,
            "--motor-status-timeout-s",
            nav_motor_status_timeout_s,
            "--track-width-m",
            motor_track_width_m,
            "--wheel-radius-m",
            motor_wheel_radius_m,
            "--ticks-per-rev",
            motor_ticks_per_rev,
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
            DeclareLaunchArgument("motor_track_width_m", default_value="0.425"),
            DeclareLaunchArgument("motor_wheel_radius_m", default_value="0.0825"),
            DeclareLaunchArgument("motor_ticks_per_rev", default_value="3200"),
            DeclareLaunchArgument("motor_teensy_control_hz", default_value="50.0"),
            DeclareLaunchArgument("motor_teensy_pid_kp", default_value="0.05"),
            DeclareLaunchArgument("motor_teensy_pid_ki", default_value="0.0"),
            DeclareLaunchArgument("motor_teensy_pid_kd", default_value="0.0"),
            DeclareLaunchArgument("motor_teensy_pid_feedforward_us_per_tps", default_value="0.04"),
            DeclareLaunchArgument("motor_teensy_pid_static_ff_us", default_value="170.0"),
            DeclareLaunchArgument("motor_teensy_left_motor_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_right_motor_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_fl_encoder_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_fr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_rl_encoder_sign", default_value="-1"),
            DeclareLaunchArgument("motor_teensy_rr_encoder_sign", default_value="1"),
            DeclareLaunchArgument("motor_teensy_side_mismatch_warn_tps", default_value="80.0"),
            DeclareLaunchArgument("motor_teensy_side_mismatch_fault_tps", default_value="180.0"),
            DeclareLaunchArgument("motor_teensy_encoder_jump_tps", default_value="12000.0"),
            DeclareLaunchArgument("nav_status_period_s", default_value="0.25"),
            DeclareLaunchArgument("nav_controller_mode", default_value="idle"),
            DeclareLaunchArgument("nav_frame_topic", default_value="/sensors/nav_frame"),
            DeclareLaunchArgument("nav_motor_status_topic", default_value="/motor_controller/status"),
            DeclareLaunchArgument("nav_straight_speed_mps", default_value="0.20"),
            DeclareLaunchArgument("nav_straight_duration_s", default_value="2.0"),
            DeclareLaunchArgument("nav_pivot_angle_deg", default_value="90.0"),
            DeclareLaunchArgument("nav_max_omega_radps", default_value="0.45"),
            DeclareLaunchArgument("nav_heading_kp", default_value="1.2"),
            DeclareLaunchArgument("nav_heading_kd", default_value="0.15"),
            DeclareLaunchArgument("nav_pivot_kp", default_value="1.0"),
            DeclareLaunchArgument("nav_heading_deadband_rad", default_value="0.025"),
            DeclareLaunchArgument("nav_stop_clearance_m", default_value="0.45"),
            DeclareLaunchArgument("nav_sensor_timeout_s", default_value="0.30"),
            DeclareLaunchArgument("nav_motor_status_timeout_s", default_value="0.50"),
            motor_controller_launch,
            nav_placeholder,
        ]
    )
