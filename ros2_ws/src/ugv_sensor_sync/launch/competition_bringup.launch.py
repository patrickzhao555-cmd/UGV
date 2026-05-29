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
    sensor_launch = workspace_root / "src" / "ugv_sensor_sync" / "launch" / "sensor_sync_launch.py"
    nav_script = workspace_root / "src" / "ugv_nav" / "ugv_nav_dual_mode.py"

    start_motor_controller = LaunchConfiguration("start_motor_controller")
    start_sensor_sync = LaunchConfiguration("start_sensor_sync")
    start_nav = LaunchConfiguration("start_nav")
    start_zed = LaunchConfiguration("start_zed")
    start_lidar = LaunchConfiguration("start_lidar")
    start_fusion = LaunchConfiguration("start_fusion")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    zed_imu_publish_rate_hz = LaunchConfiguration("zed_imu_publish_rate_hz")
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
    nav_control_period_s = LaunchConfiguration("nav_control_period_s")
    nav_controller_mode = LaunchConfiguration("nav_controller_mode")
    nav_frame_topic = LaunchConfiguration("nav_frame_topic")
    nav_imu_topic = LaunchConfiguration("nav_imu_topic")
    nav_imu_yaw_axis = LaunchConfiguration("nav_imu_yaw_axis")
    nav_imu_yaw_sign = LaunchConfiguration("nav_imu_yaw_sign")
    nav_imu_timeout_s = LaunchConfiguration("nav_imu_timeout_s")
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
    nav_max_test_duration_s = LaunchConfiguration("nav_max_test_duration_s")
    nav_gyro_bias_calibration_s = LaunchConfiguration("nav_gyro_bias_calibration_s")
    nav_gyro_bias_max_std_radps = LaunchConfiguration("nav_gyro_bias_max_std_radps")
    nav_gyro_bias_warn_abs_radps = LaunchConfiguration("nav_gyro_bias_warn_abs_radps")
    nav_gyro_bias_max_encoder_delta_ticks = LaunchConfiguration("nav_gyro_bias_max_encoder_delta_ticks")
    nav_pivot_max_omega_radps = LaunchConfiguration("nav_pivot_max_omega_radps")
    nav_pivot_min_omega_radps = LaunchConfiguration("nav_pivot_min_omega_radps")
    nav_pivot_breakaway_omega_radps = LaunchConfiguration("nav_pivot_breakaway_omega_radps")
    nav_pivot_breakaway_s = LaunchConfiguration("nav_pivot_breakaway_s")
    nav_pivot_accel_limit_radps2 = LaunchConfiguration("nav_pivot_accel_limit_radps2")
    nav_pivot_decel_limit_radps2 = LaunchConfiguration("nav_pivot_decel_limit_radps2")
    nav_pivot_approach_error_rad = LaunchConfiguration("nav_pivot_approach_error_rad")
    nav_pivot_min_omega_disable_error_rad = LaunchConfiguration("nav_pivot_min_omega_disable_error_rad")
    nav_pivot_kp_approach = LaunchConfiguration("nav_pivot_kp_approach")
    nav_pivot_kd_yaw_rate = LaunchConfiguration("nav_pivot_kd_yaw_rate")
    nav_pivot_settle_error_rad = LaunchConfiguration("nav_pivot_settle_error_rad")
    nav_pivot_settle_yaw_rate_radps = LaunchConfiguration("nav_pivot_settle_yaw_rate_radps")
    nav_pivot_settle_time_s = LaunchConfiguration("nav_pivot_settle_time_s")
    nav_pivot_brake_s = LaunchConfiguration("nav_pivot_brake_s")
    nav_pivot_timeout_s = LaunchConfiguration("nav_pivot_timeout_s")
    nav_pivot_max_correction_retries = LaunchConfiguration("nav_pivot_max_correction_retries")
    nav_pivot_clearance_m = LaunchConfiguration("nav_pivot_clearance_m")
    nav_slip_disagreement_rad = LaunchConfiguration("nav_slip_disagreement_rad")
    nav_mission_file = LaunchConfiguration("nav_mission_file")
    nav_competition_min_speed_mps = LaunchConfiguration("nav_competition_min_speed_mps")
    nav_mission_default_speed_mps = LaunchConfiguration("nav_mission_default_speed_mps")
    nav_mission_reliable_speed_mps = LaunchConfiguration("nav_mission_reliable_speed_mps")
    nav_mission_slow_speed_mps = LaunchConfiguration("nav_mission_slow_speed_mps")
    nav_mission_emergency_stop_clearance_m = LaunchConfiguration("nav_mission_emergency_stop_clearance_m")
    nav_mission_critical_sensor_timeout_s = LaunchConfiguration("nav_mission_critical_sensor_timeout_s")
    nav_mission_straight_max_omega_radps = LaunchConfiguration("nav_mission_straight_max_omega_radps")
    nav_mission_straight_omega_slew_radps2 = LaunchConfiguration("nav_mission_straight_omega_slew_radps2")
    nav_debug_allow_sub_min_crawl = LaunchConfiguration("nav_debug_allow_sub_min_crawl")
    nav_telemetry_enabled = LaunchConfiguration("nav_telemetry_enabled")
    nav_telemetry_dir = LaunchConfiguration("nav_telemetry_dir")

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

    sensor_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sensor_launch)),
        launch_arguments={
            "start_zed": start_zed,
            "start_lidar": start_lidar,
            "start_fusion": start_fusion,
            "start_uwb": "false",
            "lidar_port": lidar_port,
            "lidar_baud": lidar_baud,
            "zed_imu_publish_rate_hz": zed_imu_publish_rate_hz,
        }.items(),
        condition=IfCondition(start_sensor_sync),
    )

    nav_controller = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            str(nav_script),
            "--mode",
            "real",
            "--nav-status-period-s",
            nav_status_period_s,
            "--control-period-s",
            nav_control_period_s,
            "--controller-mode",
            nav_controller_mode,
            "--nav-frame-topic",
            nav_frame_topic,
            "--imu-topic",
            nav_imu_topic,
            "--imu-yaw-axis",
            nav_imu_yaw_axis,
            "--imu-yaw-sign",
            nav_imu_yaw_sign,
            "--imu-timeout-s",
            nav_imu_timeout_s,
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
            "--max-test-duration-s",
            nav_max_test_duration_s,
            "--gyro-bias-calibration-s",
            nav_gyro_bias_calibration_s,
            "--gyro-bias-max-std-radps",
            nav_gyro_bias_max_std_radps,
            "--gyro-bias-warn-abs-radps",
            nav_gyro_bias_warn_abs_radps,
            "--gyro-bias-max-encoder-delta-ticks",
            nav_gyro_bias_max_encoder_delta_ticks,
            "--pivot-max-omega-radps",
            nav_pivot_max_omega_radps,
            "--pivot-min-omega-radps",
            nav_pivot_min_omega_radps,
            "--pivot-breakaway-omega-radps",
            nav_pivot_breakaway_omega_radps,
            "--pivot-breakaway-s",
            nav_pivot_breakaway_s,
            "--pivot-accel-limit-radps2",
            nav_pivot_accel_limit_radps2,
            "--pivot-decel-limit-radps2",
            nav_pivot_decel_limit_radps2,
            "--pivot-approach-error-rad",
            nav_pivot_approach_error_rad,
            "--pivot-min-omega-disable-error-rad",
            nav_pivot_min_omega_disable_error_rad,
            "--pivot-kp-approach",
            nav_pivot_kp_approach,
            "--pivot-kd-yaw-rate",
            nav_pivot_kd_yaw_rate,
            "--pivot-settle-error-rad",
            nav_pivot_settle_error_rad,
            "--pivot-settle-yaw-rate-radps",
            nav_pivot_settle_yaw_rate_radps,
            "--pivot-settle-time-s",
            nav_pivot_settle_time_s,
            "--pivot-brake-s",
            nav_pivot_brake_s,
            "--pivot-timeout-s",
            nav_pivot_timeout_s,
            "--pivot-max-correction-retries",
            nav_pivot_max_correction_retries,
            "--pivot-clearance-m",
            nav_pivot_clearance_m,
            "--slip-disagreement-rad",
            nav_slip_disagreement_rad,
            "--mission-file",
            nav_mission_file,
            "--competition-min-speed-mps",
            nav_competition_min_speed_mps,
            "--mission-default-speed-mps",
            nav_mission_default_speed_mps,
            "--mission-reliable-speed-mps",
            nav_mission_reliable_speed_mps,
            "--mission-slow-speed-mps",
            nav_mission_slow_speed_mps,
            "--mission-emergency-stop-clearance-m",
            nav_mission_emergency_stop_clearance_m,
            "--mission-critical-sensor-timeout-s",
            nav_mission_critical_sensor_timeout_s,
            "--mission-straight-max-omega-radps",
            nav_mission_straight_max_omega_radps,
            "--mission-straight-omega-slew-radps2",
            nav_mission_straight_omega_slew_radps2,
            "--debug-allow-sub-min-crawl",
            nav_debug_allow_sub_min_crawl,
            "--telemetry-enabled",
            nav_telemetry_enabled,
            "--telemetry-dir",
            nav_telemetry_dir,
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
            DeclareLaunchArgument("start_sensor_sync", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("start_zed", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_fusion", default_value="true"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_baud", default_value="115200"),
            DeclareLaunchArgument("zed_imu_publish_rate_hz", default_value="100.0"),
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
            DeclareLaunchArgument("nav_control_period_s", default_value="0.02"),
            DeclareLaunchArgument("nav_controller_mode", default_value="idle"),
            DeclareLaunchArgument("nav_frame_topic", default_value="/sensors/nav_frame"),
            DeclareLaunchArgument("nav_imu_topic", default_value="/zed/imu"),
            DeclareLaunchArgument("nav_imu_yaw_axis", default_value="z"),
            DeclareLaunchArgument("nav_imu_yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("nav_imu_timeout_s", default_value="0.12"),
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
            DeclareLaunchArgument("nav_max_test_duration_s", default_value="3.0"),
            DeclareLaunchArgument("nav_gyro_bias_calibration_s", default_value="1.5"),
            DeclareLaunchArgument("nav_gyro_bias_max_std_radps", default_value="0.03"),
            DeclareLaunchArgument("nav_gyro_bias_warn_abs_radps", default_value="0.10"),
            DeclareLaunchArgument("nav_gyro_bias_max_encoder_delta_ticks", default_value="2"),
            DeclareLaunchArgument("nav_pivot_max_omega_radps", default_value="0.35"),
            DeclareLaunchArgument("nav_pivot_min_omega_radps", default_value="0.16"),
            DeclareLaunchArgument("nav_pivot_breakaway_omega_radps", default_value="0.18"),
            DeclareLaunchArgument("nav_pivot_breakaway_s", default_value="0.20"),
            DeclareLaunchArgument("nav_pivot_accel_limit_radps2", default_value="0.80"),
            DeclareLaunchArgument("nav_pivot_decel_limit_radps2", default_value="0.60"),
            DeclareLaunchArgument("nav_pivot_approach_error_rad", default_value="0.25"),
            DeclareLaunchArgument("nav_pivot_min_omega_disable_error_rad", default_value="0.10"),
            DeclareLaunchArgument("nav_pivot_kp_approach", default_value="0.75"),
            DeclareLaunchArgument("nav_pivot_kd_yaw_rate", default_value="0.10"),
            DeclareLaunchArgument("nav_pivot_settle_error_rad", default_value="0.035"),
            DeclareLaunchArgument("nav_pivot_settle_yaw_rate_radps", default_value="0.05"),
            DeclareLaunchArgument("nav_pivot_settle_time_s", default_value="0.35"),
            DeclareLaunchArgument("nav_pivot_brake_s", default_value="0.15"),
            DeclareLaunchArgument("nav_pivot_timeout_s", default_value="4.0"),
            DeclareLaunchArgument("nav_pivot_max_correction_retries", default_value="1"),
            DeclareLaunchArgument("nav_pivot_clearance_m", default_value="0.35"),
            DeclareLaunchArgument("nav_slip_disagreement_rad", default_value="0.35"),
            DeclareLaunchArgument("nav_mission_file", default_value=""),
            DeclareLaunchArgument("nav_competition_min_speed_mps", default_value="0.089408"),
            DeclareLaunchArgument("nav_mission_default_speed_mps", default_value="0.15"),
            DeclareLaunchArgument("nav_mission_reliable_speed_mps", default_value="0.15"),
            DeclareLaunchArgument("nav_mission_slow_speed_mps", default_value="0.09"),
            DeclareLaunchArgument("nav_mission_emergency_stop_clearance_m", default_value="0.18"),
            DeclareLaunchArgument("nav_mission_critical_sensor_timeout_s", default_value="1.0"),
            DeclareLaunchArgument("nav_mission_straight_max_omega_radps", default_value="0.20"),
            DeclareLaunchArgument("nav_mission_straight_omega_slew_radps2", default_value="0.80"),
            DeclareLaunchArgument("nav_debug_allow_sub_min_crawl", default_value="false"),
            DeclareLaunchArgument("nav_telemetry_enabled", default_value="true"),
            DeclareLaunchArgument("nav_telemetry_dir", default_value="~/.ros/ugv_mission_logs"),
            motor_controller_launch,
            sensor_sync_launch,
            nav_controller,
        ]
    )
