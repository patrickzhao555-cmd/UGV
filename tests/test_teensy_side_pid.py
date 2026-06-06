import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_motor_controller"))

from ugv_motor_controller.teensy_side_pid import (  # noqa: E402
    TeensyParamSyncTracker,
    apply_side_speed_scales,
    build_teensy_param_command,
    build_teensy_param_init_commands,
    build_teensy_stop_command,
    build_teensy_velocity_command,
    extract_velocity_command,
    is_stop_command,
    mps_to_ticks_per_sec,
    parse_teensy_param_ack_line,
    parse_teensy_side_pid_status_line,
    side_encoder_mismatch,
    side_mismatch_flags,
    side_speeds_to_velocity,
    side_ticks_from_wheels,
    velocity_to_side_speeds,
    velocity_to_side_pid_targets,
)


def test_velocity_to_side_target_signs_for_forward_and_pivots():
    left_mps, right_mps, left_tps, right_tps = velocity_to_side_pid_targets(
        0.20,
        0.0,
        track_width_m=0.60,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
    )
    assert left_mps > 0.0
    assert right_mps > 0.0
    assert left_tps > 0.0
    assert right_tps > 0.0

    _, _, pivot_left_left_tps, pivot_left_right_tps = velocity_to_side_pid_targets(
        0.0,
        0.35,
        track_width_m=0.60,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
    )
    assert pivot_left_left_tps < 0.0
    assert pivot_left_right_tps > 0.0

    _, _, pivot_right_left_tps, pivot_right_right_tps = velocity_to_side_pid_targets(
        0.0,
        -0.35,
        track_width_m=0.60,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
    )
    assert pivot_right_left_tps > 0.0
    assert pivot_right_right_tps < 0.0


def test_velocity_side_speed_conversion_is_reversible_for_arcs():
    left_mps, right_mps = velocity_to_side_speeds(0.30, 1.20, 0.416)
    assert left_mps == pytest.approx(0.0504)
    assert right_mps == pytest.approx(0.5496)
    v_mps, omega_radps = side_speeds_to_velocity(left_mps, right_mps, 0.416)
    assert v_mps == pytest.approx(0.30)
    assert omega_radps == pytest.approx(1.20)


def test_side_speed_scales_can_compensate_directional_side_weakness():
    left_mps, right_mps = velocity_to_side_speeds(0.30, 1.20, 0.416)
    scaled_left, scaled_right = apply_side_speed_scales(
        left_mps,
        right_mps,
        right_forward_scale=1.2,
    )
    adjusted_v, adjusted_omega = side_speeds_to_velocity(scaled_left, scaled_right, 0.416)
    assert scaled_left == pytest.approx(left_mps)
    assert scaled_right == pytest.approx(right_mps * 1.2)
    assert adjusted_v > 0.30
    assert adjusted_omega > 1.20


def test_mps_to_ticks_per_sec_conversion():
    tps = mps_to_ticks_per_sec(0.314159265, wheel_radius_m=0.10, ticks_per_rev=1000)
    assert round(tps, 1) == 500.0


def test_four_encoder_side_average_and_mismatch_helpers():
    assert side_ticks_from_wheels(100, 104) == 102
    assert side_encoder_mismatch(10.5, 11.5) == 1.0
    assert side_mismatch_flags(100.0, 185.0, warn_tps=80.0, fault_tps=180.0) == (True, False)
    assert side_mismatch_flags(100.0, 290.0, warn_tps=80.0, fault_tps=180.0) == (True, True)


def test_velocity_command_contract_is_velocity_only():
    assert extract_velocity_command({"command_type": "velocity", "v_mps": 0.2, "omega_radps": 0.1}) == (0.2, 0.1)
    assert extract_velocity_command({"command_type": "raw", "raw_left": 0.2, "raw_right": 0.2}) is None
    assert is_stop_command({"mode": "STOP"})
    assert is_stop_command({"command_type": "stop"})


def test_parse_teensy_side_pid_status_line():
    status = parse_teensy_side_pid_status_line(
        "S,1234,100,90,104,92,10.5,9.5,11.5,10.0,20.0,21.0,"
        "11.0,9.75,1510,1512,9.0,11.25,1.1,0.2,0.3,1.4,0.5,0.6,none"
    )
    assert status.controller_millis == 1234
    assert status.wheel_ticks == (100, 90, 104, 92)
    assert status.side_ticks == (102, 91)
    assert status.left_target_tps == 20.0
    assert status.right_measured_tps == 9.75
    assert status.left_pwm == 1510
    assert status.right_pwm == 1512
    assert status.fault == "none"

    bridge_status = status.as_status_dict()
    assert bridge_status["fl_tps"] == 10.5
    assert bridge_status["left_front_vs_rear_mismatch"] == 1.0
    assert bridge_status["right_front_vs_rear_mismatch"] == 0.5
    assert bridge_status["pid_left"] == {"p": 1.1, "i": 0.2, "d": 0.3}
    assert bridge_status["fault_reason"] is None


def test_teensy_serial_command_builders():
    assert build_teensy_velocity_command(0.18, 0.35) == "CMD V 0.180000 0.350000\n"
    assert build_teensy_param_command("right_motor_sign", -1) == "CMD PARAM right_motor_sign -1\n"
    assert build_teensy_param_command("wheel_radius_m", 0.0825) == "CMD PARAM wheel_radius_m 0.0825\n"
    assert build_teensy_stop_command() == "CMD STOP\n"


def test_teensy_startup_param_sequence_uses_physical_defaults():
    commands = build_teensy_param_init_commands(
        {
            "track_width_m": 0.416,
            "wheel_radius_m": 0.0825,
            "ticks_per_rev": 3200,
            "kp": 0.45,
            "ki": 0.02,
            "kd": 0.0,
            "ff_us_per_tps": 0.04,
            "static_ff_us": 170.0,
            "static_ff_full_target_tps": 2500.0,
            "static_ff_fade_start_ratio": 0.20,
            "static_ff_fade_end_ratio": 0.85,
            "left_ff_us_per_tps": 0.04,
            "right_ff_us_per_tps": 0.04,
            "right_reverse_ff_us_per_tps": 0.08,
            "left_static_ff_us": 170.0,
            "right_static_ff_us": 170.0,
            "right_reverse_static_ff_us": 260.0,
            "right_reverse_pwm_floor_us": 430.0,
            "left_pid_output_limit_us": 500.0,
            "right_pid_output_limit_us": 500.0,
            "control_hz": 50.0,
            "side_mismatch_fault_tps": 180.0,
            "sign_mismatch_target_tps": 100.0,
            "sign_mismatch_timeout_ms": 250,
            "encoder_jump_tps": 12000.0,
            "right_motor_sign": -1,
            "fl_encoder_sign": -1,
            "rl_encoder_sign": -1,
        }
    )
    assert "CMD PARAM track_width_m 0.416\n" in commands
    assert "CMD PARAM wheel_radius_m 0.0825\n" in commands
    assert "CMD PARAM ticks_per_rev 3200\n" in commands
    assert "CMD PARAM kp 0.45\n" in commands
    assert "CMD PARAM ki 0.02\n" in commands
    assert "CMD PARAM ff_us_per_tps 0.04\n" in commands
    assert "CMD PARAM static_ff_us 170\n" in commands
    assert "CMD PARAM static_ff_full_target_tps 2500\n" in commands
    assert "CMD PARAM static_ff_fade_start_ratio 0.2\n" in commands
    assert "CMD PARAM static_ff_fade_end_ratio 0.85\n" in commands
    assert "CMD PARAM left_ff_us_per_tps 0.04\n" in commands
    assert "CMD PARAM right_ff_us_per_tps 0.04\n" in commands
    assert "CMD PARAM right_reverse_ff_us_per_tps 0.08\n" in commands
    assert "CMD PARAM left_static_ff_us 170\n" in commands
    assert "CMD PARAM right_static_ff_us 170\n" in commands
    assert "CMD PARAM right_reverse_static_ff_us 260\n" in commands
    assert "CMD PARAM right_reverse_pwm_floor_us 430\n" in commands
    assert "CMD PARAM left_pid_output_limit_us 500\n" in commands
    assert "CMD PARAM right_pid_output_limit_us 500\n" in commands
    assert "CMD PARAM control_hz 50\n" in commands
    assert "CMD PARAM side_mismatch_fault_tps 180\n" in commands
    assert "CMD PARAM sign_mismatch_target_tps 100\n" in commands
    assert "CMD PARAM sign_mismatch_timeout_ms 250\n" in commands
    assert "CMD PARAM encoder_jump_tps 12000\n" in commands
    assert "CMD PARAM right_motor_sign -1\n" in commands
    assert "CMD PARAM fl_encoder_sign -1\n" in commands
    assert "CMD PARAM rl_encoder_sign -1\n" in commands
    assert "CMD PARAM ticks_per_rev 2151\n" not in commands


def test_teensy_firmware_uses_float_abs_for_speed_math():
    firmware = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "firmware"
        / "teensy_4_1_side_pid_controller"
        / "teensy_4_1_side_pid_controller.ino"
    )
    source = firmware.read_text(encoding="utf-8")
    assert "float target_abs = fabsf(target_tps);" in source
    assert "bool left_active = fabsf(left_target_tps)" in source
    assert "abs(left_target_tps)" not in source
    assert "abs(right_target_tps)" not in source


def test_teensy_firmware_shapes_feedforward_from_measured_speed():
    firmware = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "firmware"
        / "teensy_4_1_side_pid_controller"
        / "teensy_4_1_side_pid_controller.ino"
    )
    source = firmware.read_text(encoding="utf-8")
    assert "staticFeedforwardForTarget(float target_tps, float measured_tps" in source
    assert "linearFeedforwardForTarget(float target_tps, float measured_tps" in source
    assert "motion_fade" in source
    assert "static_ff_fade_start_ratio" in source
    assert "static_ff_fade_end_ratio" in source
    assert "measured_along_target > target_abs" in source
    assert "linear_ff *= clampFloat(target_abs / measured_along_target" in source


def test_motor_controller_readme_rejects_stale_one_sided_right_boost_example():
    readme = ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "README.md"
    text = readme.read_text(encoding="utf-8")
    assert "ugv_motor_closed_loop_calibrate.py" in text
    assert "making the right side faster will normally increase the left drift" in text
    assert "motor_teensy_right_pid_static_ff_us:=210.0" not in text


def test_teensy_param_ack_parser_and_sync_tracker():
    tracker = TeensyParamSyncTracker()
    tracker.start(["track_width_m", "wheel_radius_m", "ticks_per_rev"], now_s=10.0)
    assert not tracker.synced
    assert tracker.pending_names == ("track_width_m", "wheel_radius_m", "ticks_per_rev")

    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,track_width_m,ok"), now_s=10.1)
    assert tracker.pending_names == ("wheel_radius_m", "ticks_per_rev")

    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,wheel_radius_m,ok"), now_s=10.2)
    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,ticks_per_rev,ok"), now_s=10.3)
    assert tracker.synced
    assert tracker.failed is False
    assert tracker.reason == "ok"


def test_teensy_param_sync_unknown_or_timeout_leaves_sync_false():
    unknown_tracker = TeensyParamSyncTracker()
    unknown_tracker.start(["kp"], now_s=20.0)
    unknown_tracker.handle_ack(parse_teensy_param_ack_line("PARAM,kp,unknown"), now_s=20.1)
    assert not unknown_tracker.synced
    assert unknown_tracker.failed
    assert unknown_tracker.reason == "param_unknown:kp"

    timeout_tracker = TeensyParamSyncTracker()
    timeout_tracker.start(["kp", "ki"], now_s=30.0)
    assert timeout_tracker.check_timeout(now_s=30.5, timeout_s=1.0) is False
    assert timeout_tracker.check_timeout(now_s=31.1, timeout_s=1.0) is True
    assert timeout_tracker.reason == "ack_timeout:kp,ki"


def test_clean_runtime_files_do_not_reintroduce_legacy_motor_pid():
    bridge_file = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "ugv_motor_controller"
        / "motor_controller_bridge.py"
    ).read_text()
    motor_launch_file = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "launch"
        / "motor_controller.launch.py"
    ).read_text()
    bringup_launch_file = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_sensor_sync"
        / "launch"
        / "competition_bringup.launch.py"
    ).read_text()
    setup_file = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "setup.py").read_text()
    nav_file = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    field_odom_file = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_field_odom_node.py").read_text()
    firmware = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "firmware"
        / "teensy_4_1_side_pid_controller"
        / "teensy_4_1_side_pid_controller.ino"
    ).read_text()

    assert "WheelVelocityPid" not in bridge_file
    assert "velocity_control" not in bridge_file
    assert "motor_direct_test" not in setup_file
    assert "teensy_4_1_motor_bridge" not in setup_file
    assert "two_controller_four_encoder_side_pid" in bridge_file
    assert "four_pololu_37d_50_1_motors_two_gobilda_1x15a_pwm_controllers" in bridge_file
    assert "CMD V" in bridge_file
    assert "CMD RAW2" not in bridge_file
    assert "PARAM,<name>,ok" in firmware
    assert "FIRMWARE_ID[]" in firmware
    assert "firmware_id=" in firmware
    assert "parseFiniteFloatToken" in firmware
    assert "commandHasExtraTokens" in firmware
    assert "assignFloatInRange" in firmware
    assert "sendControlAck(\"parse\", \"overflow\")" in firmware
    assert "bad_velocity" in firmware
    assert "bad_raw2" in firmware
    assert "validPwmConfig" in firmware
    assert "nonfinite_state" in firmware
    assert "nonfinite_pid" in firmware
    assert "atof(" not in firmware
    assert "atoi(" not in firmware
    assert "DEFAULT_TRACK_WIDTH_M = 0.416f" in firmware
    assert "DEFAULT_WHEEL_RADIUS_M = 0.0825f" in firmware
    assert "DEFAULT_TICKS_PER_REV = 3200.0f" in firmware
    assert "DEFAULT_KP = 0.03f" in firmware
    assert "DEFAULT_KI = 0.0f" in firmware
    assert "DEFAULT_FF_US_PER_TPS = 0.02f" in firmware
    assert "DEFAULT_STATIC_FF_US = 90.0f" in firmware
    assert "DEFAULT_STATIC_FF_FULL_TARGET_TPS = 1500.0f" in firmware
    assert "DEFAULT_STATIC_FF_FADE_START_RATIO = 0.20f" in firmware
    assert "DEFAULT_STATIC_FF_FADE_END_RATIO = 0.85f" in firmware
    assert "DEFAULT_PID_OUTPUT_LIMIT_US = 180.0f" in firmware
    assert "static_ff_full_target_tps" in firmware
    assert "static_ff_fade_start_ratio" in firmware
    assert "static_ff_fade_end_ratio" in firmware
    assert "setPidOutputLimitsForBase" in firmware
    assert "resetLeftPidState" in firmware
    assert "left_feedforward_us_per_tps" in firmware
    assert "right_feedforward_us_per_tps" in firmware
    assert "left_static_ff_us" in firmware
    assert "right_static_ff_us" in firmware
    assert "right_reverse_static_ff_us" in firmware
    assert "right_reverse_ff_us_per_tps" in firmware
    assert "right_reverse_pwm_floor_us" in firmware
    assert "right_reverse_unavailable" in firmware
    assert "left_pid_output_limit_us" in firmware
    assert "right_pid_output_limit_us" in firmware
    assert 'self.declare_parameter("enable_teensy_side_specific_pid_params", True)' in bridge_file
    assert 'self.declare_parameter("teensy_pid_static_ff_fade_start_ratio", 0.20)' in bridge_file
    assert 'self.declare_parameter("teensy_pid_static_ff_fade_end_ratio", 0.85)' in bridge_file
    assert 'self.declare_parameter("teensy_right_reverse_pid_static_ff_us", -1.0)' in bridge_file
    assert 'self.declare_parameter("teensy_right_reverse_pwm_floor_us", 0.0)' in bridge_file
    assert 'self.declare_parameter("teensy_pid_param_write_interval_s", 0.03)' in bridge_file
    assert 'EnvironmentVariable("MOTOR_ENABLE_TEENSY_SIDE_SPECIFIC_PID_PARAMS", default_value="true")' in motor_launch_file
    assert "MOTOR_TEENSY_PID_STATIC_FF_FADE_START_RATIO" in motor_launch_file
    assert "MOTOR_TEENSY_PID_STATIC_FF_FADE_END_RATIO" in motor_launch_file
    assert "MOTOR_TEENSY_RIGHT_REVERSE_PID_STATIC_FF_US" in motor_launch_file
    assert "MOTOR_TEENSY_RIGHT_REVERSE_PWM_FLOOR_US" in motor_launch_file
    assert "MOTOR_TEENSY_PARAM_WRITE_INTERVAL_S" in motor_launch_file
    assert 'DeclareLaunchArgument("motor_enable_teensy_side_specific_pid_params", default_value="false")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_static_ff_us", default_value="410.0")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_feedforward_us_per_tps", default_value="0.08")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_output_limit_us", default_value="500.0")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_param_write_interval_s", default_value="0.03")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_static_ff_fade_start_ratio", default_value="0.20")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_pid_static_ff_fade_end_ratio", default_value="0.85")' in bringup_launch_file
    assert 'DeclareLaunchArgument("motor_teensy_right_reverse_pid_static_ff_us", default_value="-1.0")' in bringup_launch_file
    assert 'DeclareLaunchArgument("nav_imu_yaw_axis", default_value="y")' in bringup_launch_file
    assert 'parser.add_argument("--imu-yaw-axis", choices=["x", "y", "z"], default="y")' in nav_file
    assert 'self.declare_parameter("imu_yaw_axis", "y")' in field_odom_file
    assert "DEFAULT_CONTROL_INTERVAL_MS = 20" in firmware
    assert "int fl_encoder_sign = -1" in firmware
    assert "int rl_encoder_sign = -1" in firmware
    assert "int right_motor_sign = -1" in firmware
    assert "side_mismatch_fault_tps" in firmware
    assert "static_ff_us" in firmware
    assert "sign_mismatch_target_tps" in firmware
    assert "sign_mismatch_timeout_ms" in firmware
    assert "encoder_jump_tps" in firmware
    assert "STATUS_STREAM" in firmware
    assert "status_stream_enabled" in firmware
    assert "left_side_stall" in firmware
    assert "right_side_stall" in firmware
    assert "left_mismatch" in firmware
    assert "right_mismatch" in firmware
    assert "controller_mode == MODE_FAULT || strcmp(fault_reason, \"none\") != 0" in firmware
    assert "#include <QuickPID.h>" in firmware
    assert "pid_backend=QuickPID" in firmware
    assert "FallbackPid" not in firmware
    assert "__has_include" not in firmware
    assert not (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "ugv_motor_controller"
        / "velocity_control.py"
    ).exists()
