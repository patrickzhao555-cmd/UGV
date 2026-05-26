import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_motor_controller"))

from ugv_motor_controller.teensy_side_pid import (  # noqa: E402
    TeensyParamSyncTracker,
    build_teensy_param_command,
    build_teensy_param_init_commands,
    build_teensy_raw2_command,
    build_teensy_stop_command,
    build_teensy_velocity_command,
    mps_to_ticks_per_sec,
    parse_teensy_param_ack_line,
    parse_teensy_side_pid_status_line,
    python_velocity_pid_enabled,
    teensy_timeout_stop_command_due,
    velocity_to_side_pid_targets,
)


def test_v_omega_to_side_target_signs_for_forward_and_pivots():
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


def test_mps_to_ticks_per_sec_conversion():
    tps = mps_to_ticks_per_sec(0.314159265, wheel_radius_m=0.10, ticks_per_rev=1000)
    assert round(tps, 1) == 500.0


def test_teensy_pid_mode_disables_python_velocity_pid():
    assert not python_velocity_pid_enabled("teensy_pid", True)
    assert not python_velocity_pid_enabled("teensy_side_pid", True)
    assert python_velocity_pid_enabled("ros", True)
    assert not python_velocity_pid_enabled("ros", False)


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

    bridge_status = status.as_bridge_status_dict()
    assert bridge_status["fl_tps"] == 10.5
    assert bridge_status["left_front_vs_rear_mismatch"] == 1.0
    assert bridge_status["right_front_vs_rear_mismatch"] == 0.5
    assert bridge_status["pid_left"] == {"p": 1.1, "i": 0.2, "d": 0.3}
    assert bridge_status["fault_reason"] is None


def test_teensy_serial_command_builders():
    assert build_teensy_velocity_command(0.18, 0.35) == "CMD V 0.180000 0.350000\n"
    assert build_teensy_raw2_command(1500, 1510) == "CMD RAW2 1500 1510\n"
    assert build_teensy_param_command("right_motor_sign", -1) == "CMD PARAM right_motor_sign -1\n"
    assert build_teensy_param_command("wheel_radius_m", 0.0889) == "CMD PARAM wheel_radius_m 0.0889\n"
    assert build_teensy_stop_command() == "CMD STOP\n"


def test_teensy_startup_param_sequence_uses_bridge_ticks_and_physical_defaults():
    commands = build_teensy_param_init_commands(
        {
            "track_width_m": 0.6096,
            "wheel_radius_m": 0.0889,
            "ticks_per_rev": 3200,
            "kp": 0.45,
            "ki": 0.0,
            "kd": 0.0,
            "right_motor_sign": -1,
        }
    )

    assert "CMD PARAM wheel_radius_m 0.0889\n" in commands
    assert "CMD PARAM ticks_per_rev 3200\n" in commands
    assert "CMD PARAM kp 0.45\n" in commands
    assert "CMD PARAM right_motor_sign -1\n" in commands
    assert "CMD PARAM ticks_per_rev 2151\n" not in commands


def test_teensy_param_ack_parser_accepts_ok_and_unknown():
    ok = parse_teensy_param_ack_line("PARAM,ticks_per_rev,ok")
    unknown = parse_teensy_param_ack_line("PARAM,bad_param,unknown")

    assert ok.name == "ticks_per_rev"
    assert ok.ok
    assert unknown.name == "bad_param"
    assert not unknown.ok


def test_teensy_param_sync_tracker_waits_for_all_acks():
    tracker = TeensyParamSyncTracker()
    tracker.start(["track_width_m", "wheel_radius_m", "ticks_per_rev"], now_s=10.0)

    assert not tracker.synced
    assert tracker.pending_names == ("track_width_m", "wheel_radius_m", "ticks_per_rev")
    assert tracker.reason == "waiting_for_ack"

    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,track_width_m,ok"), now_s=10.1)
    assert not tracker.synced
    assert tracker.pending_names == ("wheel_radius_m", "ticks_per_rev")

    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,wheel_radius_m,ok"), now_s=10.2)
    tracker.handle_ack(parse_teensy_param_ack_line("PARAM,ticks_per_rev,ok"), now_s=10.3)

    assert tracker.synced
    assert tracker.pending_names == ()
    assert tracker.failed is False
    assert tracker.reason == "ok"
    assert tracker.acked_count == 3
    assert tracker.completed_s == 10.3


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
    assert not timeout_tracker.synced
    assert timeout_tracker.failed
    assert timeout_tracker.reason == "ack_timeout:kp,ki"


def test_teensy_timeout_maps_to_stop_command():
    assert teensy_timeout_stop_command_due(10.0, now_s=10.20, timeout_s=0.75) is None
    assert teensy_timeout_stop_command_due(10.0, now_s=10.80, timeout_s=0.75) == "CMD STOP\n"


def test_bridge_and_launch_wire_teensy_pid_mode_without_replacing_legacy_pid():
    bridge_file = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "ugv_motor_controller"
        / "motor_controller_bridge.py"
    ).read_text()
    launch_file = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "launch" / "motor_controller.launch.py").read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()
    firmware = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "firmware"
        / "teensy_4_1_side_pid_controller"
        / "teensy_4_1_side_pid_controller.ino"
    ).read_text()
    setup_file = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "setup.py").read_text()

    assert "motor_control_location" in bridge_file
    assert "python_velocity_pid_enabled" in bridge_file
    assert "build_teensy_velocity_command" in bridge_file
    assert "build_teensy_stop_command" in bridge_file
    assert "_sync_teensy_pid_params(reason='startup parameter sync')" in bridge_file
    assert bridge_file.index("_send_teensy_stop_command(reason='startup stop')") < bridge_file.index(
        "_sync_teensy_pid_params(reason='startup parameter sync')"
    )
    assert "'ticks_per_rev', self.ticks_per_rev" in bridge_file
    assert "left_pwm = self._raw_to_pwm(left_raw, invert=False)" in bridge_file
    assert "right_pwm = self._raw_to_pwm(right_raw, invert=False)" in bridge_file
    assert "teensy_pid_params_synced" in bridge_file
    assert "parse_teensy_param_ack_line" in bridge_file
    assert "teensy_pid_param_sync_pending" in bridge_file
    assert "teensy_pid_param_sync_failed" in bridge_file
    assert "teensy_pid_param_sync_reason" in bridge_file
    assert "teensy_pid_params_not_synced" in bridge_file
    assert "handle_ack(ack" in bridge_file
    assert "self.teensy_pid_params_synced = True" not in bridge_file
    assert "teensy_pid_wheel_radius_m" in bridge_file
    assert "teensy_right_motor_sign" in bridge_file
    assert "parse_teensy_side_pid_status_line" in bridge_file
    assert "teensy_4_1_side_pid_controller" in setup_file
    assert "teensy_pid_param_ack_timeout_s" in launch_file
    assert "EnvironmentVariable('MOTOR_WHEEL_RADIUS_M', default_value='0.0889')" in launch_file
    assert "EnvironmentVariable('MOTOR_TICKS_PER_REV', default_value='3200')" in launch_file
    assert "teensy_right_motor_sign" in launch_file
    assert "MOTOR_CONTROL_LOCATION" in launch_file
    assert 'MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-0.0889}"' in bringup
    assert 'MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-3200}"' in bringup
    assert "MOTOR_TEENSY_RIGHT_MOTOR_SIGN" in bringup
    assert "DEFAULT_WHEEL_RADIUS_M = 0.0889f" in firmware
    assert "DEFAULT_TICKS_PER_REV = 3200.0f" in firmware
    assert 'strcmp(name, "right_motor_sign")' in firmware
    assert "value < 0 ? -1 : 1" in firmware
    assert "PARAM,<name>,ok" in firmware
    assert "sendParamAck(name, ok)" in firmware
    assert 'stream.print("PARAMS,track_width_m=")' in firmware
    assert "critical = true" in firmware
    assert "resetPidState();" in firmware
    assert "neutralizeForCriticalParam();" in firmware
    assert (ROOT / "scripts" / "run_teensy_side_pid_bench.sh").read_text().count("WHEELS OFF GROUND") >= 1
    assert (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "ugv_motor_controller" / "velocity_control.py").exists()
