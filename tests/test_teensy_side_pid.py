import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_motor_controller"))

from ugv_motor_controller.teensy_side_pid import (  # noqa: E402
    build_teensy_raw2_command,
    build_teensy_stop_command,
    build_teensy_velocity_command,
    mps_to_ticks_per_sec,
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
    assert build_teensy_stop_command() == "CMD STOP\n"


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
    setup_file = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "setup.py").read_text()

    assert "motor_control_location" in bridge_file
    assert "python_velocity_pid_enabled" in bridge_file
    assert "build_teensy_velocity_command" in bridge_file
    assert "build_teensy_stop_command" in bridge_file
    assert "parse_teensy_side_pid_status_line" in bridge_file
    assert "teensy_4_1_side_pid_controller" in setup_file
    assert "MOTOR_CONTROL_LOCATION" in launch_file
    assert (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "ugv_motor_controller" / "velocity_control.py").exists()
