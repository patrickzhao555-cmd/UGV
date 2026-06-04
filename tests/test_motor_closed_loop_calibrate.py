import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_motor_closed_loop_calibrate import (  # noqa: E402
    _command_from_sides,
    _parse_pair_list,
    _status_from_line,
    _tps_to_mps,
)


def test_command_from_sides_preserves_tank_drive_sign_convention():
    command = _command_from_sides(0.22, 0.10, 0.416)
    assert command.startswith("CMD V 0.160000 -0.")

    command = _command_from_sides(0.10, 0.22, 0.416)
    assert command.startswith("CMD V 0.160000 0.")


def test_status_parser_extracts_side_targets_measurements_pwm_and_fault():
    sample = _status_from_line(
        "S,123,1,2,3,4,10,20,30,40,100,200,90,180,1510,1520,10,20,"
        "1.0,2.0,3.0,4.0,5.0,6.0,none"
    )
    assert sample is not None
    assert sample.left_target_tps == 100
    assert sample.right_measured_tps == 180
    assert sample.left_pwm == 1510
    assert sample.fault == "none"


def test_tps_to_mps_uses_wheel_geometry():
    assert _tps_to_mps(500, wheel_radius_m=0.10, ticks_per_rev=1000) == pytest.approx(0.314159265)


def test_parse_pair_list_for_differential_cases():
    assert _parse_pair_list("0.22:0.10,0.10:0.22") == [(0.22, 0.10), (0.10, 0.22)]
