import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.uav_target_input import append_esp_checksum, parse_uav_target_line  # noqa: E402


def test_parse_terminal_target_accepts_space_and_comma_meter_inputs():
    spaced = parse_uav_target_line("5.0 7.25")
    assert spaced.accepted
    assert spaced.x_m == pytest.approx(5.0)
    assert spaced.y_m == pytest.approx(7.25)
    assert spaced.seq is None

    comma = parse_uav_target_line("5.0,7.25")
    assert comma.accepted
    assert comma.x_m == pytest.approx(5.0)
    assert comma.y_m == pytest.approx(7.25)


def test_parse_esp_target_accepts_labeled_sequence_and_json():
    labeled = parse_uav_target_line("TARGET,42,3.5,4.25")
    assert labeled.accepted
    assert labeled.seq == 42
    assert labeled.x_m == pytest.approx(3.5)
    assert labeled.y_m == pytest.approx(4.25)

    labeled_units = parse_uav_target_line("TARGET,3.5,4.25,meters")
    assert labeled_units.accepted
    assert labeled_units.seq is None
    assert labeled_units.x_m == pytest.approx(3.5)
    assert labeled_units.y_m == pytest.approx(4.25)

    labeled_seq_units = parse_uav_target_line("TARGET,43,3.5,4.25,meters")
    assert labeled_seq_units.accepted
    assert labeled_seq_units.seq == 43
    assert labeled_seq_units.x_m == pytest.approx(3.5)
    assert labeled_seq_units.y_m == pytest.approx(4.25)

    json_line = parse_uav_target_line('{"seq": 43, "x_m": 8.0, "y_m": 9.0, "units": "meters"}')
    assert json_line.accepted
    assert json_line.seq == 43
    assert json_line.x_m == pytest.approx(8.0)
    assert json_line.y_m == pytest.approx(9.0)


def test_parse_target_can_convert_yards_but_defaults_to_meters():
    parsed = parse_uav_target_line("1.0,2.0,yards")
    assert parsed.accepted
    assert parsed.x_m == pytest.approx(0.9144)
    assert parsed.y_m == pytest.approx(1.8288)

    default_meters = parse_uav_target_line("1.0,2.0")
    assert default_meters.accepted
    assert default_meters.x_m == pytest.approx(1.0)
    assert default_meters.y_m == pytest.approx(2.0)


def test_parse_target_rejects_bad_values_and_bad_units():
    assert parse_uav_target_line("").reason == "empty"
    assert parse_uav_target_line("# comment").reason == "comment"
    assert parse_uav_target_line("TARGET,seq,5.0").reason == "coordinate_invalid"
    assert parse_uav_target_line("TARGET,5.5,5.0,6.0").reason == "sequence_invalid"
    assert parse_uav_target_line("nan,5.0").reason == "coordinate_invalid"
    assert parse_uav_target_line("1.0,2.0,feet").reason == "units_invalid"


def test_parse_esp_checksum_optional_or_required():
    payload = "TARGET,7,5.0,6.0"
    checksummed = append_esp_checksum(payload)

    accepted = parse_uav_target_line(checksummed, require_checksum=True)
    assert accepted.accepted
    assert accepted.seq == 7
    assert accepted.x_m == pytest.approx(5.0)
    assert accepted.y_m == pytest.approx(6.0)

    missing = parse_uav_target_line(payload, require_checksum=True)
    assert not missing.accepted
    assert missing.reason == "checksum_missing"

    bad = parse_uav_target_line(f"{payload}*00", require_checksum=True)
    assert not bad.accepted
    assert bad.reason == "checksum_mismatch"


def test_nav2_launch_can_start_terminal_or_esp_target_receiver_without_goal_bridge():
    launch_text = (ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "nav2_field_navigation.launch.py").read_text()
    assert "ugv_uav_target_receiver.py" in launch_text
    assert 'DeclareLaunchArgument("start_uav_target_receiver", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("uav_target_input_mode", default_value="serial")' in launch_text
    assert 'DeclareLaunchArgument("uav_esp_serial_port", default_value="/dev/ttyUSB1")' in launch_text
    assert 'DeclareLaunchArgument("uav_esp_require_checksum", default_value="false")' in launch_text
    assert "target_receiver_node" in launch_text
    assert 'DeclareLaunchArgument("start_goal_bridge", default_value="false")' in launch_text
