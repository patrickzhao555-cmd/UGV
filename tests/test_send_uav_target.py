import importlib.util
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "send_uav_target.py"
spec = importlib.util.spec_from_file_location("send_uav_target", TOOL_PATH)
send_uav_target = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = send_uav_target
spec.loader.exec_module(send_uav_target)


def test_send_uav_target_defaults_to_map_frame_meters_and_target_topic():
    args = send_uav_target.parse_args(["--x", "5.0", "--y", "7.0"])
    target = send_uav_target.spec_from_args(args)

    assert target.x_m == pytest.approx(5.0)
    assert target.y_m == pytest.approx(7.0)
    assert target.frame_id == "map"
    assert target.topic == "/ugv/uav_target"
    assert target.count == 1


def test_send_uav_target_can_convert_yards_for_manual_checks():
    args = send_uav_target.parse_args(["--x", "1.0", "--y", "2.0", "--units", "yards", "--count", "3"])
    target = send_uav_target.spec_from_args(args)

    assert target.x_m == pytest.approx(0.9144)
    assert target.y_m == pytest.approx(1.8288)
    assert target.count == 3


def test_send_uav_target_rejects_non_finite_coordinates_and_bad_publish_options():
    with pytest.raises(SystemExit):
        send_uav_target.parse_args(["--x", "nan", "--y", "1.0"])

    args = send_uav_target.parse_args(["--x", "1.0", "--y", "2.0", "--units", "feet"])
    with pytest.raises(ValueError, match="unsupported units"):
        send_uav_target.spec_from_args(args)

    args = send_uav_target.parse_args(["--x", "1.0", "--y", "2.0", "--topic", "relative_topic"])
    with pytest.raises(ValueError, match="absolute ROS topic"):
        send_uav_target.spec_from_args(args)
