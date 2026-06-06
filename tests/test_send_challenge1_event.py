import importlib.util
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools" / "send_challenge1_event.py"
spec = importlib.util.spec_from_file_location("send_challenge1_event", TOOL_PATH)
send_challenge1_event = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = send_challenge1_event
spec.loader.exec_module(send_challenge1_event)


def test_send_challenge1_event_defaults_to_landed_topic():
    args = send_challenge1_event.parse_args([])
    event = send_challenge1_event.spec_from_args(args)

    assert event.event == "landed"
    assert event.topic == "/ugv/uav_landed"
    assert event.value is True
    assert event.count == 3


def test_send_challenge1_event_can_publish_launched_event():
    args = send_challenge1_event.parse_args([
        "--event",
        "launched",
        "--count",
        "1",
        "--repeat-hz",
        "5.0",
    ])
    event = send_challenge1_event.spec_from_args(args)

    assert event.event == "launched"
    assert event.topic == "/ugv/uav_launched"
    assert event.repeat_hz == pytest.approx(5.0)
    assert event.count == 1


def test_send_challenge1_event_rejects_relative_topic_and_bad_rate():
    args = send_challenge1_event.parse_args(["--topic", "relative"])
    with pytest.raises(ValueError, match="absolute ROS topic"):
        send_challenge1_event.spec_from_args(args)

    args = send_challenge1_event.parse_args(["--repeat-hz", "0"])
    with pytest.raises(ValueError, match="repeat-hz"):
        send_challenge1_event.spec_from_args(args)
