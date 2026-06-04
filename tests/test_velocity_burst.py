import json
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_velocity_burst import build_stop_payload, build_velocity_payload, parse_args  # noqa: E402


def test_velocity_burst_payload_matches_motor_bridge_contract():
    payload = json.loads(build_velocity_payload(0.12, 0.0, "test"))
    assert payload["mode"] == "VELOCITY"
    assert payload["command_type"] == "velocity"
    assert payload["controller"] == "ugv_velocity_burst"
    assert payload["v_mps"] == 0.12
    assert payload["omega_radps"] == 0.0
    assert payload["reason"] == "test"


def test_velocity_burst_stop_payload_is_stop_safe():
    payload = json.loads(build_stop_payload())
    assert payload["mode"] == "STOP"
    assert payload["command_type"] == "stop"
    assert payload["v_mps"] == 0.0
    assert payload["omega_radps"] == 0.0


def test_velocity_burst_args_parse():
    args = parse_args(["--v-mps", "0.2", "--omega-radps", "0.1", "--duration-s", "2.0", "--yes"])
    assert args.v_mps == 0.2
    assert args.omega_radps == 0.1
    assert args.duration_s == 2.0
    assert args.yes
