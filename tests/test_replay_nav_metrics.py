import json
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from tools.replay_nav_metrics import summarize  # noqa: E402


def test_replay_metrics_command_type_overrides_controller(tmp_path):
    path = tmp_path / "replay.jsonl"
    rows = [
        {
            "cmd": {
                "mode": "FORWARD",
                "command_type": "raw",
                "controller": "velocity",
                "v_mps": 0.1,
                "omega_radps": 0.0,
            },
            "replans": 0,
        },
        {
            "cmd": {
                "mode": "FORWARD",
                "command_type": "velocity",
                "controller": "velocity",
                "v_mps": 0.2,
                "omega_radps": 0.1,
            },
            "active_scan": {"active": True, "remaining": 3},
            "replans": 1,
        },
        {
            "cmd": {"mode": "STOP", "command_type": "stop", "v_mps": 0.0, "omega_radps": 0.0},
            "replans": 1,
        },
    ]
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8-sig")

    metrics = summarize(path)
    assert metrics["raw_command_count"] == 1
    assert metrics["velocity_command_count"] == 1
    assert metrics["stop_count"] == 1
    assert metrics["active_scan_count"] == 1
