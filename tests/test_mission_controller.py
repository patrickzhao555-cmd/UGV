import json
import math
import pathlib
import subprocess
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.chassis_controller import ChassisControllerConfig  # noqa: E402
from ugv_nav_core.mission_controller import (  # noqa: E402
    COMPETITION_MIN_SPEED_MPS,
    apply_competition_speed_rule,
    classify_mission_safety,
    encoder_average_distance_m,
    load_mission_plan,
    motion_rule_ok,
    parse_mission_plan,
    segment_speed_mps,
    segment_timeout_s,
    summarize_mission_records,
)


def _ready_motor_status():
    return {"connected": True, "teensy_pid_params_synced": True, "fault_reason": None}


def test_competition_min_speed_is_0_2_mph_in_mps():
    assert COMPETITION_MIN_SPEED_MPS == pytest.approx(0.089408)


def test_apply_competition_speed_rule_allows_stop_and_clamps_translation():
    assert apply_competition_speed_rule(0.0) == pytest.approx(0.0)
    assert apply_competition_speed_rule(0.01) == pytest.approx(0.089408)
    assert apply_competition_speed_rule(-0.01) == pytest.approx(-0.089408)
    assert apply_competition_speed_rule(0.15) == pytest.approx(0.15)


def test_motion_rule_accepts_stop_min_speed_and_pivot_translation_zero():
    assert motion_rule_ok(0.0)
    assert motion_rule_ok(0.089408)
    assert not motion_rule_ok(0.01)


def test_parse_mission_plan_validates_segments():
    plan = parse_mission_plan(
        {
            "mission_id": "unit",
            "segments": [
                {"type": "straight", "distance_m": 1.0, "speed_mps": 0.15},
                {"type": "pivot", "angle_deg": 90.0},
                {"type": "wait", "wait_s": 0.25},
            ],
        },
        source="unit.json",
    )
    assert plan.mission_id == "unit"
    assert [segment.segment_type for segment in plan.segments] == ["straight", "pivot", "wait"]
    assert segment_speed_mps(plan.segments[0], ChassisControllerConfig()) == pytest.approx(0.15)


def test_segment_speed_never_returns_zero_for_straight_default():
    plan = parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0}]})
    config = ChassisControllerConfig(mission_default_speed_mps=0.0)
    assert segment_speed_mps(plan.segments[0], config) == pytest.approx(0.089408)
    assert segment_timeout_s(plan.segments[0], config) > 1.0 / 0.089408


def test_explicit_segment_timeout_is_preserved():
    plan = parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0, "timeout_s": 4.5}]})
    assert segment_timeout_s(plan.segments[0], ChassisControllerConfig()) == pytest.approx(4.5)


def test_parse_mission_plan_rejects_bad_segments():
    with pytest.raises(ValueError):
        parse_mission_plan({"segments": [{"type": "straight", "distance_m": -1.0}]})
    with pytest.raises(ValueError):
        parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0, "speed_mps": 0.0}]})
    with pytest.raises(ValueError):
        parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0, "timeout_s": 0.0}]})
    with pytest.raises(ValueError):
        parse_mission_plan({"segments": [{"type": "pivot", "angle_deg": 0.0}]})


def test_load_mission_plan_from_json_file(tmp_path):
    mission_path = tmp_path / "mission.json"
    mission_path.write_text(
        json.dumps({"mission_id": "file", "segments": [{"type": "straight", "distance_m": 0.5}]}),
        encoding="utf-8",
    )
    plan = load_mission_plan(str(mission_path))
    assert plan.mission_id == "file"
    assert plan.segments[0].distance_m == pytest.approx(0.5)


def test_encoder_average_distance_from_ticks():
    config = ChassisControllerConfig(wheel_radius_m=0.0825, ticks_per_rev=3200)
    distance = encoder_average_distance_m(
        start_left_ticks=100,
        start_right_ticks=200,
        current_left_ticks=3300,
        current_right_ticks=3400,
        config=config,
    )
    assert distance == pytest.approx(2.0 * math.pi * 0.0825)


def test_mission_safety_classifies_degraded_and_critical():
    config = ChassisControllerConfig(
        sensor_timeout_s=0.3,
        mission_critical_sensor_timeout_s=1.0,
        stop_clearance_m=0.45,
        mission_emergency_stop_clearance_m=0.18,
    )
    degraded = classify_mission_safety(
        now_s=10.0,
        last_sensor_s=9.5,
        last_imu_s=9.99,
        last_motor_status_s=9.99,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=config,
    )
    assert degraded.level == "degraded"
    assert degraded.reason == "sensor_stale"

    critical = classify_mission_safety(
        now_s=10.0,
        last_sensor_s=9.99,
        last_imu_s=9.99,
        last_motor_status_s=9.99,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=0.1,
        config=config,
    )
    assert critical.level == "critical"
    assert critical.reason == "front_clearance_emergency"


def test_mission_summary_detects_sub_min_and_segments():
    summary = summarize_mission_records(
        [
            {
                "segment_index": 0,
                "segment_type": "straight",
                "segment_distance_m": 0.4,
                "target_distance_m": 1.0,
                "heading_error_rad": 0.1,
                "motion_rule_ok": False,
                "safety_level": "ok",
                "safety_reason": "ok",
            },
            {
                "segment_index": 0,
                "segment_type": "straight",
                "segment_distance_m": 1.0,
                "target_distance_m": 1.0,
                "heading_error_rad": -0.2,
                "motion_rule_ok": True,
                "safety_level": "critical",
                "safety_reason": "sensor_stale",
            },
        ]
    )
    assert summary["sub_min_speed_command_count"] == 1
    assert summary["critical_stop_count"] == 1
    assert summary["sensor_stale_count"] == 1
    assert summary["segments"][0]["last_distance_m"] == pytest.approx(1.0)
    assert summary["segments"][0]["distance_error_m"] == pytest.approx(0.0)
    assert summary["segments"][0]["max_abs_heading_error_rad"] == pytest.approx(0.2)
    assert summary["segments"][0]["pivot_overshoot_rad"] == pytest.approx(0.2)


def test_analyze_mission_log_cli(tmp_path):
    log_path = tmp_path / "mission.jsonl"
    log_path.write_text(
        json.dumps(
            {
                "segment_index": 0,
                "segment_type": "pivot",
                "motion_rule_ok": True,
                "heading_error_rad": 0.03,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "analyze_mission_log.py"), str(log_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "Sub-min speed command count: 0" in result.stdout
    assert "0: type=pivot" in result.stdout
    assert "final_error_deg=" in result.stdout
