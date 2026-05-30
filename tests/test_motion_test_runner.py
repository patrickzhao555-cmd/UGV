import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from ugv_motion_test_runner import (  # noqa: E402
    BAG_TOPICS,
    MotionTestCase,
    build_bag_command,
    build_launch_command,
    builtin_suite,
    case_launch_args,
    compensation_table_draft,
    compute_case_metrics,
    expected_distance_for_case,
    expand_repeats,
    load_suite_file,
    merge_launch_args,
    parse_launch_arg,
    parse_optional_float,
    select_cases,
    write_case_mission_file,
    write_summary_files,
)


def test_builtin_basic_suite_contains_expected_cases():
    suite = builtin_suite("basic")
    ids = {case.id for case in suite.cases}
    assert "idle_health_10s" in ids
    assert "straight_open_015_2s" in ids
    assert "straight_hold_015_2s" in ids
    assert "pivot_left_45" in ids
    assert "pivot_right_90" in ids


def test_select_cases_and_expand_repeats():
    suite = builtin_suite("pivot_calibration")
    selected = select_cases(suite, ["pivot_left_45_repeat_3"])
    expanded = expand_repeats(selected)
    assert [case.id for case in expanded] == [
        "pivot_left_45_repeat_3_run1",
        "pivot_left_45_repeat_3_run2",
        "pivot_left_45_repeat_3_run3",
    ]
    with pytest.raises(ValueError):
        select_cases(suite, ["missing_case"])


def test_custom_suite_file_parsing(tmp_path):
    path = tmp_path / "suite.json"
    path.write_text(
        json.dumps(
            {
                "suite_id": "my_turn_tests",
                "cases": [
                    {
                        "id": "pivot_left_45",
                        "mode": "pivot_test",
                        "angle_deg": 45,
                        "expected_angle_deg": 45,
                        "launch_args": {"nav_pivot_angle_deg": "45"},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    suite = load_suite_file(path)
    assert suite.suite_id == "my_turn_tests"
    assert suite.cases[0].id == "pivot_left_45"
    assert suite.cases[0].expected_angle_deg == pytest.approx(45.0)


def test_launch_command_generation_for_pivot_case():
    case = next(case for case in builtin_suite("basic").cases if case.id == "pivot_right_45")
    args = case_launch_args(case)
    command = build_launch_command(args)
    assert command[:4] == ["ros2", "launch", "ugv_sensor_sync", "competition_bringup.launch.py"]
    assert "nav_controller_mode:=pivot_test" in command
    assert "nav_pivot_angle_deg:=-45" in command


def test_straight_expected_distance_calculation():
    case = next(case for case in builtin_suite("basic").cases if case.id == "straight_open_015_2s")
    assert expected_distance_for_case(case) == pytest.approx(0.30)


def test_pivot_sign_convention_left_positive_right_negative():
    suite = builtin_suite("basic")
    left = next(case for case in suite.cases if case.id == "pivot_left_30")
    right = next(case for case in suite.cases if case.id == "pivot_right_30")
    assert left.expected_angle_deg == pytest.approx(30.0)
    assert right.expected_angle_deg == pytest.approx(-30.0)


def test_mission_case_writes_mission_file_and_launch_arg(tmp_path):
    case = next(case for case in builtin_suite("mission_smoke").cases if case.id.startswith("mission_straight_05"))
    mission_path = write_case_mission_file(case, tmp_path)
    assert mission_path.exists()
    data = json.loads(mission_path.read_text(encoding="utf-8"))
    assert data["mission_id"] == case.id
    args = case_launch_args(case, case_dir=tmp_path)
    assert args["nav_controller_mode"] == "mission_sequence"
    assert args["nav_mission_file"] == str(mission_path)


def test_runner_launch_args_can_pin_telemetry_under_case_dir(tmp_path):
    case = next(case for case in builtin_suite("mission_smoke").cases if case.id.startswith("mission_straight_05"))
    args = merge_launch_args(case_launch_args(case, case_dir=tmp_path), {})
    args.setdefault("nav_telemetry_dir", str(tmp_path / "mission_telemetry"))
    command = build_launch_command(args)
    assert f"nav_telemetry_dir:={tmp_path / 'mission_telemetry'}" in command


def test_parse_optional_float_accepts_blank_skip_and_numbers():
    assert parse_optional_float("") is None
    assert parse_optional_float("skip") is None
    assert parse_optional_float("1.25") == pytest.approx(1.25)
    with pytest.raises(ValueError):
        parse_optional_float("abc")


def test_parse_launch_arg_requires_ros_launch_style():
    assert parse_launch_arg("motor_dry_run:=true") == ("motor_dry_run", "true")
    with pytest.raises(Exception):
        parse_launch_arg("motor_dry_run=true")


def test_build_bag_command_contains_expected_topics(tmp_path):
    command = build_bag_command(tmp_path / "bag")
    assert command[:4] == ["ros2", "bag", "record", "-o"]
    for topic in BAG_TOPICS:
        assert topic in command


def test_case_metrics_calculate_distance_angle_and_health_fields():
    case = MotionTestCase(
        id="pivot_left_45",
        mode="pivot_test",
        expected_angle_deg=45.0,
    )
    status_records = [
        {
            "segment_distance_m": 0.0,
            "heading_error_rad": 0.1,
            "pivot_final_error_rad": 0.05,
            "pivot_overshoot_rad": 0.02,
            "pivot_retry_count": 1,
            "imu_rate_hz": 98.0,
            "omega_saturated": True,
            "motion_rule_ok": True,
            "safety_level": "ok",
        },
        {
            "heading_error_rad": 0.05,
            "pivot_final_error_rad": 0.04,
            "imu_rate_hz": 102.0,
            "omega_saturated": False,
            "motion_rule_ok": True,
            "safety_level": "ok",
        },
    ]
    cmd_records = [
        {"command_type": "velocity", "omega_radps": 0.2, "reason": "pivot_rotate"},
        {"command_type": "stop", "omega_radps": 0.0, "reason": "pivot_test_complete"},
    ]
    manual = {"actual_angle_deg": 43.0, "actual_distance_m": None, "lateral_drift_m": None}
    metrics = compute_case_metrics(case, status_records, cmd_records, manual)
    assert metrics["actual_angle_error_deg"] == pytest.approx(-2.0)
    assert metrics["estimated_angle_deg"] == pytest.approx(45.0 - 0.04 * 180.0 / 3.141592653589793)
    assert metrics["gyro_manual_disagreement_deg"] is not None
    assert metrics["omega_saturation_percent"] == pytest.approx(50.0)
    assert metrics["imu_rate_avg_hz"] == pytest.approx(100.0)
    assert metrics["stop_reasons"] == {"pivot_test_complete": 1}


def test_case_metrics_calculate_straight_distance_errors():
    case = MotionTestCase(
        id="straight_open_015_2s",
        mode="straight_test",
        speed_mps=0.15,
        duration_s=2.0,
    )
    metrics = compute_case_metrics(
        case,
        [{"segment_distance_m": 0.28, "straight_heading_error_rms": 0.02}],
        [],
        {"actual_distance_m": 0.31, "lateral_drift_m": 0.01},
    )
    assert metrics["expected_distance_m"] == pytest.approx(0.30)
    assert metrics["distance_error_m"] == pytest.approx(0.01)
    assert metrics["encoder_distance_error_m"] == pytest.approx(-0.02)
    assert metrics["heading_rms_deg"] == pytest.approx(0.02 * 180.0 / 3.141592653589793)


def test_compensation_table_draft_uses_manual_angle_errors():
    results = [
        {"metrics": {"target_angle_deg": 45.0, "actual_angle_deg": 43.0, "actual_angle_error_deg": -2.0}},
        {"metrics": {"target_angle_deg": 45.0, "actual_angle_deg": 44.0, "actual_angle_error_deg": -1.0}},
        {"metrics": {"target_angle_deg": -45.0, "actual_angle_deg": -47.0, "actual_angle_error_deg": -2.0}},
    ]
    table = compensation_table_draft(results)
    assert table["left"]["45"]["bias_deg"] == pytest.approx(1.5)
    assert table["right"]["45"]["bias_deg"] == pytest.approx(2.0)


def test_write_summary_files_generates_json_csv_markdown(tmp_path):
    result = {
        "case_id": "straight_open_015_2s",
        "mode": "straight_test",
        "manual": {"pass_fail": "pass"},
        "metrics": {
            "expected_distance_m": 0.3,
            "actual_distance_m": 0.31,
            "distance_error_m": 0.01,
            "heading_rms_deg": 1.0,
            "stop_reasons": {"test_duration_elapsed": 1},
        },
    }
    write_summary_files(tmp_path, "unit_suite", [result])
    assert (tmp_path / "summary.json").exists()
    assert (tmp_path / "summary.csv").exists()
    assert (tmp_path / "summary.md").exists()
    assert "straight_open_015_2s" in (tmp_path / "summary.md").read_text(encoding="utf-8")
