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
    MIN_RULE_SPEED_MPH,
    MIN_RULE_SPEED_MPS,
    MOVING_TARGET_SPEED_MPS,
    MissionTelemetryRecorder,
    StuckMonitorState,
    apply_competition_speed_rule,
    classify_mission_safety,
    enforce_continuous_movement_speed,
    encoder_average_distance_m,
    load_mission_plan,
    mission_segment_start_hold_reason,
    motion_rule_ok,
    normalized_imu_qos,
    parse_mission_plan,
    reset_stuck_monitor,
    segment_speed_mps,
    segment_timeout_s,
    summarize_mission_records,
    telemetry_force_flush_key,
    update_stuck_monitor,
)
from ugv_nav_dual_mode import encoder_ticks_from_motor_status, parse_args  # noqa: E402


class _FakeTelemetryFile:
    def __init__(self, *, fail_on_write=False, fail_on_flush=False, fail_on_close=False):
        self.writes = []
        self.flush_count = 0
        self.closed = False
        self.fail_on_write = fail_on_write
        self.fail_on_flush = fail_on_flush
        self.fail_on_close = fail_on_close

    def write(self, text):
        if self.fail_on_write:
            raise OSError("write failed")
        self.writes.append(text)

    def flush(self):
        if self.fail_on_flush:
            raise OSError("flush failed")
        self.flush_count += 1

    def close(self):
        if self.fail_on_close:
            raise OSError("close failed")
        self.closed = True


def _ready_motor_status():
    return {"connected": True, "teensy_pid_params_synced": True, "fault_reason": None}


def test_competition_min_speed_is_0_2_mph_in_mps():
    assert MIN_RULE_SPEED_MPH == pytest.approx(0.2)
    assert MIN_RULE_SPEED_MPS == pytest.approx(0.0894)
    assert COMPETITION_MIN_SPEED_MPS == pytest.approx(MIN_RULE_SPEED_MPS)
    assert MOVING_TARGET_SPEED_MPS == pytest.approx(0.12)


def test_apply_competition_speed_rule_allows_stop_and_clamps_translation():
    assert apply_competition_speed_rule(0.0) == pytest.approx(0.0)
    assert apply_competition_speed_rule(0.01) == pytest.approx(0.12)
    assert apply_competition_speed_rule(-0.01) == pytest.approx(-0.12)
    assert apply_competition_speed_rule(0.15) == pytest.approx(0.15)


def test_motion_rule_accepts_stop_min_speed_and_pivot_translation_zero():
    assert motion_rule_ok(0.0)
    assert motion_rule_ok(0.0894)
    assert not motion_rule_ok(0.01)


def test_continuous_motion_enforcement_replaces_active_zero_and_sub_min_only():
    zero = enforce_continuous_movement_speed(0.0, phase="active_movement")
    assert zero.v_mps == pytest.approx(0.12)
    assert zero.zero_replaced
    assert zero.active_violation

    slow = enforce_continuous_movement_speed(0.02, phase="marker_search")
    assert slow.v_mps == pytest.approx(0.12)
    assert slow.sub_min_clamped

    waiting = enforce_continuous_movement_speed(0.0, phase="waiting_to_start")
    assert waiting.v_mps == pytest.approx(0.0)
    assert not waiting.changed

    arrived = enforce_continuous_movement_speed(0.0, phase="destination_reached")
    assert arrived.v_mps == pytest.approx(0.0)
    assert not arrived.changed

    fault = enforce_continuous_movement_speed(0.0, phase="fault")
    assert fault.v_mps == pytest.approx(0.0)
    assert not fault.changed


def test_normalized_imu_qos_defaults_to_sensor_data():
    assert normalized_imu_qos("sensor_data") == "sensor_data"
    assert normalized_imu_qos("best-effort") == "sensor_data"
    assert normalized_imu_qos("default") == "default"
    with pytest.raises(ValueError):
        normalized_imu_qos("unknown")
    args = parse_args(["--imu-qos", "best-effort"])
    assert args.imu_qos == "sensor_data"
    with pytest.raises(SystemExit):
        parse_args(["--imu-qos", "bad"])


def test_unknown_cli_args_fail_fast_unless_debug_enabled():
    with pytest.raises(SystemExit):
        parse_args(["--definitely-not-a-real-nav-arg"])
    args = parse_args(["--debug-allow-unknown-args", "true", "--definitely-not-a-real-nav-arg"])
    assert args.debug_allow_unknown_args


def test_debug_hard_run_test_flags_parse():
    args = parse_args(["--debug-ignore-nav-frame", "true", "--debug-ignore-obstacles", "true"])
    assert args.debug_ignore_nav_frame
    assert args.debug_ignore_obstacles


def test_imu_health_args_parse():
    args = parse_args([
        "--imu-min-rate-hz",
        "25.0",
        "--zed-status-topic",
        "/zed/custom_status",
        "--encoder-stamped-topic",
        "/custom/encoder_ticks",
        "--allow-encoder-heading-fallback",
        "true",
    ])
    assert args.imu_min_rate_hz == pytest.approx(25.0)
    assert args.zed_status_topic == "/zed/custom_status"
    assert args.encoder_stamped_topic == "/custom/encoder_ticks"
    assert args.allow_encoder_heading_fallback


def test_motor_status_encoder_ticks_parse_for_heading_fallback():
    assert encoder_ticks_from_motor_status({"encoder_ticks": [123, -456]}) == (123, -456)
    assert encoder_ticks_from_motor_status({"left_ticks": "12", "right_ticks": "34"}) == (12, 34)
    assert encoder_ticks_from_motor_status({"encoder_ticks": ["bad", 34]}) is None
    assert encoder_ticks_from_motor_status({"encoder_ticks": [12]}) is None


def test_pivot_test_angle_fails_fast_above_shortest_path_range():
    with pytest.raises(SystemExit):
        parse_args(["--controller-mode", "pivot_test", "--pivot-angle-deg", "181"])


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
    assert segment_speed_mps(plan.segments[0], config) == pytest.approx(0.12)
    assert segment_timeout_s(plan.segments[0], config) > 1.0 / 0.12


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
    with pytest.raises(ValueError):
        parse_mission_plan({"segments": [{"type": "pivot", "angle_deg": 181.0}]})


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


def test_mission_safety_requires_healthy_imu_rate_when_requested():
    decision = classify_mission_safety(
        now_s=10.0,
        last_sensor_s=9.99,
        last_imu_s=9.99,
        last_motor_status_s=9.99,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(imu_min_rate_hz=20.0),
        require_imu=True,
        imu_rate_hz=5.0,
    )
    assert decision.level == "critical"
    assert decision.reason == "imu_rate_low"


def test_mission_segment_start_blocks_until_straight_encoder_start_is_valid():
    segment = parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0}]}).segments[0]
    config = ChassisControllerConfig()

    assert mission_segment_start_hold_reason(
        segment,
        safety_level="ok",
        safety_reason="ok",
        encoder_available=False,
        left_ticks=None,
        right_ticks=None,
        pivot_clearance_reason=None,
        config=config,
    ) == "encoder_unavailable"
    assert mission_segment_start_hold_reason(
        segment,
        safety_level="ok",
        safety_reason="ok",
        encoder_available=True,
        left_ticks=100,
        right_ticks=200,
        pivot_clearance_reason=None,
        config=config,
    ) is None


def test_mission_segment_start_blocks_stale_or_invalid_nav_frame_before_any_segment():
    plan = parse_mission_plan(
        {
            "segments": [
                {"type": "straight", "distance_m": 1.0},
                {"type": "pivot", "angle_deg": 45.0},
                {"type": "wait", "wait_s": 0.2},
            ]
        }
    )
    for segment in plan.segments:
        assert mission_segment_start_hold_reason(
            segment,
            safety_level="degraded",
            safety_reason="sensor_stale",
            encoder_available=True,
            left_ticks=1,
            right_ticks=1,
            pivot_clearance_reason=None,
            config=ChassisControllerConfig(),
        ) == "sensor_stale"
        assert mission_segment_start_hold_reason(
            segment,
            safety_level="degraded",
            safety_reason="front_clearance_invalid",
            encoder_available=True,
            left_ticks=1,
            right_ticks=1,
            pivot_clearance_reason=None,
            config=ChassisControllerConfig(),
        ) == "front_clearance_invalid"


def test_mission_segment_start_blocks_straight_obstacle_hold_by_default():
    segment = parse_mission_plan({"segments": [{"type": "straight", "distance_m": 1.0}]}).segments[0]
    assert mission_segment_start_hold_reason(
        segment,
        safety_level="degraded",
        safety_reason="near_obstacle",
        encoder_available=True,
        left_ticks=1,
        right_ticks=1,
        pivot_clearance_reason=None,
        config=ChassisControllerConfig(mission_stop_on_degraded_obstacle=True),
    ) == "near_obstacle"
    assert mission_segment_start_hold_reason(
        segment,
        safety_level="degraded",
        safety_reason="near_obstacle",
        encoder_available=True,
        left_ticks=1,
        right_ticks=1,
        pivot_clearance_reason=None,
        config=ChassisControllerConfig(mission_stop_on_degraded_obstacle=False),
    ) is None


def test_debug_flags_allow_mission_without_nav_frame_or_obstacle_stream():
    decision = classify_mission_safety(
        now_s=10.0,
        last_sensor_s=None,
        last_imu_s=9.99,
        last_motor_status_s=9.99,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=None,
        config=ChassisControllerConfig(
            debug_ignore_nav_frame=True,
            debug_ignore_obstacles=True,
        ),
        require_imu=True,
        imu_rate_hz=30.0,
    )
    assert decision.level == "ok"
    assert decision.reason == "ok"


def test_mission_segment_start_blocks_pivot_until_clearance_is_known_and_safe():
    segment = parse_mission_plan({"segments": [{"type": "pivot", "angle_deg": 45.0}]}).segments[0]
    config = ChassisControllerConfig()

    assert mission_segment_start_hold_reason(
        segment,
        safety_level="ok",
        safety_reason="ok",
        encoder_available=False,
        left_ticks=None,
        right_ticks=None,
        pivot_clearance_reason="pivot_clearance_unknown",
        config=config,
    ) == "pivot_clearance_unknown"
    assert mission_segment_start_hold_reason(
        segment,
        safety_level="ok",
        safety_reason="ok",
        encoder_available=False,
        left_ticks=None,
        right_ticks=None,
        pivot_clearance_reason=None,
        config=config,
    ) is None


def test_stuck_monitor_detects_pivot_no_response_and_resets_on_response():
    config = ChassisControllerConfig(pivot_stuck_timeout_s=0.2, pivot_stuck_min_yaw_rate_radps=0.04)
    state = StuckMonitorState()
    result = update_stuck_monitor(
        state,
        now_s=1.0,
        segment_key="0:pivot",
        command_kind="pivot",
        v_cmd_mps=0.0,
        omega_cmd_radps=0.2,
        yaw_rate_radps=0.0,
        motor_status={"measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    assert not result.stuck
    result = update_stuck_monitor(
        state,
        now_s=1.25,
        segment_key="0:pivot",
        command_kind="pivot",
        v_cmd_mps=0.0,
        omega_cmd_radps=0.2,
        yaw_rate_radps=0.0,
        motor_status={"measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    assert result.stuck
    assert result.reason == "pivot_stuck"

    reset_stuck_monitor(state)
    result = update_stuck_monitor(
        state,
        now_s=2.0,
        segment_key="0:pivot",
        command_kind="pivot",
        v_cmd_mps=0.0,
        omega_cmd_radps=0.2,
        yaw_rate_radps=0.08,
        motor_status={"measured_left_mps": -0.04, "measured_right_mps": 0.04},
        config=config,
    )
    assert not result.stuck


def test_stuck_monitor_detects_straight_no_measured_motion():
    config = ChassisControllerConfig(straight_stuck_timeout_s=0.2, straight_stuck_min_measured_mps=0.03)
    state = StuckMonitorState()
    update_stuck_monitor(
        state,
        now_s=1.0,
        segment_key="0:straight",
        command_kind="straight",
        v_cmd_mps=0.15,
        omega_cmd_radps=0.0,
        yaw_rate_radps=0.0,
        motor_status={"measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    result = update_stuck_monitor(
        state,
        now_s=1.25,
        segment_key="0:straight",
        command_kind="straight",
        v_cmd_mps=0.15,
        omega_cmd_radps=0.0,
        yaw_rate_radps=0.0,
        motor_status={"measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    assert result.stuck
    assert result.reason == "straight_stuck"


def test_stuck_monitor_bypasses_dry_run_for_straight_and_pivot():
    config = ChassisControllerConfig(straight_stuck_timeout_s=0.2, pivot_stuck_timeout_s=0.2)

    straight_state = StuckMonitorState()
    update_stuck_monitor(
        straight_state,
        now_s=1.0,
        segment_key="0:straight",
        command_kind="straight",
        v_cmd_mps=0.15,
        omega_cmd_radps=0.0,
        yaw_rate_radps=0.0,
        motor_status={"dry_run": True, "measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    result = update_stuck_monitor(
        straight_state,
        now_s=1.5,
        segment_key="0:straight",
        command_kind="straight",
        v_cmd_mps=0.15,
        omega_cmd_radps=0.0,
        yaw_rate_radps=0.0,
        motor_status={"dry_run": True, "measured_left_mps": 0.0, "measured_right_mps": 0.0},
        config=config,
    )
    assert not result.stuck
    assert result.reason == "dry_run_bypass"
    assert straight_state.reason == "dry_run_bypass"

    pivot_state = StuckMonitorState()
    result = update_stuck_monitor(
        pivot_state,
        now_s=2.0,
        segment_key="1:pivot",
        command_kind="pivot",
        v_cmd_mps=0.0,
        omega_cmd_radps=0.2,
        yaw_rate_radps=0.0,
        motor_status={"dry_run": "true"},
        config=config,
    )
    assert not result.stuck
    assert result.reason == "dry_run_bypass"


def _fake_recorder(*, flush_period_s=0.5, flush_max_records=25, fake_file=None):
    recorder = MissionTelemetryRecorder(
        enabled=False,
        telemetry_dir="unused",
        mission_id="unit",
        flush_period_s=flush_period_s,
        flush_max_records=flush_max_records,
    )
    fake_file = _FakeTelemetryFile() if fake_file is None else fake_file
    recorder._file = fake_file
    recorder.enabled = True
    return recorder, fake_file


def test_telemetry_force_flush_key_only_for_terminal_or_critical_states():
    assert telemetry_force_flush_key(
        mission_state="mission_complete",
        safety_level="ok",
        safety_reason="ok",
        segment_index=2,
    ) == "mission_complete:ok:ok:2"
    assert telemetry_force_flush_key(
        mission_state="abort",
        safety_level="critical",
        safety_reason="segment_timeout",
        segment_index=1,
    ) == "abort:critical:segment_timeout:1"
    assert telemetry_force_flush_key(
        mission_state="straight_active",
        safety_level="critical",
        safety_reason="imu_stale",
        segment_index=0,
    ) == "straight_active:critical:imu_stale:0"
    assert telemetry_force_flush_key(
        mission_state="straight_active",
        safety_level="ok",
        safety_reason="ok",
        segment_index=0,
    ) is None


def test_duplicate_telemetry_force_flush_key_only_forces_once():
    seen_key = None
    key = telemetry_force_flush_key(
        mission_state="abort",
        safety_level="critical",
        safety_reason="segment_timeout",
        segment_index=0,
    )
    first_force = key is not None and key != seen_key
    seen_key = key if first_force else seen_key
    second_force = key is not None and key != seen_key
    assert first_force
    assert not second_force


def test_chassis_runtime_aborts_mission_on_pivot_controller_abort_and_slews_straight_test():
    controller_text = (ROOT / "ros2_ws" / "src" / "ugv_nav" / "ugv_nav_dual_mode.py").read_text()
    pivot_block = controller_text[
        controller_text.index('if segment.segment_type == "pivot":') : controller_text.index(
            '            wait_s = max(0.0, float(segment.wait_s))'
        )
    ]
    straight_test_block = controller_text[
        controller_text.index('if self.mode == "straight_test":') : controller_text.index(
            '                        elif self.mode == "pivot_test":'
        )
    ]

    assert 'if step.state == "abort":' in pivot_block
    assert 'self.mission_state = "abort"' in pivot_block
    assert 'self.mission_safety_level = "critical"' in pivot_block
    assert "straight_omega_with_slew(" in straight_test_block
    assert "self.previous_straight_omega_radps = omega" in straight_test_block
    assert "require_imu=not self._encoder_feedback_available()" not in controller_text
    assert "require_imu = not self._encoder_fallback_allowed(now_s)" in controller_text
    assert 'parser.add_argument("--allow-encoder-heading-fallback"' in controller_text


def test_telemetry_recorder_does_not_flush_every_write():
    recorder, fake_file = _fake_recorder(flush_period_s=0.5, flush_max_records=25)
    recorder.write({"seq": 1}, now_s=10.0)
    recorder.write({"seq": 2}, now_s=10.1)
    assert len(fake_file.writes) == 2
    assert fake_file.flush_count == 0


def test_telemetry_recorder_flushes_on_period_record_count_force_and_close():
    recorder, fake_file = _fake_recorder(flush_period_s=0.5, flush_max_records=25)
    recorder.write({"seq": 1}, now_s=10.0)
    recorder.write({"seq": 2}, now_s=10.5)
    assert fake_file.flush_count == 1

    recorder, fake_file = _fake_recorder(flush_period_s=10.0, flush_max_records=2)
    recorder.write({"seq": 1}, now_s=10.0)
    recorder.write({"seq": 2}, now_s=10.1)
    assert fake_file.flush_count == 1

    recorder, fake_file = _fake_recorder(flush_period_s=10.0, flush_max_records=25)
    recorder.write({"seq": 1}, now_s=10.0, force_flush=True)
    assert fake_file.flush_count == 1
    recorder.write({"seq": 2}, now_s=10.1)
    recorder.close()
    assert fake_file.flush_count == 2
    assert fake_file.closed


def test_telemetry_recorder_write_error_disables_without_raising():
    recorder, fake_file = _fake_recorder(fake_file=_FakeTelemetryFile(fail_on_write=True))
    recorder.write({"seq": 1}, now_s=10.0)
    assert recorder.failed
    assert not recorder.enabled
    assert "write failed" in str(recorder.error)
    assert fake_file.closed
    recorder.write({"seq": 2}, now_s=10.1)
    assert fake_file.writes == []


def test_telemetry_recorder_flush_error_disables_without_raising():
    recorder, fake_file = _fake_recorder(fake_file=_FakeTelemetryFile(fail_on_flush=True))
    recorder.write({"seq": 1}, now_s=10.0, force_flush=True)
    assert recorder.failed
    assert not recorder.enabled
    assert "flush failed" in str(recorder.error)
    assert fake_file.closed


def test_telemetry_recorder_close_error_disables_without_raising():
    recorder, _ = _fake_recorder(fake_file=_FakeTelemetryFile(fail_on_close=True))
    recorder.write({"seq": 1}, now_s=10.0)
    recorder.close()
    assert recorder.failed
    assert not recorder.enabled
    assert "close failed" in str(recorder.error)


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
