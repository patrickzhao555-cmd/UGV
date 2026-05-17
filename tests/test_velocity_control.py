import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_motor_controller"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_motor_controller.velocity_control import (  # noqa: E402
    EncoderSpeedSample,
    VelocityPidConfig,
    WheelVelocityPid,
    active_command_refresh_due,
    apply_velocity_raw_fallback_floor,
    command_is_timed_out,
    encoder_delta_to_wheel_speed_mps,
    encoder_speed_is_fresh,
    estimate_encoder_wheel_speeds,
    extract_raw_drive,
    extract_velocity_command,
    is_stop_command,
    reset_velocity_pid_pair,
    select_drive_command,
    stale_encoder_control_mode,
    update_encoder_wheel_speed_estimate,
    velocity_to_wheel_speeds,
)
from ugv_nav_dual_mode import ControlCommand  # noqa: E402


def test_nav_velocity_command_contract_preserves_legacy_raw_fields():
    cmd = ControlCommand(
        mode="VELOCITY",
        command_type="velocity",
        raw_left=0.0,
        raw_right=0.0,
        v_mps=0.18,
        omega_radps=0.35,
        controller="velocity",
    )
    payload = cmd.as_dict()
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == 0.18
    assert payload["omega_radps"] == 0.35
    assert "raw_left" in payload
    assert "raw_right" in payload


def test_nav_legacy_raw_command_serializes_raw_fields():
    cmd = ControlCommand(
        mode="FORWARD",
        raw_left=0.22,
        raw_right=0.55,
        reason="legacy raw smoke",
        controller="step",
    )
    payload = cmd.as_dict()
    assert payload["command_type"] == "raw"
    assert payload["raw_left"] == 0.22
    assert payload["raw_right"] == 0.55
    assert payload["v_mps"] == 0.0
    assert payload["omega_radps"] == 0.0


def test_nav_stop_command_contract_is_safe():
    payload = ControlCommand(mode="STOP").as_dict()
    assert payload["command_type"] == "stop"
    assert payload["raw_left"] == 0.0
    assert payload["raw_right"] == 0.0
    assert payload["v_mps"] == 0.0
    assert payload["omega_radps"] == 0.0
    assert is_stop_command(payload)


def test_raw_command_extraction_legacy_modes_and_fields():
    assert extract_raw_drive({"raw_left": 0.2, "raw_right": -0.1}) == (0.2, -0.1)
    assert extract_raw_drive({"mode": "FORWARD"}) == (0.35, 0.35)
    assert extract_raw_drive({"mode": "TURN_LEFT"}) == (-0.30, 0.30)


def test_velocity_raw_fallback_floor_applies_per_wheel_independently():
    floored = apply_velocity_raw_fallback_floor(
        0.005,
        0.14,
        enabled=True,
        command_type="velocity",
        mode="FORWARD",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )

    assert floored.left_raw == 0.14
    assert floored.right_raw == 0.14
    assert floored.applied_left
    assert not floored.applied_right


def test_velocity_raw_fallback_floor_preserves_turn_signs():
    floored = apply_velocity_raw_fallback_floor(
        -0.005,
        0.035,
        enabled=True,
        command_type="velocity",
        mode="TURN_LEFT",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )

    assert floored.left_raw == -0.14
    assert floored.right_raw == 0.14
    assert floored.applied_left
    assert floored.applied_right


def test_velocity_raw_fallback_floor_keeps_zero_and_stop_safe():
    zero = apply_velocity_raw_fallback_floor(
        0.0,
        0.03,
        enabled=True,
        command_type="velocity",
        mode="FORWARD",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )
    assert zero.left_raw == 0.0
    assert zero.right_raw == 0.14
    assert not zero.applied_left
    assert zero.applied_right

    stop = apply_velocity_raw_fallback_floor(
        0.005,
        0.005,
        enabled=True,
        command_type="velocity",
        mode="STOP",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )
    assert stop.left_raw == 0.005
    assert stop.right_raw == 0.005
    assert not stop.applied_left
    assert not stop.applied_right


def test_velocity_raw_fallback_floor_does_not_change_legacy_raw_or_direct_commands():
    legacy = apply_velocity_raw_fallback_floor(
        0.005,
        0.035,
        enabled=True,
        command_type="raw",
        mode="FORWARD",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )
    assert legacy.left_raw == 0.005
    assert legacy.right_raw == 0.035
    assert not legacy.applied_left
    assert not legacy.applied_right

    disabled = apply_velocity_raw_fallback_floor(
        0.005,
        0.035,
        enabled=False,
        command_type="velocity",
        mode="FORWARD",
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    )
    assert disabled.left_raw == 0.005
    assert disabled.right_raw == 0.035


def test_raw_fallback_command_refreshes_before_timeout():
    assert active_command_refresh_due(
        last_command_time_s=10.0,
        last_motor_send_time_s=10.0,
        now_s=10.30,
        timeout_s=3.0,
        refresh_period_s=0.25,
    )
    assert not command_is_timed_out(10.0, now_s=10.30, timeout_s=3.0)


def test_timeout_does_not_fire_before_configured_timeout():
    assert not command_is_timed_out(10.0, now_s=12.99, timeout_s=3.0)
    assert not active_command_refresh_due(
        last_command_time_s=10.0,
        last_motor_send_time_s=12.90,
        now_s=12.99,
        timeout_s=3.0,
        refresh_period_s=0.25,
    )


def test_timeout_fires_after_configured_timeout():
    assert command_is_timed_out(10.0, now_s=13.01, timeout_s=3.0)
    assert not active_command_refresh_due(
        last_command_time_s=10.0,
        last_motor_send_time_s=12.90,
        now_s=13.01,
        timeout_s=3.0,
        refresh_period_s=0.25,
    )


def test_stop_command_still_overrides_refresh_and_timeout_logic():
    stop_payload = {"mode": "STOP", "command_type": "stop", "raw_left": 0.14, "raw_right": 0.14}
    assert is_stop_command(stop_payload)
    assert not apply_velocity_raw_fallback_floor(
        stop_payload["raw_left"],
        stop_payload["raw_right"],
        enabled=True,
        command_type=stop_payload["command_type"],
        mode=stop_payload["mode"],
        min_wheel_raw=0.14,
        min_target_raw=0.001,
    ).applied_left


def test_velocity_command_parsing_respects_explicit_raw():
    obj = {
        "mode": "FORWARD",
        "command_type": "velocity",
        "controller": "velocity",
        "v_mps": 0.18,
        "omega_radps": 0.35,
        "raw_left": 0.0,
        "raw_right": 0.0,
    }
    assert extract_velocity_command(obj) == (0.18, 0.35)
    raw_obj = dict(obj)
    raw_obj["command_type"] = "raw"
    assert extract_velocity_command(raw_obj) is None


def test_velocity_path_does_not_depend_on_optional_raw_fields():
    obj = {
        "mode": "VELOCITY",
        "command_type": "velocity",
        "v_mps": 0.18,
        "omega_radps": 0.35,
        "raw_left": "not-a-number",
        "raw_right": "not-a-number",
    }
    path, velocity_cmd, raw_cmd = select_drive_command(obj, velocity_control_enabled=True)
    assert path == "velocity"
    assert velocity_cmd == (0.18, 0.35)
    assert raw_cmd is None

    try:
        select_drive_command(obj, velocity_control_enabled=False)
    except ValueError:
        pass
    else:
        raise AssertionError("raw fallback should still validate malformed raw fields")


def test_velocity_to_wheel_speed_conversion():
    left, right = velocity_to_wheel_speeds(0.18, 0.35, 0.60)
    assert round(left, 3) == 0.075
    assert round(right, 3) == 0.285


def test_encoder_delta_to_measured_wheel_speed():
    speed = encoder_delta_to_wheel_speed_mps(
        delta_ticks=250,
        dt_s=0.5,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
    )
    assert round(speed, 4) == 0.3142


def test_encoder_speed_estimate_prefers_controller_millis():
    prev = EncoderSpeedSample(left_ticks=1000, right_ticks=1000, host_time_s=10.0, controller_millis=1000)
    cur = EncoderSpeedSample(left_ticks=1250, right_ticks=750, host_time_s=11.0, controller_millis=1500)

    estimate = estimate_encoder_wheel_speeds(
        prev,
        cur,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        filter_alpha=1.0,
        max_abs_speed_mps=2.0,
    )

    assert estimate.dt_source == "controller_millis"
    assert round(estimate.dt_s, 3) == 0.5
    assert round(estimate.left_mps, 4) == 0.3142
    assert round(estimate.right_mps, 4) == -0.3142
    assert estimate.anomaly is None


def test_encoder_speed_estimate_falls_back_and_clamps_safely():
    prev = EncoderSpeedSample(left_ticks=0, right_ticks=0, host_time_s=10.0, controller_millis=2000)
    cur = EncoderSpeedSample(left_ticks=5000, right_ticks=-5000, host_time_s=10.2, controller_millis=1999)

    estimate = estimate_encoder_wheel_speeds(
        prev,
        cur,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        filter_alpha=1.0,
        max_abs_speed_mps=1.0,
    )

    assert estimate.dt_source == "host_time_fallback"
    assert round(estimate.dt_s, 3) == 0.2
    assert estimate.left_mps == 1.0
    assert estimate.right_mps == -1.0
    assert "controller_millis_nonpositive_dt" in (estimate.anomaly or "")
    assert "wheel_speed_sanity_clamped" in (estimate.anomaly or "")


def test_host_time_tiny_dt_sample_is_skipped_without_measured_speed_spike():
    baseline = EncoderSpeedSample(left_ticks=1000, right_ticks=1000, host_time_s=10.0)
    burst = EncoderSpeedSample(left_ticks=1020, right_ticks=1020, host_time_s=10.0008)

    update = update_encoder_wheel_speed_estimate(
        baseline,
        burst,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        previous_left_mps=0.05,
        previous_right_mps=0.05,
        filter_alpha=1.0,
        max_abs_speed_mps=1.0,
        min_host_dt_s=0.015,
    )

    assert update.skipped
    assert update.estimate is None
    assert update.baseline_sample == baseline
    assert update.dt_source == "host_time"
    assert round(update.accumulated_dt_s, 4) == 0.0008
    assert "host_dt_too_small_skipped" in (update.anomaly or "")


def test_host_time_tiny_dt_samples_accumulate_until_min_dt():
    baseline = update_encoder_wheel_speed_estimate(
        None,
        EncoderSpeedSample(left_ticks=0, right_ticks=0, host_time_s=10.0),
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        min_host_dt_s=0.015,
    ).baseline_sample

    first_burst = update_encoder_wheel_speed_estimate(
        baseline,
        EncoderSpeedSample(left_ticks=2, right_ticks=2, host_time_s=10.005),
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        min_host_dt_s=0.015,
    )
    assert first_burst.skipped
    assert first_burst.baseline_sample == baseline

    second_burst = update_encoder_wheel_speed_estimate(
        first_burst.baseline_sample,
        EncoderSpeedSample(left_ticks=4, right_ticks=4, host_time_s=10.010),
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        min_host_dt_s=0.015,
    )
    assert second_burst.skipped
    assert second_burst.baseline_sample == baseline

    valid = update_encoder_wheel_speed_estimate(
        second_burst.baseline_sample,
        EncoderSpeedSample(left_ticks=8, right_ticks=8, host_time_s=10.020),
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        filter_alpha=1.0,
        max_abs_speed_mps=2.0,
        min_host_dt_s=0.015,
    )

    assert not valid.skipped
    assert valid.estimate is not None
    assert valid.estimate.dt_source == "host_time"
    assert round(valid.estimate.dt_s, 3) == 0.020
    assert round(valid.estimate.left_mps, 4) == 0.2513
    assert valid.baseline_sample.left_ticks == 8


def test_tiny_host_dt_does_not_clamp_measured_speed_to_max():
    baseline = EncoderSpeedSample(left_ticks=0, right_ticks=0, host_time_s=20.0)
    tiny = EncoderSpeedSample(left_ticks=500, right_ticks=500, host_time_s=20.001)

    skipped = update_encoder_wheel_speed_estimate(
        baseline,
        tiny,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        previous_left_mps=0.04,
        previous_right_mps=0.04,
        filter_alpha=1.0,
        max_abs_speed_mps=1.0,
        min_host_dt_s=0.015,
    )

    assert skipped.estimate is None
    assert "wheel_speed_sanity_clamped" not in (skipped.anomaly or "")


def test_controller_millis_bypasses_host_min_dt_and_is_preferred():
    baseline = EncoderSpeedSample(left_ticks=0, right_ticks=0, host_time_s=30.0, controller_millis=1000)
    current = EncoderSpeedSample(left_ticks=10, right_ticks=10, host_time_s=30.0008, controller_millis=1020)

    update = update_encoder_wheel_speed_estimate(
        baseline,
        current,
        wheel_radius_m=0.10,
        ticks_per_rev=1000,
        filter_alpha=1.0,
        max_abs_speed_mps=2.0,
        min_host_dt_s=0.015,
    )

    assert not update.skipped
    assert update.estimate is not None
    assert update.estimate.dt_source == "controller_millis"
    assert round(update.estimate.dt_s, 3) == 0.020
    assert round(update.estimate.left_mps, 4) == 0.3142


def test_velocity_encoder_min_dt_launch_and_bringup_wiring():
    bridge_file = (
        ROOT
        / "ros2_ws"
        / "src"
        / "ugv_motor_controller"
        / "ugv_motor_controller"
        / "motor_controller_bridge.py"
    ).read_text()
    launch_file = (ROOT / "ros2_ws" / "src" / "ugv_motor_controller" / "launch" / "motor_controller.launch.py").read_text()
    competition_launch = (
        ROOT / "ros2_ws" / "src" / "ugv_sensor_sync" / "launch" / "competition_bringup.launch.py"
    ).read_text()
    bringup = (ROOT / "ros2_ws" / "jetson_bringup.sh").read_text()

    assert "velocity_encoder_speed_min_dt_s" in launch_file
    assert "motor_velocity_encoder_speed_min_dt_s" in competition_launch
    assert "MOTOR_VELOCITY_ENCODER_SPEED_MIN_DT_S" in bringup
    assert "velocity_raw_fallback_floor_enabled" in launch_file
    assert "velocity_raw_fallback_min_wheel_raw" in launch_file
    assert "velocity_raw_fallback_min_target_raw" in launch_file
    assert "motor_velocity_raw_fallback_floor_enabled" in competition_launch
    assert "motor_velocity_raw_fallback_min_wheel_raw" in competition_launch
    assert "motor_velocity_raw_fallback_min_target_raw" in competition_launch
    assert "MOTOR_VELOCITY_RAW_FALLBACK_FLOOR_ENABLED" in bringup
    assert "MOTOR_VELOCITY_RAW_FALLBACK_MIN_WHEEL_RAW" in bringup
    assert "MOTOR_VELOCITY_RAW_FALLBACK_MIN_TARGET_RAW" in bringup
    assert "velocity_raw_fallback_floor_applied_left" in bridge_file
    assert "velocity_raw_fallback_floor_applied_right" in bridge_file
    assert "selected_raw_left" in bridge_file
    assert "selected_raw_right" in bridge_file
    assert "timeout_stop_count" in bridge_file
    assert "command_refresh_count" in bridge_file
    assert "last_motor_send_time_s" in bridge_file
    assert "command_refresh_period_s" in launch_file
    assert "motor_command_timeout_s" in competition_launch
    assert "motor_command_refresh_period_s" in competition_launch
    assert "MOTOR_COMMAND_TIMEOUT_S" in bringup
    assert "MOTOR_COMMAND_REFRESH_PERIOD_S" in bringup


def test_pid_correction_uses_encoder_feedback():
    cfg = VelocityPidConfig(kp=1.0, ki=0.0, kd=0.0, feedforward_raw_per_mps=0.0)
    slow_pid = WheelVelocityPid(cfg)
    fast_pid = WheelVelocityPid(cfg)
    needs_more_pwm = slow_pid.update(target_mps=0.20, measured_mps=0.05, dt_s=0.1)
    needs_less_pwm = fast_pid.update(target_mps=0.20, measured_mps=0.25, dt_s=0.1)
    assert needs_more_pwm.error_mps > 0.0
    assert needs_less_pwm.error_mps < 0.0
    assert needs_more_pwm.output_raw > needs_less_pwm.output_raw


def test_pid_output_sign_and_reset_for_stop():
    pid = WheelVelocityPid(VelocityPidConfig(kp=1.0, kd=0.0, feedforward_raw_per_mps=0.0))
    forward = pid.update(target_mps=0.20, measured_mps=0.05, dt_s=0.1)
    assert forward.output_raw > 0.0
    reverse = pid.update(target_mps=-0.20, measured_mps=-0.05, dt_s=0.1)
    assert reverse.output_raw < 0.0
    pid.reset()
    assert pid.integral == 0.0
    assert pid.prev_error is None
    assert is_stop_command({"mode": "STOP", "command_type": "stop"})


def test_stop_reset_helper_clears_both_pid_integrators():
    left_pid = WheelVelocityPid(VelocityPidConfig(kp=1.0, ki=0.5, kd=0.0, feedforward_raw_per_mps=0.0))
    right_pid = WheelVelocityPid(VelocityPidConfig(kp=1.0, ki=0.5, kd=0.0, feedforward_raw_per_mps=0.0))
    left_pid.update(target_mps=0.20, measured_mps=0.0, dt_s=0.5)
    right_pid.update(target_mps=-0.20, measured_mps=0.0, dt_s=0.5)
    assert left_pid.integral != 0.0
    assert right_pid.integral != 0.0

    reset_velocity_pid_pair(left_pid, right_pid)

    assert left_pid.integral == 0.0
    assert right_pid.integral == 0.0
    assert left_pid.prev_error is None
    assert right_pid.prev_error is None


def test_stale_encoder_detection_defaults_safe():
    assert encoder_speed_is_fresh(10.0, now=10.10, timeout_s=0.25)
    assert not encoder_speed_is_fresh(10.0, now=10.40, timeout_s=0.25)
    assert not encoder_speed_is_fresh(0.0, now=10.0, timeout_s=0.25)


def test_stale_encoder_policy_defaults_to_safe_neutral():
    assert stale_encoder_control_mode(fallback_to_raw_without_encoder=False) == "velocity_safe_neutral"
    assert stale_encoder_control_mode(fallback_to_raw_without_encoder=True) == "velocity_raw_fallback"
