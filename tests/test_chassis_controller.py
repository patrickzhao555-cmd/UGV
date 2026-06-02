import json
import math
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.chassis_controller import (  # noqa: E402
    ChassisControllerConfig,
    ChassisEstimatorState,
    GyroBiasCalibrationState,
    PivotControllerState,
    compute_pivot_omega,
    compute_profiled_pivot_omega,
    compute_straight_omega,
    encoder_yaw_delta,
    evaluate_pivot_clearance,
    evaluate_safety,
    reset_pivot,
    slew_limit,
    start_pivot,
    step_profiled_pivot,
    update_gyro_bias_calibration,
    update_estimator,
    update_gyro_heading,
    wrap_pi,
)
from ugv_nav_dual_mode import build_velocity_command  # noqa: E402


def _ready_motor_status():
    return {"connected": True, "teensy_pid_params_synced": True, "fault_reason": None}


def test_angle_wrap_handles_pi_boundary():
    assert wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)


def test_gyro_integration_uses_yaw_rate_z():
    config = ChassisControllerConfig()
    state = ChassisEstimatorState()
    update_estimator(
        state,
        stamp_s=10.0,
        yaw_rate_radps=0.0,
        left_ticks=0,
        right_ticks=0,
        encoder_available=False,
        config=config,
    )
    update_estimator(
        state,
        stamp_s=10.5,
        yaw_rate_radps=0.4,
        left_ticks=0,
        right_ticks=0,
        encoder_available=False,
        config=config,
    )
    assert state.heading_rad == pytest.approx(0.08)


def test_high_rate_imu_integration_subtracts_bias_and_rejects_bad_dt():
    state = ChassisEstimatorState()
    update_gyro_heading(
        state,
        stamp_s=1.0,
        raw_yaw_rate_radps=0.5,
        gyro_bias_radps=0.1,
    )
    state, yaw_rate, integrated = update_gyro_heading(
        state,
        stamp_s=1.02,
        raw_yaw_rate_radps=0.5,
        gyro_bias_radps=0.1,
    )
    assert integrated
    assert yaw_rate == pytest.approx(0.4)
    assert state.gyro_heading_rad == pytest.approx(0.008)

    state, _, integrated = update_gyro_heading(
        state,
        stamp_s=1.20,
        raw_yaw_rate_radps=0.5,
        gyro_bias_radps=0.1,
    )
    assert not integrated
    assert state.gyro_heading_rad == pytest.approx(0.008)


def test_encoder_yaw_delta_from_left_right_ticks():
    yaw = encoder_yaw_delta(
        0,
        3200,
        wheel_radius_m=0.0825,
        ticks_per_rev=3200,
        track_width_m=0.416,
    )
    assert yaw == pytest.approx((2.0 * math.pi * 0.0825) / 0.416)


def test_straight_heading_correction_sign():
    config = ChassisControllerConfig(heading_kp=1.2, heading_kd=0.0, max_omega_radps=0.45)
    omega_left, error_left = compute_straight_omega(
        target_heading_rad=0.0,
        heading_rad=0.10,
        yaw_rate_radps=0.0,
        config=config,
    )
    omega_right, error_right = compute_straight_omega(
        target_heading_rad=0.0,
        heading_rad=-0.10,
        yaw_rate_radps=0.0,
        config=config,
    )
    assert error_left < 0.0
    assert omega_left < 0.0
    assert error_right > 0.0
    assert omega_right > 0.0


def test_pivot_left_right_omega_sign():
    config = ChassisControllerConfig(pivot_kp=1.0, max_omega_radps=0.45)
    left_omega, _ = compute_pivot_omega(
        target_heading_rad=math.radians(45.0),
        heading_rad=0.0,
        config=config,
    )
    right_omega, _ = compute_pivot_omega(
        target_heading_rad=math.radians(-45.0),
        heading_rad=0.0,
        config=config,
    )
    assert left_omega > 0.0
    assert right_omega < 0.0


def test_gyro_bias_calibration_accepts_stable_samples():
    config = ChassisControllerConfig(gyro_bias_calibration_s=0.1, gyro_bias_max_std_radps=0.03)
    state = GyroBiasCalibrationState()
    result = None
    for i in range(8):
        result = update_gyro_bias_calibration(
            state,
            now_s=10.0 + i * 0.02,
            raw_yaw_rate_radps=0.021 + (0.001 if i % 2 else 0.0),
            left_ticks=100,
            right_ticks=100,
            config=config,
        )
    assert result is not None
    assert result.ready
    assert not result.unstable
    assert result.bias_radps == pytest.approx(0.0215, abs=0.001)


def test_gyro_bias_calibration_rejects_encoder_motion_and_variance():
    config = ChassisControllerConfig(
        gyro_bias_calibration_s=0.1,
        gyro_bias_max_std_radps=0.01,
        gyro_bias_max_encoder_delta_ticks=2,
    )
    moved_state = GyroBiasCalibrationState()
    result = None
    for i in range(8):
        result = update_gyro_bias_calibration(
            moved_state,
            now_s=10.0 + i * 0.02,
            raw_yaw_rate_radps=0.02,
            left_ticks=100 + i,
            right_ticks=100,
            config=config,
        )
        if result.unstable:
            break
    assert result is not None
    assert result.unstable
    assert result.reason == "gyro_bias_encoder_motion"

    noisy_state = GyroBiasCalibrationState()
    for i, sample in enumerate([0.0, 0.05, -0.04, 0.04, -0.05, 0.04, -0.04, 0.05]):
        result = update_gyro_bias_calibration(
            noisy_state,
            now_s=20.0 + i * 0.02,
            raw_yaw_rate_radps=sample,
            left_ticks=100,
            right_ticks=100,
            config=config,
        )
        if result.unstable:
            break
    assert result is not None
    assert result.unstable
    assert result.reason == "gyro_bias_unstable"


def test_pivot_profile_signs_breakaway_minimum_and_approach():
    config = ChassisControllerConfig(
        max_omega_radps=0.45,
        pivot_max_omega_radps=0.35,
        pivot_min_omega_radps=0.16,
        pivot_breakaway_omega_radps=0.18,
        pivot_kp_approach=0.75,
        pivot_kd_yaw_rate=0.0,
        pivot_accel_limit_radps2=100.0,
    )
    assert compute_profiled_pivot_omega(
        heading_error_rad=0.5,
        yaw_rate_radps=0.0,
        previous_omega_radps=0.0,
        dt_s=0.02,
        state="breakaway",
        config=config,
    ) == pytest.approx(0.18)
    assert compute_profiled_pivot_omega(
        heading_error_rad=-0.5,
        yaw_rate_radps=0.0,
        previous_omega_radps=0.0,
        dt_s=0.02,
        state="breakaway",
        config=config,
    ) == pytest.approx(-0.18)
    assert abs(
        compute_profiled_pivot_omega(
            heading_error_rad=0.18,
            yaw_rate_radps=0.0,
            previous_omega_radps=0.0,
            dt_s=0.02,
            state="rotate",
            config=config,
        )
    ) >= 0.16
    assert compute_profiled_pivot_omega(
        heading_error_rad=0.04,
        yaw_rate_radps=0.0,
        previous_omega_radps=0.0,
        dt_s=0.02,
        state="approach",
        config=config,
    ) == pytest.approx(0.03)


def test_slew_limit_caps_omega_rate_change():
    assert slew_limit(0.0, 1.0, 0.2) == pytest.approx(0.2)
    assert slew_limit(0.5, -1.0, 0.3) == pytest.approx(0.2)


def test_profiled_pivot_state_machine_reaches_complete():
    config = ChassisControllerConfig(
        pivot_angle_deg=30.0,
        pivot_breakaway_s=0.02,
        pivot_brake_s=0.02,
        pivot_settle_time_s=0.04,
        pivot_settle_error_rad=0.05,
        pivot_settle_yaw_rate_radps=0.05,
        pivot_max_correction_retries=0,
    )
    state = PivotControllerState()
    start_pivot(
        state,
        now_s=0.0,
        current_heading_rad=0.0,
        encoder_heading_rad=0.0,
        target_angle_rad=math.radians(30.0),
    )
    result = step_profiled_pivot(
        state,
        now_s=0.01,
        heading_rad=0.0,
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    assert result.command_type == "velocity"
    assert result.omega_radps > 0.0

    step_profiled_pivot(
        state,
        now_s=0.05,
        heading_rad=math.radians(29.0),
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    step_profiled_pivot(
        state,
        now_s=0.08,
        heading_rad=math.radians(30.0),
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    step_profiled_pivot(
        state,
        now_s=0.11,
        heading_rad=math.radians(30.0),
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    result = step_profiled_pivot(
        state,
        now_s=0.16,
        heading_rad=math.radians(30.0),
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    assert result.complete
    assert result.reason == "pivot_test_complete"

    reset_pivot(state)
    assert state.state == "idle"


def test_profiled_pivot_waits_for_settle_and_aborts_if_final_error_persists():
    config = ChassisControllerConfig(
        pivot_settle_time_s=0.20,
        pivot_max_correction_retries=0,
        pivot_timeout_s=3.0,
    )
    state = PivotControllerState(
        state="settle",
        state_start_s=1.0,
        motion_start_s=0.0,
        target_heading_rad=0.5,
        start_heading_rad=0.0,
        last_update_s=1.0,
        last_error_rad=0.5,
    )

    waiting = step_profiled_pivot(
        state,
        now_s=1.05,
        heading_rad=0.0,
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    assert waiting.reason == "pivot_settling"
    assert waiting.state == "settle"
    assert not waiting.complete

    failed = step_profiled_pivot(
        state,
        now_s=1.25,
        heading_rad=0.0,
        yaw_rate_radps=0.0,
        encoder_heading_rad=0.0,
        config=config,
    )
    assert failed.reason == "pivot_settle_failed"
    assert failed.state == "abort"
    assert not failed.complete
    assert state.abort_reason == "pivot_settle_failed"


def test_stale_sensor_stops_chassis_test():
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.0,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(sensor_timeout_s=0.3),
    )
    assert not decision.safe
    assert decision.reason == "sensor_stale"


def test_stale_imu_stops_active_chassis_test_when_required():
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_imu_s=9.0,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(imu_timeout_s=0.12),
        require_imu=True,
    )
    assert not decision.safe
    assert decision.reason == "imu_stale"


def test_motor_fault_stops_chassis_test():
    status = _ready_motor_status()
    status["fault_reason"] = "left_stall"
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=status,
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(),
    )
    assert not decision.safe
    assert decision.reason == "motor_fault:left_stall"


def test_unsynced_or_stale_motor_status_stops_chassis_test():
    unsynced_status = {"connected": True, "teensy_pid_params_synced": False}
    unsynced = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=unsynced_status,
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(),
    )
    stale = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.0,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(motor_status_timeout_s=0.5),
    )
    assert not unsynced.safe
    assert unsynced.reason == "motor_params_not_synced"
    assert not stale.safe
    assert stale.reason == "motor_status_stale"


def test_obstacle_and_low_clearance_stop_chassis_test():
    obstacle = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=True,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(),
    )
    low_clearance = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=0.2,
        config=ChassisControllerConfig(stop_clearance_m=0.45),
    )
    assert not obstacle.safe
    assert obstacle.reason == "near_obstacle"
    assert not low_clearance.safe
    assert low_clearance.reason == "front_clearance_low"


def test_low_pivot_clearance_stops_pivot_test():
    decision = evaluate_pivot_clearance([float("inf"), 0.6, 0.2, float("nan")], ChassisControllerConfig())
    assert not decision.safe
    assert decision.reason == "pivot_clearance_low"


def test_unknown_pivot_clearance_is_unsafe_by_default():
    missing = evaluate_pivot_clearance(None, ChassisControllerConfig())
    empty = evaluate_pivot_clearance([], ChassisControllerConfig())
    all_invalid = evaluate_pivot_clearance([float("inf"), float("nan")], ChassisControllerConfig())
    assert not missing.safe
    assert missing.reason == "pivot_clearance_unknown"
    assert not empty.safe
    assert empty.reason == "pivot_clearance_unknown"
    assert not all_invalid.safe
    assert all_invalid.reason == "pivot_clearance_unknown"


def test_debug_flag_allows_unknown_pivot_clearance():
    decision = evaluate_pivot_clearance(None, ChassisControllerConfig(debug_allow_unknown_pivot_clearance=True))
    assert decision.safe
    assert decision.reason == "ok"


def test_slip_disagreement_is_diagnostic_only():
    from ugv_nav_core.chassis_controller import pivot_encoder_gyro_disagreement

    disagreement = pivot_encoder_gyro_disagreement(
        pivot_start_gyro_heading_rad=0.0,
        pivot_start_encoder_heading_rad=0.0,
        current_gyro_heading_rad=0.2,
        current_encoder_heading_rad=0.7,
    )
    assert disagreement == pytest.approx(0.5)


def test_velocity_command_json_is_velocity_only():
    payload = json.loads(build_velocity_command(0.2, -0.1, "unit_test").to_json())
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.2)
    assert payload["omega_radps"] == pytest.approx(-0.1)
    assert "raw_left" not in payload
    assert "raw_right" not in payload
    assert "left_pwm" not in payload
    assert "right_pwm" not in payload
