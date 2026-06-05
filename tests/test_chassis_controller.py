import json
import math
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ugv_nav"))

from ugv_nav_core.chassis_controller import (  # noqa: E402
    ChassisControllerConfig,
    CurveControllerState,
    ChassisEstimatorState,
    GyroBiasCalibrationState,
    PivotControllerState,
    compute_pivot_omega,
    compute_profiled_pivot_omega,
    compute_straight_omega,
    estimate_curve_timeout_s,
    encoder_yaw_delta,
    evaluate_pivot_clearance,
    evaluate_safety,
    reset_pivot,
    start_curve,
    slew_limit,
    start_pivot,
    step_curve,
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


def test_curve_test_uses_forward_arc_and_completes_on_target_yaw():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.15,
        curve_radius_m=1.0,
        arc_min_turn_radius_m=0.75,
        max_test_duration_s=10.0,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)
    assert state.target_heading_rad == pytest.approx(math.pi / 2.0)
    assert state.arc_length_m == pytest.approx(math.pi / 2.0)

    step = step_curve(state, now_s=0.02, heading_rad=0.0, config=config)
    assert step.command_type == "velocity"
    assert step.v_mps == pytest.approx(0.15)
    assert step.omega_radps > 0.0
    left_mps = step.v_mps - step.omega_radps * config.track_width_m / 2.0
    right_mps = step.v_mps + step.omega_radps * config.track_width_m / 2.0
    assert left_mps > 0.0
    assert right_mps > 0.0

    done = step_curve(state, now_s=1.0, heading_rad=math.radians(90.0), config=config)
    assert done.command_type == "stop"
    assert done.complete
    assert done.reason == "curve_test_complete"


def test_curve_timeout_is_derived_from_motion_when_generic_test_duration_is_short():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.15,
        curve_radius_m=1.0,
        max_test_duration_s=3.0,
        curve_omega_slew_radps2=100.0,
    )
    assert estimate_curve_timeout_s(config) > 10.0
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)
    still_running = step_curve(state, now_s=3.5, heading_rad=0.2, yaw_rate_radps=0.0, config=config)
    assert still_running.command_type == "velocity"
    assert still_running.reason in {"curve_arc", "curve_approach"}


def test_curve_continues_past_nominal_time_when_heading_is_still_progressing():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.42,
        curve_radius_m=1.0,
        arc_max_omega_radps=0.85,
        max_omega_radps=0.85,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)

    still_running = step_curve(state, now_s=8.0, heading_rad=0.4, yaw_rate_radps=0.05, config=config)

    assert still_running.command_type == "velocity"
    assert still_running.reason == "curve_arc"
    assert state.abort_reason is None


def test_curve_aborts_when_heading_makes_no_progress():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.42,
        curve_radius_m=1.0,
        arc_max_omega_radps=0.85,
        max_omega_radps=0.85,
        curve_no_progress_timeout_s=1.5,
        curve_min_progress_rad=0.025,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)
    first = step_curve(state, now_s=0.1, heading_rad=0.0, yaw_rate_radps=0.0, config=config)
    assert first.command_type == "velocity"

    stuck = step_curve(state, now_s=1.6, heading_rad=0.0, yaw_rate_radps=0.0, config=config)

    assert stuck.command_type == "stop"
    assert stuck.reason == "curve_no_yaw_progress"
    assert state.abort_reason == "curve_no_yaw_progress"


def test_curve_explicit_timeout_is_hard_safety_limit():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.42,
        curve_radius_m=1.0,
        curve_timeout_s=0.5,
        curve_no_progress_timeout_s=10.0,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)

    timeout = step_curve(state, now_s=0.6, heading_rad=0.1, yaw_rate_radps=0.1, config=config)

    assert timeout.command_type == "stop"
    assert timeout.reason == "curve_hard_timeout"
    assert state.abort_reason == "curve_hard_timeout"


def test_curve_approach_reduces_omega_near_target():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.15,
        curve_radius_m=1.0,
        curve_approach_error_rad=0.3,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)
    approach = step_curve(state, now_s=0.1, heading_rad=math.radians(85.0), yaw_rate_radps=0.0, config=config)
    assert approach.reason == "curve_approach"
    assert 0.0 < approach.omega_radps < 0.15


def test_curve_approach_applies_minimum_omega_until_close_to_target():
    config = ChassisControllerConfig(
        curve_angle_deg=90.0,
        curve_direction="left",
        curve_speed_mps=0.15,
        curve_radius_m=1.0,
        curve_approach_error_rad=0.3,
        curve_min_omega_radps=0.14,
        curve_min_omega_disable_error_rad=0.08,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)

    approach = step_curve(state, now_s=0.1, heading_rad=math.radians(85.0), yaw_rate_radps=0.0, config=config)

    assert approach.reason == "curve_approach"
    assert approach.omega_radps == pytest.approx(0.14)


def test_curve_test_right_turn_has_negative_omega():
    config = ChassisControllerConfig(
        curve_angle_deg=45.0,
        curve_direction="right",
        curve_speed_mps=0.15,
        curve_radius_m=1.0,
        curve_omega_slew_radps2=100.0,
    )
    state = CurveControllerState()
    start_curve(state, now_s=0.0, current_heading_rad=0.0, encoder_heading_rad=0.0, config=config)
    step = step_curve(state, now_s=0.02, heading_rad=0.0, config=config)
    assert step.command_type == "velocity"
    assert step.omega_radps < 0.0


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


def test_debug_ignore_nav_frame_allows_imu_closed_loop_test_without_fusion_frame():
    decision = evaluate_safety(
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
    )
    assert decision.safe
    assert decision.reason == "ok"


def test_encoder_heading_fallback_does_not_require_imu():
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=None,
        last_imu_s=None,
        last_motor_status_s=9.99,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=None,
        config=ChassisControllerConfig(
            debug_ignore_nav_frame=True,
            debug_ignore_obstacles=True,
        ),
        require_imu=False,
    )
    assert decision.safe
    assert decision.reason == "ok"


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


def test_low_imu_rate_stops_active_chassis_test_when_required():
    low_rate = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_imu_s=9.99,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(imu_timeout_s=0.30, imu_min_rate_hz=20.0),
        require_imu=True,
        imu_rate_hz=5.0,
    )
    assert not low_rate.safe
    assert low_rate.reason == "imu_rate_low"

    healthy = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_imu_s=9.99,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(imu_timeout_s=0.30, imu_min_rate_hz=20.0),
        require_imu=True,
        imu_rate_hz=100.0,
    )
    assert healthy.safe
    assert healthy.reason == "ok"


def test_encoder_feedback_does_not_bypass_mandatory_imu_health_gate():
    status = _ready_motor_status()
    status["encoder_ticks"] = [123, 456]
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_imu_s=None,
        last_motor_status_s=9.9,
        motor_status=status,
        near_obstacle=False,
        front_clearance_m=1.0,
        config=ChassisControllerConfig(imu_timeout_s=0.30),
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


def test_debug_ignore_obstacles_bypasses_clearance_only_after_motor_is_ready():
    decision = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=_ready_motor_status(),
        near_obstacle=True,
        front_clearance_m=0.0,
        config=ChassisControllerConfig(debug_ignore_obstacles=True),
    )
    assert decision.safe
    assert decision.reason == "ok"

    motor_fault = _ready_motor_status()
    motor_fault["fault_reason"] = "right_stall"
    blocked = evaluate_safety(
        now_s=10.0,
        last_sensor_s=9.9,
        last_motor_status_s=9.9,
        motor_status=motor_fault,
        near_obstacle=True,
        front_clearance_m=0.0,
        config=ChassisControllerConfig(debug_ignore_obstacles=True),
    )
    assert not blocked.safe
    assert blocked.reason == "motor_fault:right_stall"


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
