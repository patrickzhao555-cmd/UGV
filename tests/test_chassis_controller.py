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
    compute_pivot_omega,
    compute_straight_omega,
    encoder_yaw_delta,
    evaluate_safety,
    update_estimator,
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


def test_encoder_yaw_delta_from_left_right_ticks():
    yaw = encoder_yaw_delta(
        0,
        3200,
        wheel_radius_m=0.0825,
        ticks_per_rev=3200,
        track_width_m=0.425,
    )
    assert yaw == pytest.approx((2.0 * math.pi * 0.0825) / 0.425)


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


def test_velocity_command_json_is_velocity_only():
    payload = json.loads(build_velocity_command(0.2, -0.1, "unit_test").to_json())
    assert payload["command_type"] == "velocity"
    assert payload["v_mps"] == pytest.approx(0.2)
    assert payload["omega_radps"] == pytest.approx(-0.1)
    assert "raw_left" not in payload
    assert "raw_right" not in payload
    assert "left_pwm" not in payload
    assert "right_pwm" not in payload
