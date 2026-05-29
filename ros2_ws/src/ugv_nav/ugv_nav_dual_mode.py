#!/usr/bin/env python3
"""Safe Jetson chassis controller entrypoint.

Default real mode stays in ``idle`` and publishes STOP. Explicit test modes
publish velocity-only JSON:

    {"command_type": "velocity", "v_mps": <float>, "omega_radps": <float>}
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from ugv_nav_core.chassis_controller import (
    ChassisControllerConfig,
    ChassisEstimatorState,
    GyroBiasCalibrationState,
    PivotControllerState,
    bounded_duration,
    compute_straight_omega,
    evaluate_pivot_clearance,
    evaluate_safety,
    min_finite_range,
    pivot_encoder_gyro_disagreement,
    reset_gyro_bias_calibration,
    reset_pivot,
    start_pivot,
    step_profiled_pivot,
    update_encoder_heading,
    update_gyro_bias_calibration,
    update_gyro_heading,
)


@dataclass(frozen=True)
class ControlCommand:
    command_type: str = "stop"
    v_mps: float = 0.0
    omega_radps: float = 0.0
    reason: str = "idle"
    controller: str = "ugv_chassis_controller"

    def to_json(self) -> str:
        mode = "STOP" if self.command_type == "stop" else "VELOCITY"
        return json.dumps(
            {
                "mode": mode,
                "command_type": self.command_type,
                "controller": self.controller,
                "v_mps": float(self.v_mps),
                "omega_radps": float(self.omega_radps),
                "reason": self.reason,
            },
            sort_keys=True,
        )


def build_stop_command(reason: str = "idle") -> ControlCommand:
    return ControlCommand(command_type="stop", reason=reason)


def build_velocity_command(v_mps: float, omega_radps: float, reason: str) -> ControlCommand:
    return ControlCommand(
        command_type="velocity",
        v_mps=float(v_mps),
        omega_radps=float(omega_radps),
        reason=reason,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe UGV chassis controller. Default mode publishes STOP.",
        allow_abbrev=False,
    )
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--controller-mode", choices=["idle", "straight_test", "pivot_test"], default="idle")
    parser.add_argument("--command-topic", default="/ugv_nav_cmd")
    parser.add_argument("--status-topic", default="/ugv_nav_status")
    parser.add_argument("--nav-frame-topic", default="/sensors/nav_frame")
    parser.add_argument("--imu-topic", default="/zed/imu")
    parser.add_argument("--imu-yaw-axis", choices=["x", "y", "z"], default="z")
    parser.add_argument("--imu-yaw-sign", type=float, default=1.0)
    parser.add_argument("--motor-status-topic", default="/motor_controller/status")
    parser.add_argument("--nav-status-period-s", type=float, default=0.25)
    parser.add_argument("--control-period-s", type=float, default=0.02)
    parser.add_argument("--straight-speed-mps", type=float, default=0.20)
    parser.add_argument("--straight-duration-s", type=float, default=2.0)
    parser.add_argument("--pivot-angle-deg", type=float, default=90.0)
    parser.add_argument("--max-omega-radps", type=float, default=0.45)
    parser.add_argument("--heading-kp", type=float, default=1.2)
    parser.add_argument("--heading-kd", type=float, default=0.15)
    parser.add_argument("--pivot-kp", type=float, default=1.0)
    parser.add_argument("--heading-deadband-rad", type=float, default=0.025)
    parser.add_argument("--stop-clearance-m", type=float, default=0.45)
    parser.add_argument("--sensor-timeout-s", type=float, default=0.30)
    parser.add_argument("--imu-timeout-s", type=float, default=0.12)
    parser.add_argument("--motor-status-timeout-s", type=float, default=0.50)
    parser.add_argument("--max-test-duration-s", type=float, default=3.0)
    parser.add_argument("--gyro-bias-calibration-s", type=float, default=1.5)
    parser.add_argument("--gyro-bias-max-std-radps", type=float, default=0.03)
    parser.add_argument("--gyro-bias-warn-abs-radps", type=float, default=0.10)
    parser.add_argument("--gyro-bias-max-encoder-delta-ticks", type=int, default=2)
    parser.add_argument("--pivot-max-omega-radps", type=float, default=0.35)
    parser.add_argument("--pivot-min-omega-radps", type=float, default=0.16)
    parser.add_argument("--pivot-breakaway-omega-radps", type=float, default=0.18)
    parser.add_argument("--pivot-breakaway-s", type=float, default=0.20)
    parser.add_argument("--pivot-accel-limit-radps2", type=float, default=0.80)
    parser.add_argument("--pivot-decel-limit-radps2", type=float, default=0.60)
    parser.add_argument("--pivot-approach-error-rad", type=float, default=0.25)
    parser.add_argument("--pivot-min-omega-disable-error-rad", type=float, default=0.10)
    parser.add_argument("--pivot-kp-approach", type=float, default=0.75)
    parser.add_argument("--pivot-kd-yaw-rate", type=float, default=0.10)
    parser.add_argument("--pivot-settle-error-rad", type=float, default=0.035)
    parser.add_argument("--pivot-settle-yaw-rate-radps", type=float, default=0.05)
    parser.add_argument("--pivot-settle-time-s", type=float, default=0.35)
    parser.add_argument("--pivot-brake-s", type=float, default=0.15)
    parser.add_argument("--pivot-timeout-s", type=float, default=4.0)
    parser.add_argument("--pivot-max-correction-retries", type=int, default=1)
    parser.add_argument("--pivot-clearance-m", type=float, default=0.35)
    parser.add_argument("--slip-disagreement-rad", type=float, default=0.35)
    parser.add_argument("--track-width-m", type=float, default=0.425)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0825)
    parser.add_argument("--ticks-per-rev", type=float, default=3200.0)
    parser.add_argument("--once", action="store_true", help="publish one command/status cycle and exit in real mode")
    args, unknown = parser.parse_known_args()
    if unknown:
        print(f"Ignoring unsupported navigation arguments: {' '.join(unknown)}")
    return args


def config_from_args(args: argparse.Namespace) -> ChassisControllerConfig:
    return ChassisControllerConfig(
        straight_speed_mps=float(args.straight_speed_mps),
        straight_duration_s=float(args.straight_duration_s),
        pivot_angle_deg=float(args.pivot_angle_deg),
        max_omega_radps=float(args.max_omega_radps),
        heading_kp=float(args.heading_kp),
        heading_kd=float(args.heading_kd),
        pivot_kp=float(args.pivot_kp),
        heading_deadband_rad=float(args.heading_deadband_rad),
        stop_clearance_m=float(args.stop_clearance_m),
        sensor_timeout_s=float(args.sensor_timeout_s),
        imu_timeout_s=float(args.imu_timeout_s),
        motor_status_timeout_s=float(args.motor_status_timeout_s),
        max_test_duration_s=float(args.max_test_duration_s),
        gyro_bias_calibration_s=float(args.gyro_bias_calibration_s),
        gyro_bias_max_std_radps=float(args.gyro_bias_max_std_radps),
        gyro_bias_warn_abs_radps=float(args.gyro_bias_warn_abs_radps),
        gyro_bias_max_encoder_delta_ticks=int(args.gyro_bias_max_encoder_delta_ticks),
        pivot_max_omega_radps=float(args.pivot_max_omega_radps),
        pivot_min_omega_radps=float(args.pivot_min_omega_radps),
        pivot_breakaway_omega_radps=float(args.pivot_breakaway_omega_radps),
        pivot_breakaway_s=float(args.pivot_breakaway_s),
        pivot_accel_limit_radps2=float(args.pivot_accel_limit_radps2),
        pivot_decel_limit_radps2=float(args.pivot_decel_limit_radps2),
        pivot_approach_error_rad=float(args.pivot_approach_error_rad),
        pivot_min_omega_disable_error_rad=float(args.pivot_min_omega_disable_error_rad),
        pivot_kp_approach=float(args.pivot_kp_approach),
        pivot_kd_yaw_rate=float(args.pivot_kd_yaw_rate),
        pivot_settle_error_rad=float(args.pivot_settle_error_rad),
        pivot_settle_yaw_rate_radps=float(args.pivot_settle_yaw_rate_radps),
        pivot_settle_time_s=float(args.pivot_settle_time_s),
        pivot_brake_s=float(args.pivot_brake_s),
        pivot_timeout_s=float(args.pivot_timeout_s),
        pivot_max_correction_retries=int(args.pivot_max_correction_retries),
        pivot_clearance_m=float(args.pivot_clearance_m),
        slip_disagreement_rad=float(args.slip_disagreement_rad),
        track_width_m=float(args.track_width_m),
        wheel_radius_m=float(args.wheel_radius_m),
        ticks_per_rev=float(args.ticks_per_rev),
    )


def run_sim() -> None:
    print("Clean navigation placeholder: safe chassis controller is idle unless a test mode is selected.")


def run_real(args: argparse.Namespace) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        from ugv_sensor_sync.msg import NavSensorFrame
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Python packages are required for real mode: {exc}") from exc

    config = config_from_args(args)

    class ChassisControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("ugv_chassis_controller")
            self.config = config
            self.mode = str(args.controller_mode)
            self.estimator = ChassisEstimatorState()
            self.gyro_bias = GyroBiasCalibrationState()
            self.pivot = PivotControllerState()
            self.last_sensor_s: Optional[float] = None
            self.last_imu_s: Optional[float] = None
            self.last_motor_status_s: Optional[float] = None
            self.motor_status: Optional[Dict[str, Any]] = None
            self.near_obstacle = False
            self.front_clearance_m: Optional[float] = None
            self.pivot_clearance_m: Optional[float] = None
            self.last_scan_ranges: Optional[list[float]] = None
            self.raw_yaw_rate_radps = 0.0
            self.yaw_rate_radps = 0.0
            self.imu_dt_integrated = False
            self.mode_start_s: Optional[float] = None
            self.target_heading_rad: Optional[float] = None
            self.encoder_gyro_disagreement_rad = 0.0
            self.slip_detected = False
            self.last_status_publish_s = 0.0
            self.last_safety_log_s = 0.0
            self.last_safety_reason: Optional[str] = None
            self.last_command_payload: Dict[str, Any] = json.loads(build_stop_command().to_json())

            self.imu_axis = str(args.imu_yaw_axis)
            self.imu_sign = -1.0 if float(args.imu_yaw_sign) < 0.0 else 1.0

            self.cmd_pub = self.create_publisher(String, args.command_topic, 10)
            self.status_pub = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(NavSensorFrame, args.nav_frame_topic, self.nav_frame_callback, 10)
            self.create_subscription(Imu, args.imu_topic, self.imu_callback, 50)
            self.create_subscription(String, args.motor_status_topic, self.motor_status_callback, 10)
            self.timer = self.create_timer(max(0.01, float(args.control_period_s)), self.tick)
            self.get_logger().warn(
                f"Chassis controller started in {self.mode}; only velocity/STOP JSON is published."
            )

        def nav_frame_callback(self, msg: Any) -> None:
            now_s = time.monotonic()
            self.last_sensor_s = now_s
            self.near_obstacle = bool(msg.near_obstacle)
            self.front_clearance_m = float(msg.front_clearance_m)
            self.last_scan_ranges = [float(value) for value in getattr(msg.scan, "ranges", [])]
            self.pivot_clearance_m = min_finite_range(self.last_scan_ranges)
            self.estimator = update_encoder_heading(
                self.estimator,
                left_ticks=int(msg.left_encoder_ticks),
                right_ticks=int(msg.right_encoder_ticks),
                encoder_available=bool(msg.encoder_available),
                config=self.config,
            )

        def imu_callback(self, msg: Any) -> None:
            now_s = time.monotonic()
            stamp_s = self._stamp_to_seconds(msg.header.stamp) or now_s
            self.last_imu_s = now_s
            angular_velocity = msg.angular_velocity
            raw_value = float(getattr(angular_velocity, self.imu_axis))
            self.raw_yaw_rate_radps = self.imu_sign * raw_value
            bias = self.gyro_bias.bias_radps if self.gyro_bias.ready else 0.0
            self.estimator, self.yaw_rate_radps, self.imu_dt_integrated = update_gyro_heading(
                self.estimator,
                stamp_s=stamp_s,
                raw_yaw_rate_radps=self.raw_yaw_rate_radps,
                gyro_bias_radps=bias,
            )

        def motor_status_callback(self, msg: Any) -> None:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f"Ignoring invalid motor status JSON: {exc}")
                return
            if isinstance(payload, dict):
                self.motor_status = payload
                self.last_motor_status_s = time.monotonic()

        @staticmethod
        def _stamp_to_seconds(stamp: Any) -> Optional[float]:
            sec = int(getattr(stamp, "sec", 0))
            nanosec = int(getattr(stamp, "nanosec", 0))
            if sec == 0 and nanosec == 0:
                return None
            return float(sec) + float(nanosec) * 1e-9

        def _start_straight_if_needed(self, now_s: float) -> None:
            if self.mode_start_s is not None:
                return
            self.mode_start_s = now_s
            self.target_heading_rad = self.estimator.heading_rad

        def _start_pivot_if_needed(self, now_s: float) -> None:
            if self.pivot.state != "idle":
                return
            self.mode_start_s = now_s
            start_pivot(
                self.pivot,
                now_s=now_s,
                current_heading_rad=self.estimator.heading_rad,
                encoder_heading_rad=self.estimator.encoder_heading_rad,
                target_angle_rad=math.radians(self.config.pivot_angle_deg),
            )
            self.target_heading_rad = self.pivot.target_heading_rad

        def _reset_test_state(self) -> None:
            self.mode_start_s = None
            self.target_heading_rad = None
            reset_pivot(self.pivot)
            self.encoder_gyro_disagreement_rad = 0.0
            self.slip_detected = False

        def _reset_bias_and_heading(self) -> None:
            reset_gyro_bias_calibration(self.gyro_bias)
            self.estimator.gyro_heading_rad = 0.0
            self.estimator.last_stamp_s = None
            self.estimator.gyro_available = False

        def _calibrate_gyro_bias_if_needed(self, now_s: float) -> tuple[bool, str]:
            if self.gyro_bias.ready:
                return True, "gyro_bias_ready"
            result = update_gyro_bias_calibration(
                self.gyro_bias,
                now_s=now_s,
                raw_yaw_rate_radps=self.raw_yaw_rate_radps,
                left_ticks=self.estimator.last_left_ticks,
                right_ticks=self.estimator.last_right_ticks,
                config=self.config,
            )
            if result.unstable:
                self.get_logger().warn(
                    f"Gyro bias calibration rejected: {result.reason} "
                    f"(bias={result.bias_radps:.5f}, std={result.std_radps:.5f})"
                )
                return False, result.reason
            if result.ready:
                self.estimator.gyro_heading_rad = 0.0
                self.estimator.gyro_available = True
                self.estimator.last_stamp_s = None
                if result.reason == "gyro_bias_large":
                    self.get_logger().warn(
                        f"Gyro bias calibrated but large: {result.bias_radps:.5f} rad/s "
                        f"(std={result.std_radps:.5f})"
                    )
                else:
                    self.get_logger().info(
                        f"Gyro bias calibrated: {result.bias_radps:.5f} rad/s "
                        f"(std={result.std_radps:.5f})"
                    )
                return True, result.reason
            return False, result.reason

        def _evaluate_active_safety(self, now_s: float) -> str:
            safety = evaluate_safety(
                now_s=now_s,
                last_sensor_s=self.last_sensor_s,
                last_imu_s=self.last_imu_s,
                last_motor_status_s=self.last_motor_status_s,
                motor_status=self.motor_status,
                near_obstacle=self.near_obstacle,
                front_clearance_m=self.front_clearance_m,
                config=self.config,
                require_imu=True,
            )
            if not safety.safe:
                return safety.reason
            if self.mode == "pivot_test":
                pivot_safety = evaluate_pivot_clearance(self.last_scan_ranges, self.config)
                if not pivot_safety.safe:
                    return pivot_safety.reason
            return "ok"

        def tick(self) -> None:
            now_s = time.monotonic()
            heading = self.estimator.heading_rad
            heading_error = 0.0
            safety_state = "idle"
            cmd = build_stop_command("idle")

            if self.mode == "idle":
                self._reset_test_state()
            else:
                safety_state = self._evaluate_active_safety(now_s)
                if safety_state != "ok":
                    self._reset_test_state()
                    self._reset_bias_and_heading()
                    self._log_safety_stop(safety_state, now_s)
                    cmd = build_stop_command(safety_state)
                else:
                    calibrated, calibration_reason = self._calibrate_gyro_bias_if_needed(now_s)
                    if not calibrated:
                        safety_state = calibration_reason
                        self._reset_test_state()
                        cmd = build_stop_command(calibration_reason)
                    else:
                        heading = self.estimator.heading_rad
                        if self.mode == "straight_test":
                            self._start_straight_if_needed(now_s)
                            mode_start_s = (
                                float(self.mode_start_s) if self.mode_start_s is not None else float(now_s)
                            )
                            elapsed_s = now_s - mode_start_s
                            max_duration_s = bounded_duration(self.config.max_test_duration_s, self.config)
                            duration_s = bounded_duration(self.config.straight_duration_s, self.config)
                            if elapsed_s >= max_duration_s:
                                cmd = build_stop_command("test_duration_elapsed")
                            elif elapsed_s >= duration_s:
                                cmd = build_stop_command("straight_test_complete")
                            else:
                                omega, heading_error = compute_straight_omega(
                                    target_heading_rad=float(self.target_heading_rad),
                                    heading_rad=heading,
                                    yaw_rate_radps=self.yaw_rate_radps,
                                    config=self.config,
                                )
                                cmd = build_velocity_command(
                                    self.config.straight_speed_mps,
                                    omega,
                                    "straight_heading_hold",
                                )
                        elif self.mode == "pivot_test":
                            self._start_pivot_if_needed(now_s)
                            step = step_profiled_pivot(
                                self.pivot,
                                now_s=now_s,
                                heading_rad=heading,
                                yaw_rate_radps=self.yaw_rate_radps,
                                encoder_heading_rad=self.estimator.encoder_heading_rad,
                                config=self.config,
                            )
                            heading_error = step.heading_error_rad
                            self.target_heading_rad = self.pivot.target_heading_rad
                            self.encoder_gyro_disagreement_rad = pivot_encoder_gyro_disagreement(
                                pivot_start_gyro_heading_rad=self.pivot.start_heading_rad,
                                pivot_start_encoder_heading_rad=self.pivot.start_encoder_heading_rad,
                                current_gyro_heading_rad=self.estimator.gyro_heading_rad,
                                current_encoder_heading_rad=self.estimator.encoder_heading_rad,
                            )
                            self.slip_detected = (
                                self.encoder_gyro_disagreement_rad > self.config.slip_disagreement_rad
                            )
                            if step.command_type == "velocity":
                                cmd = build_velocity_command(0.0, step.omega_radps, step.reason)
                            else:
                                cmd = build_stop_command(step.reason)

            self._publish_command(cmd)
            self._publish_status_if_needed(now_s, heading, heading_error, safety_state)
            if args.once:
                raise KeyboardInterrupt

        def _publish_command(self, cmd: ControlCommand) -> None:
            payload = cmd.to_json()
            self.last_command_payload = json.loads(payload)
            self.cmd_pub.publish(String(data=payload))

        def _log_safety_stop(self, reason: str, now_s: float) -> None:
            if reason == self.last_safety_reason and now_s - self.last_safety_log_s < 1.0:
                return
            self.get_logger().warn(f"Chassis test holding STOP: {reason}")
            self.last_safety_reason = reason
            self.last_safety_log_s = now_s

        def _publish_status_if_needed(
            self,
            now_s: float,
            heading: float,
            heading_error: float,
            safety_state: str,
        ) -> None:
            period_s = max(0.05, float(args.nav_status_period_s))
            if now_s - self.last_status_publish_s < period_s:
                return
            pivot_state = self.pivot.state
            if self.mode == "pivot_test":
                if safety_state in {
                    "gyro_bias_calibration",
                    "gyro_bias_unstable",
                    "gyro_bias_encoder_motion",
                }:
                    pivot_state = "gyro_bias_calibration"
                elif safety_state not in {"ok", "idle"}:
                    pivot_state = "precheck" if self.pivot.state == "idle" else "abort"
            status = {
                "nav_runtime": "chassis_controller",
                "controller_mode": self.mode,
                "active": self.last_command_payload.get("command_type") == "velocity",
                "pivot_state": pivot_state,
                "target_heading_rad": self.target_heading_rad,
                "estimated_heading_rad": heading,
                "gyro_heading_rad": self.estimator.gyro_heading_rad,
                "encoder_heading_rad": self.estimator.encoder_heading_rad,
                "encoder_gyro_disagreement_rad": self.encoder_gyro_disagreement_rad,
                "slip_detected": self.slip_detected,
                "heading_error_rad": heading_error,
                "raw_yaw_rate_radps": self.raw_yaw_rate_radps,
                "yaw_rate_radps": self.yaw_rate_radps,
                "gyro_bias_radps": self.gyro_bias.bias_radps,
                "gyro_bias_std_radps": self.gyro_bias.std_radps,
                "gyro_bias_ready": self.gyro_bias.ready,
                "pivot_retry_count": self.pivot.retry_count,
                "v_mps": self.last_command_payload.get("v_mps", 0.0),
                "omega_radps": self.last_command_payload.get("omega_radps", 0.0),
                "safety_state": safety_state,
                "near_obstacle": self.near_obstacle,
                "front_clearance_m": self.front_clearance_m,
                "pivot_clearance_m": self.pivot_clearance_m,
                "sensor_age_s": None if self.last_sensor_s is None else round(now_s - self.last_sensor_s, 3),
                "imu_age_s": None if self.last_imu_s is None else round(now_s - self.last_imu_s, 3),
                "motor_status_age_s": (
                    None if self.last_motor_status_s is None else round(now_s - self.last_motor_status_s, 3)
                ),
                "last_command": self.last_command_payload,
                "timestamp_s": round(time.time(), 3),
            }
            self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status_publish_s = now_s

    rclpy.init()
    node = ChassisControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    args = parse_args()
    if args.mode == "real":
        run_real(args)
    else:
        run_sim()


if __name__ == "__main__":
    main()
