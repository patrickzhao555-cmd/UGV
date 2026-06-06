#!/usr/bin/env python3
"""Standalone Challenge 3 corridor bypass controller.

This node deliberately stays separate from the C1/C2 controller.  It builds a
dead-reckoned field pose from encoder distance plus IMU yaw, tracks the
start-to-target baseline, and performs early large-radius lane changes when the
route corridor is occupied.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Optional, Sequence

import rclpy
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from ugv_sensor_sync.msg import EncoderTicksStamped

from ugv_nav_core.challenge3_corridor import (
    CHALLENGE3_DEFAULT_LANE_OFFSETS_M,
    CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS,
    CHALLENGE3_TURN_ENVELOPE_SPEED_MPS,
    Challenge3CorridorConfig,
    Challenge3CorridorState,
    Challenge3Step,
    FieldPose,
    RouteFrame,
    RouteObservation,
    integrate_encoder_imu_pose,
    make_route_frame,
    scan_ranges_to_route_observations,
    step_corridor_controller,
)
from ugv_nav_core.chassis_controller import (
    GyroBiasCalibrationState,
    gyro_bias_sample_std,
    motor_status_ready,
    reset_gyro_bias_calibration,
    wrap_pi,
)
from ugv_nav_core.nav2_bridge import VelocityCommand, build_stop_command


def _parse_float_list(value: Any, *, default: Sequence[float]) -> tuple[float, ...]:
    if value is None:
        return tuple(float(v) for v in default)
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",") if part.strip()]
    elif isinstance(value, (list, tuple)):
        parts = list(value)
    else:
        return tuple(float(v) for v in default)
    parsed: list[float] = []
    for part in parts:
        try:
            number = float(part)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number) and not any(abs(number - existing) < 1e-9 for existing in parsed):
            parsed.append(number)
    return tuple(parsed) if parsed else tuple(float(v) for v in default)


def _normalize_imu_yaw_axis(value: Any) -> str:
    # ROS 2 parameter YAML treats unquoted "y" as boolean true. Keep C3 alive
    # even if a launch file or manual command forgets to quote the axis.
    if isinstance(value, bool):
        return "y" if value else "z"
    axis = str(value).strip().lower()
    return axis if axis in {"x", "y", "z"} else "y"


def encoder_ticks_from_motor_status(payload: Any) -> Optional[tuple[int, int]]:
    if not isinstance(payload, dict):
        return None
    ticks = payload.get("encoder_ticks")
    if isinstance(ticks, (list, tuple)) and len(ticks) >= 2:
        try:
            return int(ticks[0]), int(ticks[1])
        except (TypeError, ValueError):
            return None
    if "left_ticks" in payload and "right_ticks" in payload:
        try:
            return int(payload["left_ticks"]), int(payload["right_ticks"])
        except (TypeError, ValueError):
            return None
    return None


class Challenge3CorridorNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_challenge3_corridor")

        self.declare_parameter("command_topic", "/ugv_nav_cmd")
        self.declare_parameter("status_topic", "/ugv_nav_status")
        self.declare_parameter("target_topic", "/ugv/uav_target")
        self.declare_parameter("encoder_topic", "/encoder_ticks_stamped")
        self.declare_parameter("imu_topic", "/zed/imu")
        self.declare_parameter("scan_topic", "/scan/filtered")
        self.declare_parameter("motor_status_topic", "/motor_controller/status")
        self.declare_parameter("uav_landed_topic", "/ugv/uav_landed")
        self.declare_parameter("control_period_s", 0.02)
        self.declare_parameter("status_period_s", 0.25)
        self.declare_parameter("start_pose_set", False)
        self.declare_parameter("start_x_m", 0.0)
        self.declare_parameter("start_y_m", 0.0)
        self.declare_parameter("start_yaw_deg", 0.0)
        self.declare_parameter(
            "imu_yaw_axis",
            "y",
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("imu_yaw_sign", -1.0)
        self.declare_parameter("imu_timeout_s", 0.30)
        self.declare_parameter("imu_min_rate_hz", 20.0)
        self.declare_parameter("imu_rate_window_s", 2.0)
        self.declare_parameter("encoder_timeout_s", 0.50)
        self.declare_parameter("scan_timeout_s", 0.60)
        self.declare_parameter("motor_status_timeout_s", 0.60)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("gyro_bias_calibration_s", 1.5)
        self.declare_parameter("gyro_bias_max_std_radps", 0.03)
        self.declare_parameter("gyro_bias_max_encoder_delta_ticks", 2)
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("field_margin_m", 0.45)
        self.declare_parameter("lane_offsets_m", "0.0,1.6,-1.6,2.2,-2.2")
        self.declare_parameter("obstacle_lookahead_m", 5.5)
        self.declare_parameter("route_corridor_half_width_m", 0.70)
        self.declare_parameter("emergency_stop_m", 0.65)
        self.declare_parameter("obstacle_memory_ttl_s", 8.0)
        self.declare_parameter("obstacle_passed_behind_m", 1.0)
        self.declare_parameter("lidar_min_cluster_points", 3)
        self.declare_parameter("lidar_cluster_max_gap_m", 0.35)
        self.declare_parameter("lane_change_distance_m", 5.0)
        self.declare_parameter("rejoin_distance_m", 6.0)
        self.declare_parameter("lookahead_m", 2.4)
        self.declare_parameter("cruise_speed_mps", 0.24)
        self.declare_parameter("hard_turn_speed_mps", CHALLENGE3_TURN_ENVELOPE_SPEED_MPS)
        self.declare_parameter("hard_turn_max_omega_radps", CHALLENGE3_TURN_ENVELOPE_OMEGA_RADPS)
        self.declare_parameter("hard_turn_error_rad", 0.35)
        self.declare_parameter("cruise_heading_kp", 0.85)
        self.declare_parameter("cruise_max_omega_radps", 0.85)
        self.declare_parameter("track_width_m", 0.416)
        self.declare_parameter("wheel_radius_m", 0.0825)
        self.declare_parameter("ticks_per_rev", 3200.0)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.encoder_topic = str(self.get_parameter("encoder_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.motor_status_topic = str(self.get_parameter("motor_status_topic").value)
        self.uav_landed_topic = str(self.get_parameter("uav_landed_topic").value)
        self.control_period_s = max(0.01, float(self.get_parameter("control_period_s").value))
        self.status_period_s = max(0.05, float(self.get_parameter("status_period_s").value))
        self.start_pose_set = bool(self.get_parameter("start_pose_set").value)
        self.start_pose = FieldPose(
            x_m=float(self.get_parameter("start_x_m").value),
            y_m=float(self.get_parameter("start_y_m").value),
            yaw_rad=math.radians(float(self.get_parameter("start_yaw_deg").value)),
        )
        self.pose = self.start_pose
        self.previous_pose_yaw_rad = self.pose.yaw_rad
        self.field_yaw_rad = self.pose.yaw_rad
        self.imu_axis = _normalize_imu_yaw_axis(self.get_parameter("imu_yaw_axis").value)
        self.imu_sign = float(self.get_parameter("imu_yaw_sign").value)
        self.imu_timeout_s = max(0.0, float(self.get_parameter("imu_timeout_s").value))
        self.imu_min_rate_hz = max(0.0, float(self.get_parameter("imu_min_rate_hz").value))
        self.imu_rate_window_s = max(0.1, float(self.get_parameter("imu_rate_window_s").value))
        self.encoder_timeout_s = max(0.0, float(self.get_parameter("encoder_timeout_s").value))
        self.scan_timeout_s = max(0.0, float(self.get_parameter("scan_timeout_s").value))
        self.motor_status_timeout_s = max(0.0, float(self.get_parameter("motor_status_timeout_s").value))
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.gyro_bias_calibration_s = max(0.0, float(self.get_parameter("gyro_bias_calibration_s").value))
        self.gyro_bias_max_std_radps = max(0.0, float(self.get_parameter("gyro_bias_max_std_radps").value))
        self.gyro_bias_max_encoder_delta_ticks = max(
            0,
            int(self.get_parameter("gyro_bias_max_encoder_delta_ticks").value),
        )

        lane_offsets = _parse_float_list(
            self.get_parameter("lane_offsets_m").value,
            default=CHALLENGE3_DEFAULT_LANE_OFFSETS_M,
        )
        self.config = Challenge3CorridorConfig(
            field_width_m=float(self.get_parameter("field_width_m").value),
            field_height_m=float(self.get_parameter("field_height_m").value),
            field_margin_m=float(self.get_parameter("field_margin_m").value),
            lane_offsets_m=lane_offsets,
            obstacle_lookahead_m=float(self.get_parameter("obstacle_lookahead_m").value),
            route_corridor_half_width_m=float(self.get_parameter("route_corridor_half_width_m").value),
            emergency_stop_m=float(self.get_parameter("emergency_stop_m").value),
            obstacle_memory_ttl_s=float(self.get_parameter("obstacle_memory_ttl_s").value),
            obstacle_passed_behind_m=float(self.get_parameter("obstacle_passed_behind_m").value),
            lidar_min_cluster_points=max(1, int(self.get_parameter("lidar_min_cluster_points").value)),
            lidar_cluster_max_gap_m=max(0.0, float(self.get_parameter("lidar_cluster_max_gap_m").value)),
            lane_change_distance_m=float(self.get_parameter("lane_change_distance_m").value),
            rejoin_distance_m=float(self.get_parameter("rejoin_distance_m").value),
            lookahead_m=float(self.get_parameter("lookahead_m").value),
            cruise_speed_mps=float(self.get_parameter("cruise_speed_mps").value),
            hard_turn_speed_mps=float(self.get_parameter("hard_turn_speed_mps").value),
            hard_turn_error_rad=float(self.get_parameter("hard_turn_error_rad").value),
            cruise_heading_kp=float(self.get_parameter("cruise_heading_kp").value),
            cruise_max_omega_radps=float(self.get_parameter("cruise_max_omega_radps").value),
            hard_turn_max_omega_radps=float(self.get_parameter("hard_turn_max_omega_radps").value),
            track_width_m=float(self.get_parameter("track_width_m").value),
            wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
            ticks_per_rev=float(self.get_parameter("ticks_per_rev").value),
        )

        self.state = Challenge3CorridorState(mode="GYRO_BIAS_CALIBRATION")
        self.route: Optional[RouteFrame] = None
        self.target: Optional[tuple[float, float]] = None
        self.latest_step: Optional[Challenge3Step] = None
        self.last_command: VelocityCommand = build_stop_command("startup", controller="challenge3_corridor")
        self.last_command_s: Optional[float] = None

        self.gyro_bias = GyroBiasCalibrationState()
        self.raw_yaw_rate_radps = 0.0
        self.yaw_rate_radps = 0.0
        self.last_imu_s: Optional[float] = None
        self.last_imu_stamp_s: Optional[float] = None
        self.imu_arrivals_s: deque[float] = deque()

        self.current_left_ticks: Optional[int] = None
        self.current_right_ticks: Optional[int] = None
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_encoder_s: Optional[float] = None
        self.gyro_bias_start_left_ticks: Optional[int] = None
        self.gyro_bias_start_right_ticks: Optional[int] = None

        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_s: Optional[float] = None
        self.latest_observations: list[RouteObservation] = []

        self.motor_status: Optional[dict[str, Any]] = None
        self.last_motor_status_s: Optional[float] = None
        self.uav_landed = False
        self.last_uav_landed_s: Optional[float] = None

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PointStamped, self.target_topic, self.target_callback, 10)
        self.create_subscription(EncoderTicksStamped, self.encoder_topic, self.encoder_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.motor_status_topic, self.motor_status_callback, 10)
        self.create_subscription(Bool, self.uav_landed_topic, self.uav_landed_callback, 10)
        self.create_timer(self.control_period_s, self.control_timer)
        self.create_timer(self.status_period_s, self.publish_status)

        self.get_logger().warn(
            "Challenge 3 corridor controller started "
            f"(target={self.target_topic}, encoder={self.encoder_topic}, imu={self.imu_topic}, "
            f"scan={self.scan_topic}, start_pose_set={self.start_pose_set}, "
            f"hard_arc={self.config.hard_turn_speed_mps:.2f}/{self.config.hard_turn_max_omega_radps:.2f}, "
            f"lanes={self.config.lane_offsets_m})"
        )

    def _now_s(self) -> float:
        return time.monotonic()

    @staticmethod
    def _stamp_to_seconds(stamp: Any) -> Optional[float]:
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None
        return float(sec) + float(nanosec) * 1e-9

    def target_callback(self, msg: PointStamped) -> None:
        x_m = float(msg.point.x)
        y_m = float(msg.point.y)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            self.get_logger().warn(f"Ignoring non-finite Challenge 3 target: x={x_m} y={y_m}")
            return
        try:
            route = make_route_frame(
                start_x_m=self.start_pose.x_m,
                start_y_m=self.start_pose.y_m,
                target_x_m=x_m,
                target_y_m=y_m,
            )
        except ValueError as exc:
            self.get_logger().warn(f"Ignoring invalid Challenge 3 target: {exc}")
            return
        self.target = (x_m, y_m)
        self.route = route
        self.state = Challenge3CorridorState(mode="TRACK_BASELINE")
        self.latest_step = None
        self.get_logger().warn(
            f"Challenge 3 target accepted: field_target=({x_m:.3f},{y_m:.3f})m "
            f"route_length={route.length_m:.3f}m"
        )

    def uav_landed_callback(self, msg: Bool) -> None:
        self.uav_landed = bool(msg.data)
        self.last_uav_landed_s = self._now_s()

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_s = self._now_s()

    def motor_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid motor status JSON: {exc}")
            return
        if isinstance(payload, dict):
            self.motor_status = payload
            self.last_motor_status_s = self._now_s()
            ticks = encoder_ticks_from_motor_status(payload)
            if ticks is not None:
                self._update_encoder_ticks(left_ticks=ticks[0], right_ticks=ticks[1], now_s=self.last_motor_status_s)

    def encoder_callback(self, msg: EncoderTicksStamped) -> None:
        self._update_encoder_ticks(
            left_ticks=int(msg.left_ticks),
            right_ticks=int(msg.right_ticks),
            now_s=self._now_s(),
        )

    def imu_callback(self, msg: Imu) -> None:
        now_s = self._now_s()
        stamp_s = self._stamp_to_seconds(msg.header.stamp) or now_s
        angular_velocity = msg.angular_velocity
        raw_value = float(getattr(angular_velocity, self.imu_axis))
        self.raw_yaw_rate_radps = self.imu_sign * raw_value
        self.last_imu_s = now_s
        self.imu_arrivals_s.append(now_s)
        self._trim_imu_arrivals(now_s)

        if not self.gyro_bias.ready:
            self._update_gyro_bias(now_s)
            self.last_imu_stamp_s = stamp_s
            return

        if self.last_imu_stamp_s is None:
            self.last_imu_stamp_s = stamp_s
            return

        dt_s = stamp_s - self.last_imu_stamp_s
        self.last_imu_stamp_s = stamp_s
        if not math.isfinite(dt_s) or dt_s < 0.001 or dt_s > 0.05:
            return
        self.yaw_rate_radps = self.raw_yaw_rate_radps - self.gyro_bias.bias_radps
        self.field_yaw_rad = wrap_pi(self.field_yaw_rad + self.yaw_rate_radps * dt_s)
        self.pose = FieldPose(self.pose.x_m, self.pose.y_m, self.field_yaw_rad)

    def _update_gyro_bias(self, now_s: float) -> None:
        if self.gyro_bias_calibration_s <= 0.0:
            self.gyro_bias.ready = True
            self.gyro_bias.bias_radps = 0.0
            self.gyro_bias.std_radps = 0.0
            return
        if self.gyro_bias.start_s is None:
            self.gyro_bias.start_s = now_s
            self.gyro_bias.samples.clear()
            self.gyro_bias_start_left_ticks = self.current_left_ticks
            self.gyro_bias_start_right_ticks = self.current_right_ticks
            self.state.mode = "GYRO_BIAS_CALIBRATION"
        self.gyro_bias.samples.append(self.raw_yaw_rate_radps)
        if now_s - self.gyro_bias.start_s < self.gyro_bias_calibration_s:
            return

        mean = sum(self.gyro_bias.samples) / float(max(1, len(self.gyro_bias.samples)))
        std = gyro_bias_sample_std(self.gyro_bias.samples)
        moved = False
        if (
            self.current_left_ticks is not None
            and self.current_right_ticks is not None
            and self.gyro_bias_start_left_ticks is not None
            and self.gyro_bias_start_right_ticks is not None
        ):
            moved = max(
                abs(self.current_left_ticks - self.gyro_bias_start_left_ticks),
                abs(self.current_right_ticks - self.gyro_bias_start_right_ticks),
            ) > self.gyro_bias_max_encoder_delta_ticks
        if moved or std > self.gyro_bias_max_std_radps:
            reason = "encoder moved" if moved else f"std={std:.5f}"
            self.get_logger().warn(f"Challenge 3 gyro bias retry: {reason}")
            reset_gyro_bias_calibration(self.gyro_bias)
            self.gyro_bias_start_left_ticks = self.current_left_ticks
            self.gyro_bias_start_right_ticks = self.current_right_ticks
            return

        self.gyro_bias.bias_radps = mean
        self.gyro_bias.std_radps = std
        self.gyro_bias.ready = True
        self.get_logger().info(f"Challenge 3 gyro bias calibrated: {mean:.5f} rad/s (std={std:.5f})")

    def _update_encoder_ticks(self, *, left_ticks: int, right_ticks: int, now_s: float) -> None:
        self.current_left_ticks = int(left_ticks)
        self.current_right_ticks = int(right_ticks)
        self.last_encoder_s = float(now_s)
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = self.current_left_ticks
            self.last_right_ticks = self.current_right_ticks
            return
        left_delta = self.current_left_ticks - self.last_left_ticks
        right_delta = self.current_right_ticks - self.last_right_ticks
        self.last_left_ticks = self.current_left_ticks
        self.last_right_ticks = self.current_right_ticks
        self.pose = integrate_encoder_imu_pose(
            self.pose,
            left_delta_ticks=left_delta,
            right_delta_ticks=right_delta,
            yaw_rad=self.field_yaw_rad,
            previous_yaw_rad=self.previous_pose_yaw_rad,
            wheel_radius_m=self.config.wheel_radius_m,
            ticks_per_rev=self.config.ticks_per_rev,
        )
        self.previous_pose_yaw_rad = self.field_yaw_rad

    def _trim_imu_arrivals(self, now_s: float) -> None:
        while self.imu_arrivals_s and now_s - self.imu_arrivals_s[0] > self.imu_rate_window_s:
            self.imu_arrivals_s.popleft()

    def _imu_rate_hz(self) -> float:
        if len(self.imu_arrivals_s) < 2:
            return 0.0
        elapsed_s = self.imu_arrivals_s[-1] - self.imu_arrivals_s[0]
        if elapsed_s <= 0.0:
            return 0.0
        return float(len(self.imu_arrivals_s) - 1) / elapsed_s

    def _safety_stop_reason(self, now_s: float) -> Optional[str]:
        if not self.start_pose_set:
            return "challenge3_start_pose_missing"
        if not self.gyro_bias.ready:
            return "gyro_bias_calibration"
        if self.last_imu_s is None or now_s - self.last_imu_s > self.imu_timeout_s:
            return "imu_stale"
        if self.imu_min_rate_hz > 0.0 and self._imu_rate_hz() < self.imu_min_rate_hz:
            return "imu_rate_low"
        if self.last_encoder_s is None or now_s - self.last_encoder_s > self.encoder_timeout_s:
            return "encoder_stale"
        if self.require_scan and (self.last_scan_s is None or now_s - self.last_scan_s > self.scan_timeout_s):
            return "scan_stale"
        if self.last_motor_status_s is None or now_s - self.last_motor_status_s > self.motor_status_timeout_s:
            return "motor_status_stale"
        motor = motor_status_ready(self.motor_status)
        if not motor.safe:
            return motor.reason
        return None

    def _build_observations(self) -> list[RouteObservation]:
        if self.latest_scan is None or self.route is None:
            return []
        return scan_ranges_to_route_observations(
            ranges=list(self.latest_scan.ranges),
            angle_min_rad=float(self.latest_scan.angle_min),
            angle_increment_rad=float(self.latest_scan.angle_increment),
            range_min_m=float(self.latest_scan.range_min),
            range_max_m=float(self.latest_scan.range_max),
            pose=self.pose,
            route=self.route,
            max_range_m=self.config.obstacle_lookahead_m,
            min_cluster_points=self.config.lidar_min_cluster_points,
            cluster_max_gap_m=self.config.lidar_cluster_max_gap_m,
        )

    def control_timer(self) -> None:
        now_s = self._now_s()
        safety_reason = self._safety_stop_reason(now_s)
        if safety_reason is not None:
            if safety_reason == "gyro_bias_calibration":
                self.state.mode = "GYRO_BIAS_CALIBRATION"
            elif self.route is None:
                self.state.mode = "WAIT_TARGET"
            else:
                self.state.mode = "FAULT"
                self.state.fault_reason = safety_reason
            self._publish_command(build_stop_command(safety_reason, controller="challenge3_corridor"))
            return

        if self.route is None:
            self.state.mode = "WAIT_TARGET"
            self._publish_command(build_stop_command("challenge3_wait_target", controller="challenge3_corridor"))
            return

        self.latest_observations = self._build_observations()
        step = step_corridor_controller(
            self.state,
            route=self.route,
            pose=self.pose,
            now_s=now_s,
            observations=self.latest_observations,
            config=self.config,
        )
        self.latest_step = step
        self._publish_command(step.command)

    def _publish_command(self, command: VelocityCommand) -> None:
        self.last_command = command
        self.last_command_s = self._now_s()
        self.command_pub.publish(String(data=command.to_json()))

    def publish_status(self) -> None:
        now_s = self._now_s()
        step = self.latest_step
        route_point = step.route_point if step is not None else self.state.last_route_point
        last_command_payload = self.last_command.to_payload()
        status = {
            "controller": "challenge3_corridor",
            "mission_state": "challenge3",
            "safety_level": "ok" if self.last_command.command_type == "velocity" else "critical",
            "safety_reason": self.last_command.reason,
            "last_command": last_command_payload,
            "cmd": last_command_payload,
            "v_mps": self.last_command.v_mps,
            "omega_radps": self.last_command.omega_radps,
            "heading_error_rad": None if step is None else step.heading_error_rad,
            "heading_source": "imu",
            "yaw_rate_radps": self.yaw_rate_radps,
            "raw_yaw_rate_radps": self.raw_yaw_rate_radps,
            "gyro_bias_radps": self.gyro_bias.bias_radps,
            "gyro_bias_std_radps": self.gyro_bias.std_radps,
            "gyro_bias_ready": self.gyro_bias.ready,
            "imu_rate_hz": round(self._imu_rate_hz(), 3),
            "imu_age_s": None if self.last_imu_s is None else round(max(0.0, now_s - self.last_imu_s), 3),
            "encoder_age_s": None
            if self.last_encoder_s is None
            else round(max(0.0, now_s - self.last_encoder_s), 3),
            "scan_age_s": None if self.last_scan_s is None else round(max(0.0, now_s - self.last_scan_s), 3),
            "motor_status_age_s": None
            if self.last_motor_status_s is None
            else round(max(0.0, now_s - self.last_motor_status_s), 3),
            "challenge3_state": self.state.mode,
            "challenge3_pose_m": [round(self.pose.x_m, 4), round(self.pose.y_m, 4), round(self.pose.yaw_rad, 5)],
            "challenge3_start_pose_m": [
                round(self.start_pose.x_m, 4),
                round(self.start_pose.y_m, 4),
                round(self.start_pose.yaw_rad, 5),
            ],
            "challenge3_target_m": None
            if self.target is None
            else [round(self.target[0], 4), round(self.target[1], 4)],
            "challenge3_route_uv_m": None
            if route_point is None
            else [round(route_point.u_m, 4), round(route_point.v_m, 4)],
            "challenge3_target_distance_m": None if step is None else round(step.target_distance_m or 0.0, 4),
            "challenge3_lane_offset_m": round(self.state.last_lane_offset_m, 4),
            "challenge3_desired_lane_offset_m": round(self.state.desired_lane_offset_m, 4),
            "challenge3_obstacle_memory_count": len(self.state.obstacles),
            "challenge3_scan_observation_count": len(self.latest_observations),
            "challenge3_lidar_min_cluster_points": self.config.lidar_min_cluster_points,
            "challenge3_lidar_cluster_max_gap_m": round(float(self.config.lidar_cluster_max_gap_m), 3),
            "challenge3_result_reason": self.last_command.reason,
            "challenge3_target_left_mps": None if step is None else round(step.left_mps, 4),
            "challenge3_target_right_mps": None if step is None else round(step.right_mps, 4),
            "challenge3_hard_turn_speed_mps": self.config.hard_turn_speed_mps,
            "challenge3_hard_turn_max_omega_radps": self.config.hard_turn_max_omega_radps,
            "challenge3_lane_offsets_m": list(self.config.lane_offsets_m),
            "uav_landed": self.uav_landed,
            "uav_landed_age_s": None
            if self.last_uav_landed_s is None
            else round(max(0.0, now_s - self.last_uav_landed_s), 3),
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = Challenge3CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_command(build_stop_command("shutdown", controller="challenge3_corridor"))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
