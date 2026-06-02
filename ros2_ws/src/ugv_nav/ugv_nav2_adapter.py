#!/usr/bin/env python3
"""Bridge Nav2 /cmd_vel into the project's /ugv_nav_cmd JSON contract."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from ugv_sensor_sync.msg import NavSensorFrame

from ugv_nav_core.nav2_bridge import (
    FieldBounds,
    VelocityCommand,
    build_stop_command,
    field_boundary_decision,
    nav_command_from_twist,
)
from ugv_nav_core.safety_status import motor_fault_reason


class Nav2AdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_nav2_adapter")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_topic", "/ugv_nav_cmd")
        self.declare_parameter("status_topic", "/ugv_nav2_adapter/status")
        self.declare_parameter("nav_frame_topic", "/sensors/nav_frame")
        self.declare_parameter("imu_topic", "/zed/imu")
        self.declare_parameter("motor_status_topic", "/motor_controller/status")
        self.declare_parameter("localization_status_topic", "/ugv_localization/status")
        self.declare_parameter("kill_switch_topic", "/ugv/kill_switch")
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("cmd_vel_timeout_s", 0.35)
        self.declare_parameter("sensor_timeout_s", 0.40)
        self.declare_parameter("imu_timeout_s", 0.15)
        self.declare_parameter("motor_status_timeout_s", 0.60)
        self.declare_parameter("localization_timeout_s", 0.60)
        self.declare_parameter("require_imu", True)
        self.declare_parameter("require_nav_frame", True)
        self.declare_parameter("require_localization_ready", True)
        self.declare_parameter("stop_on_near_obstacle", True)
        self.declare_parameter("emergency_stop_clearance_m", 0.18)
        self.declare_parameter("competition_min_speed_mps", 0.089408)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("field_boundary_margin_m", 0.45)
        self.declare_parameter("field_boundary_prediction_time_s", 0.75)
        self.declare_parameter("require_field_bounds", True)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.nav_frame_topic = str(self.get_parameter("nav_frame_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.motor_status_topic = str(self.get_parameter("motor_status_topic").value)
        self.localization_status_topic = str(self.get_parameter("localization_status_topic").value)
        self.kill_switch_topic = str(self.get_parameter("kill_switch_topic").value)
        self.publish_period_s = max(0.01, float(self.get_parameter("publish_period_s").value))
        self.cmd_vel_timeout_s = max(0.0, float(self.get_parameter("cmd_vel_timeout_s").value))
        self.sensor_timeout_s = max(0.0, float(self.get_parameter("sensor_timeout_s").value))
        self.imu_timeout_s = max(0.0, float(self.get_parameter("imu_timeout_s").value))
        self.motor_status_timeout_s = max(0.0, float(self.get_parameter("motor_status_timeout_s").value))
        self.localization_timeout_s = max(0.0, float(self.get_parameter("localization_timeout_s").value))
        self.require_imu = bool(self.get_parameter("require_imu").value)
        self.require_nav_frame = bool(self.get_parameter("require_nav_frame").value)
        self.require_localization_ready = bool(self.get_parameter("require_localization_ready").value)
        self.stop_on_near_obstacle = bool(self.get_parameter("stop_on_near_obstacle").value)
        self.emergency_stop_clearance_m = float(self.get_parameter("emergency_stop_clearance_m").value)
        self.competition_min_speed_mps = float(self.get_parameter("competition_min_speed_mps").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.field_bounds = FieldBounds(
            width_m=float(self.get_parameter("field_width_m").value),
            height_m=float(self.get_parameter("field_height_m").value),
            margin_m=0.0,
        )
        self.field_boundary_margin_m = max(0.0, float(self.get_parameter("field_boundary_margin_m").value))
        self.field_boundary_prediction_time_s = max(
            0.0,
            float(self.get_parameter("field_boundary_prediction_time_s").value),
        )
        self.require_field_bounds = bool(self.get_parameter("require_field_bounds").value)

        self.latest_cmd: VelocityCommand = build_stop_command("startup")
        self.last_cmd_vel_s: Optional[float] = None
        self.last_nav_frame_s: Optional[float] = None
        self.last_imu_s: Optional[float] = None
        self.last_motor_status_s: Optional[float] = None
        self.last_localization_status_s: Optional[float] = None
        self.motor_status: dict[str, Any] = {}
        self.localization_status: dict[str, Any] = {}
        self.front_clearance_m: Optional[float] = None
        self.near_obstacle = False
        self.last_published_payload: dict[str, Any] = self.latest_cmd.to_payload()
        self.sub_min_speed_command_blocked = 0
        self.kill_switch_active = False
        self.last_kill_switch_s: Optional[float] = None

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(NavSensorFrame, self.nav_frame_topic, self.nav_frame_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.motor_status_topic, self.motor_status_callback, 10)
        self.create_subscription(String, self.localization_status_topic, self.localization_status_callback, 10)
        self.create_subscription(Bool, self.kill_switch_topic, self.kill_switch_callback, 10)
        self.create_timer(self.publish_period_s, self.timer_callback)

        self.get_logger().info(
            "Nav2 adapter started: /cmd_vel -> /ugv_nav_cmd "
            f"(cmd_vel={self.cmd_vel_topic}, command={self.command_topic}, "
            f"min_speed={self.competition_min_speed_mps:.6f} m/s, "
            f"allow_reverse={self.allow_reverse}, boundary_gate={self.require_field_bounds})"
        )

    def _now_s(self) -> float:
        return time.monotonic()

    def cmd_vel_callback(self, msg: Twist) -> None:
        command = nav_command_from_twist(
            linear_x_mps=msg.linear.x,
            linear_y_mps=msg.linear.y,
            angular_z_radps=msg.angular.z,
            competition_min_speed_mps=self.competition_min_speed_mps,
            allow_reverse=self.allow_reverse,
        )
        if command.reason == "nav2_cmd_vel_speed_clamped":
            self.sub_min_speed_command_blocked += 1
        self.latest_cmd = command
        self.last_cmd_vel_s = self._now_s()

    def nav_frame_callback(self, msg: NavSensorFrame) -> None:
        self.last_nav_frame_s = self._now_s()
        self.near_obstacle = bool(msg.near_obstacle)
        self.front_clearance_m = float(msg.front_clearance_m)

    def imu_callback(self, _msg: Imu) -> None:
        self.last_imu_s = self._now_s()

    def motor_status_callback(self, msg: String) -> None:
        self.last_motor_status_s = self._now_s()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"status_parse_error": True}
        self.motor_status = payload if isinstance(payload, dict) else {"status_not_object": True}

    def localization_status_callback(self, msg: String) -> None:
        self.last_localization_status_s = self._now_s()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"status_parse_error": True}
        self.localization_status = payload if isinstance(payload, dict) else {"status_not_object": True}

    def kill_switch_callback(self, msg: Bool) -> None:
        self.kill_switch_active = bool(msg.data)
        self.last_kill_switch_s = self._now_s()

    def _motor_stop_reason(self, now_s: float) -> Optional[str]:
        if self.last_motor_status_s is None or now_s - self.last_motor_status_s > self.motor_status_timeout_s:
            return "motor_status_stale"
        if not bool(self.motor_status.get("connected", False)):
            return "motor_disconnected"
        if not bool(self.motor_status.get("teensy_pid_params_synced", False)):
            return "teensy_params_not_synced"
        fault = motor_fault_reason(self.motor_status)
        if fault is not None:
            return f"motor_fault:{fault}"
        return None

    def _field_boundary_stop_reason(self, command: VelocityCommand) -> Optional[str]:
        if not self.require_field_bounds:
            return None
        pose = self.localization_status.get("pose_m")
        if not isinstance(pose, list) or len(pose) < 3:
            return "field_pose_invalid"
        try:
            x_m = float(pose[0])
            y_m = float(pose[1])
            yaw_rad = math.radians(float(pose[2]))
        except (TypeError, ValueError):
            return "field_pose_invalid"
        decision = field_boundary_decision(
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            v_mps=command.v_mps,
            omega_radps=command.omega_radps,
            bounds=self.field_bounds,
            safety_margin_m=self.field_boundary_margin_m,
            prediction_time_s=self.field_boundary_prediction_time_s,
        )
        return None if decision.safe else decision.reason

    def _safety_stop_reason(self, now_s: float) -> Optional[str]:
        if self.kill_switch_active:
            return "kill_switch"
        motor_reason = self._motor_stop_reason(now_s)
        if motor_reason:
            return motor_reason
        if self.require_imu and (self.last_imu_s is None or now_s - self.last_imu_s > self.imu_timeout_s):
            return "imu_stale"
        if self.require_nav_frame and (
            self.last_nav_frame_s is None or now_s - self.last_nav_frame_s > self.sensor_timeout_s
        ):
            return "sensor_stale"
        if self.require_localization_ready:
            stale = (
                self.last_localization_status_s is None
                or now_s - self.last_localization_status_s > self.localization_timeout_s
            )
            if stale:
                return "localization_status_stale"
            if not bool(self.localization_status.get("localization_ready", False)):
                return "localization_not_ready"
        if self.front_clearance_m is not None and math.isfinite(self.front_clearance_m):
            if self.front_clearance_m < self.emergency_stop_clearance_m:
                return "front_clearance_emergency"
        if self.stop_on_near_obstacle and self.near_obstacle:
            return "near_obstacle"
        return None

    def timer_callback(self) -> None:
        now_s = self._now_s()
        safety_reason = self._safety_stop_reason(now_s)
        if safety_reason:
            command = build_stop_command(safety_reason)
        elif self.last_cmd_vel_s is None or now_s - self.last_cmd_vel_s > self.cmd_vel_timeout_s:
            command = build_stop_command("cmd_vel_timeout")
        else:
            command = self.latest_cmd
            boundary_reason = self._field_boundary_stop_reason(command)
            if boundary_reason is not None:
                safety_reason = boundary_reason
                command = build_stop_command(boundary_reason)

        self.last_published_payload = command.to_payload()
        self.command_pub.publish(String(data=command.to_json()))

        status = {
            "node": "ugv_nav2_adapter",
            "active": command.command_type == "velocity",
            "last_command": self.last_published_payload,
            "motion_rule_ok": command.motion_rule_ok,
            "competition_min_speed_mps": self.competition_min_speed_mps,
            "sub_min_speed_command_blocked": self.sub_min_speed_command_blocked,
            "safety_reason": safety_reason or "ok",
            "cmd_vel_age_s": None if self.last_cmd_vel_s is None else now_s - self.last_cmd_vel_s,
            "nav_frame_age_s": None if self.last_nav_frame_s is None else now_s - self.last_nav_frame_s,
            "imu_age_s": None if self.last_imu_s is None else now_s - self.last_imu_s,
            "motor_status_age_s": None
            if self.last_motor_status_s is None
            else now_s - self.last_motor_status_s,
            "localization_status_age_s": None
            if self.last_localization_status_s is None
            else now_s - self.last_localization_status_s,
            "front_clearance_m": self.front_clearance_m,
            "near_obstacle": self.near_obstacle,
            "field_width_m": self.field_bounds.width_m,
            "field_height_m": self.field_bounds.height_m,
            "field_boundary_margin_m": self.field_boundary_margin_m,
            "field_boundary_prediction_time_s": self.field_boundary_prediction_time_s,
            "field_boundary_gate_enabled": self.require_field_bounds,
            "reverse_allowed": self.allow_reverse,
            "kill_switch_active": self.kill_switch_active,
            "kill_switch_age_s": None
            if self.last_kill_switch_s is None
            else now_s - self.last_kill_switch_s,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main() -> None:
    rclpy.init()
    node = Nav2AdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.command_pub.publish(String(data=build_stop_command("shutdown").to_json()))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
