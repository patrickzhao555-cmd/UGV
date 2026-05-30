#!/usr/bin/env python3
"""GPS-denied field odometry and TF anchor for Nav2.

The node publishes:

- odom -> base_link from wheel encoder distance plus ZED gyro yaw.
- map -> odom from a manual initial field pose or /initialpose update.
- /ugv_localization/status and /ugv_nav_status pose diagnostics.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from ugv_sensor_sync.msg import NavSensorFrame

from ugv_nav_core.chassis_controller import encoder_ticks_to_distance_m, encoder_yaw_delta
from ugv_nav_core.nav2_bridge import (
    Transform2D,
    compose_transform_2d,
    evaluate_gyro_bias_samples,
    integrate_planar_odometry,
    map_to_odom_from_pose,
    select_imu_timing_step,
    wrap_pi,
)


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * float(yaw_rad)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FieldOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_field_odom")

        self.declare_parameter("nav_frame_topic", "/sensors/nav_frame")
        self.declare_parameter("imu_topic", "/zed/imu")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("status_topic", "/ugv_localization/status")
        self.declare_parameter("nav_status_topic", "/ugv_nav_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initial_x_m", 0.0)
        self.declare_parameter("initial_y_m", 0.0)
        self.declare_parameter("initial_yaw_deg", 0.0)
        self.declare_parameter("track_width_m", 0.425)
        self.declare_parameter("wheel_radius_m", 0.0825)
        self.declare_parameter("ticks_per_rev", 3200.0)
        self.declare_parameter("gyro_bias_calibration_s", 1.5)
        self.declare_parameter("gyro_bias_radps", 0.0)
        self.declare_parameter("gyro_bias_max_stddev_radps", 0.03)
        self.declare_parameter("gyro_bias_max_encoder_delta_ticks", 2)
        self.declare_parameter("imu_yaw_axis", "z")
        self.declare_parameter("imu_yaw_sign", 1.0)
        self.declare_parameter("status_period_s", 0.25)
        self.declare_parameter("min_imu_dt_s", 0.001)
        self.declare_parameter("max_imu_dt_s", 0.05)

        self.nav_frame_topic = str(self.get_parameter("nav_frame_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.nav_status_topic = str(self.get_parameter("nav_status_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        initial_pose = Transform2D(
            x=float(self.get_parameter("initial_x_m").value),
            y=float(self.get_parameter("initial_y_m").value),
            yaw=math.radians(float(self.get_parameter("initial_yaw_deg").value)),
        )
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.gyro_bias_calibration_s = max(0.0, float(self.get_parameter("gyro_bias_calibration_s").value))
        self.gyro_bias_radps = float(self.get_parameter("gyro_bias_radps").value)
        self.gyro_bias_max_stddev_radps = max(
            0.0,
            float(self.get_parameter("gyro_bias_max_stddev_radps").value),
        )
        self.gyro_bias_max_encoder_delta_ticks = max(
            0,
            int(self.get_parameter("gyro_bias_max_encoder_delta_ticks").value),
        )
        self.imu_yaw_axis = str(self.get_parameter("imu_yaw_axis").value)
        self.imu_yaw_sign = float(self.get_parameter("imu_yaw_sign").value)
        self.status_period_s = max(0.05, float(self.get_parameter("status_period_s").value))
        self.min_imu_dt_s = max(0.0, float(self.get_parameter("min_imu_dt_s").value))
        self.max_imu_dt_s = max(0.001, float(self.get_parameter("max_imu_dt_s").value))

        self.odom_pose = Transform2D()
        self.map_to_odom = map_to_odom_from_pose(desired_map_base=initial_pose, current_odom_base=self.odom_pose)
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_encoder_s: Optional[float] = None
        self.last_imu_s: Optional[float] = None
        self.last_imu_stamp_s: Optional[float] = None
        self.last_imu_fallback_s: Optional[float] = None
        self.last_nav_frame_s: Optional[float] = None
        self.last_map_to_odom_update_s = time.monotonic()
        self.gyro_heading_rad = 0.0
        self.encoder_heading_rad = 0.0
        self.imu_ready = False
        self.encoder_ready = False
        self.imu_skipped_integrations = 0
        self.imu_timestamp_fallbacks = 0
        self.imu_max_observed_dt_s = 0.0
        self.imu_time_source = "none"
        self.imu_timing_reason = "startup"
        self.bias_samples: list[float] = []
        self.bias_start_s: Optional[float] = None
        self.bias_ready = self.gyro_bias_calibration_s <= 0.0
        self.gyro_bias_std_radps = 0.0
        self.gyro_bias_sample_count = 0
        self.gyro_bias_status = "ready_configured" if self.bias_ready else "calibrating"
        self.bias_encoder_start_ticks: Optional[tuple[int, int]] = None
        self.bias_encoder_current_ticks: Optional[tuple[int, int]] = None
        self.last_linear_speed_mps = 0.0
        self.last_angular_speed_radps = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.nav_status_pub = self.create_publisher(String, self.nav_status_topic, 10)
        self.create_subscription(NavSensorFrame, self.nav_frame_topic, self.nav_frame_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initialpose_callback, 10)
        self.create_timer(self.status_period_s, self.status_timer)

        self.get_logger().info(
            "Field odometry started "
            f"(initial=({initial_pose.x:.3f}, {initial_pose.y:.3f}, {math.degrees(initial_pose.yaw):.1f}deg), "
            f"frames={self.map_frame}->{self.odom_frame}->{self.base_frame})"
        )

    def _now_s(self) -> float:
        return time.monotonic()

    def _stamp_now(self):
        return self.get_clock().now().to_msg()

    def _imu_axis_value(self, msg: Imu) -> float:
        axis = self.imu_yaw_axis.lower()
        if axis == "x":
            value = msg.angular_velocity.x
        elif axis == "y":
            value = msg.angular_velocity.y
        else:
            value = msg.angular_velocity.z
        return self.imu_yaw_sign * float(value)

    def _imu_stamp_s(self, msg: Imu) -> Optional[float]:
        stamp_s = int(msg.header.stamp.sec)
        stamp_ns = int(msg.header.stamp.nanosec)
        if stamp_s == 0 and stamp_ns == 0:
            return None
        value = float(stamp_s) + float(stamp_ns) * 1e-9
        return value if math.isfinite(value) and value > 0.0 else None

    def _reset_bias_calibration(self, now_s: float, reason: str) -> None:
        self.bias_samples = []
        self.bias_start_s = now_s
        self.bias_ready = False
        self.gyro_bias_std_radps = 0.0
        self.gyro_bias_sample_count = 0
        self.gyro_bias_status = reason
        self.bias_encoder_start_ticks = self.bias_encoder_current_ticks

    def _update_bias_encoder_ticks(self, left_ticks: int, right_ticks: int, now_s: float) -> None:
        if self.bias_ready:
            return
        current = (int(left_ticks), int(right_ticks))
        self.bias_encoder_current_ticks = current
        if self.bias_encoder_start_ticks is None:
            self.bias_encoder_start_ticks = current
            return
        left_delta = abs(current[0] - self.bias_encoder_start_ticks[0])
        right_delta = abs(current[1] - self.bias_encoder_start_ticks[1])
        if max(left_delta, right_delta) > self.gyro_bias_max_encoder_delta_ticks:
            self.get_logger().warn("Field odom gyro bias calibration reset: encoder moved during calibration")
            self._reset_bias_calibration(now_s, "gyro_bias_encoder_motion")

    def imu_callback(self, msg: Imu) -> None:
        now_s = self._now_s()
        stamp_s = self._imu_stamp_s(msg)
        raw_rate = self._imu_axis_value(msg)
        if self.bias_start_s is None:
            self.bias_start_s = now_s
        if not self.bias_ready:
            if not math.isfinite(raw_rate):
                self._reset_bias_calibration(now_s, "gyro_bias_non_finite")
                self.last_imu_s = now_s
                self.last_imu_fallback_s = now_s
                return
            self.bias_samples.append(raw_rate)
            self.gyro_bias_sample_count = len(self.bias_samples)
            if now_s - self.bias_start_s >= self.gyro_bias_calibration_s and self.bias_samples:
                evaluation = evaluate_gyro_bias_samples(
                    self.bias_samples,
                    max_stddev_radps=self.gyro_bias_max_stddev_radps,
                    encoder_start_ticks=self.bias_encoder_start_ticks,
                    encoder_current_ticks=self.bias_encoder_current_ticks,
                    max_encoder_delta_ticks=self.gyro_bias_max_encoder_delta_ticks,
                )
                self.gyro_bias_std_radps = evaluation.stddev_radps
                self.gyro_bias_sample_count = evaluation.sample_count
                self.gyro_bias_status = evaluation.reason
                if evaluation.accepted:
                    self.gyro_bias_radps = evaluation.mean_radps
                    self.bias_ready = True
                    self.get_logger().info(
                        "Field odom gyro bias calibrated: "
                        f"{self.gyro_bias_radps:.5f} rad/s std={self.gyro_bias_std_radps:.5f}"
                    )
                else:
                    self.get_logger().warn(
                        "Field odom gyro bias calibration rejected: "
                        f"{evaluation.reason} std={evaluation.stddev_radps:.5f}"
                    )
                    self._reset_bias_calibration(now_s, evaluation.reason)
            self.last_imu_s = now_s
            self.last_imu_fallback_s = now_s
            if stamp_s is not None and (self.last_imu_stamp_s is None or stamp_s > self.last_imu_stamp_s):
                self.last_imu_stamp_s = stamp_s
            return

        yaw_rate = raw_rate - self.gyro_bias_radps
        timing = select_imu_timing_step(
            stamp_s=stamp_s,
            fallback_now_s=now_s,
            last_stamp_s=self.last_imu_stamp_s,
            last_fallback_s=self.last_imu_fallback_s,
            min_dt_s=self.min_imu_dt_s,
            max_dt_s=self.max_imu_dt_s,
        )
        self.imu_time_source = timing.time_source
        self.imu_timing_reason = timing.reason
        if timing.time_source == "monotonic_fallback":
            self.imu_timestamp_fallbacks += 1
        if timing.dt_s is not None and not timing.skipped:
            self.gyro_heading_rad = wrap_pi(self.gyro_heading_rad + yaw_rate * timing.dt_s)
            self.imu_ready = True
            self.last_angular_speed_radps = yaw_rate
            self.imu_max_observed_dt_s = max(self.imu_max_observed_dt_s, timing.dt_s)
        else:
            self.imu_skipped_integrations += 1
        if timing.stamp_s is not None:
            self.last_imu_stamp_s = timing.stamp_s
        self.last_imu_s = now_s
        self.last_imu_fallback_s = now_s

    def nav_frame_callback(self, msg: NavSensorFrame) -> None:
        now_s = self._now_s()
        left_ticks = int(msg.left_encoder_ticks)
        right_ticks = int(msg.right_encoder_ticks)
        self.last_nav_frame_s = now_s
        if not bool(msg.encoder_available):
            self._publish_transforms_and_odom()
            return
        self._update_bias_encoder_ticks(left_ticks, right_ticks, now_s)
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_encoder_s = now_s
            self.encoder_ready = True
            self._publish_transforms_and_odom()
            return

        left_delta = left_ticks - self.last_left_ticks
        right_delta = right_ticks - self.last_right_ticks
        left_m = encoder_ticks_to_distance_m(
            left_delta,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
        )
        right_m = encoder_ticks_to_distance_m(
            right_delta,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
        )
        distance_m = 0.5 * (left_m + right_m)
        encoder_delta_yaw = encoder_yaw_delta(
            left_delta,
            right_delta,
            wheel_radius_m=self.wheel_radius_m,
            ticks_per_rev=self.ticks_per_rev,
            track_width_m=self.track_width_m,
        )
        self.encoder_heading_rad = wrap_pi(self.encoder_heading_rad + encoder_delta_yaw)
        desired_yaw = self.gyro_heading_rad if self.imu_ready else self.encoder_heading_rad
        delta_yaw = wrap_pi(desired_yaw - self.odom_pose.yaw)
        self.odom_pose = integrate_planar_odometry(
            self.odom_pose,
            distance_m=distance_m,
            delta_yaw_rad=delta_yaw,
        )
        if self.last_encoder_s is not None:
            dt_s = max(1e-6, now_s - self.last_encoder_s)
            self.last_linear_speed_mps = distance_m / dt_s
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_encoder_s = now_s
        self.encoder_ready = True
        self._publish_transforms_and_odom()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        desired = Transform2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=_yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.map_to_odom = map_to_odom_from_pose(desired_map_base=desired, current_odom_base=self.odom_pose)
        self.last_map_to_odom_update_s = self._now_s()
        self.get_logger().warn(
            "Manual /initialpose accepted: "
            f"map pose=({desired.x:.3f}, {desired.y:.3f}, {math.degrees(desired.yaw):.1f}deg)"
        )
        self._publish_transforms_and_odom()

    def _publish_transforms_and_odom(self) -> None:
        stamp = self._stamp_now()
        self._publish_transform(self.map_frame, self.odom_frame, self.map_to_odom, stamp)
        self._publish_transform(self.odom_frame, self.base_frame, self.odom_pose, stamp)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.odom_pose.x)
        odom.pose.pose.position.y = float(self.odom_pose.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = _quaternion_from_yaw(self.odom_pose.yaw)
        odom.twist.twist.linear.x = float(self.last_linear_speed_mps)
        odom.twist.twist.angular.z = float(self.last_angular_speed_radps)
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.10
        self.odom_pub.publish(odom)

    def _publish_transform(self, parent: str, child: str, pose: Transform2D, stamp) -> None:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(pose.x)
        msg.transform.translation.y = float(pose.y)
        msg.transform.translation.z = 0.0
        msg.transform.rotation = _quaternion_from_yaw(pose.yaw)
        self.tf_broadcaster.sendTransform(msg)

    def _status_payload(self) -> dict:
        now_s = self._now_s()
        map_pose = compose_transform_2d(self.map_to_odom, self.odom_pose)
        localization_ready = bool(self.bias_ready and self.encoder_ready)
        confidence = 1.0 if localization_ready and self.imu_ready else (0.5 if self.encoder_ready else 0.0)
        payload = {
            "node": "ugv_field_odom",
            "localization_ready": localization_ready,
            "localization_confidence": confidence,
            "pose_source": "manual_field_pose_plus_wheel_imu_odom",
            "pose_m": [round(map_pose.x, 4), round(map_pose.y, 4), round(math.degrees(map_pose.yaw), 3)],
            "odom_pose_m": [
                round(self.odom_pose.x, 4),
                round(self.odom_pose.y, 4),
                round(math.degrees(self.odom_pose.yaw), 3),
            ],
            "map_to_odom_m": [
                round(self.map_to_odom.x, 4),
                round(self.map_to_odom.y, 4),
                round(math.degrees(self.map_to_odom.yaw), 3),
            ],
            "map_to_odom_age_s": now_s - self.last_map_to_odom_update_s,
            "gyro_bias_radps": self.gyro_bias_radps,
            "gyro_bias_std_radps": self.gyro_bias_std_radps,
            "gyro_bias_sample_count": self.gyro_bias_sample_count,
            "gyro_bias_status": self.gyro_bias_status,
            "gyro_bias_ready": self.bias_ready,
            "imu_ready": self.imu_ready,
            "encoder_ready": self.encoder_ready,
            "imu_skipped_integrations": self.imu_skipped_integrations,
            "imu_timestamp_fallbacks": self.imu_timestamp_fallbacks,
            "imu_time_source": self.imu_time_source,
            "imu_timing_reason": self.imu_timing_reason,
            "imu_max_dt_s": self.imu_max_observed_dt_s,
            "last_nav_frame_age_s": None if self.last_nav_frame_s is None else now_s - self.last_nav_frame_s,
            "last_imu_age_s": None if self.last_imu_s is None else now_s - self.last_imu_s,
        }
        return payload

    def status_timer(self) -> None:
        self._publish_transforms_and_odom()
        payload = self._status_payload()
        text = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=text))
        self.nav_status_pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = FieldOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
