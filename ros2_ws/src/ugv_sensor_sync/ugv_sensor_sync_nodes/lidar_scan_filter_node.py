#!/usr/bin/env python3
"""Filter LiDAR sectors blocked by the UGV body before navigation consumers."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:
    from ugv_nav_core.nav2_bridge import angle_within_centered_fov
except ModuleNotFoundError:
    angle_within_centered_fov = None


def _within_centered_fov(angle_rad: float, fov_deg: float, center_rad: float) -> bool:
    if angle_within_centered_fov is not None:
        return angle_within_centered_fov(angle_rad, fov_deg, center_rad=center_rad)
    half_fov = 0.5 * math.radians(max(0.0, min(360.0, float(fov_deg))))
    wrapped = (float(angle_rad) - float(center_rad) + math.pi) % (2.0 * math.pi) - math.pi
    return abs(wrapped) <= half_fov


class LidarScanFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_scan_filter_node")
        self.declare_parameter("input_topic", "/scan/synced")
        self.declare_parameter("output_topic", "/scan/filtered")
        self.declare_parameter("forward_fov_deg", 230.0)
        self.declare_parameter("forward_center_rad", 0.0)
        self.declare_parameter("invalid_replacement", "inf")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.forward_fov_deg = max(0.0, min(360.0, float(self.get_parameter("forward_fov_deg").value)))
        self.forward_center_rad = float(self.get_parameter("forward_center_rad").value)
        self.invalid_replacement = str(self.get_parameter("invalid_replacement").value).strip().lower()
        self.last_publish_s = 0.0

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.input_topic, self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            "LiDAR scan filter started: "
            f"{self.input_topic} -> {self.output_topic}, forward_fov={self.forward_fov_deg:.1f} deg"
        )

    def _invalid_range(self, msg: LaserScan) -> float:
        if self.invalid_replacement == "nan":
            return float("nan")
        if self.invalid_replacement == "range_max":
            return float(msg.range_max) + 1.0
        return float("inf")

    def scan_callback(self, msg: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = list(msg.intensities)

        invalid = self._invalid_range(msg)
        ranges = []
        angle = float(msg.angle_min)
        increment = float(msg.angle_increment)
        for range_m in msg.ranges:
            keep = _within_centered_fov(angle, self.forward_fov_deg, self.forward_center_rad)
            ranges.append(float(range_m) if keep else invalid)
            angle += increment
        filtered.ranges = ranges
        self.pub.publish(filtered)
        self.last_publish_s = time.monotonic()


def main() -> None:
    rclpy.init()
    node = LidarScanFilterNode()
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
