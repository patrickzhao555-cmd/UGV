#!/usr/bin/env python3
import json
import math
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class ObstacleWarning(Node):
    def __init__(self):
        super().__init__("obstacle_warning")

        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("warning_topic", "/ugv/obstacle_warning")
        self.declare_parameter("distance_topic", "/ugv/obstacle_distance_m")
        self.declare_parameter("debug_topic", "/ugv/obstacle_depth_debug")
        self.declare_parameter("debug_period_s", 1.0)
        self.declare_parameter("threshold_on_m", 0.30)
        self.declare_parameter("threshold_off_m", 0.35)
        self.declare_parameter("min_valid_m", 0.05)
        self.declare_parameter("max_valid_m", 10.0)
        self.declare_parameter("roi_w_frac", 0.50)
        self.declare_parameter("roi_h_frac", 0.40)
        self.declare_parameter("roi_y_center_frac", 0.55)
        self.declare_parameter("near_percentile", 10.0)
        self.declare_parameter("smooth_window", 5)
        self.declare_parameter("invalid_warn_frames", 3)

        self.depth_topic = self.get_parameter("depth_topic").value
        self.warning_topic = self.get_parameter("warning_topic").value
        self.distance_topic = self.get_parameter("distance_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.debug_period_s = max(0.2, float(self.get_parameter("debug_period_s").value))
        self.threshold_on_m = float(self.get_parameter("threshold_on_m").value)
        self.threshold_off_m = float(self.get_parameter("threshold_off_m").value)
        self.min_valid_m = float(self.get_parameter("min_valid_m").value)
        self.max_valid_m = float(self.get_parameter("max_valid_m").value)
        self.roi_w_frac = float(self.get_parameter("roi_w_frac").value)
        self.roi_h_frac = float(self.get_parameter("roi_h_frac").value)
        self.roi_y_center_frac = float(self.get_parameter("roi_y_center_frac").value)
        self.near_percentile = float(self.get_parameter("near_percentile").value)
        self.smooth_window = max(1, int(self.get_parameter("smooth_window").value))
        self.invalid_warn_frames = max(1, int(self.get_parameter("invalid_warn_frames").value))

        self.bridge = CvBridge()
        self.invalid_count = 0
        self.warn_state = False
        self.last_distance_m = float("nan")
        self.last_debug_s = 0.0
        self.frame_count = 0
        self.distance_window = deque(maxlen=self.smooth_window)

        self.warning_pub = self.create_publisher(Bool, self.warning_topic, 10)
        self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)

        self.get_logger().info(
            "Depth warning ready: "
            f"depth_topic={self.depth_topic}, "
            f"warning_topic={self.warning_topic}, "
            f"distance_topic={self.distance_topic}, "
            f"debug_topic={self.debug_topic}, "
            f"threshold_on={self.threshold_on_m:.2f}m, "
            f"threshold_off={self.threshold_off_m:.2f}m"
        )

    def depth_callback(self, msg: Image) -> None:
        self.frame_count += 1
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        roi = self._extract_roi(depth)

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_valid_m) & (valid <= self.max_valid_m)]

        if valid.size == 0:
            self.invalid_count += 1
            if self.invalid_count >= self.invalid_warn_frames:
                self.warn_state = True
            self._publish(self.last_distance_m, self.warn_state)
            self._publish_debug(msg, roi.shape, 0, self.last_distance_m, self.warn_state)
            return

        self.invalid_count = 0

        near_distance_m = float(np.percentile(valid, self.near_percentile))
        self.distance_window.append(near_distance_m)
        smooth_distance_m = float(np.mean(self.distance_window))
        self.last_distance_m = smooth_distance_m

        if not self.warn_state and smooth_distance_m < self.threshold_on_m:
            self.warn_state = True
        elif self.warn_state and smooth_distance_m > self.threshold_off_m:
            self.warn_state = False

        self._publish(smooth_distance_m, self.warn_state)
        self._publish_debug(msg, roi.shape, int(valid.size), smooth_distance_m, self.warn_state)

    def _extract_roi(self, depth: np.ndarray) -> np.ndarray:
        height, width = depth.shape[:2]

        roi_w = max(10, int(width * self.roi_w_frac))
        roi_h = max(10, int(height * self.roi_h_frac))

        center_x = width // 2
        center_y = int(height * self.roi_y_center_frac)

        x0 = max(0, center_x - roi_w // 2)
        x1 = min(width, center_x + roi_w // 2)
        y0 = max(0, center_y - roi_h // 2)
        y1 = min(height, center_y + roi_h // 2)
        return depth[y0:y1, x0:x1]

    def _publish(self, distance_m: float, warn_state: bool) -> None:
        self.distance_pub.publish(Float32(data=float(distance_m)))
        self.warning_pub.publish(Bool(data=warn_state))

    def _publish_debug(self, msg: Image, roi_shape, valid_count: int, distance_m: float, warn_state: bool) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self.last_debug_s < self.debug_period_s:
            return
        self.last_debug_s = now_s
        status = {
            "stamp_sec": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
            "frame_count": self.frame_count,
            "image_shape": [int(msg.height), int(msg.width)],
            "roi_shape": [int(roi_shape[0]), int(roi_shape[1])],
            "valid_depth_samples": int(valid_count),
            "distance_m": round(float(distance_m), 3) if math.isfinite(float(distance_m)) else None,
            "warn": bool(warn_state),
            "invalid_streak": int(self.invalid_count),
        }
        self.debug_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = ObstacleWarning()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
