#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObstacleWarning(Node):
    def __init__(self):
        super().__init__('obstacle_warning')

        # Params
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('threshold_m', 0.30) # 30 cm
        self.declare_parameter('roi_w_frac', 0.30)  # middle 30% width
        self.declare_parameter('roi_h_frac', 0.30)  # middle 30% height
        self.declare_parameter('min_valid_m', 0.15) #ignore noise
        self.declare_parameter('max_valid_m', 10.0)
        self.declare_parameter('percentile', 10.0)    
        self.declare_parameter('smooth_window', 5)  # reduce flicker
        self.declare_parameter("threshold_on_m", 0.30)
        self.declare_parameter("threshold_off_m", 0.35)
        self.declare_parameter("min_valid_m", 0.05)
        self.declare_parameter("max_valid_m", 10.0)
        self.declare_parameter("roi_w_frac", 0.50)
        self.declare_parameter("roi_h_frac", 0.40)
        self.declare_parameter("roi_y_center_frac", 0.55)
        self.declare_parameter("near_percentile", 10.0)
        self.declare_parameter("invalid_warn_frames", 3)


        self.bridge = CvBridge()
        self.last_dists = []
        self.invalid_count = 0
        self.warn_state = False
        self.last_dist = None
        self.last_dists = []
        self.smooth_window = 5  # or set via parameter


        self.pub_warn = self.create_publisher(Bool, '/ugv/obstacle_warning', 10)
        self.pub_dist = self.create_publisher(Float32, '/ugv/obstacle_distance_m', 10)

        depth_topic = self.get_parameter('depth_topic').value
        self.sub = self.create_subscription(Image, depth_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(f"Subscribed to depth: {depth_topic}")

    def cb(self, msg):
        # Convert depth image to float32 meters (ZED depth_registered is 32FC1)
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

        h, w = depth.shape[:2]

        # --- Parameters ---
        threshold_on = float(self.get_parameter("threshold_on_m").value)   # e.g., 0.30
        threshold_off = float(self.get_parameter("threshold_off_m").value) # e.g., 0.35
        min_valid = float(self.get_parameter("min_valid_m").value)         # e.g., 0.05
        max_valid = float(self.get_parameter("max_valid_m").value)         # e.g., 10.0
        roi_w_frac = float(self.get_parameter("roi_w_frac").value)         # e.g., 0.50
        roi_h_frac = float(self.get_parameter("roi_h_frac").value)         # e.g., 0.40
        roi_y_center_frac = float(self.get_parameter("roi_y_center_frac").value) # e.g., 0.55
        percentile = float(self.get_parameter("near_percentile").value)    # e.g., 10.0
        invalid_warn_frames = int(self.get_parameter("invalid_warn_frames").value) # e.g., 3

        # --- ROI: center-ish (slightly lower is better for obstacles on ground) ---
        roi_w = max(10, int(w * roi_w_frac))
        roi_h = max(10, int(h * roi_h_frac))

        cx = w // 2
        cy = int(h * roi_y_center_frac)

        x0 = max(0, cx - roi_w // 2)
        x1 = min(w, cx + roi_w // 2)
        y0 = max(0, cy - roi_h // 2)
        y1 = min(h, cy + roi_h // 2)

        roi = depth[y0:y1, x0:x1]

        # Filter valid depth
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= min_valid) & (valid <= max_valid)]

        # If no valid depth: treat as "unsafe" after a few frames (camera covered / too close)
        if valid.size == 0:
            self.invalid_count += 1
            if self.invalid_count >= invalid_warn_frames:
                self.warn_state = True
                self.pub_warn.publish(Bool(data=True))
            # distance can be NaN or last known; choose last known for nicer demo
            if self.last_dist is not None:
                self.pub_dist.publish(Float32(data=float(self.last_dist)))
            else:
                self.pub_dist.publish(Float32(data=float("nan")))
            return

        # Got valid depth -> reset invalid counter
        self.invalid_count = 0

        # Robust "closest" estimate: near-percentile distance
        d = float(np.percentile(valid, percentile))

        # Smooth a little
        self.last_dists.append(d)
        if len(self.last_dists) > self.smooth_window:
            self.last_dists.pop(0)
        smooth_d = float(np.mean(self.last_dists))
        self.last_dist = smooth_d

        # Hysteresis warning logic
        if not self.warn_state and smooth_d < threshold_on:
            self.warn_state = True
        elif self.warn_state and smooth_d > threshold_off:
            self.warn_state = False

        self.pub_dist.publish(Float32(data=smooth_d))
        self.pub_warn.publish(Bool(data=self.warn_state))


def main():
    rclpy.init()
    node = ObstacleWarning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

