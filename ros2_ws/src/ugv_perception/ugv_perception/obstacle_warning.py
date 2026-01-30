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

        self.bridge = CvBridge()
        self.last_dists = []

        self.pub_warn = self.create_publisher(Bool, '/ugv/obstacle_warning', 10)
        self.pub_dist = self.create_publisher(Float32, '/ugv/obstacle_distance_m', 10)

        depth_topic = self.get_parameter('depth_topic').value
        self.sub = self.create_subscription(Image, depth_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(f"Subscribed to depth: {depth_topic}")

    def cb(self, msg: Image):
        # ZED depth is typically 32FC1 meters
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = depth.shape[:2]
        roi_w = int(w * float(self.get_parameter('roi_w_frac').value))
        roi_h = int(h * float(self.get_parameter('roi_h_frac').value))
        x0 = (w - roi_w) // 2
        y0 = (h - roi_h) // 2

        roi = depth[y0:y0+roi_h, x0:x0+roi_w]
        roi = roi.astype(np.float32)

        min_valid = float(self.get_parameter('min_valid_m').value)
        max_valid = float(self.get_parameter('max_valid_m').value)

        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > min_valid) & (valid < max_valid)]

	if valid.size == 0:
	    # keep last distance if we have one, avoids flicker during invalid depth frames
	    if len(self.last_dists) > 0:
		smooth_d = float(np.mean(self.last_dists))
		threshold = float(self.get_parameter('threshold_m').value)
		warn = smooth_d < threshold
		self.pub_warn.publish(Bool(data=warn))
		self.pub_dist.publish(Float32(data=smooth_d))
	    else:
		self.pub_warn.publish(Bool(data=False))
		self.pub_dist.publish(Float32(data=10.0))  # "far"
	    return


        p = float(self.get_parameter('percentile').value)
        close_d = float(np.percentile(valid, p))

        # Smooth a bit (moving average)
        win = int(self.get_parameter('smooth_window').value)
        self.last_dists.append(close_d)
        if len(self.last_dists) > max(1, win):
            self.last_dists.pop(0)
        smooth_d = float(np.mean(self.last_dists))

        threshold = float(self.get_parameter('threshold_m').value)
        warn = smooth_d < threshold

        self.pub_warn.publish(Bool(data=warn))
        self.pub_dist.publish(Float32(data=smooth_d))

        # Optional: print occasionally
        # self.get_logger().info(f"dist={smooth_d:.2f}m warn={warn}")

def main():
    rclpy.init()
    node = ObstacleWarning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

