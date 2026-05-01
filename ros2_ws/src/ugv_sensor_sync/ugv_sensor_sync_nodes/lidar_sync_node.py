#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


SCAN_FREQ_HZ = 10.0  # Match the standard A1/A2 bring-up unless explicitly overridden


class LidarSyncNode(Node):
    def __init__(self):
        super().__init__('lidar_sync_node')

        self.declare_parameter('scan_freq_hz', SCAN_FREQ_HZ)
        self.freq = self.get_parameter('scan_freq_hz').value

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan/synced', qos_profile_sensor_data)

        self.get_logger().info(f'Lidar sync node started (scan_freq={self.freq} Hz)')

    def scan_callback(self, msg: LaserScan):
        t_rx_ns = self.get_clock().now().nanoseconds
        scan_period_ns = int(1e9 / self.freq)

        # Midpoint is a better representative timestamp than arrival time
        # when the robot is moving because beams span a full revolution.
        t_mid_ns = t_rx_ns - scan_period_ns // 2
        t_mid = Time(nanoseconds=t_mid_ns)

        out = LaserScan()
        out.header.stamp = t_mid.to_msg()
        out.header.frame_id = msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
