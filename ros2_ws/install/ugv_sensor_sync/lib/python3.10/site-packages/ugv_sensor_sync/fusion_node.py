#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PointStamped
import message_filters


SLOP_S = 0.05   # 50 ms tolerance window — widen if messages drop


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Synchronized subscribers — all keyed on header.stamp in Jetson time
        scan_sub = message_filters.Subscriber(
            self, LaserScan,    '/scan/synced')
        imu_sub  = message_filters.Subscriber(
            self, Imu,          '/zed/imu')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, imu_sub],
            queue_size=20,
            slop=SLOP_S
        )
        self.sync.registerCallback(self.fused_callback)

        # UWB at lower update rate — subscribe separately, store latest
        self.latest_uwb = None
        self.create_subscription(
            PointStamped, '/uwb/range', self.uwb_callback, 10)

        self.get_logger().info(
            f'Fusion node started (slop={SLOP_S*1000:.0f}ms)')

    def uwb_callback(self, msg: PointStamped):
        self.latest_uwb = msg

    def fused_callback(self, scan_msg: LaserScan, imu_msg: Imu):
        t_s = Time.from_msg(scan_msg.header.stamp).nanoseconds / 1e9

        ranges      = scan_msg.ranges
        valid       = [r for r in ranges
                       if scan_msg.range_min < r < scan_msg.range_max]
        min_dist    = min(valid) if valid else float('inf')

        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        uwb_str = 'none'
        if self.latest_uwb is not None:
            uwb_str = (f'anchor={int(self.latest_uwb.point.y)} '
                       f'range={self.latest_uwb.point.x:.2f}m')

        self.get_logger().info(
            f't={t_s:.3f}s | '
            f'lidar_min={min_dist:.2f}m | '
            f'imu_ax={ax:.2f} ay={ay:.2f} az={az:.2f} | '
            f'uwb={uwb_str}'
        )

        # ---- Hand off to your pathing algorithm here ----
        # from ugv_nav.ugv_nav.avoid import your_function
        # your_function(scan_msg, imu_msg, self.latest_uwb)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()