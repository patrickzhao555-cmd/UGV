#!/usr/bin/env python3
"""Convert fused PoseArray obstacle points into PointCloud2 for Nav2 costmaps."""

from __future__ import annotations

import struct
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from ugv_nav_core.nav2_bridge import finite_xyz_points


class PoseArrayToCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_posearray_to_cloud")
        self.declare_parameter("input_topic", "/sensors/zed_obstacle_points")
        self.declare_parameter("output_topic", "/sensors/zed_obstacle_cloud")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("publish_empty_cloud", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.publish_empty_cloud = bool(self.get_parameter("publish_empty_cloud").value)
        self.last_publish_s = 0.0

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(PoseArray, self.input_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(f"PoseArray obstacle cloud bridge started: {self.input_topic} -> {self.output_topic}")

    def callback(self, msg: PoseArray) -> None:
        points = finite_xyz_points((pose.position.x, pose.position.y, pose.position.z) for pose in msg.poses)
        if not points and not self.publish_empty_cloud:
            return
        cloud = PointCloud2()
        cloud.header = msg.header
        if self.target_frame:
            cloud.header.frame_id = self.target_frame
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * len(points)
        cloud.is_dense = True
        cloud.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)
        self.pub.publish(cloud)
        self.last_publish_s = time.monotonic()


def main() -> None:
    rclpy.init()
    node = PoseArrayToCloudNode()
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

