#!/usr/bin/env python3
"""Publish a simple competition-field occupancy map for Nav2.

The map is intentionally conservative and deterministic: the interior of the
field is known free space, while the configured boundary margin is occupied.
Dynamic obstacles still come from LiDAR/ZED costmap layers, and the adapter
keeps its independent field-boundary stop gate as the final containment layer.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


def _build_field_map_data(
    *,
    width: int,
    height: int,
    resolution_m: float,
    field_width_m: float,
    field_height_m: float,
    margin_m: float,
) -> list[int]:
    data: list[int] = []
    max_free_x = max(0.0, float(field_width_m) - float(margin_m))
    max_free_y = max(0.0, float(field_height_m) - float(margin_m))
    for row in range(int(height)):
        y_m = (row + 0.5) * float(resolution_m)
        for col in range(int(width)):
            x_m = (col + 0.5) * float(resolution_m)
            inside_free = (
                float(margin_m) <= x_m <= max_free_x
                and float(margin_m) <= y_m <= max_free_y
                and x_m <= float(field_width_m)
                and y_m <= float(field_height_m)
            )
            data.append(0 if inside_free else 100)
    return data


class FieldMapNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_field_map")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("field_margin_m", 0.45)
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("publish_period_s", 1.0)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.field_width_m = max(0.1, float(self.get_parameter("field_width_m").value))
        self.field_height_m = max(0.1, float(self.get_parameter("field_height_m").value))
        self.field_margin_m = max(0.0, float(self.get_parameter("field_margin_m").value))
        self.resolution_m = max(0.01, float(self.get_parameter("resolution_m").value))
        self.publish_period_s = max(0.2, float(self.get_parameter("publish_period_s").value))
        if self.field_margin_m * 2.0 >= min(self.field_width_m, self.field_height_m):
            raise ValueError("field_margin_m is too large for the configured field size")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(OccupancyGrid, self.map_topic, qos)
        self.map_msg: Optional[OccupancyGrid] = self._build_map()
        self.create_timer(self.publish_period_s, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            "Field map publisher started "
            f"(topic={self.map_topic}, size={self.field_width_m:.3f}x{self.field_height_m:.3f}m, "
            f"margin={self.field_margin_m:.3f}m, resolution={self.resolution_m:.3f}m)"
        )

    def _build_map(self) -> OccupancyGrid:
        width = int(math.ceil(self.field_width_m / self.resolution_m))
        height = int(math.ceil(self.field_height_m / self.resolution_m))
        msg = OccupancyGrid()
        msg.header = Header(frame_id=self.map_frame)
        msg.info = MapMetaData()
        msg.info.resolution = float(self.resolution_m)
        msg.info.width = int(width)
        msg.info.height = int(height)
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = _build_field_map_data(
            width=width,
            height=height,
            resolution_m=self.resolution_m,
            field_width_m=self.field_width_m,
            field_height_m=self.field_height_m,
            margin_m=self.field_margin_m,
        )
        return msg

    def publish_map(self) -> None:
        if self.map_msg is None:
            return
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main() -> None:
    rclpy.init()
    node = FieldMapNode()
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
