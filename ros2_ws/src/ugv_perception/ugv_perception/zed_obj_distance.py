#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped


class ZedObjDistance(Node):
    def __init__(self):
        super().__init__("zed_obj_distance")
        self.create_subscription(
            ObjectsStamped,
            "/zed/zed_node/obj_det/objects",
            self.callback,
            10,
        )

    def callback(self, msg: ObjectsStamped) -> None:
        if not msg.objects:
            return

        closest = None
        closest_distance = float("inf")
        for obj in msg.objects:
            x, y, z = obj.position
            distance = math.sqrt(x * x + y * y + z * z)
            if distance < closest_distance:
                closest_distance = distance
                closest = obj

        if closest is None:
            return

        x, y, z = closest.position
        self.get_logger().info(
            f"{closest.label}/{closest.sublabel} conf={closest.confidence:.1f}% "
            f"d={closest_distance:.2f}m pos=({x:.2f},{y:.2f},{z:.2f}) frame={msg.header.frame_id}"
        )


def main():
    rclpy.init()
    node = ZedObjDistance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

