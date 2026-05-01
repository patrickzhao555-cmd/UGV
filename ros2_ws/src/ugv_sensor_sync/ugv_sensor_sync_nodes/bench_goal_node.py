#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class BenchGoalNode(Node):
    def __init__(self):
        super().__init__('bench_goal_node')

        self.declare_parameter('topic', '/ugv_goal')
        self.declare_parameter('goal_x_m', 12.2)
        self.declare_parameter('goal_y_m', 12.0)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('frame_id', 'map')

        topic = self.get_parameter('topic').value
        self.goal_x_m = float(self.get_parameter('goal_x_m').value)
        self.goal_y_m = float(self.get_parameter('goal_y_m').value)
        self.frame_id = self.get_parameter('frame_id').value
        publish_period_s = max(0.25, float(self.get_parameter('publish_period_s').value))

        self.pub = self.create_publisher(PointStamped, topic, 10)
        self.create_timer(publish_period_s, self.publish_goal)
        self.get_logger().info(
            f'Bench goal publisher started (topic={topic}, goal=({self.goal_x_m:.2f}, {self.goal_y_m:.2f}) m, '
            f'period={publish_period_s:.2f}s)'
        )
        self.publish_goal()

    def publish_goal(self) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = self.goal_x_m
        msg.point.y = self.goal_y_m
        msg.point.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BenchGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
