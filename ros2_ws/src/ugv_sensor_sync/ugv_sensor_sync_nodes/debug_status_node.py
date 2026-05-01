#!/usr/bin/env python3
import json
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DebugStatusNode(Node):
    def __init__(self):
        super().__init__('ugv_debug_status_node')

        self.declare_parameter('publish_topic', '/ugv/debug_status')
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('zed_status_topic', '/zed/status')
        self.declare_parameter('fusion_summary_topic', '/sensors/synced_summary')
        self.declare_parameter('motor_status_topic', '/motor_controller/status')
        self.declare_parameter('nav_status_topic', '/ugv_nav_status')
        self.declare_parameter('uav_flag_topic', '/ugv/uav_flag')

        publish_topic = self.get_parameter('publish_topic').value
        self.publish_period_s = max(0.2, float(self.get_parameter('publish_period_s').value))
        self.latest: Dict[str, Tuple[dict, float]] = {}

        self.pub = self.create_publisher(String, publish_topic, 10)
        self._subscribe_json('zed', self.get_parameter('zed_status_topic').value)
        self._subscribe_json('fusion', self.get_parameter('fusion_summary_topic').value)
        self._subscribe_json('motor', self.get_parameter('motor_status_topic').value)
        self._subscribe_json('nav', self.get_parameter('nav_status_topic').value)
        self._subscribe_json('uav_flag', self.get_parameter('uav_flag_topic').value)
        self.create_timer(self.publish_period_s, self.publish_debug)
        self.get_logger().info(f'UGV debug status node publishing {publish_topic}')

    def _subscribe_json(self, key: str, topic: str) -> None:
        def callback(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                data = {'raw': msg.data}
            self.latest[key] = (data, self._now_s())

        self.create_subscription(String, topic, callback, 10)

    def publish_debug(self) -> None:
        now_s = self._now_s()
        status = {}
        for key, (data, stamp_s) in self.latest.items():
            status[key] = {
                'age_s': round(max(0.0, now_s - stamp_s), 3),
                'data': data,
            }
        self.pub.publish(String(data=json.dumps(status)))
        self._log_one_line(status)

    def _log_one_line(self, status: dict) -> None:
        zed = self._data(status, 'zed')
        fusion = self._data(status, 'fusion')
        motor = self._data(status, 'motor')
        nav = self._data(status, 'nav')
        mission = nav.get('mission', {}) if isinstance(nav, dict) else {}
        cmd = nav.get('cmd', {}) if isinstance(nav, dict) else {}

        self.get_logger().info(
            'debug '
            f"zed_valid={zed.get('valid_depth_samples')} "
            f"depth_p10={zed.get('depth_p10_m')} "
            f"front_clearance={fusion.get('front_clearance_m')} "
            f"encoder={fusion.get('encoder_available')} "
            f"motor_connected={motor.get('connected')} "
            f"phase={mission.get('phase')} "
            f"cmd={cmd.get('mode')} "
            f"pose={nav.get('pose_m')}"
        )

    @staticmethod
    def _data(status: dict, key: str) -> dict:
        item: Optional[dict] = status.get(key)
        if not item:
            return {}
        data = item.get('data')
        return data if isinstance(data, dict) else {}

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = DebugStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
