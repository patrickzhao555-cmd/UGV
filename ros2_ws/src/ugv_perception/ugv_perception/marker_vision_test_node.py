import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class MarkerVisionTestNode(Node):
    def __init__(self):
        super().__init__('marker_vision_test_node')

        self.declare_parameter('debug_topic', '/ugv/marker_vision_debug')
        self.declare_parameter('marker_topic', '/ugv/marker_detection')
        self.declare_parameter('searching_period_s', 3.0)
        self.declare_parameter('detected_period_s', 1.0)
        self.declare_parameter('stale_warn_s', 5.0)

        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.searching_period_s = max(0.5, float(self.get_parameter('searching_period_s').value))
        self.detected_period_s = max(0.2, float(self.get_parameter('detected_period_s').value))
        self.stale_warn_s = max(1.0, float(self.get_parameter('stale_warn_s').value))

        self.frame_count = 0
        self.detected_frame_count = 0
        self.confirmed_count = 0
        self.last_debug_received_s: Optional[float] = None
        self.last_status_print_s = 0.0
        self.last_detected_print_s = 0.0
        self.last_detected = False
        self.last_reason = 'waiting_for_debug'

        self.create_subscription(String, self.debug_topic, self.debug_callback, 10)
        self.create_subscription(PointStamped, self.marker_topic, self.marker_callback, 10)
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            f'Marker vision test started (debug={self.debug_topic}, '
            f'marker={self.marker_topic}, searching_period_s={self.searching_period_s})'
        )

    def debug_callback(self, msg: String) -> None:
        now_s = time.monotonic()
        self.last_debug_received_s = now_s
        self.frame_count += 1

        try:
            debug = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_reason = 'bad_debug_json'
            return

        detected = bool(debug.get('detected'))
        self.last_reason = str(debug.get('reason') or 'no_marker_candidate')
        if not detected:
            self.last_detected = False
            return

        self.detected_frame_count += 1
        should_print = (
            not self.last_detected
            or now_s - self.last_detected_print_s >= self.detected_period_s
        )
        self.last_detected = True
        if not should_print:
            return

        self.last_detected_print_s = now_s
        fields = [
            f"method={debug.get('method')}",
            f"frames={self.detected_frame_count}/{self.frame_count}",
        ]

        distance_m = _finite_number(debug.get('distance_m'))
        if distance_m is None:
            distance_m = _finite_number(debug.get('depth_m'))
        if distance_m is not None:
            fields.append(f'distance={distance_m:.2f}m')

        bearing_deg = _finite_number(debug.get('bearing_deg'))
        if bearing_deg is not None:
            fields.append(f'bearing={bearing_deg:.1f}deg')

        center_px = debug.get('center_px')
        if isinstance(center_px, list) and len(center_px) >= 2:
            fields.append(f'center=({float(center_px[0]):.0f},{float(center_px[1]):.0f})px')

        if debug.get('target_reached_by_depth') is True:
            fields.append('inside_1yd_radius=true')

        reason = debug.get('reason')
        if reason:
            fields.append(f'not_published_reason={reason}')

        self.get_logger().info('MARKER DETECTED ' + ' '.join(fields))

    def marker_callback(self, msg: PointStamped) -> None:
        self.confirmed_count += 1
        fields = [
            f'map=({msg.point.x:.2f},{msg.point.y:.2f})m',
            f'confirmed_count={self.confirmed_count}',
        ]
        if math.isfinite(msg.point.z) and msg.point.z > 0.0:
            fields.append(f'distance={msg.point.z:.2f}m')
        self.get_logger().info('CONFIRMED MARKER PUBLISHED ' + ' '.join(fields))

    def timer_callback(self) -> None:
        now_s = time.monotonic()
        if now_s - self.last_status_print_s < self.searching_period_s:
            return
        self.last_status_print_s = now_s

        if self.last_debug_received_s is None:
            self.get_logger().info(f'WAITING for marker vision debug on {self.debug_topic}')
            return

        debug_age_s = now_s - self.last_debug_received_s
        if debug_age_s > self.stale_warn_s:
            self.get_logger().warn(
                f'No recent marker vision debug for {debug_age_s:.1f}s. '
                'Check that ZED image publishing and marker_vision_node are running.'
            )
            return

        if not self.last_detected:
            self.get_logger().info(
                f'SEARCHING no marker yet frames={self.frame_count} '
                f'detections={self.detected_frame_count} last_reason={self.last_reason}'
            )


def _finite_number(value) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def main(args=None):
    rclpy.init(args=args)
    node = MarkerVisionTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
