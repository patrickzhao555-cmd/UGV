#!/usr/bin/env python3
from collections import deque
import json
import math
from typing import List, Optional, Tuple

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int32MultiArray, String

from ugv_sensor_sync.msg import EncoderTicksStamped, NavSensorFrame, SyncedSensorPacket


DEFAULT_SLOP_S = 0.08


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('scan_topic', '/scan/synced')
        self.declare_parameter('image_topic', '/zed/image')
        self.declare_parameter('depth_topic', '/zed/depth')
        self.declare_parameter('imu_topic', '/zed/imu')
        self.declare_parameter('encoder_stamped_topic', '/encoder_ticks_stamped')
        self.declare_parameter('encoder_topic', '/encoder_ticks')
        self.declare_parameter('output_topic', '/sensors/synced')
        self.declare_parameter('nav_frame_topic', '/sensors/nav_frame')
        self.declare_parameter('obstacle_points_topic', '/sensors/zed_obstacle_points')
        self.declare_parameter('obstacle_points_frame_id', 'base_link')
        self.declare_parameter('front_clearance_topic', '/sensors/front_clearance_m')
        self.declare_parameter('near_obstacle_topic', '/sensors/near_obstacle')
        self.declare_parameter('summary_topic', '/sensors/synced_summary')
        self.declare_parameter('use_legacy_encoder_fallback', True)
        self.declare_parameter('encoder_stale_timeout_s', 0.25)
        self.declare_parameter('encoder_buffer_duration_s', 5.0)
        self.declare_parameter('max_frame_age_s', 0.40)
        self.declare_parameter('sync_slop_s', DEFAULT_SLOP_S)
        self.declare_parameter('sync_queue_size', 5)
        self.declare_parameter('depth_warning_threshold_m', 0.30)
        self.declare_parameter('depth_min_valid_m', 0.05)
        self.declare_parameter('depth_max_valid_m', 10.0)
        self.declare_parameter('depth_roi_w_frac', 0.50)
        self.declare_parameter('depth_roi_h_frac', 0.40)
        self.declare_parameter('depth_roi_y_center_frac', 0.55)
        self.declare_parameter('depth_near_percentile', 10.0)
        self.declare_parameter('depth_projection_hfov_deg', 110.0)
        self.declare_parameter('depth_projection_stride_px', 16)
        self.declare_parameter('depth_obstacle_max_m', 3.5)
        self.declare_parameter('depth_obstacle_max_points', 160)

        scan_topic = self.get_parameter('scan_topic').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        encoder_stamped_topic = self.get_parameter('encoder_stamped_topic').value
        encoder_topic = self.get_parameter('encoder_topic').value
        output_topic = self.get_parameter('output_topic').value
        nav_frame_topic = self.get_parameter('nav_frame_topic').value
        obstacle_points_topic = self.get_parameter('obstacle_points_topic').value
        self.obstacle_points_frame_id = self.get_parameter('obstacle_points_frame_id').value
        front_clearance_topic = self.get_parameter('front_clearance_topic').value
        near_obstacle_topic = self.get_parameter('near_obstacle_topic').value
        summary_topic = self.get_parameter('summary_topic').value
        self.use_legacy_encoder_fallback = bool(self.get_parameter('use_legacy_encoder_fallback').value)
        self.encoder_stale_timeout_s = float(self.get_parameter('encoder_stale_timeout_s').value)
        self.encoder_buffer_duration_s = float(self.get_parameter('encoder_buffer_duration_s').value)
        self.max_frame_age_s = float(self.get_parameter('max_frame_age_s').value)
        sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)

        self.depth_warning_threshold_m = float(self.get_parameter('depth_warning_threshold_m').value)
        self.depth_min_valid_m = float(self.get_parameter('depth_min_valid_m').value)
        self.depth_max_valid_m = float(self.get_parameter('depth_max_valid_m').value)
        self.depth_roi_w_frac = float(self.get_parameter('depth_roi_w_frac').value)
        self.depth_roi_h_frac = float(self.get_parameter('depth_roi_h_frac').value)
        self.depth_roi_y_center_frac = float(self.get_parameter('depth_roi_y_center_frac').value)
        self.depth_near_percentile = float(self.get_parameter('depth_near_percentile').value)
        self.depth_projection_hfov_rad = math.radians(
            float(self.get_parameter('depth_projection_hfov_deg').value)
        )
        self.depth_projection_stride_px = max(1, int(self.get_parameter('depth_projection_stride_px').value))
        self.depth_obstacle_max_m = float(self.get_parameter('depth_obstacle_max_m').value)
        self.depth_obstacle_max_points = max(1, int(self.get_parameter('depth_obstacle_max_points').value))

        self.latest_encoder_stamped: Optional[dict] = None
        self.encoder_stamped_history = deque()
        self.latest_encoder_legacy: Optional[dict] = None
        self.last_encoder_warning_s = 0.0
        self.last_frame_age_warning_s = 0.0
        self.create_subscription(EncoderTicksStamped, encoder_stamped_topic, self.encoder_stamped_callback, 10)
        if self.use_legacy_encoder_fallback:
            self.create_subscription(Int32MultiArray, encoder_topic, self.encoder_callback, 10)

        scan_sub = message_filters.Subscriber(self, LaserScan, scan_topic)
        image_sub = message_filters.Subscriber(self, Image, image_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        imu_sub = message_filters.Subscriber(self, Imu, imu_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [scan_sub, image_sub, depth_sub, imu_sub],
            queue_size=max(sync_queue_size, 5),
            slop=sync_slop_s,
        )
        self.sync.registerCallback(self.fused_callback)

        self.bundle_pub = self.create_publisher(SyncedSensorPacket, output_topic, 10)
        self.nav_frame_pub = self.create_publisher(NavSensorFrame, nav_frame_topic, 10)
        self.obstacle_points_pub = self.create_publisher(PoseArray, obstacle_points_topic, 10)
        self.front_clearance_pub = self.create_publisher(Float32, front_clearance_topic, 10)
        self.near_obstacle_pub = self.create_publisher(Bool, near_obstacle_topic, 10)
        self.summary_pub = self.create_publisher(String, summary_topic, 10)

        self.get_logger().info(
            'Fusion node started '
            f'(scan={scan_topic}, image={image_topic}, depth={depth_topic}, imu={imu_topic}, '
            f'encoder_stamped={encoder_stamped_topic}, encoder_legacy={encoder_topic}, '
            f'out={output_topic}, nav_frame={nav_frame_topic}, obstacle_points={obstacle_points_topic})'
        )

    def encoder_stamped_callback(self, msg: EncoderTicksStamped) -> None:
        stamp_s = self._header_stamp_to_seconds(msg.header)
        self.latest_encoder_stamped = {
            'left': int(msg.left_ticks),
            'right': int(msg.right_ticks),
            'stamp_s': stamp_s,
            'source': 'stamped',
        }
        self.encoder_stamped_history.append(self.latest_encoder_stamped.copy())
        self._trim_encoder_history(stamp_s)

    def encoder_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= 2:
            now_s = self._clock_now_seconds()
            self.latest_encoder_legacy = {
                'left': int(msg.data[0]),
                'right': int(msg.data[1]),
                'stamp_s': now_s,
                'source': 'legacy',
            }

    def fused_callback(
        self,
        scan_msg: LaserScan,
        image_msg: Image,
        depth_msg: Image,
        imu_msg: Imu,
    ) -> None:
        frame_stamp_s = self._header_stamp_to_seconds(scan_msg.header)
        frame_age_s = max(0.0, self._clock_now_seconds() - frame_stamp_s)
        if frame_age_s > self.max_frame_age_s:
            self._warn_frame_delay(
                f'dropping stale fused frame before publish (frame_age={frame_age_s:.3f}s)'
            )
            return

        lidar_min_range_m = self._compute_lidar_min_range(scan_msg)
        depth_min_range_m, depth_warning, valid_depth_samples = self._compute_depth_stats(depth_msg)
        zed_obstacle_points = self._build_depth_obstacle_pose_array(depth_msg)
        front_clearance_m = min(lidar_min_range_m, depth_min_range_m)
        near_obstacle = math.isfinite(front_clearance_m) and front_clearance_m < self.depth_warning_threshold_m
        encoder_left, encoder_right, encoder_available, encoder_age_s, encoder_source = self._select_encoder_for_frame(
            scan_msg.header
        )

        bundle = SyncedSensorPacket()
        bundle.header = scan_msg.header
        bundle.scan = scan_msg
        bundle.image = image_msg
        bundle.depth = depth_msg
        bundle.imu = imu_msg
        bundle.zed_obstacle_points = zed_obstacle_points
        bundle.left_encoder_ticks = int(encoder_left)
        bundle.right_encoder_ticks = int(encoder_right)
        bundle.encoder_available = bool(encoder_available)
        bundle.min_lidar_range_m = float(lidar_min_range_m)
        bundle.min_depth_range_m = float(depth_min_range_m)
        bundle.front_clearance_m = float(front_clearance_m)
        bundle.depth_warning = bool(depth_warning)
        bundle.near_obstacle = bool(near_obstacle)
        self.bundle_pub.publish(bundle)

        nav_frame = NavSensorFrame()
        nav_frame.header = bundle.header
        nav_frame.scan = scan_msg
        nav_frame.zed_obstacle_points = zed_obstacle_points
        nav_frame.left_encoder_ticks = int(encoder_left)
        nav_frame.right_encoder_ticks = int(encoder_right)
        nav_frame.encoder_available = bool(encoder_available)
        nav_frame.min_lidar_range_m = float(lidar_min_range_m)
        nav_frame.min_depth_range_m = float(depth_min_range_m)
        nav_frame.front_clearance_m = float(front_clearance_m)
        nav_frame.near_obstacle = bool(near_obstacle)
        self.nav_frame_pub.publish(nav_frame)

        self.obstacle_points_pub.publish(zed_obstacle_points)
        self.front_clearance_pub.publish(Float32(data=float(front_clearance_m)))
        self.near_obstacle_pub.publish(Bool(data=bool(near_obstacle)))

        summary = {
            'stamp_sec': self._stamp_to_seconds(scan_msg),
            'frame_age_s': round(frame_age_s, 3),
            'scan_frame': scan_msg.header.frame_id,
            'image_frame': image_msg.header.frame_id,
            'depth_frame': depth_msg.header.frame_id,
            'imu_frame': imu_msg.header.frame_id,
            'scan_points': len(scan_msg.ranges),
            'zed_obstacle_points': len(zed_obstacle_points.poses),
            'valid_depth_samples': valid_depth_samples,
            'encoder_available': bool(bundle.encoder_available),
            'left_encoder_ticks': int(bundle.left_encoder_ticks),
            'right_encoder_ticks': int(bundle.right_encoder_ticks),
            'encoder_age_s': self._finite_or_none(encoder_age_s),
            'encoder_source': encoder_source,
            'min_lidar_range_m': self._finite_or_none(lidar_min_range_m),
            'min_depth_range_m': self._finite_or_none(depth_min_range_m),
            'front_clearance_m': self._finite_or_none(front_clearance_m),
            'depth_warning': bool(depth_warning),
            'near_obstacle': bool(near_obstacle),
        }
        self.summary_pub.publish(String(data=json.dumps(summary)))

    def _select_encoder_for_frame(self, frame_header):
        frame_stamp_s = self._header_stamp_to_seconds(frame_header)
        candidates = []
        candidates.extend(self.encoder_stamped_history)
        if not candidates and self.latest_encoder_stamped is not None:
            candidates.append(self.latest_encoder_stamped)
        if self.use_legacy_encoder_fallback and self.latest_encoder_legacy is not None:
            candidates.append(self.latest_encoder_legacy)

        best = None
        best_age_s = float('inf')
        for candidate in candidates:
            age_s = abs(frame_stamp_s - float(candidate['stamp_s']))
            if age_s < best_age_s:
                best = candidate
                best_age_s = age_s

        if best is None:
            self._warn_encoder_gap('no encoder frames available yet')
            return 0, 0, False, float('inf'), 'none'

        if best_age_s > self.encoder_stale_timeout_s:
            self._warn_encoder_gap(
                f'encoder data stale for fused frame (age={best_age_s:.3f}s, source={best["source"]})'
            )
            return 0, 0, False, best_age_s, str(best['source'])

        return (
            int(best['left']),
            int(best['right']),
            True,
            float(best_age_s),
            str(best['source']),
        )

    def _compute_lidar_min_range(self, scan_msg: LaserScan) -> float:
        valid = [r for r in scan_msg.ranges if scan_msg.range_min < r < scan_msg.range_max]
        return min(valid) if valid else float('inf')

    def _compute_depth_stats(self, depth_msg: Image):
        depth = self._depth_image_to_numpy(depth_msg)
        if depth is None:
            return float('inf'), True, 0

        roi = self._extract_depth_roi(depth)
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.depth_min_valid_m) & (valid <= self.depth_max_valid_m)]
        if valid.size == 0:
            return float('inf'), True, 0

        near_distance_m = float(np.percentile(valid, self.depth_near_percentile))
        depth_warning = near_distance_m < self.depth_warning_threshold_m
        return near_distance_m, depth_warning, int(valid.size)

    def _build_depth_obstacle_pose_array(self, depth_msg: Image) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header.stamp = depth_msg.header.stamp
        pose_array.header.frame_id = self.obstacle_points_frame_id or depth_msg.header.frame_id

        depth = self._depth_image_to_numpy(depth_msg)
        if depth is None:
            return pose_array

        roi, x0, _, y0, _ = self._extract_depth_roi(depth, return_bounds=True)
        points = self._project_depth_roi_to_points(roi, x0, y0, depth.shape[1])

        for x_m, y_m in points:
            pose = Pose()
            pose.position.x = float(x_m)
            pose.position.y = float(y_m)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array

    def _project_depth_roi_to_points(
        self,
        roi: np.ndarray,
        roi_x0: int,
        roi_y0: int,
        full_width: int,
    ) -> List[Tuple[float, float]]:
        del roi_y0
        points: List[Tuple[float, float]] = []
        if roi.size == 0:
            return points

        center_x = 0.5 * max(full_width - 1, 1)
        half_fov = 0.5 * self.depth_projection_hfov_rad
        stride = self.depth_projection_stride_px

        for row in range(0, roi.shape[0], stride):
            for col in range(0, roi.shape[1], stride):
                depth_m = float(roi[row, col])
                if not math.isfinite(depth_m):
                    continue
                if depth_m < self.depth_min_valid_m or depth_m > min(self.depth_max_valid_m, self.depth_obstacle_max_m):
                    continue

                pixel_x = roi_x0 + col
                norm_x = (pixel_x - center_x) / max(center_x, 1.0)
                angle_x = norm_x * half_fov
                x_m = depth_m * math.cos(angle_x)
                y_m = depth_m * math.sin(angle_x)
                points.append((x_m, y_m))

        if len(points) <= self.depth_obstacle_max_points:
            return points

        step = max(1, int(math.ceil(len(points) / self.depth_obstacle_max_points)))
        return points[::step][:self.depth_obstacle_max_points]

    def _depth_image_to_numpy(self, depth_msg: Image):
        if depth_msg.encoding != '32FC1':
            self.get_logger().warn(
                f"Expected depth encoding 32FC1, received {depth_msg.encoding}. "
                'Depth-based pathing outputs will be marked unsafe.'
            )
            return None

        try:
            return np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
        except ValueError:
            self.get_logger().warn('Depth image shape mismatch; skipping this fused frame.')
            return None

    def _extract_depth_roi(self, depth: np.ndarray, return_bounds: bool = False):
        height, width = depth.shape[:2]

        roi_w = max(10, int(width * self.depth_roi_w_frac))
        roi_h = max(10, int(height * self.depth_roi_h_frac))
        center_x = width // 2
        center_y = int(height * self.depth_roi_y_center_frac)

        x0 = max(0, center_x - roi_w // 2)
        x1 = min(width, center_x + roi_w // 2)
        y0 = max(0, center_y - roi_h // 2)
        y1 = min(height, center_y + roi_h // 2)

        roi = depth[y0:y1, x0:x1]
        if return_bounds:
            return roi, x0, x1, y0, y1
        return roi

    @staticmethod
    def _stamp_to_seconds(msg: LaserScan) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    @staticmethod
    def _header_stamp_to_seconds(header) -> float:
        return float(header.stamp.sec) + float(header.stamp.nanosec) / 1e9

    def _clock_now_seconds(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) / 1e9

    def _trim_encoder_history(self, newest_stamp_s: float) -> None:
        cutoff_s = newest_stamp_s - max(self.encoder_buffer_duration_s, 0.5)
        while self.encoder_stamped_history and float(self.encoder_stamped_history[0]['stamp_s']) < cutoff_s:
            self.encoder_stamped_history.popleft()

    def _warn_encoder_gap(self, message: str, period_s: float = 2.0) -> None:
        now_s = self._clock_now_seconds()
        if now_s - self.last_encoder_warning_s >= period_s:
            self.get_logger().warn(message)
            self.last_encoder_warning_s = now_s

    def _warn_frame_delay(self, message: str, period_s: float = 2.0) -> None:
        now_s = self._clock_now_seconds()
        if now_s - self.last_frame_age_warning_s >= period_s:
            self.get_logger().warn(message)
            self.last_frame_age_warning_s = now_s

    @staticmethod
    def _finite_or_none(value: float):
        return value if math.isfinite(value) else None


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
