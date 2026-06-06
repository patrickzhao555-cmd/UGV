#!/usr/bin/env python3
from collections import deque
import json
import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int32MultiArray, String

from ugv_sensor_sync.msg import EncoderTicksStamped, NavSensorFrame, SyncedSensorPacket
from ugv_nav_core.nav2_bridge import clustered_lidar_min_range


DEFAULT_SLOP_S = 0.25
FRONT_OBSTACLE_DEBUG_THRESHOLD_M = 2.0
FRONT_STOP_DEBUG_CLEARANCE_M = 1.0


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
        self.declare_parameter('semantic_obstacle_points_topic', '/sensors/yolo_semantic_obstacle_points')
        self.declare_parameter('semantic_obstacle_timeout_s', 0.75)
        self.declare_parameter('front_clearance_topic', '/sensors/front_clearance_m')
        self.declare_parameter('near_obstacle_topic', '/sensors/near_obstacle')
        self.declare_parameter('summary_topic', '/sensors/synced_summary')
        self.declare_parameter('subscribe_image_topic', False)
        self.declare_parameter('use_encoder_pair_fallback', True)
        self.declare_parameter('encoder_stale_timeout_s', 0.25)
        self.declare_parameter('encoder_buffer_duration_s', 5.0)
        self.declare_parameter('max_frame_age_s', 0.40)
        self.declare_parameter('zed_frame_buffer_duration_s', 2.0)
        self.declare_parameter('sync_slop_s', DEFAULT_SLOP_S)
        self.declare_parameter('zed_fresh_timeout_s', 0.75)
        self.declare_parameter('allow_lidar_only_fallback', True)
        self.declare_parameter('depth_warning_threshold_m', 0.30)
        self.declare_parameter('depth_min_valid_m', 0.05)
        self.declare_parameter('depth_max_valid_m', 10.0)
        self.declare_parameter('depth_roi_w_frac', 0.50)
        self.declare_parameter('depth_roi_h_frac', 0.40)
        self.declare_parameter('depth_roi_y_center_frac', 0.55)
        self.declare_parameter('depth_near_percentile', 10.0)
        self.declare_parameter('depth_invalid_warn_frames', 2)
        self.declare_parameter('lidar_front_fov_deg', 70.0)
        self.declare_parameter('lidar_front_min_cluster_points', 3)
        self.declare_parameter('lidar_front_cluster_max_gap_m', 0.35)
        self.declare_parameter('depth_projection_hfov_deg', 110.0)
        self.declare_parameter('depth_projection_stride_px', 8)
        self.declare_parameter('depth_obstacle_max_m', 3.5)
        self.declare_parameter('depth_obstacle_max_points', 240)
        self.declare_parameter('depth_ground_filter_enabled', True)
        self.declare_parameter('depth_ground_row_percentile', 72.0)
        self.declare_parameter('depth_ground_min_delta_m', 0.18)
        self.declare_parameter('depth_ground_ratio', 0.88)
        self.declare_parameter('depth_obstacle_min_block_pixels', 2)
        self.declare_parameter('depth_obstacle_min_component_cells', 2)
        self.declare_parameter('depth_obstacle_min_component_height_px', 14)
        self.declare_parameter('depth_front_corridor_half_width_m', 0.42)
        self.declare_parameter('imu_smoothing_alpha', 0.25)

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
        semantic_obstacle_points_topic = self.get_parameter('semantic_obstacle_points_topic').value
        self.semantic_obstacle_timeout_s = float(self.get_parameter('semantic_obstacle_timeout_s').value)
        front_clearance_topic = self.get_parameter('front_clearance_topic').value
        near_obstacle_topic = self.get_parameter('near_obstacle_topic').value
        summary_topic = self.get_parameter('summary_topic').value
        self.subscribe_image_topic = bool(self.get_parameter('subscribe_image_topic').value)
        self.use_encoder_pair_fallback = bool(self.get_parameter('use_encoder_pair_fallback').value)
        self.encoder_stale_timeout_s = float(self.get_parameter('encoder_stale_timeout_s').value)
        self.encoder_buffer_duration_s = float(self.get_parameter('encoder_buffer_duration_s').value)
        self.max_frame_age_s = float(self.get_parameter('max_frame_age_s').value)
        self.zed_frame_buffer_duration_s = float(self.get_parameter('zed_frame_buffer_duration_s').value)
        self.sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        self.zed_fresh_timeout_s = float(self.get_parameter('zed_fresh_timeout_s').value)
        self.allow_lidar_only_fallback = bool(self.get_parameter('allow_lidar_only_fallback').value)

        self.depth_warning_threshold_m = float(self.get_parameter('depth_warning_threshold_m').value)
        self.depth_min_valid_m = float(self.get_parameter('depth_min_valid_m').value)
        self.depth_max_valid_m = float(self.get_parameter('depth_max_valid_m').value)
        self.depth_roi_w_frac = float(self.get_parameter('depth_roi_w_frac').value)
        self.depth_roi_h_frac = float(self.get_parameter('depth_roi_h_frac').value)
        self.depth_roi_y_center_frac = float(self.get_parameter('depth_roi_y_center_frac').value)
        self.depth_near_percentile = float(self.get_parameter('depth_near_percentile').value)
        self.depth_invalid_warn_frames = max(1, int(self.get_parameter('depth_invalid_warn_frames').value))
        self.lidar_front_fov_deg = float(self.get_parameter('lidar_front_fov_deg').value)
        self.lidar_front_half_fov_rad = 0.5 * math.radians(self.lidar_front_fov_deg)
        self.lidar_front_min_cluster_points = max(1, int(self.get_parameter('lidar_front_min_cluster_points').value))
        self.lidar_front_cluster_max_gap_m = max(0.0, float(self.get_parameter('lidar_front_cluster_max_gap_m').value))
        self.depth_projection_hfov_rad = math.radians(
            float(self.get_parameter('depth_projection_hfov_deg').value)
        )
        self.depth_projection_stride_px = max(1, int(self.get_parameter('depth_projection_stride_px').value))
        self.depth_obstacle_max_m = float(self.get_parameter('depth_obstacle_max_m').value)
        self.depth_obstacle_max_points = max(1, int(self.get_parameter('depth_obstacle_max_points').value))
        self.depth_ground_filter_enabled = bool(self.get_parameter('depth_ground_filter_enabled').value)
        self.depth_ground_row_percentile = float(self.get_parameter('depth_ground_row_percentile').value)
        self.depth_ground_min_delta_m = max(0.0, float(self.get_parameter('depth_ground_min_delta_m').value))
        self.depth_ground_ratio = min(0.99, max(0.10, float(self.get_parameter('depth_ground_ratio').value)))
        self.depth_obstacle_min_block_pixels = max(1, int(self.get_parameter('depth_obstacle_min_block_pixels').value))
        self.depth_obstacle_min_component_cells = max(1, int(self.get_parameter('depth_obstacle_min_component_cells').value))
        self.depth_obstacle_min_component_height_px = max(1, int(self.get_parameter('depth_obstacle_min_component_height_px').value))
        self.depth_front_corridor_half_width_m = max(0.05, float(self.get_parameter('depth_front_corridor_half_width_m').value))
        self.imu_smoothing_alpha = min(1.0, max(0.0, float(self.get_parameter('imu_smoothing_alpha').value)))

        self.latest_encoder_stamped: Optional[dict] = None
        self.encoder_stamped_history = deque()
        self.latest_encoder_pair: Optional[dict] = None
        self.zed_frame_history = deque()
        self.zed_frame_map = {}
        self.latest_imu_msg: Optional[Imu] = None
        self.latest_imu_stamp_s: Optional[float] = None
        self.latest_imu_received_s: Optional[float] = None
        self.latest_semantic_obstacles: Optional[dict] = None
        self.last_encoder_warning_s = 0.0
        self.last_frame_age_warning_s = 0.0
        self.last_zed_warning_s = 0.0
        self.last_lidar_only_notice_s = 0.0
        self.depth_invalid_streak = 0
        self.smoothed_imu_msg: Optional[Imu] = None
        self._depth_obstacle_cache = None
        self.last_depth_obstacle_metrics = {
            'depth_ground_filter_enabled': self.depth_ground_filter_enabled,
            'depth_obstacle_candidate_cells': 0,
            'depth_obstacle_components': 0,
            'depth_obstacle_points_filtered': 0,
        }
        self.create_subscription(EncoderTicksStamped, encoder_stamped_topic, self.encoder_stamped_callback, qos_profile_sensor_data)
        if self.use_encoder_pair_fallback:
            self.create_subscription(Int32MultiArray, encoder_topic, self.encoder_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        if semantic_obstacle_points_topic:
            self.create_subscription(
                PoseArray,
                semantic_obstacle_points_topic,
                self.semantic_obstacles_callback,
                qos_profile_sensor_data,
            )
        if self.subscribe_image_topic:
            self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sensor_data)

        self.bundle_pub = self.create_publisher(SyncedSensorPacket, output_topic, 10)
        self.nav_frame_pub = self.create_publisher(NavSensorFrame, nav_frame_topic, 10)
        self.obstacle_points_pub = self.create_publisher(PoseArray, obstacle_points_topic, 10)
        self.front_clearance_pub = self.create_publisher(Float32, front_clearance_topic, 10)
        self.near_obstacle_pub = self.create_publisher(Bool, near_obstacle_topic, 10)
        self.summary_pub = self.create_publisher(String, summary_topic, 10)

        self.get_logger().info(
            'Fusion node started '
            f'(scan={scan_topic}, image={"disabled" if not self.subscribe_image_topic else image_topic}, '
            f'depth={depth_topic}, imu={imu_topic}, '
            f'encoder_stamped={encoder_stamped_topic}, encoder_pair={encoder_topic}, '
            f'out={output_topic}, nav_frame={nav_frame_topic}, obstacle_points={obstacle_points_topic}, '
            f'semantic_obstacles={semantic_obstacle_points_topic or "disabled"})'
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
            self.latest_encoder_pair = {
                'left': int(msg.data[0]),
                'right': int(msg.data[1]),
                'stamp_s': now_s,
                'source': 'encoder_pair',
            }

    def image_callback(self, msg: Image) -> None:
        frame = self._upsert_zed_frame(msg.header)
        frame['image'] = msg

    def depth_callback(self, msg: Image) -> None:
        frame = self._upsert_zed_frame(msg.header)
        frame['depth'] = msg

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu_msg = msg
        self.latest_imu_stamp_s = self._header_stamp_to_seconds(msg.header)
        self.latest_imu_received_s = self._clock_now_seconds()

    def semantic_obstacles_callback(self, msg: PoseArray) -> None:
        self.latest_semantic_obstacles = {
            'msg': msg,
            'received_s': self._clock_now_seconds(),
        }

    def scan_callback(self, scan_msg: LaserScan) -> None:
        zed_selection = self._select_zed_frame_for_scan(
            scan_msg.header,
            warn_if_missing=not self.allow_lidar_only_fallback,
        )
        if zed_selection is None:
            if not self.allow_lidar_only_fallback:
                return
            self._notice_lidar_only_fallback()
            self.fused_callback(
                scan_msg,
                self._empty_image_like(scan_msg.header),
                self._empty_depth_like(scan_msg.header),
                self._empty_imu_like(scan_msg.header),
                zed_stamp_age_s=float('inf'),
                zed_receive_age_s=float('inf'),
                zed_available=False,
            )
            return
        zed_frame, zed_stamp_age_s, zed_receive_age_s = zed_selection
        image_msg = zed_frame.get('image')
        if image_msg is None:
            image_msg = self._empty_image_like(zed_frame['depth'].header)
        imu_msg = self._select_imu_for_scan(scan_msg.header)
        if imu_msg is None:
            imu_msg = self._empty_imu_like(scan_msg.header)
        self.fused_callback(
            scan_msg,
            image_msg,
            zed_frame['depth'],
            imu_msg,
            zed_stamp_age_s=zed_stamp_age_s,
            zed_receive_age_s=zed_receive_age_s,
            zed_available=True,
        )

    def fused_callback(
        self,
        scan_msg: LaserScan,
        image_msg: Image,
        depth_msg: Image,
        imu_msg: Imu,
        zed_stamp_age_s: float = 0.0,
        zed_receive_age_s: float = 0.0,
        zed_available: bool = True,
    ) -> None:
        frame_stamp_s = self._header_stamp_to_seconds(scan_msg.header)
        frame_age_s = max(0.0, self._clock_now_seconds() - frame_stamp_s)
        if frame_age_s > self.max_frame_age_s:
            self._warn_frame_delay(
                f'dropping stale fused frame before publish (frame_age={frame_age_s:.3f}s)'
            )
            return

        lidar_min_range_m = self._compute_lidar_min_range(scan_msg)
        front_lidar_range_m = self._compute_front_lidar_min_range(scan_msg)
        if zed_available:
            depth_min_range_m, depth_warning, valid_depth_samples = self._compute_depth_stats(depth_msg)
            if valid_depth_samples > 0:
                self.depth_invalid_streak = 0
            else:
                self.depth_invalid_streak += 1
            depth_blind_hazard = self.depth_invalid_streak >= self.depth_invalid_warn_frames
            zed_obstacle_points = self._build_depth_obstacle_pose_array(depth_msg)
        else:
            depth_min_range_m = float('inf')
            depth_warning = False
            valid_depth_samples = 0
            self.depth_invalid_streak = 0
            depth_blind_hazard = False
            zed_obstacle_points = self._empty_obstacle_pose_array(scan_msg.header)
        depth_obstacle_point_count = len(zed_obstacle_points.poses)
        semantic_obstacle_points = self._select_semantic_obstacles(scan_msg.header)
        semantic_obstacle_point_count = len(semantic_obstacle_points.poses)
        if semantic_obstacle_point_count:
            zed_obstacle_points = self._merge_pose_arrays(
                zed_obstacle_points,
                semantic_obstacle_points,
                scan_msg.header,
            )
        front_clearance_m = min(front_lidar_range_m, depth_min_range_m)
        front_clearance_source = self._clearance_source(front_lidar_range_m, depth_min_range_m)
        lidar_front_clear = (
            math.isfinite(front_lidar_range_m)
            and front_lidar_range_m >= self.depth_warning_threshold_m
        )
        depth_blind_hazard_active = depth_blind_hazard and not lidar_front_clear
        near_obstacle = depth_blind_hazard_active or (
            math.isfinite(front_clearance_m) and front_clearance_m < self.depth_warning_threshold_m
        )
        front_obstacle_within_2m = (
            math.isfinite(front_clearance_m)
            and front_clearance_m <= FRONT_OBSTACLE_DEBUG_THRESHOLD_M
        )
        front_stop_required_1m = (
            math.isfinite(front_clearance_m)
            and front_clearance_m < FRONT_STOP_DEBUG_CLEARANCE_M
        )
        front_sensor_health = self._front_sensor_health(
            zed_available=zed_available,
            valid_depth_samples=valid_depth_samples,
            depth_blind_hazard_active=depth_blind_hazard_active,
            front_lidar_range_m=front_lidar_range_m,
            depth_min_range_m=depth_min_range_m,
        )
        smoothed_imu_msg = self._smooth_imu(imu_msg)
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
        if hasattr(nav_frame, 'imu'):
            nav_frame.imu = smoothed_imu_msg
        nav_frame.zed_obstacle_points = zed_obstacle_points
        nav_frame.left_encoder_ticks = int(encoder_left)
        nav_frame.right_encoder_ticks = int(encoder_right)
        nav_frame.encoder_available = bool(encoder_available)
        nav_frame.min_lidar_range_m = float(lidar_min_range_m)
        nav_frame.min_depth_range_m = float(depth_min_range_m)
        nav_frame.front_clearance_m = float(front_clearance_m)
        nav_frame.near_obstacle = bool(near_obstacle)
        if hasattr(nav_frame, 'depth_blind_hazard'):
            nav_frame.depth_blind_hazard = bool(depth_blind_hazard)
        if hasattr(nav_frame, 'front_clearance_source'):
            nav_frame.front_clearance_source = str(front_clearance_source)
        if hasattr(nav_frame, 'front_lidar_range_m'):
            nav_frame.front_lidar_range_m = float(front_lidar_range_m)
        self.nav_frame_pub.publish(nav_frame)

        self.obstacle_points_pub.publish(zed_obstacle_points)
        self.front_clearance_pub.publish(Float32(data=float(front_clearance_m)))
        self.near_obstacle_pub.publish(Bool(data=bool(near_obstacle)))

        summary = {
            'stamp_sec': self._stamp_to_seconds(scan_msg),
            'frame_age_s': round(frame_age_s, 3),
            'zed_available': bool(zed_available),
            'zed_stamp_age_s': self._finite_or_none(zed_stamp_age_s),
            'zed_receive_age_s': self._finite_or_none(zed_receive_age_s),
            'scan_frame': scan_msg.header.frame_id,
            'image_frame': image_msg.header.frame_id,
            'depth_frame': depth_msg.header.frame_id,
            'imu_frame': imu_msg.header.frame_id,
            'scan_points': len(scan_msg.ranges),
            'zed_obstacle_points': len(zed_obstacle_points.poses),
            'depth_obstacle_points': depth_obstacle_point_count,
            **self.last_depth_obstacle_metrics,
            'semantic_obstacle_points': semantic_obstacle_point_count,
            'valid_depth_samples': valid_depth_samples,
            'depth_invalid_streak': self.depth_invalid_streak,
            'encoder_available': bool(bundle.encoder_available),
            'left_encoder_ticks': int(bundle.left_encoder_ticks),
            'right_encoder_ticks': int(bundle.right_encoder_ticks),
            'encoder_age_s': self._finite_or_none(encoder_age_s),
            'encoder_source': encoder_source,
            'imu_linear_accel_mps2': [
                round(float(imu_msg.linear_acceleration.x), 4),
                round(float(imu_msg.linear_acceleration.y), 4),
                round(float(imu_msg.linear_acceleration.z), 4),
            ],
            'imu_angular_velocity_rps': [
                round(float(imu_msg.angular_velocity.x), 4),
                round(float(imu_msg.angular_velocity.y), 4),
                round(float(imu_msg.angular_velocity.z), 4),
            ],
            'imu_linear_accel_smoothed_mps2': [
                round(float(smoothed_imu_msg.linear_acceleration.x), 4),
                round(float(smoothed_imu_msg.linear_acceleration.y), 4),
                round(float(smoothed_imu_msg.linear_acceleration.z), 4),
            ],
            'imu_angular_velocity_smoothed_rps': [
                round(float(smoothed_imu_msg.angular_velocity.x), 4),
                round(float(smoothed_imu_msg.angular_velocity.y), 4),
                round(float(smoothed_imu_msg.angular_velocity.z), 4),
            ],
            'min_lidar_range_m': self._finite_or_none(lidar_min_range_m),
            'lidar_any_min_range_m': self._finite_or_none(lidar_min_range_m),
            'front_lidar_range_m': self._finite_or_none(front_lidar_range_m),
            'front_lidar_fov_deg': round(float(self.lidar_front_fov_deg), 3),
            'front_lidar_min_cluster_points': int(self.lidar_front_min_cluster_points),
            'front_lidar_cluster_max_gap_m': round(float(self.lidar_front_cluster_max_gap_m), 3),
            'min_depth_range_m': self._finite_or_none(depth_min_range_m),
            'depth_corridor_half_width_m': round(float(self.depth_front_corridor_half_width_m), 3),
            'front_clearance_m': self._finite_or_none(front_clearance_m),
            'front_clearance_source': front_clearance_source,
            'front_obstacle_within_2m': bool(front_obstacle_within_2m),
            'front_stop_required_1m': bool(front_stop_required_1m),
            'front_sensor_health': front_sensor_health,
            'depth_warning': bool(depth_warning),
            'depth_blind_hazard': bool(depth_blind_hazard),
            'depth_blind_hazard_active': bool(depth_blind_hazard_active),
            'near_obstacle': bool(near_obstacle),
        }
        self.summary_pub.publish(String(data=json.dumps(summary)))

    def _select_semantic_obstacles(self, header) -> PoseArray:
        if self.latest_semantic_obstacles is None:
            return self._empty_obstacle_pose_array(header)
        msg = self.latest_semantic_obstacles['msg']
        receive_age_s = max(0.0, self._clock_now_seconds() - float(self.latest_semantic_obstacles['received_s']))
        if receive_age_s > self.semantic_obstacle_timeout_s:
            return self._empty_obstacle_pose_array(header)
        return msg

    def _merge_pose_arrays(self, primary: PoseArray, semantic: PoseArray, header) -> PoseArray:
        merged = PoseArray()
        merged.header.stamp = header.stamp
        merged.header.frame_id = self.obstacle_points_frame_id or primary.header.frame_id or semantic.header.frame_id
        merged.poses.extend(primary.poses)
        merged.poses.extend(semantic.poses)
        return merged

    def _smooth_imu(self, imu_msg: Imu) -> Imu:
        alpha = self.imu_smoothing_alpha
        if self.smoothed_imu_msg is None or alpha >= 1.0:
            out = Imu()
            out.header = imu_msg.header
            out.orientation = imu_msg.orientation
            out.orientation_covariance = imu_msg.orientation_covariance
            out.linear_acceleration = imu_msg.linear_acceleration
            out.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
            out.angular_velocity = imu_msg.angular_velocity
            out.angular_velocity_covariance = imu_msg.angular_velocity_covariance
            self.smoothed_imu_msg = out
            return out
        if alpha <= 0.0:
            self.smoothed_imu_msg.header = imu_msg.header
            return self.smoothed_imu_msg

        prev = self.smoothed_imu_msg
        out = Imu()
        out.header = imu_msg.header
        out.orientation = imu_msg.orientation
        out.orientation_covariance = imu_msg.orientation_covariance
        out.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        out.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        out.linear_acceleration.x = self._low_pass(prev.linear_acceleration.x, imu_msg.linear_acceleration.x, alpha)
        out.linear_acceleration.y = self._low_pass(prev.linear_acceleration.y, imu_msg.linear_acceleration.y, alpha)
        out.linear_acceleration.z = self._low_pass(prev.linear_acceleration.z, imu_msg.linear_acceleration.z, alpha)
        out.angular_velocity.x = self._low_pass(prev.angular_velocity.x, imu_msg.angular_velocity.x, alpha)
        out.angular_velocity.y = self._low_pass(prev.angular_velocity.y, imu_msg.angular_velocity.y, alpha)
        out.angular_velocity.z = self._low_pass(prev.angular_velocity.z, imu_msg.angular_velocity.z, alpha)
        self.smoothed_imu_msg = out
        return out

    @staticmethod
    def _low_pass(prev: float, cur: float, alpha: float) -> float:
        return (1.0 - alpha) * float(prev) + alpha * float(cur)

    def _select_encoder_for_frame(self, frame_header):
        frame_stamp_s = self._header_stamp_to_seconds(frame_header)
        candidates = []
        candidates.extend(self.encoder_stamped_history)
        if not candidates and self.latest_encoder_stamped is not None:
            candidates.append(self.latest_encoder_stamped)
        if self.use_encoder_pair_fallback and self.latest_encoder_pair is not None:
            candidates.append(self.latest_encoder_pair)

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

    def _upsert_zed_frame(self, header):
        stamp_s = self._header_stamp_to_seconds(header)
        received_s = self._clock_now_seconds()
        key = self._header_stamp_key(header)
        frame = self.zed_frame_map.get(key)
        if frame is None:
            frame = {
                'stamp_s': stamp_s,
                'received_s': received_s,
                'image': None,
                'depth': None,
                'imu': None,
            }
            self.zed_frame_map[key] = frame
            self.zed_frame_history.append((key, frame))
        else:
            frame['stamp_s'] = stamp_s
            frame['received_s'] = received_s
        self._trim_zed_history(stamp_s)
        return frame

    def _select_zed_frame_for_scan(self, scan_header, warn_if_missing: bool = True):
        frame_stamp_s = self._header_stamp_to_seconds(scan_header)
        now_s = self._clock_now_seconds()
        best = None
        best_age_s = float('inf')
        freshest = None
        freshest_receive_age_s = float('inf')
        freshest_stamp_age_s = float('inf')
        for _, frame in self.zed_frame_history:
            if frame.get('depth') is None:
                continue
            age_s = abs(frame_stamp_s - float(frame['stamp_s']))
            receive_age_s = max(0.0, now_s - float(frame.get('received_s', frame['stamp_s'])))
            if age_s < best_age_s:
                best = frame
                best_age_s = age_s
            if receive_age_s < freshest_receive_age_s:
                freshest = frame
                freshest_receive_age_s = receive_age_s
                freshest_stamp_age_s = age_s

        if best is None:
            if warn_if_missing:
                self._warn_zed_gap('no complete ZED frame available yet for scan fusion')
            return None

        if best_age_s > self.sync_slop_s:
            if freshest is not None and freshest_receive_age_s <= self.zed_fresh_timeout_s:
                if warn_if_missing:
                    self._warn_zed_gap(
                        'using freshest complete ZED frame despite header mismatch '
                        f'(stamp_age={freshest_stamp_age_s:.3f}s, recv_age={freshest_receive_age_s:.3f}s)'
                    )
                return freshest, freshest_stamp_age_s, freshest_receive_age_s
            if warn_if_missing:
                self._warn_zed_gap(
                    'no fresh ZED frame available for scan fusion '
                    f'(best_stamp_age={best_age_s:.3f}s, freshest_stamp_age={freshest_stamp_age_s:.3f}s, '
                    f'freshest_recv_age={freshest_receive_age_s:.3f}s)'
                )
            return None

        best_receive_age_s = max(0.0, now_s - float(best.get('received_s', best['stamp_s'])))
        return best, best_age_s, best_receive_age_s

    def _select_imu_for_scan(self, scan_header) -> Optional[Imu]:
        if self.latest_imu_msg is None or self.latest_imu_stamp_s is None or self.latest_imu_received_s is None:
            return None
        now_s = self._clock_now_seconds()
        receive_age_s = max(0.0, now_s - float(self.latest_imu_received_s))
        if receive_age_s > self.zed_fresh_timeout_s:
            self._warn_zed_gap(f'latest ZED IMU stale for scan fusion (recv_age={receive_age_s:.3f}s)')
            return None
        scan_stamp_s = self._header_stamp_to_seconds(scan_header)
        stamp_age_s = abs(scan_stamp_s - float(self.latest_imu_stamp_s))
        if stamp_age_s > max(self.sync_slop_s, self.zed_fresh_timeout_s):
            self._warn_zed_gap(f'latest ZED IMU stamp far from scan (stamp_age={stamp_age_s:.3f}s)')
        return self.latest_imu_msg

    def _compute_lidar_min_range(self, scan_msg: LaserScan) -> float:
        valid = [r for r in scan_msg.ranges if scan_msg.range_min < r < scan_msg.range_max]
        return min(valid) if valid else float('inf')

    @staticmethod
    def _clearance_source(front_lidar_range_m: float, depth_min_range_m: float) -> str:
        lidar_ok = math.isfinite(front_lidar_range_m)
        depth_ok = math.isfinite(depth_min_range_m)
        if lidar_ok and depth_ok:
            if abs(front_lidar_range_m - depth_min_range_m) < 0.03:
                return 'lidar+zed'
            return 'lidar' if front_lidar_range_m < depth_min_range_m else 'zed'
        if lidar_ok:
            return 'lidar'
        if depth_ok:
            return 'zed'
        return 'none'

    @staticmethod
    def _front_sensor_health(
        *,
        zed_available: bool,
        valid_depth_samples: int,
        depth_blind_hazard_active: bool,
        front_lidar_range_m: float,
        depth_min_range_m: float,
    ) -> str:
        lidar_ok = math.isfinite(front_lidar_range_m)
        depth_ok = math.isfinite(depth_min_range_m)
        if lidar_ok and depth_ok:
            return 'lidar+zed_ok'
        if lidar_ok:
            if not zed_available:
                return 'lidar_only_zed_unavailable'
            if depth_blind_hazard_active:
                return 'lidar_only_depth_blind'
            if valid_depth_samples <= 0:
                return 'lidar_only_depth_invalid'
            return 'lidar_only_no_depth_corridor_obstacle'
        if depth_ok:
            return 'zed_only_no_lidar_front'
        if not zed_available:
            return 'no_front_sensor_zed_unavailable'
        if valid_depth_samples <= 0:
            return 'no_front_sensor_depth_invalid'
        return 'no_front_obstacle_points'

    def _compute_front_lidar_min_range(self, scan_msg: LaserScan) -> float:
        return clustered_lidar_min_range(
            ranges=list(scan_msg.ranges),
            angle_min_rad=float(scan_msg.angle_min),
            angle_increment_rad=float(scan_msg.angle_increment),
            range_min_m=float(scan_msg.range_min),
            range_max_m=float(scan_msg.range_max),
            fov_deg=float(self.lidar_front_fov_deg),
            min_cluster_points=self.lidar_front_min_cluster_points,
            max_cluster_gap_m=self.lidar_front_cluster_max_gap_m,
        )

    def _compute_depth_stats(self, depth_msg: Image):
        points, valid_count, _ = self._depth_obstacle_points_for_msg(depth_msg)
        if valid_count == 0:
            return float('inf'), False, 0

        corridor_x = [
            float(x)
            for x, y in points
            if abs(float(y)) <= self.depth_front_corridor_half_width_m
        ]
        if not corridor_x:
            return float('inf'), False, int(valid_count)

        near_distance_m = float(np.percentile(corridor_x, self.depth_near_percentile))
        depth_warning = near_distance_m < self.depth_warning_threshold_m
        return near_distance_m, depth_warning, int(valid_count)

    def _build_depth_obstacle_pose_array(self, depth_msg: Image) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header.stamp = depth_msg.header.stamp
        pose_array.header.frame_id = self.obstacle_points_frame_id or depth_msg.header.frame_id

        points, _, _ = self._depth_obstacle_points_for_msg(depth_msg)
        for x_m, y_m in points:
            pose = Pose()
            pose.position.x = float(x_m)
            pose.position.y = float(y_m)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array

    def _depth_obstacle_points_for_msg(self, depth_msg: Image) -> Tuple[List[Tuple[float, float]], int, dict]:
        cache_key = self._header_stamp_key(depth_msg.header)
        if self._depth_obstacle_cache is not None and self._depth_obstacle_cache.get('key') == cache_key:
            return (
                list(self._depth_obstacle_cache['points']),
                int(self._depth_obstacle_cache['valid_count']),
                dict(self._depth_obstacle_cache['metrics']),
            )

        depth = self._depth_image_to_numpy(depth_msg)
        if depth is None:
            metrics = {
                'depth_ground_filter_enabled': self.depth_ground_filter_enabled,
                'depth_obstacle_candidate_cells': 0,
                'depth_obstacle_components': 0,
                'depth_obstacle_points_filtered': 0,
            }
            self.last_depth_obstacle_metrics = metrics
            self._depth_obstacle_cache = {
                'key': cache_key,
                'points': [],
                'valid_count': 0,
                'metrics': metrics,
            }
            return [], 0, metrics

        roi, x0, _, y0, _ = self._extract_depth_roi(depth, return_bounds=True)
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.depth_min_valid_m) & (valid <= self.depth_max_valid_m)]
        valid_count = int(valid.size)
        if valid_count == 0:
            points: List[Tuple[float, float]] = []
            metrics = {
                'depth_ground_filter_enabled': self.depth_ground_filter_enabled,
                'depth_obstacle_candidate_cells': 0,
                'depth_obstacle_components': 0,
                'depth_obstacle_points_filtered': 0,
            }
        elif self.depth_ground_filter_enabled:
            mask = self._ground_filtered_depth_mask(roi)
            points, metrics = self._project_depth_mask_to_points(
                roi,
                mask,
                x0,
                y0,
                depth.shape[1],
            )
        else:
            points = self._project_depth_roi_to_points(roi, x0, y0, depth.shape[1])
            metrics = {
                'depth_ground_filter_enabled': False,
                'depth_obstacle_candidate_cells': len(points),
                'depth_obstacle_components': 0,
                'depth_obstacle_points_filtered': len(points),
            }
        self.last_depth_obstacle_metrics = metrics
        self._depth_obstacle_cache = {
            'key': cache_key,
            'points': list(points),
            'valid_count': valid_count,
            'metrics': dict(metrics),
        }
        return points, valid_count, metrics

    def _ground_filtered_depth_mask(self, roi: np.ndarray) -> np.ndarray:
        valid = np.isfinite(roi)
        valid &= roi >= self.depth_min_valid_m
        valid &= roi <= min(self.depth_max_valid_m, self.depth_obstacle_max_m)
        if roi.size == 0 or not np.any(valid):
            return np.zeros_like(roi, dtype=bool)

        row_background = np.full(roi.shape[0], np.nan, dtype=np.float32)
        for row in range(roi.shape[0]):
            vals = roi[row, valid[row]]
            if vals.size >= 6:
                row_background[row] = float(np.percentile(vals, self.depth_ground_row_percentile))
        finite_rows = np.isfinite(row_background)
        if not np.any(finite_rows):
            return np.zeros_like(roi, dtype=bool)
        if np.count_nonzero(finite_rows) >= 2:
            idx = np.arange(roi.shape[0], dtype=np.float32)
            row_background = np.interp(
                idx,
                idx[finite_rows],
                row_background[finite_rows],
            ).astype(np.float32)
        else:
            row_background[:] = row_background[finite_rows][0]

        bg = row_background[:, None]
        delta = np.maximum(self.depth_ground_min_delta_m, bg * (1.0 - self.depth_ground_ratio))
        closer_than_floor_or_background = roi <= (bg - delta)
        return valid & np.isfinite(bg) & closer_than_floor_or_background

    def _project_depth_mask_to_points(
        self,
        roi: np.ndarray,
        mask: np.ndarray,
        roi_x0: int,
        roi_y0: int,
        full_width: int,
    ) -> Tuple[List[Tuple[float, float]], dict]:
        del roi_y0
        stride = max(2, self.depth_projection_stride_px)
        grid_h = int(math.ceil(roi.shape[0] / stride))
        grid_w = int(math.ceil(roi.shape[1] / stride))
        cells = {}
        for gr in range(grid_h):
            row0 = gr * stride
            row1 = min(roi.shape[0], row0 + stride)
            for gc in range(grid_w):
                col0 = gc * stride
                col1 = min(roi.shape[1], col0 + stride)
                block_mask = mask[row0:row1, col0:col1]
                count = int(np.count_nonzero(block_mask))
                if count < self.depth_obstacle_min_block_pixels:
                    continue
                block_depth = roi[row0:row1, col0:col1]
                vals = block_depth[block_mask]
                vals = vals[np.isfinite(vals)]
                if vals.size == 0:
                    continue
                rr, cc = np.nonzero(block_mask)
                depth_m = float(np.percentile(vals, 35.0))
                pixel_x = float(roi_x0 + col0 + np.median(cc))
                cells[(gr, gc)] = {
                    'depth_m': depth_m,
                    'pixel_x': pixel_x,
                    'candidate_pixels': count,
                }

        remaining = set(cells.keys())
        accepted_components = 0
        accepted_cells = []
        while remaining:
            start = remaining.pop()
            stack = [start]
            comp = [start]
            while stack:
                gr, gc = stack.pop()
                for nr in range(gr - 1, gr + 2):
                    for nc in range(gc - 1, gc + 2):
                        if nr == gr and nc == gc:
                            continue
                        key = (nr, nc)
                        if key in remaining:
                            remaining.remove(key)
                            stack.append(key)
                            comp.append(key)
            rows = [p[0] for p in comp]
            comp_height_px = (max(rows) - min(rows) + 1) * stride
            if len(comp) < self.depth_obstacle_min_component_cells:
                continue
            if comp_height_px < self.depth_obstacle_min_component_height_px:
                continue
            accepted_components += 1
            accepted_cells.extend(comp)

        center_x = 0.5 * max(full_width - 1, 1)
        half_fov = 0.5 * self.depth_projection_hfov_rad
        points: List[Tuple[float, float]] = []
        for key in accepted_cells:
            cell = cells[key]
            norm_x = (cell['pixel_x'] - center_x) / max(center_x, 1.0)
            angle_x = norm_x * half_fov
            depth_m = float(cell['depth_m'])
            points.append((depth_m * math.cos(angle_x), depth_m * math.sin(angle_x)))

        if len(points) > self.depth_obstacle_max_points:
            step = max(1, int(math.ceil(len(points) / self.depth_obstacle_max_points)))
            points = points[::step][:self.depth_obstacle_max_points]

        metrics = {
            'depth_ground_filter_enabled': True,
            'depth_obstacle_candidate_cells': len(cells),
            'depth_obstacle_components': accepted_components,
            'depth_obstacle_points_filtered': len(points),
        }
        return points, metrics

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
    def _empty_image_like(header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = 0
        msg.width = 0
        msg.encoding = ''
        msg.is_bigendian = 0
        msg.step = 0
        msg.data = b''
        return msg

    @staticmethod
    def _empty_depth_like(header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = 0
        msg.width = 0
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = 0
        msg.data = b''
        return msg

    @staticmethod
    def _empty_imu_like(header) -> Imu:
        msg = Imu()
        msg.header = header
        return msg

    def _empty_obstacle_pose_array(self, header) -> PoseArray:
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = self.obstacle_points_frame_id or header.frame_id
        return pose_array

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

    def _trim_zed_history(self, newest_stamp_s: float) -> None:
        cutoff_s = newest_stamp_s - max(self.zed_frame_buffer_duration_s, 0.5)
        while self.zed_frame_history and float(self.zed_frame_history[0][1]['stamp_s']) < cutoff_s:
            old_key, _ = self.zed_frame_history.popleft()
            self.zed_frame_map.pop(old_key, None)

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

    def _warn_zed_gap(self, message: str, period_s: float = 2.0) -> None:
        now_s = self._clock_now_seconds()
        if now_s - self.last_zed_warning_s >= period_s:
            self.get_logger().warn(message)
            self.last_zed_warning_s = now_s

    def _notice_lidar_only_fallback(self, period_s: float = 10.0) -> None:
        now_s = self._clock_now_seconds()
        if now_s - self.last_lidar_only_notice_s >= period_s:
            self.get_logger().info('using LiDAR-only fusion fallback; no complete ZED frame is available')
            self.last_lidar_only_notice_s = now_s

    @staticmethod
    def _finite_or_none(value: float):
        return value if math.isfinite(value) else None

    @staticmethod
    def _wrap_to_pi(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def _header_stamp_key(header):
        return int(header.stamp.sec), int(header.stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
