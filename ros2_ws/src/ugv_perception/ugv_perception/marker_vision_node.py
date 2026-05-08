#!/usr/bin/env python3
import json
import math
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class MarkerVisionNode(Node):
    def __init__(self):
        super().__init__('marker_vision_node')

        self.declare_parameter('image_topic', '/zed/image')
        self.declare_parameter('depth_topic', '/zed/depth')
        self.declare_parameter('nav_status_topic', '/ugv_nav_status')
        self.declare_parameter('marker_topic', '/ugv/marker_detection')
        self.declare_parameter('debug_topic', '/ugv/marker_vision_debug')
        self.declare_parameter('model_path', 'models/marker_orb_model.npz')
        self.declare_parameter('camera_hfov_deg', 110.0)
        self.declare_parameter('min_good_matches', 18)
        self.declare_parameter('match_ratio', 0.75)
        self.declare_parameter('max_publish_hz', 2.0)
        self.declare_parameter('depth_roi_px', 8)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('max_features', 900)
        self.declare_parameter('confirmation_frames', 2)
        self.declare_parameter('confirmation_radius_m', 0.75)
        self.declare_parameter('max_depth_stamp_delta_s', 0.35)
        self.declare_parameter('max_pose_age_s', 1.5)
        self.declare_parameter('enable_generic_detector', True)
        self.declare_parameter('generic_min_area_frac', 0.002)
        self.declare_parameter('generic_max_area_frac', 0.65)
        self.declare_parameter('generic_min_y_frac', 0.20)
        self.declare_parameter('generic_min_contrast', 55.0)
        self.declare_parameter('generic_min_score', 0.55)

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.nav_status_topic = self.get_parameter('nav_status_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.model_path = self._resolve_model_path(str(self.get_parameter('model_path').value))
        self.camera_hfov_rad = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        self.min_good_matches = int(self.get_parameter('min_good_matches').value)
        self.match_ratio = float(self.get_parameter('match_ratio').value)
        self.max_publish_period_s = 1.0 / max(0.1, float(self.get_parameter('max_publish_hz').value))
        self.depth_roi_px = max(1, int(self.get_parameter('depth_roi_px').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        max_features = int(self.get_parameter('max_features').value)
        self.confirmation_frames = max(1, int(self.get_parameter('confirmation_frames').value))
        self.confirmation_radius_m = max(0.05, float(self.get_parameter('confirmation_radius_m').value))
        self.max_depth_stamp_delta_s = max(0.0, float(self.get_parameter('max_depth_stamp_delta_s').value))
        self.max_pose_age_s = max(0.0, float(self.get_parameter('max_pose_age_s').value))
        self.enable_generic_detector = bool(self.get_parameter('enable_generic_detector').value)
        self.generic_min_area_frac = max(0.0, float(self.get_parameter('generic_min_area_frac').value))
        self.generic_max_area_frac = min(1.0, max(self.generic_min_area_frac, float(self.get_parameter('generic_max_area_frac').value)))
        self.generic_min_y_frac = min(1.0, max(0.0, float(self.get_parameter('generic_min_y_frac').value)))
        self.generic_min_contrast = max(0.0, float(self.get_parameter('generic_min_contrast').value))
        self.generic_min_score = max(0.0, float(self.get_parameter('generic_min_score').value))

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=max_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.model_descriptors: Optional[np.ndarray] = None
        self.model_metadata = {}
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_stamp_s: Optional[float] = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None
        self.latest_pose_received_s: Optional[float] = None
        self.pending_marker_xy: Optional[Tuple[float, float]] = None
        self.pending_marker_count = 0
        self.last_publish_s = 0.0
        self.last_missing_model_log_s = 0.0

        self._load_model()
        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.nav_status_topic, self.nav_status_callback, 10)
        self.marker_pub = self.create_publisher(PointStamped, self.marker_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.get_logger().info(
            f'Marker vision node started (image={self.image_topic}, depth={self.depth_topic}, '
            f'model={self.model_path}, marker_topic={self.marker_topic}, '
            f'confirmation_frames={self.confirmation_frames}, '
            f'generic_detector={self.enable_generic_detector})'
        )

    def _load_model(self) -> None:
        if not self.model_path.exists():
            self.get_logger().warn(f'Marker model not found yet: {self.model_path}')
            return
        try:
            data = np.load(self.model_path, allow_pickle=False)
        except Exception as exc:
            self.get_logger().warn(f'Could not load marker model {self.model_path}: {exc}')
            return
        descriptors = data['descriptors'] if 'descriptors' in data.files else None
        if descriptors is None or descriptors.size == 0:
            self.get_logger().warn(f'Marker model has no descriptors: {self.model_path}')
            return
        self.model_descriptors = descriptors.astype(np.uint8, copy=False)
        try:
            metadata_raw = data['metadata'].item() if 'metadata' in data.files else '{}'
            self.model_metadata = json.loads(metadata_raw)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            self.model_metadata = {}
        self.get_logger().info(
            f'Loaded marker model with {len(self.model_descriptors)} descriptors from {self.model_path}'
        )

    @staticmethod
    def _resolve_model_path(raw_path: str) -> Path:
        path = Path(raw_path).expanduser()
        if path.is_absolute() or path.exists():
            return path
        package_root = Path(__file__).resolve().parents[1]
        source_candidate = package_root / path
        if source_candidate.exists():
            return source_candidate
        return path

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Could not convert depth image: {exc}')
            return
        self.latest_depth = np.asarray(depth, dtype=np.float32)
        self.latest_depth_stamp_s = self._stamp_to_seconds(msg.header.stamp)

    def nav_status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
            pose = status.get('pose_m')
            if isinstance(pose, list) and len(pose) >= 3:
                self.latest_pose = (float(pose[0]), float(pose[1]), math.radians(float(pose[2])))
                self.latest_pose_received_s = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def image_callback(self, msg: Image) -> None:
        now_s = time.monotonic()
        if now_s - self.last_publish_s < self.max_publish_period_s:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception as exc:
                self.get_logger().warn(f'Could not convert marker image: {exc}')
                return
        try:
            gray = self._to_gray(image)
        except Exception as exc:
            self.get_logger().warn(f'Could not convert marker image to grayscale: {exc}')
            return

        generic_center, generic_debug = self._detect_generic_marker(gray)
        orb_center, orb_debug = self._detect_orb_marker(gray, now_s)
        center_px = generic_center if generic_center is not None else orb_center
        method = 'generic_dark_light_marker' if generic_center is not None else 'orb_model'

        debug = {
            'detected': center_px is not None,
            'method': method if center_px is not None else None,
            'generic': generic_debug,
            'orb': orb_debug,
        }
        if center_px is None:
            debug.update({'reason': 'no_marker_candidate'})
            self._publish_debug(debug)
            return

        depth_m = self._depth_at(center_px)
        debug.update({
            'center_px': [round(center_px[0], 1), round(center_px[1], 1)],
            'depth_m': None if depth_m is None else round(depth_m, 3),
            'pose_available': self.latest_pose is not None,
        })

        image_stamp_s = self._stamp_to_seconds(msg.header.stamp)
        if not self._depth_is_fresh(image_stamp_s):
            debug.update({'reason': 'stale_depth'})
            self._publish_debug(debug)
            return
        if not self._pose_is_fresh(now_s):
            debug.update({'reason': 'stale_pose_or_missing'})
            self._publish_debug(debug)
            return

        if depth_m is None or self.latest_pose is None:
            debug.update({'reason': 'missing_depth_or_pose'})
            self._publish_debug(debug)
            return

        marker_xy = self._estimate_marker_world(center_px, depth_m, gray.shape[1])
        if marker_xy is None:
            self._publish_debug(debug)
            return

        confirmed = self._update_confirmation(marker_xy)
        debug.update({
            'confirmation_count': self.pending_marker_count,
            'confirmation_needed': self.confirmation_frames,
            'confirmed': confirmed,
        })
        if not confirmed:
            self._publish_debug(debug)
            return

        publish_xy = self.pending_marker_xy if self.pending_marker_xy is not None else marker_xy
        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = 'map'
        point_msg.point.x = float(publish_xy[0])
        point_msg.point.y = float(publish_xy[1])
        point_msg.point.z = 0.0
        self.marker_pub.publish(point_msg)
        self.last_publish_s = now_s
        debug.update({'published_marker_m': [round(publish_xy[0], 3), round(publish_xy[1], 3)]})
        self._publish_debug(debug)

    def _detect_orb_marker(self, gray: np.ndarray, now_s: float):
        debug = {
            'model_loaded': self.model_descriptors is not None,
            'good_matches': 0,
            'min_good_matches': self.min_good_matches,
            'keypoints': 0,
        }
        if self.model_descriptors is None:
            if not self.enable_generic_detector and now_s - self.last_missing_model_log_s > 5.0:
                self.get_logger().warn('Marker model missing; train it before enabling ORB-only marker vision.')
                self.last_missing_model_log_s = now_s
            return None, debug

        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        debug['keypoints'] = 0 if keypoints is None else len(keypoints)
        if descriptors is None or keypoints is None or len(keypoints) == 0:
            debug['reason'] = 'no_keypoints'
            return None, debug

        good = self._good_matches(descriptors)
        debug['good_matches'] = len(good)
        if len(good) < self.min_good_matches:
            debug['reason'] = 'not_enough_good_matches'
            return None, debug
        return self._matched_center_px(keypoints, good), debug

    def _detect_generic_marker(self, gray: np.ndarray):
        debug = {'enabled': self.enable_generic_detector, 'candidates': 0, 'sources': {}}
        if not self.enable_generic_detector:
            return None, debug

        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, dark_mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = np.ones((5, 5), np.uint8)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        dark_contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        median = float(np.median(blur))
        lower = int(max(20.0, 0.55 * median))
        upper = int(min(255.0, max(lower + 30.0, 1.35 * median)))
        edges = cv2.Canny(blur, lower, upper)
        edge_kernel = np.ones((5, 5), np.uint8)
        edge_mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, edge_kernel, iterations=2)
        edge_mask = cv2.dilate(edge_mask, edge_kernel, iterations=1)
        edge_contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidate_sets = (
            ('dark_region', dark_contours),
            ('edge_region', edge_contours),
        )
        best = None
        best_score = -1.0
        for source, contours in candidate_sets:
            source_candidates = 0
            for contour in contours:
                candidate = self._score_generic_candidate(gray, contour, source)
                if candidate is None:
                    continue
                source_candidates += 1
                if candidate['score'] > best_score:
                    best_score = candidate['score']
                    best = candidate
            debug['sources'][source] = source_candidates
            debug['candidates'] += source_candidates

        if best is None or best_score < self.generic_min_score:
            debug['best_score'] = None if best is None else round(best_score, 3)
            return None, debug

        debug.update({
            'best_score': round(best['score'], 3),
            'bbox': best['bbox'],
            'area_frac': round(best['area_frac'], 4),
            'aspect': round(best['aspect'], 3),
            'contrast': round(best['contrast'], 1),
            'dark_ratio': round(best['dark_ratio'], 3),
            'light_ratio': round(best['light_ratio'], 3),
            'source': best['source'],
        })
        return best['center'], debug

    def _score_generic_candidate(self, gray: np.ndarray, contour, source: str):
        h, w = gray.shape[:2]
        image_area = float(max(1, h * w))
        area = float(cv2.contourArea(contour))
        area_frac = area / image_area
        if area_frac < self.generic_min_area_frac or area_frac > self.generic_max_area_frac:
            return None

        x, y, bw, bh = cv2.boundingRect(contour)
        if bw <= 0 or bh <= 0:
            return None
        center_y_frac = (y + 0.5 * bh) / max(1.0, float(h))
        if center_y_frac < self.generic_min_y_frac:
            return None

        aspect = bw / float(bh)
        if aspect < 0.25 or aspect > 4.5:
            return None

        touches_borders = int(x <= 2) + int(y <= 2) + int(x + bw >= w - 2) + int(y + bh >= h - 2)
        if touches_borders >= 3 and area_frac > 0.15:
            return None

        roi = gray[y:y + bh, x:x + bw]
        if roi.size == 0:
            return None
        p10, p90 = np.percentile(roi, [10.0, 90.0])
        contrast = float(p90 - p10)
        if contrast < self.generic_min_contrast:
            return None

        dark_cut = float(p10 + 0.30 * contrast)
        light_cut = float(p90 - 0.25 * contrast)
        dark_ratio = max(float(np.mean(roi < 85)), float(np.mean(roi <= dark_cut)))
        light_ratio = max(float(np.mean(roi > 170)), float(np.mean(roi >= light_cut)))
        if dark_ratio < 0.10 or light_ratio < 0.04:
            return None

        rect_area = float(max(1, bw * bh))
        extent = area / rect_area
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.045 * perimeter, True) if perimeter > 0 else []
        corner_bonus = 0.15 if 4 <= len(approx) <= 10 else 0.0
        if source == 'edge_region':
            corner_bonus += 0.05
        square_score = max(0.0, 1.0 - min(abs(math.log(max(aspect, 1e-6))), 1.15))
        contrast_score = min(1.0, contrast / 150.0)
        area_score = min(1.0, area_frac / 0.08)
        balance_score = min(1.0, dark_ratio + light_ratio)
        extent_target = 0.35 if source == 'edge_region' else 0.60
        score = (
            0.30 * contrast_score
            + 0.18 * square_score
            + 0.20 * area_score
            + 0.17 * min(1.0, extent / extent_target)
            + 0.15 * balance_score
            + corner_bonus
        )

        m = cv2.moments(contour)
        if abs(m.get('m00', 0.0)) > 1e-6:
            cx = float(m['m10'] / m['m00'])
            cy = float(m['m01'] / m['m00'])
        else:
            cx = float(x + 0.5 * bw)
            cy = float(y + 0.5 * bh)
        return {
            'center': (cx, cy),
            'bbox': [int(x), int(y), int(bw), int(bh)],
            'area_frac': area_frac,
            'aspect': aspect,
            'contrast': contrast,
            'dark_ratio': dark_ratio,
            'light_ratio': light_ratio,
            'score': score,
            'source': source,
        }

    def _good_matches(self, descriptors: np.ndarray):
        raw_matches = self.matcher.knnMatch(descriptors.astype(np.uint8, copy=False), self.model_descriptors, k=2)
        good = []
        for pair in raw_matches:
            if len(pair) < 2:
                continue
            first, second = pair
            if first.distance < self.match_ratio * second.distance:
                good.append(first)
        return good

    @staticmethod
    def _matched_center_px(keypoints, matches) -> Tuple[float, float]:
        pts = np.array([keypoints[m.queryIdx].pt for m in matches], dtype=np.float32)
        return float(np.median(pts[:, 0])), float(np.median(pts[:, 1]))

    def _depth_at(self, center_px: Tuple[float, float]) -> Optional[float]:
        if self.latest_depth is None:
            return None
        x = int(round(center_px[0]))
        y = int(round(center_px[1]))
        h, w = self.latest_depth.shape[:2]
        x0 = max(0, x - self.depth_roi_px)
        x1 = min(w, x + self.depth_roi_px + 1)
        y0 = max(0, y - self.depth_roi_px)
        y1 = min(h, y + self.depth_roi_px + 1)
        roi = self.latest_depth[y0:y1, x0:x1]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _depth_is_fresh(self, image_stamp_s: float) -> bool:
        if self.latest_depth is None or self.latest_depth_stamp_s is None:
            return False
        if self.max_depth_stamp_delta_s <= 0.0:
            return True
        return abs(self.latest_depth_stamp_s - image_stamp_s) <= self.max_depth_stamp_delta_s

    def _pose_is_fresh(self, now_s: float) -> bool:
        if self.latest_pose is None or self.latest_pose_received_s is None:
            return False
        if self.max_pose_age_s <= 0.0:
            return True
        return now_s - self.latest_pose_received_s <= self.max_pose_age_s

    def _estimate_marker_world(self, center_px: Tuple[float, float], depth_m: float, width_px: int) -> Optional[Tuple[float, float]]:
        if self.latest_pose is None:
            return None
        pose_x, pose_y, yaw = self.latest_pose
        normalized_x = (center_px[0] - 0.5 * width_px) / max(1.0, 0.5 * width_px)
        bearing = normalized_x * (0.5 * self.camera_hfov_rad)
        heading = yaw + bearing
        return pose_x + depth_m * math.cos(heading), pose_y + depth_m * math.sin(heading)

    def _update_confirmation(self, marker_xy: Tuple[float, float]) -> bool:
        if self.pending_marker_xy is None:
            self.pending_marker_xy = marker_xy
            self.pending_marker_count = 1
            return self.pending_marker_count >= self.confirmation_frames

        dist = math.hypot(marker_xy[0] - self.pending_marker_xy[0], marker_xy[1] - self.pending_marker_xy[1])
        if dist <= self.confirmation_radius_m:
            count = self.pending_marker_count + 1
            self.pending_marker_xy = (
                0.65 * self.pending_marker_xy[0] + 0.35 * marker_xy[0],
                0.65 * self.pending_marker_xy[1] + 0.35 * marker_xy[1],
            )
            self.pending_marker_count = min(count, self.confirmation_frames)
        else:
            self.pending_marker_xy = marker_xy
            self.pending_marker_count = 1
        return self.pending_marker_count >= self.confirmation_frames

    @staticmethod
    def _to_gray(image) -> np.ndarray:
        arr = np.asarray(image)
        if arr.ndim == 2:
            return arr
        if arr.shape[2] == 1:
            return arr[:, :, 0]
        if arr.shape[2] == 4:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2GRAY)
        return cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)

    def _publish_debug(self, debug: dict) -> None:
        self.debug_pub.publish(String(data=json.dumps(debug)))

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = MarkerVisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
