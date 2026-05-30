#!/usr/bin/env python3
"""Competition ArUco marker detector for Operation Touchdown.

This node is intentionally stricter than the legacy marker_vision_node:
only decoded OpenCV ArUco IDs are accepted, and the output marker position is
published only after repeated, stable detections.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    if int(str(np.__version__).split(".", 1)[0]) >= 2:
        raise ImportError("cv_bridge from ROS Humble is not compatible with NumPy 2.x")
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - depends on robot runtime packages
    CvBridge = None

ARUCO_MARKER_SIDE_M = 0.3048
DEFAULT_ALLOWED_ARUCO_IDS = (0, 1, 2, 3, 4)


def parse_allowed_marker_ids(raw: object, *, default=DEFAULT_ALLOWED_ARUCO_IDS) -> tuple[int, ...]:
    if raw is None:
        return tuple(default)
    text = str(raw).replace(";", ",").replace(" ", ",").strip()
    if not text:
        return tuple(default)
    parsed = set()
    for chunk in text.split(","):
        try:
            parsed.add(int(chunk.strip()))
        except (TypeError, ValueError):
            continue
    return tuple(sorted(parsed)) if parsed else tuple(default)


class ArucoMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("ugv_aruco_marker_node")

        self.declare_parameter("image_topic", "/zed/image")
        self.declare_parameter("camera_info_topic", "/zed/left/camera_info")
        self.declare_parameter("localization_status_topic", "/ugv_localization/status")
        self.declare_parameter("marker_topic", "/ugv/aruco_detection")
        self.declare_parameter("debug_topic", "/ugv/aruco_debug")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("dictionary", "DICT_6X6_250")
        self.declare_parameter("allowed_marker_ids", "0,1,2,3,4")
        self.declare_parameter("marker_size_m", ARUCO_MARKER_SIDE_M)
        self.declare_parameter("confirmation_frames", 3)
        self.declare_parameter("confirmation_radius_m", 0.35)
        self.declare_parameter("max_pose_age_s", 1.0)
        self.declare_parameter("max_camera_info_age_s", 2.0)
        self.declare_parameter("max_publish_hz", 10.0)
        self.declare_parameter("min_marker_perimeter_px", 45.0)
        self.declare_parameter("min_distance_to_border_px", 6.0)
        self.declare_parameter("max_reprojection_error_px", 8.0)
        self.declare_parameter("max_pose_jump_m", 0.75)
        self.declare_parameter("tf_timeout_s", 0.05)
        self.declare_parameter("allow_planar_projection_fallback", False)
        self.declare_parameter("camera_x_m", 0.0)
        self.declare_parameter("camera_y_m", 0.0)
        self.declare_parameter("camera_yaw_offset_rad", 0.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.localization_status_topic = str(self.get_parameter("localization_status_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.dictionary_name = str(self.get_parameter("dictionary").value)
        self.allowed_marker_ids = set(
            parse_allowed_marker_ids(
                self.get_parameter("allowed_marker_ids").value,
                default=DEFAULT_ALLOWED_ARUCO_IDS,
            )
        )
        self.marker_size_m = max(0.02, float(self.get_parameter("marker_size_m").value))
        self.confirmation_frames = max(1, int(self.get_parameter("confirmation_frames").value))
        self.confirmation_radius_m = max(0.02, float(self.get_parameter("confirmation_radius_m").value))
        self.max_pose_age_s = max(0.0, float(self.get_parameter("max_pose_age_s").value))
        self.max_camera_info_age_s = max(0.0, float(self.get_parameter("max_camera_info_age_s").value))
        self.max_publish_period_s = 1.0 / max(0.1, float(self.get_parameter("max_publish_hz").value))
        self.min_marker_perimeter_px = max(0.0, float(self.get_parameter("min_marker_perimeter_px").value))
        self.min_distance_to_border_px = max(0.0, float(self.get_parameter("min_distance_to_border_px").value))
        self.max_reprojection_error_px = max(0.0, float(self.get_parameter("max_reprojection_error_px").value))
        self.max_pose_jump_m = max(0.0, float(self.get_parameter("max_pose_jump_m").value))
        self.tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self.allow_planar_projection_fallback = bool(self.get_parameter("allow_planar_projection_fallback").value)
        self.camera_x_m = float(self.get_parameter("camera_x_m").value)
        self.camera_y_m = float(self.get_parameter("camera_y_m").value)
        self.camera_yaw_offset_rad = float(self.get_parameter("camera_yaw_offset_rad").value)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.last_camera_info_s: Optional[float] = None
        self.latest_pose: Optional[tuple[float, float, float]] = None
        self.latest_pose_received_s: Optional[float] = None
        self.pending_marker_id: Optional[int] = None
        self.pending_marker_xy: Optional[tuple[float, float]] = None
        self.pending_count = 0
        self.last_publish_s = 0.0
        self.last_tf_warn_s = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dictionary = self._build_dictionary(self.dictionary_name)
        self.detector = self._build_detector(self.dictionary)

        self.marker_pub = self.create_publisher(PointStamped, self.marker_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(String, self.localization_status_topic, self.localization_status_callback, 10)

        self.get_logger().info(
            "ArUco marker node started "
            f"(image={self.image_topic}, camera_info={self.camera_info_topic}, "
            f"dictionary={self.dictionary_name}, ids={sorted(self.allowed_marker_ids)}, "
            f"marker_size={self.marker_size_m:.4f}m)"
        )

    def _build_dictionary(self, name: str):
        aruco = getattr(cv2, "aruco", None)
        if aruco is None:
            raise RuntimeError("OpenCV was built without cv2.aruco; install opencv-contrib-python or python3-opencv with aruco.")
        dictionary_id = getattr(aruco, str(name), None)
        if dictionary_id is None:
            raise RuntimeError(f"Unsupported ArUco dictionary {name!r}")
        return aruco.getPredefinedDictionary(dictionary_id)

    @staticmethod
    def _build_detector(dictionary):
        aruco = cv2.aruco
        if hasattr(aruco, "DetectorParameters"):
            parameters = aruco.DetectorParameters()
        else:  # OpenCV 4.5 on some ROS Humble images
            parameters = aruco.DetectorParameters_create()
        if hasattr(aruco, "CORNER_REFINE_SUBPIX"):
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        if hasattr(aruco, "ArucoDetector"):
            return aruco.ArucoDetector(dictionary, parameters)
        return (dictionary, parameters)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) < 9:
            return
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1)) if msg.d else np.zeros((5, 1), dtype=np.float64)
        self.last_camera_info_s = time.monotonic()

    def localization_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            pose = payload.get("pose_m")
            if isinstance(pose, list) and len(pose) >= 3:
                self.latest_pose = (float(pose[0]), float(pose[1]), math.radians(float(pose[2])))
                self.latest_pose_received_s = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def image_callback(self, msg: Image) -> None:
        now_s = time.monotonic()
        if now_s - self.last_publish_s < self.max_publish_period_s:
            return
        debug = {
            "node": "ugv_aruco_marker_node",
            "detected": False,
            "confirmed": False,
            "allowed_marker_ids": sorted(self.allowed_marker_ids),
        }
        if self.camera_matrix is None or self.dist_coeffs is None or self.last_camera_info_s is None:
            debug["reason"] = "missing_camera_info"
            self._publish_debug(debug)
            return
        if self.max_camera_info_age_s > 0.0 and now_s - self.last_camera_info_s > self.max_camera_info_age_s:
            debug["reason"] = "camera_info_stale"
            self._publish_debug(debug)
            return
        if self.latest_pose is None or self.latest_pose_received_s is None:
            debug["reason"] = "missing_localization_pose"
            self._publish_debug(debug)
            return
        if self.max_pose_age_s > 0.0 and now_s - self.latest_pose_received_s > self.max_pose_age_s:
            debug["reason"] = "localization_pose_stale"
            self._publish_debug(debug)
            return

        try:
            image = self._image_msg_to_bgr(msg)
        except Exception as exc:
            debug["reason"] = f"image_convert_failed:{exc}"
            self._publish_debug(debug)
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        corners, ids, rejected = self._detect_markers(gray)
        debug["candidate_count"] = 0 if corners is None else len(corners)
        debug["rejected_count"] = 0 if rejected is None else len(rejected)
        if ids is None or corners is None or len(ids) == 0:
            debug["reason"] = "no_aruco_marker"
            self._publish_debug(debug)
            return

        candidate = self._best_candidate(corners, ids, gray.shape)
        if candidate is None:
            debug["reason"] = "no_allowed_marker"
            debug["detected_ids"] = [int(v) for v in np.asarray(ids).reshape(-1).tolist()]
            self._publish_debug(debug)
            return

        marker_id, marker_corners = candidate
        pose_result = self._estimate_marker_pose(marker_corners)
        debug.update({"detected": True, "marker_id": int(marker_id)})
        if pose_result is None:
            debug["reason"] = "pnp_failed"
            self._publish_debug(debug)
            return
        tvec, reprojection_error_px = pose_result
        marker_xy = self._marker_map_xy_from_tvec(tvec, msg.header.frame_id, msg.header.stamp)
        if marker_xy is None:
            debug["reason"] = "marker_map_projection_failed"
            self._publish_debug(debug)
            return
        if self.max_reprojection_error_px > 0.0 and reprojection_error_px > self.max_reprojection_error_px:
            debug.update(
                {
                    "reason": "reprojection_error_high",
                    "reprojection_error_px": round(reprojection_error_px, 3),
                }
            )
            self._publish_debug(debug)
            return

        confirmed = self._update_confirmation(marker_id, marker_xy)
        distance_m = math.hypot(float(tvec[0]), float(tvec[2]))
        bearing_rad = math.atan2(float(tvec[0]), float(tvec[2]))
        debug.update(
            {
                "reason": "confirmed" if confirmed else "confirmation_pending",
                "confirmed": confirmed,
                "confirmation_count": self.pending_count,
                "confirmation_needed": self.confirmation_frames,
                "marker_map_m": [round(marker_xy[0], 3), round(marker_xy[1], 3)],
                "distance_m": round(distance_m, 3),
                "bearing_deg": round(math.degrees(bearing_rad), 2),
                "reprojection_error_px": round(reprojection_error_px, 3),
            }
        )
        if confirmed:
            msg_out = PointStamped()
            msg_out.header.stamp = msg.header.stamp
            msg_out.header.frame_id = self.map_frame
            msg_out.point.x = float(self.pending_marker_xy[0] if self.pending_marker_xy else marker_xy[0])
            msg_out.point.y = float(self.pending_marker_xy[1] if self.pending_marker_xy else marker_xy[1])
            msg_out.point.z = float(distance_m)
            self.marker_pub.publish(msg_out)
            self.last_publish_s = now_s
        self._publish_debug(debug)

    def _detect_markers(self, gray: np.ndarray):
        if hasattr(cv2.aruco, "ArucoDetector") and not isinstance(self.detector, tuple):
            return self.detector.detectMarkers(gray)
        dictionary, parameters = self.detector
        return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    def _best_candidate(self, corners, ids, image_shape) -> Optional[tuple[int, np.ndarray]]:
        h, w = image_shape[:2]
        flat_ids = np.asarray(ids).reshape(-1)
        best = None
        best_perimeter = -1.0
        for idx, marker_id_raw in enumerate(flat_ids):
            marker_id = int(marker_id_raw)
            if marker_id not in self.allowed_marker_ids:
                continue
            marker_corners = np.asarray(corners[idx], dtype=np.float64).reshape((4, 2))
            xs = marker_corners[:, 0]
            ys = marker_corners[:, 1]
            border_distance = min(float(np.min(xs)), float(np.min(ys)), float(w - np.max(xs)), float(h - np.max(ys)))
            if border_distance < self.min_distance_to_border_px:
                continue
            perimeter = 0.0
            for corner_idx in range(4):
                a = marker_corners[corner_idx]
                b = marker_corners[(corner_idx + 1) % 4]
                perimeter += float(np.linalg.norm(a - b))
            if perimeter < self.min_marker_perimeter_px:
                continue
            if perimeter > best_perimeter:
                best_perimeter = perimeter
                best = (marker_id, marker_corners)
        return best

    def _estimate_marker_pose(self, marker_corners: np.ndarray) -> Optional[tuple[np.ndarray, float]]:
        half = 0.5 * self.marker_size_m
        obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                marker_corners.astype(np.float64),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE,
            )
        except cv2.error:
            return None
        if not ok:
            return None
        projected, _ = cv2.projectPoints(obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        projected = projected.reshape((4, 2))
        error = float(np.mean(np.linalg.norm(projected - marker_corners, axis=1)))
        return tvec.reshape(3), error

    def _marker_map_xy_from_tvec(self, tvec: np.ndarray, camera_frame: str, stamp) -> Optional[tuple[float, float]]:
        if not all(math.isfinite(float(v)) for v in tvec):
            return None
        frame_id = str(camera_frame).strip()
        if frame_id:
            point = PointStamped()
            point.header.frame_id = frame_id
            lookup_time = self._lookup_time_from_stamp(stamp)
            point.header.stamp = lookup_time.to_msg()
            point.point.x = float(tvec[0])
            point.point.y = float(tvec[1])
            point.point.z = float(tvec[2])
            try:
                transform = self._lookup_transform(frame_id, lookup_time)
                map_point = self._transform_point(point, transform)
                if math.isfinite(map_point.point.x) and math.isfinite(map_point.point.y):
                    return float(map_point.point.x), float(map_point.point.y)
            except TransformException as exc:
                if not self.allow_planar_projection_fallback:
                    now_s = time.monotonic()
                    if now_s - self.last_tf_warn_s >= 2.0:
                        self.last_tf_warn_s = now_s
                        self.get_logger().warn(f"ArUco TF projection unavailable {frame_id}->{self.map_frame}: {exc}")
                    return None

        if not self.allow_planar_projection_fallback or self.latest_pose is None:
            return None
        pose_x, pose_y, pose_yaw = self.latest_pose
        camera_right_m = float(tvec[0])
        camera_forward_m = float(tvec[2])
        if camera_forward_m <= 0.0:
            return None
        camera_yaw = pose_yaw + self.camera_yaw_offset_rad
        base_dx = self.camera_x_m + camera_forward_m * math.cos(self.camera_yaw_offset_rad) + camera_right_m * math.sin(self.camera_yaw_offset_rad)
        base_dy = self.camera_y_m + camera_forward_m * math.sin(self.camera_yaw_offset_rad) - camera_right_m * math.cos(self.camera_yaw_offset_rad)
        _ = camera_yaw  # Kept explicit for readability when tuning camera_yaw_offset_rad.
        map_x = pose_x + base_dx * math.cos(pose_yaw) - base_dy * math.sin(pose_yaw)
        map_y = pose_y + base_dx * math.sin(pose_yaw) + base_dy * math.cos(pose_yaw)
        if not math.isfinite(map_x) or not math.isfinite(map_y):
            return None
        return map_x, map_y

    @staticmethod
    def _lookup_time_from_stamp(stamp) -> Time:
        try:
            if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
                return Time()
            return Time.from_msg(stamp)
        except Exception:
            return Time()

    def _lookup_transform(self, frame_id: str, lookup_time: Time):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                lookup_time,
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException:
            if lookup_time.nanoseconds == 0:
                raise
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )

    @staticmethod
    def _transform_point(point: PointStamped, transform) -> PointStamped:
        q = transform.transform.rotation
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)
        x = float(point.point.x)
        y = float(point.point.y)
        z = float(point.point.z)
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        # Rotate v by q using the expanded q * v * q^-1 form.
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw + iw * -qz + ix * -qy - iy * -qx

        out = PointStamped()
        out.header = transform.header
        out.point.x = rx + tx
        out.point.y = ry + ty
        out.point.z = rz + tz
        return out

    def _update_confirmation(self, marker_id: int, marker_xy: tuple[float, float]) -> bool:
        if self.pending_marker_id != int(marker_id) or self.pending_marker_xy is None:
            self.pending_marker_id = int(marker_id)
            self.pending_marker_xy = marker_xy
            self.pending_count = 1
            return self.pending_count >= self.confirmation_frames
        jump = math.hypot(marker_xy[0] - self.pending_marker_xy[0], marker_xy[1] - self.pending_marker_xy[1])
        if self.max_pose_jump_m > 0.0 and jump > self.max_pose_jump_m:
            self.pending_marker_xy = marker_xy
            self.pending_count = 1
            return False
        if jump <= self.confirmation_radius_m:
            self.pending_marker_xy = (
                0.65 * self.pending_marker_xy[0] + 0.35 * marker_xy[0],
                0.65 * self.pending_marker_xy[1] + 0.35 * marker_xy[1],
            )
            self.pending_count = min(self.confirmation_frames, self.pending_count + 1)
        else:
            self.pending_marker_xy = marker_xy
            self.pending_count = 1
        return self.pending_count >= self.confirmation_frames

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray:
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                pass
        return self._manual_image_msg_to_bgr(msg)

    @classmethod
    def _manual_image_msg_to_bgr(cls, msg: Image) -> np.ndarray:
        arr = cls._manual_image_msg_to_numpy(msg)
        encoding = str(msg.encoding).lower()
        if encoding == "bgr8":
            return arr
        if encoding == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return arr[:, :, :3]
        if encoding == "rgba8":
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if arr.ndim == 2:
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if arr.ndim == 3 and arr.shape[2] == 3:
            return arr
        if arr.ndim == 3 and arr.shape[2] == 4:
            return arr[:, :, :3]
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    @staticmethod
    def _manual_image_msg_to_numpy(msg: Image) -> np.ndarray:
        encoding = str(msg.encoding).lower()
        encoding_info = {
            "mono8": (np.uint8, 1),
            "8uc1": (np.uint8, 1),
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "bgra8": (np.uint8, 4),
            "rgba8": (np.uint8, 4),
        }
        if encoding not in encoding_info:
            raise ValueError(f"Unsupported image encoding without cv_bridge: {msg.encoding}")
        dtype_raw, channels = encoding_info[encoding]
        dtype = np.dtype(dtype_raw)
        height = int(msg.height)
        width = int(msg.width)
        row_bytes = int(msg.step) if int(msg.step) > 0 else width * channels * dtype.itemsize
        row_elems = row_bytes // dtype.itemsize
        needed_elems = height * row_elems
        data = np.frombuffer(msg.data, dtype=dtype, count=needed_elems)
        if data.size < needed_elems:
            raise ValueError(f"Image data is short for encoding {msg.encoding}")
        data = data.reshape((height, row_elems))
        useful = data[:, : width * channels]
        if channels == 1:
            return np.ascontiguousarray(useful.reshape((height, width)))
        return np.ascontiguousarray(useful.reshape((height, width, channels)))

    def _publish_debug(self, payload: dict) -> None:
        self.debug_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    rclpy.init()
    node = ArucoMarkerNode()
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
