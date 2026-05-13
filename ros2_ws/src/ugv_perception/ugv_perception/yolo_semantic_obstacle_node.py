#!/usr/bin/env python3
import json
import math
import sys
import time
from collections import Counter
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    if int(str(np.__version__).split('.', 1)[0]) >= 2:
        raise ImportError('cv_bridge from ROS Humble is not compatible with NumPy 2.x')
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - depends on robot runtime packages
    CvBridge = None


DEFAULT_OBSTACLE_CLASSES = (
    "person,chair,couch,dining table,bench,potted plant,backpack,suitcase"
)


class YoloSemanticObstacleNode(Node):
    def __init__(self):
        super().__init__("yolo_semantic_obstacle_node")

        self.declare_parameter("image_topic", "/zed/image")
        self.declare_parameter("depth_topic", "/zed/depth")
        self.declare_parameter("obstacle_points_topic", "/sensors/yolo_semantic_obstacle_points")
        self.declare_parameter("debug_topic", "/sensors/yolo_semantic_debug")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "")
        self.declare_parameter("imgsz", 416)
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("max_hz", 2.0)
        self.declare_parameter("camera_hfov_deg", 110.0)
        self.declare_parameter("max_depth_stamp_delta_s", 0.35)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("depth_roi_lower_frac", 0.55)
        self.declare_parameter("min_bbox_area_frac", 0.003)
        self.declare_parameter("max_points_per_detection", 21)
        self.declare_parameter("max_total_points", 120)
        self.declare_parameter("semantic_width_pad_m", 0.20)
        self.declare_parameter("semantic_depth_thickness_m", 0.30)
        self.declare_parameter("obstacle_classes", DEFAULT_OBSTACLE_CLASSES)

        self.image_topic = self.get_parameter("image_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.obstacle_points_topic = self.get_parameter("obstacle_points_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.model_path = str(self.get_parameter("model_path").value)
        self.device = self._normalize_device(str(self.get_parameter("device").value))
        self.imgsz = max(64, int(self.get_parameter("imgsz").value))
        self.confidence = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.max_period_s = 1.0 / max(0.1, float(self.get_parameter("max_hz").value))
        self.camera_hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.max_depth_stamp_delta_s = max(0.0, float(self.get_parameter("max_depth_stamp_delta_s").value))
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.depth_roi_lower_frac = min(0.95, max(0.05, float(self.get_parameter("depth_roi_lower_frac").value)))
        self.min_bbox_area_frac = max(0.0, float(self.get_parameter("min_bbox_area_frac").value))
        self.max_points_per_detection = max(3, int(self.get_parameter("max_points_per_detection").value))
        self.max_total_points = max(1, int(self.get_parameter("max_total_points").value))
        self.semantic_width_pad_m = max(0.0, float(self.get_parameter("semantic_width_pad_m").value))
        self.semantic_depth_thickness_m = max(0.05, float(self.get_parameter("semantic_depth_thickness_m").value))
        self.obstacle_classes = self._parse_classes(str(self.get_parameter("obstacle_classes").value))

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = None
        self.model_names = {}
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_stamp_s: Optional[float] = None
        self.last_infer_s = 0.0
        self.last_debug_s = 0.0

        self.points_pub = self.create_publisher(PoseArray, self.obstacle_points_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)

        self._load_model()
        self.get_logger().info(
            "YOLO semantic obstacle node started "
            f"(image={self.image_topic}, depth={self.depth_topic}, "
            f"out={self.obstacle_points_topic}, model={self.model_path}, "
            f"classes={sorted(self.obstacle_classes)})"
        )

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.get_logger().warn(
                "ultralytics is not installed; YOLO semantic obstacle points are disabled. "
                f"Install it on the Jetson or set START_YOLO_OBSTACLES=false. ({exc})"
            )
            return

        try:
            self.model = YOLO(self.model_path)
            self.model_names = getattr(self.model, "names", {}) or {}
        except Exception as exc:
            self.model = None
            self.get_logger().warn(f"Could not load YOLO model {self.model_path}: {exc}")

    @staticmethod
    def _parse_classes(raw: str) -> set:
        return {
            item.strip().lower().replace("_", " ")
            for item in raw.split(",")
            if item.strip()
        }

    @staticmethod
    def _normalize_device(raw: str) -> str:
        device = str(raw or "").strip()
        if device.lower() in {"", "auto", "default", "none"}:
            return ""
        return device

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self._depth_msg_to_numpy(msg)
        except Exception as exc:
            self.get_logger().warn(f"Could not convert depth image for YOLO semantic obstacles: {exc}")
            return
        self.latest_depth = np.asarray(depth, dtype=np.float32)
        self.latest_depth_stamp_s = self._stamp_to_seconds(msg.header.stamp)

    def image_callback(self, msg: Image) -> None:
        now_s = time.monotonic()
        if now_s - self.last_infer_s < self.max_period_s:
            return
        self.last_infer_s = now_s

        if self.model is None:
            self._publish_empty(msg, "model_unavailable")
            return
        if self.latest_depth is None or self.latest_depth_stamp_s is None:
            self._publish_empty(msg, "missing_depth")
            return

        image_stamp_s = self._stamp_to_seconds(msg.header.stamp)
        depth_age_s = abs(image_stamp_s - self.latest_depth_stamp_s)
        if depth_age_s > self.max_depth_stamp_delta_s:
            self._publish_empty(msg, f"stale_depth age={depth_age_s:.3f}s")
            return

        try:
            image_bgr = self._image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"Could not convert image for YOLO semantic obstacles: {exc}")
            self._publish_empty(msg, "image_convert_failed")
            return

        try:
            points, debug = self._detect_points(image_bgr, self.latest_depth)
        except Exception as exc:
            self.get_logger().warn(f"YOLO semantic obstacle inference failed: {exc}")
            self._publish_empty(msg, "inference_failed")
            return
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "base_link"
        for x_m, y_m in points[: self.max_total_points]:
            pose = Pose()
            pose.position.x = float(x_m)
            pose.position.y = float(y_m)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.points_pub.publish(pose_array)
        debug["published_points"] = len(pose_array.poses)
        self._publish_debug(msg, debug)

    def _detect_points(self, image_bgr: np.ndarray, depth: np.ndarray) -> Tuple[List[Tuple[float, float]], dict]:
        kwargs = {
            "source": image_bgr,
            "imgsz": self.imgsz,
            "conf": self.confidence,
            "iou": self.iou,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        results = self.model.predict(**kwargs)
        if not results:
            return [], {"reason": "no_results", "detections": 0, "accepted": 0, "classes": {}}

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return [], {"reason": "no_boxes", "detections": 0, "accepted": 0, "classes": {}}

        image_h, image_w = image_bgr.shape[:2]
        depth_h, depth_w = depth.shape[:2]
        points: List[Tuple[float, float]] = []
        accepted_classes = Counter()
        detections = 0
        accepted = 0

        for box in boxes:
            detections += 1
            cls_id = int(box.cls[0]) if getattr(box, "cls", None) is not None else -1
            cls_name = str(self.model_names.get(cls_id, cls_id)).lower()
            if cls_name not in self.obstacle_classes:
                continue

            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            box_area_frac = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, float(image_w * image_h))
            if box_area_frac < self.min_bbox_area_frac:
                continue

            depth_m = self._depth_for_box(depth, (x1, y1, x2, y2), image_w, image_h, depth_w, depth_h)
            if depth_m is None:
                continue

            det_points = self._points_for_box(depth_m, (x1, y1, x2, y2), image_w)
            points.extend(det_points[: self.max_points_per_detection])
            accepted += 1
            accepted_classes[cls_name] += 1

        return points[: self.max_total_points], {
            "reason": "ok",
            "detections": detections,
            "accepted": accepted,
            "classes": dict(accepted_classes),
        }

    def _depth_for_box(
        self,
        depth: np.ndarray,
        box_xyxy: Sequence[float],
        image_w: int,
        image_h: int,
        depth_w: int,
        depth_h: int,
    ) -> Optional[float]:
        x1, y1, x2, y2 = box_xyxy
        lower_y1 = y1 + self.depth_roi_lower_frac * max(1.0, y2 - y1)
        sx = depth_w / max(1.0, float(image_w))
        sy = depth_h / max(1.0, float(image_h))
        dx1 = max(0, min(depth_w - 1, int(round(x1 * sx))))
        dx2 = max(0, min(depth_w, int(round(x2 * sx))))
        dy1 = max(0, min(depth_h - 1, int(round(lower_y1 * sy))))
        dy2 = max(0, min(depth_h, int(round(y2 * sy))))
        if dx2 <= dx1 or dy2 <= dy1:
            return None

        roi = depth[dy1:dy2, dx1:dx2]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.percentile(valid, 35.0))

    def _points_for_box(self, depth_m: float, box_xyxy: Sequence[float], image_w: int) -> List[Tuple[float, float]]:
        x1, _, x2, _ = box_xyxy
        center_x = 0.5 * max(image_w - 1, 1)
        half_fov = 0.5 * self.camera_hfov_rad

        def bearing(pixel_x: float) -> float:
            return ((float(pixel_x) - center_x) / max(center_x, 1.0)) * half_fov

        a1 = bearing(x1)
        a2 = bearing(x2)
        ac = bearing(0.5 * (x1 + x2))
        y1 = depth_m * math.sin(a1)
        y2 = depth_m * math.sin(a2)
        yc = depth_m * math.sin(ac)
        half_width = max(abs(y2 - y1) * 0.5 + self.semantic_width_pad_m, 0.22)
        half_depth = 0.5 * self.semantic_depth_thickness_m

        y_samples = np.linspace(yc - half_width, yc + half_width, num=5)
        x_samples = (max(self.min_depth_m, depth_m - half_depth), depth_m, depth_m + half_depth)
        return [(float(x), float(y)) for x in x_samples for y in y_samples]

    def _publish_empty(self, msg: Image, reason: str) -> None:
        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = "base_link"
        self.points_pub.publish(pose_array)
        self._publish_debug(msg, {
            "reason": reason,
            "detections": 0,
            "accepted": 0,
            "classes": {},
            "published_points": 0,
        })

    def _publish_debug(self, msg: Image, debug: dict) -> None:
        now_s = time.monotonic()
        if now_s - self.last_debug_s < 0.5:
            return
        self.last_debug_s = now_s
        payload = {
            "stamp_sec": self._stamp_to_seconds(msg.header.stamp),
            "model_loaded": self.model is not None,
            "model_path": self.model_path,
            **debug,
        }
        self.debug_pub.publish(String(data=json.dumps(payload)))

    def _depth_msg_to_numpy(self, msg: Image) -> np.ndarray:
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            except Exception:
                pass
        return self._manual_image_msg_to_numpy(msg)

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
            return np.ascontiguousarray(arr[:, :, ::-1])
        if encoding == "bgra8":
            return np.ascontiguousarray(arr[:, :, :3])
        if encoding == "rgba8":
            return np.ascontiguousarray(arr[:, :, [2, 1, 0]])
        if arr.ndim == 2:
            return np.repeat(arr[:, :, None], 3, axis=2)
        if arr.ndim == 3 and arr.shape[2] == 3:
            return arr
        if arr.ndim == 3 and arr.shape[2] == 4:
            return np.ascontiguousarray(arr[:, :, :3])
        raise ValueError(f"Unsupported YOLO image encoding: {msg.encoding}")

    @staticmethod
    def _manual_image_msg_to_numpy(msg: Image) -> np.ndarray:
        encoding = str(msg.encoding).lower()
        encoding_info = {
            "32fc1": (np.float32, 1),
            "16uc1": (np.uint16, 1),
            "mono16": (np.uint16, 1),
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
        if bool(msg.is_bigendian) != (sys.byteorder == "big") and dtype.itemsize > 1:
            data = data.byteswap().view(dtype)
        data = data.reshape((height, row_elems))
        useful = data[:, : width * channels]
        if channels == 1:
            return np.ascontiguousarray(useful.reshape((height, width)))
        return np.ascontiguousarray(useful.reshape((height, width, channels)))

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = YoloSemanticObstacleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
