#!/usr/bin/env python3
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ugv_sensor_sync.msg import NavSensorFrame

try:
    if int(str(np.__version__).split(".", 1)[0]) >= 2:
        raise ImportError("cv_bridge from ROS Humble is not compatible with NumPy 2.x")
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - depends on robot runtime packages
    CvBridge = None


Color = Tuple[int, int, int]


class UgvDebugDashboard(Node):
    def __init__(self):
        super().__init__("ugv_debug_dashboard")

        self.declare_parameter("image_topic", "/zed/image")
        self.declare_parameter("depth_topic", "/zed/depth")
        self.declare_parameter("nav_frame_topic", "/sensors/nav_frame")
        self.declare_parameter("semantic_points_topic", "/sensors/yolo_semantic_obstacle_points")
        self.declare_parameter("fusion_summary_topic", "/sensors/synced_summary")
        self.declare_parameter("nav_status_topic", "/ugv_nav_status")
        self.declare_parameter("marker_debug_topic", "/ugv/marker_vision_debug")
        self.declare_parameter("yolo_debug_topic", "/sensors/yolo_semantic_debug")
        self.declare_parameter("window_name", "UGV Perception Dashboard")
        self.declare_parameter("update_hz", 8.0)
        self.declare_parameter("canvas_width_px", 1600)
        self.declare_parameter("canvas_height_px", 920)
        self.declare_parameter("camera_hfov_deg", 110.0)
        self.declare_parameter("camera_search_depth_m", 0.30)
        self.declare_parameter("lidar_used_fov_deg", 180.0)
        self.declare_parameter("lidar_offset_x_m", 0.30)
        self.declare_parameter("lidar_offset_y_m", 0.0)
        self.declare_parameter("robot_length_m", 0.762)
        self.declare_parameter("robot_width_m", 0.762)
        self.declare_parameter("field_width_m", 13.716)
        self.declare_parameter("field_height_m", 13.716)
        self.declare_parameter("session_map_resolution_m", 0.15)
        self.declare_parameter("max_sensor_age_s", 1.0)
        self.declare_parameter("screenshot_dir", str(Path.home() / "ugv_dashboard_screenshots"))

        self.window_name = str(self.get_parameter("window_name").value)
        self.canvas_w = max(1000, int(self.get_parameter("canvas_width_px").value))
        self.canvas_h = max(700, int(self.get_parameter("canvas_height_px").value))
        update_hz = max(1.0, float(self.get_parameter("update_hz").value))
        self.camera_hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.camera_search_depth_m = max(0.05, float(self.get_parameter("camera_search_depth_m").value))
        self.lidar_used_fov_deg = float(self.get_parameter("lidar_used_fov_deg").value)
        self.lidar_offset = (
            float(self.get_parameter("lidar_offset_x_m").value),
            float(self.get_parameter("lidar_offset_y_m").value),
        )
        self.robot_length_m = float(self.get_parameter("robot_length_m").value)
        self.robot_width_m = float(self.get_parameter("robot_width_m").value)
        self.field_w_m = max(1.0, float(self.get_parameter("field_width_m").value))
        self.field_h_m = max(1.0, float(self.get_parameter("field_height_m").value))
        self.map_res_m = max(0.05, float(self.get_parameter("session_map_resolution_m").value))
        self.max_sensor_age_s = max(0.1, float(self.get_parameter("max_sensor_age_s").value))
        self.screenshot_dir = Path(str(self.get_parameter("screenshot_dir").value)).expanduser()

        self.bridge = CvBridge() if CvBridge is not None else None
        self.latest_image: Optional[Tuple[np.ndarray, float, float, Tuple[int, int]]] = None
        self.latest_depth: Optional[Tuple[np.ndarray, float, float]] = None
        self.latest_nav_frame: Optional[Tuple[NavSensorFrame, float, float]] = None
        self.latest_semantic_points: Optional[Tuple[PoseArray, float, float]] = None
        self.latest_yolo_debug: Dict = {}
        self.latest_marker_debug: Dict = {}
        self.latest_nav_status: Dict = {}
        self.latest_fusion_summary: Dict = {}
        self.latest_json_recv_s: Dict[str, float] = {}

        self.pose_trace: List[Tuple[float, float, float]] = []
        self.camera_seen_cells = set()
        self.lidar_hit_cells = set()
        self._last_map_pose: Optional[Tuple[float, float, float]] = None
        self._last_integrated_scan_stamp: Optional[float] = None
        self._window_ready = False
        self._gui_available = self._detect_gui_available()
        self._last_canvas: Optional[np.ndarray] = None

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSensorFrame,
            str(self.get_parameter("nav_frame_topic").value),
            self.nav_frame_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("semantic_points_topic").value),
            self.semantic_points_callback,
            qos_profile_sensor_data,
        )
        self._subscribe_json("fusion", str(self.get_parameter("fusion_summary_topic").value))
        self._subscribe_json("nav", str(self.get_parameter("nav_status_topic").value))
        self._subscribe_json("marker", str(self.get_parameter("marker_debug_topic").value))
        self._subscribe_json("yolo", str(self.get_parameter("yolo_debug_topic").value))
        self.create_timer(1.0 / update_hz, self.draw)

        if not self._gui_available:
            self.get_logger().warn(
                "No GUI display detected. Start this from a VNC desktop or set DISPLAY before launching."
            )
        self.get_logger().info(
            "UGV debug dashboard ready "
            f"(image={self.get_parameter('image_topic').value}, "
            f"depth={self.get_parameter('depth_topic').value}, "
            f"nav_frame={self.get_parameter('nav_frame_topic').value})"
        )

    def _detect_gui_available(self) -> bool:
        if os.name == "nt":
            return True
        if os.environ.get("WAYLAND_DISPLAY"):
            return True
        display = os.environ.get("DISPLAY")
        if not display:
            return False
        display_num = display.split(":", 1)[1].split(".", 1)[0] if ":" in display else ""
        if display_num and not Path(f"/tmp/.X11-unix/X{display_num}").exists():
            return False
        for tool in ("xdpyinfo", "xset"):
            exe = shutil.which(tool)
            if not exe:
                continue
            try:
                result = subprocess.run(
                    [exe, "q"] if tool == "xset" else [exe],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
                return result.returncode == 0
            except Exception:
                return False
        return True

    def _subscribe_json(self, key: str, topic: str) -> None:
        def callback(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                data = {"raw": msg.data}
            if key == "fusion":
                self.latest_fusion_summary = data
            elif key == "nav":
                self.latest_nav_status = data
            elif key == "marker":
                self.latest_marker_debug = data
            elif key == "yolo":
                self.latest_yolo_debug = data
            self.latest_json_recv_s[key] = time.monotonic()

        self.create_subscription(String, topic, callback, 10)

    def image_callback(self, msg: Image) -> None:
        try:
            image = self._image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"Could not convert dashboard image: {exc}")
            return
        self.latest_image = (
            image,
            self._stamp_to_seconds(msg.header.stamp),
            time.monotonic(),
            (int(msg.width), int(msg.height)),
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self._depth_msg_to_numpy(msg)
        except Exception as exc:
            self.get_logger().warn(f"Could not convert dashboard depth: {exc}")
            return
        self.latest_depth = (np.asarray(depth, dtype=np.float32), self._stamp_to_seconds(msg.header.stamp), time.monotonic())

    def nav_frame_callback(self, msg: NavSensorFrame) -> None:
        self.latest_nav_frame = (msg, self._stamp_to_seconds(msg.header.stamp), time.monotonic())

    def semantic_points_callback(self, msg: PoseArray) -> None:
        self.latest_semantic_points = (msg, self._stamp_to_seconds(msg.header.stamp), time.monotonic())

    def draw(self) -> None:
        self._integrate_session_map()
        canvas = np.full((self.canvas_h, self.canvas_w, 3), (24, 27, 31), dtype=np.uint8)

        margin = 12
        status_h = 182
        left_w = int(self.canvas_w * 0.61)
        right_w = self.canvas_w - left_w - 3 * margin
        top_h = int((self.canvas_h - status_h - 3 * margin) * 0.62)
        bottom_h = self.canvas_h - status_h - top_h - 4 * margin
        camera_rect = (margin, margin, left_w, top_h)
        map_rect = (margin, margin + top_h + margin, left_w, bottom_h)
        depth_rect = (2 * margin + left_w, margin, right_w, int(top_h * 0.48))
        lidar_rect = (
            2 * margin + left_w,
            margin + int(top_h * 0.48) + margin,
            right_w,
            top_h - int(top_h * 0.48) - margin,
        )
        status_rect = (margin, self.canvas_h - status_h - margin, self.canvas_w - 2 * margin, status_h)

        self._paste_panel(canvas, camera_rect, self._render_camera_panel(camera_rect[2], camera_rect[3]))
        self._paste_panel(canvas, depth_rect, self._render_depth_panel(depth_rect[2], depth_rect[3]))
        self._paste_panel(canvas, lidar_rect, self._render_lidar_panel(lidar_rect[2], lidar_rect[3]))
        self._paste_panel(canvas, map_rect, self._render_session_map_panel(map_rect[2], map_rect[3]))
        self._paste_panel(canvas, status_rect, self._render_status_panel(status_rect[2], status_rect[3]))

        self._last_canvas = canvas
        if not self._gui_available:
            return
        try:
            if not self._window_ready:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, self.canvas_w, self.canvas_h)
                self._window_ready = True
            cv2.imshow(self.window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("Dashboard close requested")
                rclpy.shutdown()
            elif key == ord("s"):
                self._save_screenshot()
        except Exception as exc:
            self._gui_available = False
            self.get_logger().warn(f"Dashboard GUI disabled after OpenCV window error: {exc}")

    def _render_camera_panel(self, w: int, h: int) -> np.ndarray:
        panel = self._panel_base(w, h, "ZED image + YOLO + marker")
        if self.latest_image is None:
            self._draw_center_text(panel, "waiting for /zed/image")
            return panel
        image, _, recv_s, original_size = self.latest_image
        view, scale, dx, dy = self._fit_image(image, w, h - 28)
        y0 = 28
        panel[y0 + dy:y0 + dy + view.shape[0], dx:dx + view.shape[1]] = view
        self._draw_yolo_boxes(panel, scale, dx, y0 + dy, original_size)
        self._draw_marker_box(panel, scale, dx, y0 + dy, original_size)
        self._put_text(panel, f"age {self._age_text(recv_s)}", (w - 155, 20), (180, 210, 255), 0.48)
        return panel

    def _render_depth_panel(self, w: int, h: int) -> np.ndarray:
        panel = self._panel_base(w, h, "ZED depth map")
        if self.latest_depth is None:
            self._draw_center_text(panel, "waiting for /zed/depth")
            return panel
        depth, _, recv_s = self.latest_depth
        depth_view = np.asarray(depth, dtype=np.float32)
        valid = depth_view[np.isfinite(depth_view)]
        valid = valid[(valid >= 0.05) & (valid <= 10.0)]
        if valid.size == 0:
            self._draw_center_text(panel, "no valid depth")
            return panel
        depth_for_color = np.nan_to_num(depth_view, nan=4.0, posinf=4.0, neginf=4.0)
        clipped = np.clip(depth_for_color, 0.2, 4.0)
        norm = ((4.0 - clipped) / 3.8 * 255.0).astype(np.uint8)
        norm[~np.isfinite(depth_view)] = 0
        color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        view, _, dx, dy = self._fit_image(color, w, h - 28)
        y0 = 28
        panel[y0 + dy:y0 + dy + view.shape[0], dx:dx + view.shape[1]] = view
        p10 = float(np.percentile(valid, 10.0))
        p50 = float(np.percentile(valid, 50.0))
        self._put_text(panel, f"p10={p10:.2f}m median={p50:.2f}m age {self._age_text(recv_s)}", (10, h - 12), (230, 230, 230), 0.45)
        return panel

    def _render_lidar_panel(self, w: int, h: int) -> np.ndarray:
        panel = self._panel_base(w, h, "Local LiDAR / fused obstacles")
        center = (w // 2, h - 48)
        scale = min(w * 0.42, h * 0.78) / 3.2

        def to_px(x_m: float, y_m: float) -> Tuple[int, int]:
            return int(round(center[0] + y_m * scale)), int(round(center[1] - x_m * scale))

        for r in (0.5, 1.0, 1.5, 2.0, 3.0):
            cv2.circle(panel, center, int(round(r * scale)), (50, 58, 68), 1, cv2.LINE_AA)
            self._put_text(panel, f"{r:.1f}", (center[0] + 4, int(center[1] - r * scale)), (100, 110, 120), 0.35)

        half = math.radians(0.5 * self.lidar_used_fov_deg)
        fov_pts = [(0.0, 0.0)]
        for a in np.linspace(-half, half, 40):
            fov_pts.append((3.0 * math.cos(a), 3.0 * math.sin(a)))
        poly = np.array([to_px(x, y) for x, y in fov_pts], dtype=np.int32)
        cv2.polylines(panel, [poly], True, (38, 95, 92), 1, cv2.LINE_AA)

        nav_frame = self.latest_nav_frame[0] if self.latest_nav_frame is not None else None
        if nav_frame is None:
            self._draw_center_text(panel, "waiting for /sensors/nav_frame")
        else:
            scan = nav_frame.scan
            ox, oy = self.lidar_offset
            angle = float(scan.angle_min)
            step = max(1, int(len(scan.ranges) / 720))
            points_used = 0
            for i, r in enumerate(scan.ranges):
                if i % step != 0:
                    angle += float(scan.angle_increment)
                    continue
                if math.isfinite(float(r)) and scan.range_min <= float(r) <= scan.range_max:
                    lx = float(r) * math.cos(angle) + ox
                    ly = float(r) * math.sin(angle) + oy
                    in_fov = abs(_wrap_pi(angle)) <= half
                    color = (70, 220, 115) if in_fov else (65, 70, 78)
                    cv2.circle(panel, to_px(lx, ly), 2 if in_fov else 1, color, -1, cv2.LINE_AA)
                    points_used += int(in_fov)
                angle += float(scan.angle_increment)
            self._draw_pose_array(panel, nav_frame.zed_obstacle_points, to_px, (0, 178, 255), 2)
        if self.latest_semantic_points is not None:
            self._draw_pose_array(panel, self.latest_semantic_points[0], to_px, (255, 80, 210), 3)

        self._draw_robot(panel, to_px, (220, 230, 240))
        self._draw_camera_wedge(panel, to_px)
        fusion = self.latest_fusion_summary
        if fusion:
            line = (
                f"front lidar={_fmt_m(fusion.get('front_lidar_range_m'))} "
                f"depth={_fmt_m(fusion.get('min_depth_range_m'))} "
                f"clear={_fmt_m(fusion.get('front_clearance_m'))} src={fusion.get('front_clearance_source')}"
            )
            self._put_text(panel, line, (10, h - 12), (230, 230, 230), 0.42)
        return panel

    def _render_session_map_panel(self, w: int, h: int) -> np.ndarray:
        panel = self._panel_base(w, h, "Session map: pose trace, LiDAR hits, camera-searched cells")
        plot = panel[28:h - 8, 8:w - 8]
        ph, pw = plot.shape[:2]
        scale = min(pw / self.field_w_m, ph / self.field_h_m)
        ox = int((pw - self.field_w_m * scale) * 0.5)
        oy = int(ph - (ph - self.field_h_m * scale) * 0.5)

        def world_to_px(x: float, y: float) -> Tuple[int, int]:
            return int(round(ox + x * scale)), int(round(oy - y * scale))

        cv2.rectangle(plot, world_to_px(0.0, 0.0), world_to_px(self.field_w_m, self.field_h_m), (85, 90, 100), 1)
        for gx, gy in list(self.camera_seen_cells)[-6000:]:
            x = (gx + 0.5) * self.map_res_m
            y = (gy + 0.5) * self.map_res_m
            cv2.circle(plot, world_to_px(x, y), 1, (105, 175, 185), -1)
        for gx, gy in list(self.lidar_hit_cells)[-6000:]:
            x = (gx + 0.5) * self.map_res_m
            y = (gy + 0.5) * self.map_res_m
            cv2.circle(plot, world_to_px(x, y), 1, (75, 220, 120), -1)
        if len(self.pose_trace) >= 2:
            pts = np.array([world_to_px(x, y) for x, y, _ in self.pose_trace[-1500:]], dtype=np.int32)
            cv2.polylines(plot, [pts], False, (210, 210, 250), 1, cv2.LINE_AA)
        pose = self._current_pose()
        if pose is not None:
            x, y, yaw = pose
            self._draw_robot_world(plot, world_to_px, x, y, yaw)
            self._put_text(panel, f"pose x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg", (12, h - 12), (230, 230, 230), 0.42)
        return panel

    def _render_status_panel(self, w: int, h: int) -> np.ndarray:
        panel = self._panel_base(w, h, "Decision / sensor health")
        nav = self.latest_nav_status or {}
        fusion = self.latest_fusion_summary or {}
        yolo = self.latest_yolo_debug or {}
        marker = self.latest_marker_debug or {}
        cmd = nav.get("cmd", {}) if isinstance(nav.get("cmd"), dict) else {}
        odom = nav.get("odom_delta", {}) if isinstance(nav.get("odom_delta"), dict) else {}
        mission = nav.get("mission", {}) if isinstance(nav.get("mission"), dict) else {}
        sectors = nav.get("sectors_m", {}) if isinstance(nav.get("sectors_m"), dict) else {}
        active_scan = nav.get("active_scan", {}) if isinstance(nav.get("active_scan"), dict) else {}
        velocity = nav.get("velocity_control", {}) if isinstance(nav.get("velocity_control"), dict) else {}
        ages = {
            "img": self._tuple_age(self.latest_image),
            "depth": self._tuple_age(self.latest_depth),
            "nav_frame": self._tuple_age(self.latest_nav_frame),
            "fusion": self._json_age("fusion"),
            "nav": self._json_age("nav"),
            "yolo": self._json_age("yolo"),
            "marker": self._json_age("marker"),
        }
        warn = odom.get("warning") or fusion.get("depth_blind_hazard") or ""
        lines = [
            f"phase={mission.get('phase')} cmd={cmd.get('mode')} reason={cmd.get('reason')} raw=({cmd.get('raw_left')},{cmd.get('raw_right')})",
            f"pose={nav.get('pose_m')} goal={mission.get('active_goal_m')} plan={nav.get('planner')} {nav.get('plan_time_ms')}ms",
            f"clearance front={_fmt_m(fusion.get('front_clearance_m'))} src={fusion.get('front_clearance_source')} sectors f/fl/fr={sectors.get('front')}/{sectors.get('front_left')}/{sectors.get('front_right')}",
            f"velocity ctrl enabled={velocity.get('enabled')} safe={velocity.get('safe_samples')}/{velocity.get('samples')} v={velocity.get('selected_v_mps')} omega={velocity.get('selected_omega_radps')} gap={velocity.get('best_gap_heading_deg')}deg/{velocity.get('best_gap_depth_m')}m clear={velocity.get('min_clearance_m')} path_clear={velocity.get('path_clearance_m')}:{velocity.get('path_clearance_source')} state={velocity.get('safety_state')}",
            f"ZED depth obstacles pts={fusion.get('depth_obstacle_points')} filtered={fusion.get('depth_obstacle_points_filtered')} comps={fusion.get('depth_obstacle_components')} cells={fusion.get('depth_obstacle_candidate_cells')}",
            f"active scan rem={active_scan.get('remaining')} dir={active_scan.get('direction')} cd={active_scan.get('cooldown_steps')} probe={active_scan.get('probe_steps')} alt={active_scan.get('needs_opposite_side')} evidence={active_scan.get('front_blocked_evidence')}/{active_scan.get('plan_failed_evidence')} corridor_pass={active_scan.get('front_corridor_passable')} corridor_gap={active_scan.get('front_corridor_gap_heading_deg')}deg/{active_scan.get('front_corridor_gap_depth_m')}m safe_dirs={active_scan.get('front_corridor_safe_headings')} depth_corridor={active_scan.get('front_depth_corridor_points')} min={active_scan.get('front_depth_corridor_min_m')} reason={active_scan.get('reason')}",
            f"YOLO loaded={yolo.get('model_loaded')} accepted={yolo.get('accepted')} boxes={len(yolo.get('boxes', []))} classes={yolo.get('classes')}",
            f"marker detected={marker.get('detected')} method={marker.get('method')} reason={marker.get('reason')} bbox={marker.get('candidate_bbox')}",
            f"ages {', '.join(f'{k}={v:.2f}s' if v is not None else f'{k}=None' for k, v in ages.items())}",
            f"odom={odom} warning={warn}",
            "keys: q/esc close, s screenshot",
        ]
        y = 34
        for line in lines:
            color = (90, 110, 255) if "warning=" in line and warn else (226, 230, 235)
            self._put_text(panel, line[:220], (12, y), color, 0.40)
            y += 14
        return panel

    def _draw_yolo_boxes(self, panel: np.ndarray, scale: float, dx: int, dy: int, original_size: Tuple[int, int]) -> None:
        boxes = self.latest_yolo_debug.get("boxes", []) if isinstance(self.latest_yolo_debug, dict) else []
        if not isinstance(boxes, list):
            return
        for box in boxes:
            if not isinstance(box, dict):
                continue
            coords = box.get("bbox_xyxy")
            if not isinstance(coords, list) or len(coords) < 4:
                continue
            x1, y1, x2, y2 = [int(round(float(v) * scale)) for v in coords[:4]]
            p1 = (dx + x1, dy + y1)
            p2 = (dx + x2, dy + y2)
            accepted = bool(box.get("accepted"))
            color = (70, 230, 120) if accepted else (85, 95, 105)
            cv2.rectangle(panel, p1, p2, color, 2 if accepted else 1, cv2.LINE_AA)
            label = f"{box.get('class')} {box.get('confidence', 0)}"
            if box.get("depth_m") is not None:
                label += f" {box.get('depth_m')}m"
            self._label(panel, label, (p1[0], max(34, p1[1] - 4)), color)

    def _draw_marker_box(self, panel: np.ndarray, scale: float, dx: int, dy: int, original_size: Tuple[int, int]) -> None:
        marker = self.latest_marker_debug
        bbox = marker.get("candidate_bbox") if isinstance(marker, dict) else None
        if not isinstance(bbox, list) or len(bbox) < 4:
            return
        x, y, bw, bh = [int(round(float(v) * scale)) for v in bbox[:4]]
        p1 = (dx + x, dy + y)
        p2 = (dx + x + bw, dy + y + bh)
        color = (0, 220, 255) if marker.get("detected") else (40, 160, 230)
        cv2.rectangle(panel, p1, p2, color, 2, cv2.LINE_AA)
        label = f"marker {marker.get('method') or marker.get('reason')}"
        self._label(panel, label, (p1[0], p2[1] + 16), color)

    def _integrate_session_map(self) -> None:
        pose = self._current_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self._last_map_pose is None or math.hypot(x - self._last_map_pose[0], y - self._last_map_pose[1]) > 0.03 or abs(_wrap_pi(yaw - self._last_map_pose[2])) > math.radians(3.0):
            self.pose_trace.append((x, y, yaw))
            self.pose_trace = self.pose_trace[-2500:]
            self._integrate_camera_wedge_world(x, y, yaw)
            self._last_map_pose = pose
        if self.latest_nav_frame is not None:
            msg, stamp_s, _ = self.latest_nav_frame
            if self._last_integrated_scan_stamp != stamp_s:
                self._integrate_scan_world(msg.scan, x, y, yaw)
                self._last_integrated_scan_stamp = stamp_s

    def _integrate_camera_wedge_world(self, x: float, y: float, yaw: float) -> None:
        half = 0.5 * self.camera_hfov_rad
        for r in np.linspace(0.05, self.camera_search_depth_m, 6):
            for a in np.linspace(-half, half, 21):
                wx = x + float(r) * math.cos(yaw + float(a))
                wy = y + float(r) * math.sin(yaw + float(a))
                self._add_cell(self.camera_seen_cells, wx, wy)

    def _integrate_scan_world(self, scan, pose_x: float, pose_y: float, yaw: float) -> None:
        half = math.radians(0.5 * self.lidar_used_fov_deg)
        angle = float(scan.angle_min)
        ox, oy = self.lidar_offset
        step = max(1, int(len(scan.ranges) / 360))
        c = math.cos(yaw)
        s = math.sin(yaw)
        for i, r in enumerate(scan.ranges):
            if i % step != 0:
                angle += float(scan.angle_increment)
                continue
            if math.isfinite(float(r)) and scan.range_min <= float(r) <= min(scan.range_max, 6.0) and abs(_wrap_pi(angle)) <= half:
                lx = float(r) * math.cos(angle) + ox
                ly = float(r) * math.sin(angle) + oy
                wx = pose_x + lx * c - ly * s
                wy = pose_y + lx * s + ly * c
                self._add_cell(self.lidar_hit_cells, wx, wy)
            angle += float(scan.angle_increment)

    def _add_cell(self, cells, x: float, y: float) -> None:
        if 0.0 <= x <= self.field_w_m and 0.0 <= y <= self.field_h_m:
            cells.add((int(math.floor(x / self.map_res_m)), int(math.floor(y / self.map_res_m))))
            if len(cells) > 30000:
                for cell in list(cells)[:5000]:
                    cells.discard(cell)

    def _current_pose(self) -> Optional[Tuple[float, float, float]]:
        nav = self.latest_nav_status
        pose = nav.get("pose_m") if isinstance(nav, dict) else None
        if isinstance(pose, list) and len(pose) >= 3:
            try:
                return float(pose[0]), float(pose[1]), math.radians(float(pose[2]))
            except (TypeError, ValueError):
                return None
        return None

    def _draw_pose_array(self, panel: np.ndarray, poses: PoseArray, to_px, color: Color, radius: int) -> None:
        for pose in poses.poses[:300]:
            cv2.circle(panel, to_px(float(pose.position.x), float(pose.position.y)), radius, color, -1, cv2.LINE_AA)

    def _draw_robot(self, panel: np.ndarray, to_px, color: Color) -> None:
        hl = 0.5 * self.robot_length_m
        hw = 0.5 * self.robot_width_m
        pts = np.array([to_px(hl, hw), to_px(hl, -hw), to_px(-hl, -hw), to_px(-hl, hw)], dtype=np.int32)
        cv2.polylines(panel, [pts], True, color, 2, cv2.LINE_AA)
        cv2.arrowedLine(panel, to_px(0.0, 0.0), to_px(0.55, 0.0), (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)

    def _draw_camera_wedge(self, panel: np.ndarray, to_px) -> None:
        half = 0.5 * self.camera_hfov_rad
        pts = [(0.0, 0.0)]
        for a in np.linspace(-half, half, 28):
            pts.append((self.camera_search_depth_m * math.cos(a), self.camera_search_depth_m * math.sin(a)))
        cv2.polylines(panel, [np.array([to_px(x, y) for x, y in pts], dtype=np.int32)], True, (120, 210, 230), 1, cv2.LINE_AA)

    def _draw_robot_world(self, panel: np.ndarray, world_to_px, x: float, y: float, yaw: float) -> None:
        hl = 0.5 * self.robot_length_m
        hw = 0.5 * self.robot_width_m
        local = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
        c = math.cos(yaw)
        s = math.sin(yaw)
        pts = []
        for lx, ly in local:
            pts.append(world_to_px(x + lx * c - ly * s, y + lx * s + ly * c))
        cv2.polylines(panel, [np.array(pts, dtype=np.int32)], True, (245, 245, 255), 2, cv2.LINE_AA)
        cv2.arrowedLine(panel, world_to_px(x, y), world_to_px(x + 0.55 * c, y + 0.55 * s), (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)

    def _fit_image(self, image: np.ndarray, w: int, h: int) -> Tuple[np.ndarray, float, int, int]:
        ih, iw = image.shape[:2]
        scale = min(w / max(1, iw), h / max(1, ih))
        nw = max(1, int(round(iw * scale)))
        nh = max(1, int(round(ih * scale)))
        resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
        return resized, scale, (w - nw) // 2, (h - nh) // 2

    @staticmethod
    def _panel_base(w: int, h: int, title: str) -> np.ndarray:
        panel = np.full((h, w, 3), (34, 38, 44), dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (w - 1, h - 1), (70, 78, 88), 1)
        cv2.putText(panel, title, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 238, 242), 1, cv2.LINE_AA)
        return panel

    @staticmethod
    def _paste_panel(canvas: np.ndarray, rect: Tuple[int, int, int, int], panel: np.ndarray) -> None:
        x, y, w, h = rect
        canvas[y:y + h, x:x + w] = panel[:h, :w]

    @staticmethod
    def _put_text(panel: np.ndarray, text: str, org: Tuple[int, int], color: Color, scale: float = 0.45, thickness: int = 1) -> None:
        cv2.putText(panel, str(text), org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    def _label(self, panel: np.ndarray, text: str, org: Tuple[int, int], color: Color) -> None:
        x, y = org
        (tw, th), _ = cv2.getTextSize(str(text), cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(panel, (x, y - th - 5), (x + tw + 6, y + 3), (20, 22, 26), -1)
        self._put_text(panel, text, (x + 3, y), color, 0.45)

    def _draw_center_text(self, panel: np.ndarray, text: str) -> None:
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        x = max(10, (panel.shape[1] - tw) // 2)
        y = max(35, (panel.shape[0] + th) // 2)
        self._put_text(panel, text, (x, y), (180, 188, 198), 0.55)

    def _save_screenshot(self) -> None:
        if self._last_canvas is None:
            return
        self.screenshot_dir.mkdir(parents=True, exist_ok=True)
        path = self.screenshot_dir / f"ugv_dashboard_{time.strftime('%Y%m%d_%H%M%S')}.png"
        cv2.imwrite(str(path), self._last_canvas)
        self.get_logger().info(f"Saved dashboard screenshot: {path}")

    def _tuple_age(self, item) -> Optional[float]:
        if item is None:
            return None
        return max(0.0, time.monotonic() - float(item[2]))

    def _json_age(self, key: str) -> Optional[float]:
        if key not in self.latest_json_recv_s:
            return None
        return max(0.0, time.monotonic() - self.latest_json_recv_s[key])

    def _age_text(self, recv_s: float) -> str:
        age = max(0.0, time.monotonic() - recv_s)
        return f"{age:.2f}s"

    def _depth_msg_to_numpy(self, msg: Image) -> np.ndarray:
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
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
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

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


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _fmt_m(value) -> str:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "None"
    if not math.isfinite(v):
        return "inf"
    return f"{v:.2f}m"


def main(args=None):
    rclpy.init(args=args)
    node = UgvDebugDashboard()
    try:
        rclpy.spin(node)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
