#!/usr/bin/env python3
from collections import deque
import json
import math

import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Header, String

try:
    if int(str(np.__version__).split('.', 1)[0]) >= 2:
        raise ImportError('cv_bridge from ROS Humble is not compatible with NumPy 2.x')
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - optional runtime optimization
    CvBridge = None


class ZedSyncNode(Node):
    def __init__(self):
        super().__init__('zed_sync_node')

        self.declare_parameter('image_topic', '/zed/image')
        self.declare_parameter('camera_info_topic', '/zed/left/camera_info')
        self.declare_parameter('depth_topic', '/zed/depth')
        self.declare_parameter('imu_topic', '/zed/imu')
        self.declare_parameter('image_frame_id', 'zed_left')
        self.declare_parameter('depth_frame_id', 'zed_depth')
        self.declare_parameter('imu_frame_id', 'zed_imu')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('imu_publish_rate_hz', 100.0)
        self.declare_parameter('publish_image', False)
        self.declare_parameter('depth_downsample_factor', 2)
        self.declare_parameter('image_downsample_factor', 1)
        self.declare_parameter('status_topic', '/zed/status')
        self.declare_parameter('status_period_s', 1.0)
        self.declare_parameter('imu_rate_window_s', 2.0)
        self.declare_parameter('publish_depth_without_subscribers', False)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.image_frame_id = self.get_parameter('image_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        imu_publish_rate_hz = float(self.get_parameter('imu_publish_rate_hz').value)
        self.publish_image = bool(self.get_parameter('publish_image').value)
        self.depth_downsample_factor = max(1, int(self.get_parameter('depth_downsample_factor').value))
        self.image_downsample_factor = max(1, int(self.get_parameter('image_downsample_factor').value))
        status_topic = self.get_parameter('status_topic').value
        self.status_period_s = max(0.2, float(self.get_parameter('status_period_s').value))
        self.imu_rate_window_s = max(0.1, float(self.get_parameter('imu_rate_window_s').value))
        self.publish_depth_without_subscribers = bool(
            self.get_parameter('publish_depth_without_subscribers').value
        )
        self.depth_period_s = 1.0 / max(publish_rate_hz, 0.1)
        self.poll_period_s = 1.0 / max(publish_rate_hz, imu_publish_rate_hz, 1.0)
        self.bridge = CvBridge() if CvBridge is not None else None

        self.image_pub = None
        self.camera_info_pub = None
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, image_topic, qos_profile_sensor_data)
            self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.frame_count = 0
        self.camera_grab_count = 0
        self.imu_count = 0
        self.imu_publish_failures = 0
        self.last_imu_error = None
        self.imu_busy_skips = 0
        self.last_imu_publish_s = None
        self.last_imu_ang_radps = [0.0, 0.0, 0.0]
        self.last_imu_ang_degps = [0.0, 0.0, 0.0]
        self.last_imu_lin_mps2 = [0.0, 0.0, 0.0]
        self.last_imu_timestamp_ns = None
        self.imu_duplicate_samples = 0
        self.imu_publish_times_s = deque()
        self.camera_grab_times_s = deque()
        self.last_imu_failure_log_s = 0.0
        self.next_depth_publish_s = 0.0
        self.latest_depth_stamp_s = None
        self.latest_depth_shape = []
        self.latest_valid_depth_samples = 0
        self.latest_depth_min_m = None
        self.latest_depth_p10_m = None
        self.latest_depth_mean_m = None

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'ZED failed to open: {status}')
            raise RuntimeError(str(status))
        self.camera_model = self._camera_model_text()

        self.runtime = sl.RuntimeParameters()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.sensor_data = sl.SensorsData()
        self.left_camera_info_template = self._build_left_camera_info_template()

        self.create_timer(
            self.poll_period_s,
            self.poll_camera,
        )
        self.create_timer(
            self.status_period_s,
            self.publish_status,
        )
        self.get_logger().info(
            'ZED sync node started '
            f'(image={"disabled" if not self.publish_image else image_topic}, '
            f'depth={depth_topic}, imu={imu_topic}, '
            f'depth_rate={publish_rate_hz:.1f}Hz, imu_rate={imu_publish_rate_hz:.1f}Hz, '
            f'status={status_topic}, '
            f'depth_downsample_factor={self.depth_downsample_factor}, '
            f'poll_rate={1.0 / self.poll_period_s:.1f}Hz, '
            f'cv_bridge={"enabled" if self.bridge is not None else "disabled"})'
        )

    def poll_camera(self):
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        now_s = self._clock_now_s()
        self.camera_grab_count += 1
        self.camera_grab_times_s.append(now_s)
        self._trim_camera_grab_times_locked(now_s)
        self.publish_imu()

        if now_s + 1e-9 < self.next_depth_publish_s:
            return
        self.next_depth_publish_s = now_s + self.depth_period_s
        self.grab_frame()

    def grab_frame(self):
        if not self._should_publish_depth_or_image():
            return

        stamp = self.get_clock().now().to_msg()
        self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
        depth_np = np.array(self.depth_image.get_data(), copy=True)
        left_np = None
        if self.publish_image and self.image_pub is not None:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            left_np = np.array(self.left_image.get_data(), copy=True)

        self.frame_count += 1

        if depth_np.ndim == 3:
            depth_np = depth_np[:, :, 0]
        if self.depth_downsample_factor > 1:
            depth_np = depth_np[::self.depth_downsample_factor, ::self.depth_downsample_factor]

        if self.publish_image and self.image_pub is not None and left_np is not None:
            if self.image_downsample_factor > 1:
                left_np = left_np[::self.image_downsample_factor, ::self.image_downsample_factor]
            self.image_pub.publish(self._image_from_array(left_np, 'bgra8', self.image_frame_id, stamp))
            if self.camera_info_pub is not None:
                self.camera_info_pub.publish(self._camera_info_for_image(left_np.shape, stamp))
        self.depth_pub.publish(
            self._image_from_array(
                np.ascontiguousarray(depth_np.astype(np.float32, copy=False)),
                '32FC1',
                self.depth_frame_id,
                stamp,
            )
        )

        self._update_depth_status(depth_np, stamp)

    def _should_publish_depth_or_image(self) -> bool:
        if self.publish_image and self.image_pub is not None:
            if self.image_pub.get_subscription_count() > 0:
                return True
            if self.camera_info_pub is not None and self.camera_info_pub.get_subscription_count() > 0:
                return True
        if self.depth_pub.get_subscription_count() > 0:
            return True
        return self.publish_depth_without_subscribers

    def publish_imu(self):
        status = self.zed.get_sensors_data(self.sensor_data, sl.TIME_REFERENCE.CURRENT)
        if status != sl.ERROR_CODE.SUCCESS:
            self._record_imu_failure(f'get_sensors_data:{status}')
            return

        imu = self.sensor_data.get_imu_data()
        imu_timestamp_ns = self._timestamp_to_ns(getattr(imu, 'timestamp', None))
        if imu_timestamp_ns is not None and imu_timestamp_ns == self.last_imu_timestamp_ns:
            self.imu_duplicate_samples += 1
            return
        if imu_timestamp_ns is not None:
            self.last_imu_timestamp_ns = imu_timestamp_ns

        lin = imu.get_linear_acceleration()
        ang = imu.get_angular_velocity()
        try:
            lin_values = [float(lin[0]), float(lin[1]), float(lin[2])]
            ang_values_degps = [float(ang[0]), float(ang[1]), float(ang[2])]
        except (TypeError, ValueError, IndexError) as exc:
            self._record_imu_failure(f'invalid_imu_vector:{exc}')
            return
        if not all(math.isfinite(value) for value in [*lin_values, *ang_values_degps]):
            self._record_imu_failure('nonfinite_imu_vector')
            return
        ang_values_radps = [math.radians(value) for value in ang_values_degps]

        stamp = self._timestamp_to_stamp(imu_timestamp_ns)
        imu_msg = Imu()
        imu_msg.header = Header(stamp=stamp, frame_id=self.imu_frame_id)
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.linear_acceleration.x = lin_values[0]
        imu_msg.linear_acceleration.y = lin_values[1]
        imu_msg.linear_acceleration.z = lin_values[2]
        imu_msg.angular_velocity.x = ang_values_radps[0]
        imu_msg.angular_velocity.y = ang_values_radps[1]
        imu_msg.angular_velocity.z = ang_values_radps[2]
        imu_msg.angular_velocity_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-4,
        ]
        imu_msg.linear_acceleration_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3,
        ]
        self.imu_pub.publish(imu_msg)

        now_s = self._clock_now_s()
        self.last_imu_error = None
        self.imu_count += 1
        self.last_imu_publish_s = now_s
        self.last_imu_ang_radps = ang_values_radps
        self.last_imu_ang_degps = ang_values_degps
        self.last_imu_lin_mps2 = lin_values
        self.imu_publish_times_s.append(now_s)
        self._trim_imu_publish_times_locked(now_s)

    def _record_imu_failure(self, reason: str) -> None:
        now_s = self._clock_now_s()
        self.imu_publish_failures += 1
        self.last_imu_error = str(reason)
        if now_s - self.last_imu_failure_log_s >= 2.0:
            self.last_imu_failure_log_s = now_s
            self.get_logger().warn(f'ZED IMU publish skipped: {reason}')

    def _update_depth_status(self, depth_np, stamp) -> None:
        valid = depth_np[np.isfinite(depth_np)]
        valid = valid[(valid > 0.05) & (valid < 20.0)]
        if valid.size:
            depth_min = float(np.min(valid))
            depth_p10 = float(np.percentile(valid, 10.0))
            depth_mean = float(np.mean(valid))
        else:
            depth_min = float('inf')
            depth_p10 = float('inf')
            depth_mean = float('inf')

        self.latest_depth_stamp_s = float(stamp.sec) + float(stamp.nanosec) / 1e9
        self.latest_depth_shape = [int(depth_np.shape[0]), int(depth_np.shape[1])]
        self.latest_valid_depth_samples = int(valid.size)
        self.latest_depth_min_m = self._finite_or_none(depth_min)
        self.latest_depth_p10_m = self._finite_or_none(depth_p10)
        self.latest_depth_mean_m = self._finite_or_none(depth_mean)

    def publish_status(self) -> None:
        now_s = self._clock_now_s()
        self._trim_imu_publish_times_locked(now_s)
        imu_age_s = None if self.last_imu_publish_s is None else max(0.0, now_s - self.last_imu_publish_s)
        status = {
            'stamp_sec': self.latest_depth_stamp_s,
            'frame_count': self.frame_count,
            'camera_grab_count': self.camera_grab_count,
            'camera_grab_rate_hz': self._camera_grab_rate_hz_locked(),
            'imu_count': self.imu_count,
            'imu_rate_hz': self._imu_rate_hz_locked(),
            'imu_age_s': self._finite_or_none(imu_age_s),
            'imu_topic': self.imu_pub.topic_name,
            'imu_subscribers': self.imu_pub.get_subscription_count(),
            'last_imu_publish_s': self.last_imu_publish_s,
            'last_imu_timestamp_ns': self.last_imu_timestamp_ns,
            'imu_publish_failures': self.imu_publish_failures,
            'last_imu_error': self.last_imu_error,
            'imu_busy_skips': self.imu_busy_skips,
            'imu_duplicate_samples': self.imu_duplicate_samples,
            'last_imu_ang_radps': list(self.last_imu_ang_radps),
            'last_imu_ang_degps': list(self.last_imu_ang_degps),
            'last_imu_lin_mps2': list(self.last_imu_lin_mps2),
            'camera_model': self.camera_model,
            'depth_shape': list(self.latest_depth_shape),
            'valid_depth_samples': self.latest_valid_depth_samples,
            'depth_min_m': self.latest_depth_min_m,
            'depth_p10_m': self.latest_depth_p10_m,
            'depth_mean_m': self.latest_depth_mean_m,
            'publish_image': self.publish_image,
            'publish_depth_without_subscribers': self.publish_depth_without_subscribers,
            'depth_downsample_factor': self.depth_downsample_factor,
            'image_downsample_factor': self.image_downsample_factor,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _trim_imu_publish_times_locked(self, now_s: float) -> None:
        while self.imu_publish_times_s and now_s - self.imu_publish_times_s[0] > self.imu_rate_window_s:
            self.imu_publish_times_s.popleft()

    def _trim_camera_grab_times_locked(self, now_s: float) -> None:
        while self.camera_grab_times_s and now_s - self.camera_grab_times_s[0] > self.imu_rate_window_s:
            self.camera_grab_times_s.popleft()

    def _imu_rate_hz_locked(self) -> float:
        if len(self.imu_publish_times_s) < 2:
            return 0.0
        elapsed_s = self.imu_publish_times_s[-1] - self.imu_publish_times_s[0]
        if elapsed_s <= 0.0:
            return 0.0
        return float(len(self.imu_publish_times_s) - 1) / elapsed_s

    def _camera_grab_rate_hz_locked(self) -> float:
        if len(self.camera_grab_times_s) < 2:
            return 0.0
        elapsed_s = self.camera_grab_times_s[-1] - self.camera_grab_times_s[0]
        if elapsed_s <= 0.0:
            return 0.0
        return float(len(self.camera_grab_times_s) - 1) / elapsed_s

    def _clock_now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _timestamp_to_stamp(self, timestamp_ns):
        if timestamp_ns is None:
            return self.get_clock().now().to_msg()
        sec = int(timestamp_ns // 1_000_000_000)
        nanosec = int(timestamp_ns % 1_000_000_000)
        return Time(seconds=sec, nanoseconds=nanosec).to_msg()

    @staticmethod
    def _timestamp_to_ns(timestamp) -> int | None:
        if timestamp is None:
            return None
        for name, scale in (
            ('get_nanoseconds', 1),
            ('get_microseconds', 1_000),
            ('get_milliseconds', 1_000_000),
        ):
            method = getattr(timestamp, name, None)
            if callable(method):
                try:
                    value = int(method())
                except (TypeError, ValueError):
                    continue
                if value > 0:
                    return value * scale
        method = getattr(timestamp, 'get_seconds', None)
        if callable(method):
            try:
                value_s = float(method())
            except (TypeError, ValueError):
                return None
            if math.isfinite(value_s) and value_s > 0.0:
                return int(value_s * 1_000_000_000)
        return None

    def _camera_model_text(self) -> str:
        try:
            info = self.zed.get_camera_information()
            model = getattr(info, 'camera_model', None)
            if model is not None:
                return str(model)
        except Exception as exc:  # pragma: no cover - depends on ZED SDK runtime
            self.get_logger().warn(f'Could not read ZED camera model: {exc}')
        return 'unknown'

    @staticmethod
    def _finite_or_none(value: float):
        if value is None:
            return None
        return float(value) if math.isfinite(float(value)) else None

    def _build_left_camera_info_template(self) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self.image_frame_id
        try:
            camera_info = self.zed.get_camera_information()
            config = camera_info.camera_configuration
            resolution = config.resolution
            calib = config.calibration_parameters.left_cam
            width = int(getattr(resolution, 'width', 0) or 0)
            height = int(getattr(resolution, 'height', 0) or 0)
            fx = float(calib.fx)
            fy = float(calib.fy)
            cx = float(calib.cx)
            cy = float(calib.cy)
            distortion = [float(v) for v in getattr(calib, 'disto', [])]
        except Exception as exc:  # pragma: no cover - depends on ZED SDK runtime
            self.get_logger().warn(f'Could not read ZED left camera calibration; CameraInfo will be approximate: {exc}')
            width = 1280
            height = 720
            fx = fy = 700.0
            cx = width * 0.5
            cy = height * 0.5
            distortion = [0.0, 0.0, 0.0, 0.0, 0.0]

        scale = float(max(1, self.image_downsample_factor))
        msg.width = max(1, int(round(width / scale)))
        msg.height = max(1, int(round(height / scale)))
        msg.distortion_model = 'plumb_bob'
        msg.d = distortion[:5] if distortion else [0.0, 0.0, 0.0, 0.0, 0.0]
        fx_s = fx / scale
        fy_s = fy / scale
        cx_s = cx / scale
        cy_s = cy / scale
        msg.k = [
            fx_s, 0.0, cx_s,
            0.0, fy_s, cy_s,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            fx_s, 0.0, cx_s, 0.0,
            0.0, fy_s, cy_s, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _camera_info_for_image(self, image_shape, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header = Header(stamp=stamp, frame_id=self.image_frame_id)
        msg.height = int(image_shape[0])
        msg.width = int(image_shape[1])
        msg.distortion_model = self.left_camera_info_template.distortion_model
        msg.d = list(self.left_camera_info_template.d)
        msg.k = list(self.left_camera_info_template.k)
        msg.r = list(self.left_camera_info_template.r)
        msg.p = list(self.left_camera_info_template.p)
        return msg

    def _image_from_array(self, array, encoding: str, frame_id: str, stamp) -> Image:
        if self.bridge is not None:
            msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(array), encoding=encoding)
            msg.header = Header(stamp=stamp, frame_id=frame_id)
            return msg

        msg = Image()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0

        if array.ndim == 2:
            msg.step = int(array.shape[1] * array.dtype.itemsize)
        else:
            msg.step = int(array.shape[1] * array.shape[2] * array.dtype.itemsize)

        msg.data = np.ascontiguousarray(array).tobytes()
        return msg

    def destroy_node(self):
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedSyncNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
