#!/usr/bin/env python3
import json
import math

import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
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
        self.declare_parameter('depth_topic', '/zed/depth')
        self.declare_parameter('imu_topic', '/zed/imu')
        self.declare_parameter('image_frame_id', 'zed_left')
        self.declare_parameter('depth_frame_id', 'zed_depth')
        self.declare_parameter('imu_frame_id', 'zed_imu')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('publish_image', False)
        self.declare_parameter('depth_downsample_factor', 2)
        self.declare_parameter('status_topic', '/zed/status')
        self.declare_parameter('status_period_s', 1.0)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.image_frame_id = self.get_parameter('image_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_image = bool(self.get_parameter('publish_image').value)
        self.depth_downsample_factor = max(1, int(self.get_parameter('depth_downsample_factor').value))
        status_topic = self.get_parameter('status_topic').value
        self.status_period_s = max(0.2, float(self.get_parameter('status_period_s').value))
        self.bridge = CvBridge() if CvBridge is not None else None

        self.image_pub = None
        if self.publish_image:
            self.image_pub = self.create_publisher(Image, image_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.frame_count = 0
        self.last_status_s = 0.0

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'ZED failed to open: {status}')
            raise RuntimeError(str(status))

        self.runtime = sl.RuntimeParameters()
        self.left_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.sensor_data = sl.SensorsData()

        self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.grab_frame)
        self.get_logger().info(
            'ZED sync node started '
            f'(image={"disabled" if not self.publish_image else image_topic}, '
            f'depth={depth_topic}, imu={imu_topic}, '
            f'status={status_topic}, '
            f'depth_downsample_factor={self.depth_downsample_factor}, '
            f'cv_bridge={"enabled" if self.bridge is not None else "disabled"})'
        )

    def grab_frame(self):
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        stamp = self.get_clock().now().to_msg()
        self.frame_count += 1

        self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)

        depth_np = np.array(self.depth_image.get_data(), copy=True)
        if depth_np.ndim == 3:
            depth_np = depth_np[:, :, 0]
        if self.depth_downsample_factor > 1:
            depth_np = depth_np[::self.depth_downsample_factor, ::self.depth_downsample_factor]

        if self.publish_image and self.image_pub is not None:
            self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
            left_np = np.array(self.left_image.get_data(), copy=True)
            if self.depth_downsample_factor > 1:
                left_np = left_np[::self.depth_downsample_factor, ::self.depth_downsample_factor]
            self.image_pub.publish(self._image_from_array(left_np, 'bgra8', self.image_frame_id, stamp))
        self.depth_pub.publish(
            self._image_from_array(
                np.ascontiguousarray(depth_np.astype(np.float32, copy=False)),
                '32FC1',
                self.depth_frame_id,
                stamp,
            )
        )

        self.zed.get_sensors_data(self.sensor_data, sl.TIME_REFERENCE.IMAGE)
        imu = self.sensor_data.get_imu_data()
        lin = imu.get_linear_acceleration()
        ang = imu.get_angular_velocity()

        imu_msg = Imu()
        imu_msg.header = Header(stamp=stamp, frame_id=self.imu_frame_id)
        imu_msg.linear_acceleration.x = lin[0]
        imu_msg.linear_acceleration.y = lin[1]
        imu_msg.linear_acceleration.z = lin[2]
        imu_msg.angular_velocity.x = ang[0]
        imu_msg.angular_velocity.y = ang[1]
        imu_msg.angular_velocity.z = ang[2]
        self.imu_pub.publish(imu_msg)
        self._publish_status(depth_np, stamp)

    def _publish_status(self, depth_np, stamp) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self.last_status_s < self.status_period_s:
            return
        self.last_status_s = now_s

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

        status = {
            'stamp_sec': float(stamp.sec) + float(stamp.nanosec) / 1e9,
            'frame_count': self.frame_count,
            'depth_shape': [int(depth_np.shape[0]), int(depth_np.shape[1])],
            'valid_depth_samples': int(valid.size),
            'depth_min_m': self._finite_or_none(depth_min),
            'depth_p10_m': self._finite_or_none(depth_p10),
            'depth_mean_m': self._finite_or_none(depth_mean),
            'publish_image': self.publish_image,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    @staticmethod
    def _finite_or_none(value: float):
        return value if math.isfinite(value) else None

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
    node = ZedSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
