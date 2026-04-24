#!/usr/bin/env python3
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header


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

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.image_frame_id = self.get_parameter('image_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.image_pub = self.create_publisher(Image, image_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
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
            f'(image={image_topic}, depth={depth_topic}, imu={imu_topic})'
        )

    def grab_frame(self):
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        stamp = self.get_clock().now().to_msg()

        self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)

        left_np = np.array(self.left_image.get_data(), copy=True)
        depth_np = np.array(self.depth_image.get_data(), copy=True)
        if depth_np.ndim == 3:
            depth_np = depth_np[:, :, 0]

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

    @staticmethod
    def _image_from_array(array, encoding: str, frame_id: str, stamp) -> Image:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
