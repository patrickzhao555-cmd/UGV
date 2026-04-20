#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
import pyzed.sl as sl


class ZedSyncNode(Node):
    def __init__(self):
        super().__init__('zed_sync_node')

        self.image_pub = self.create_publisher(Image, '/zed/image', 10)
        self.imu_pub   = self.create_publisher(Imu,   '/zed/imu',   10)

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'ZED failed to open: {status}')
            raise RuntimeError(str(status))

        self.runtime = sl.RuntimeParameters()
        # ~30 Hz grab loop
        self.create_timer(1.0 / 30.0, self.grab_frame)
        self.get_logger().info('ZED sync node started')

    def grab_frame(self):
        if self.zed.grab(self.runtime) != sl.ERROR_CODE.SUCCESS:
            return

        # Single Jetson-clock stamp shared by both image and IMU messages
        stamp = self.get_clock().now().to_msg()

        # --- Image ---
        zed_image = sl.Mat()
        self.zed.retrieve_image(zed_image, sl.VIEW.LEFT)
        img_msg = Image()
        img_msg.header = Header(stamp=stamp, frame_id='zed_left')
        img_msg.height   = zed_image.get_height()
        img_msg.width    = zed_image.get_width()
        img_msg.encoding = 'bgra8'
        img_msg.step     = img_msg.width * 4
        img_msg.data     = zed_image.get_data().tobytes()
        self.image_pub.publish(img_msg)

        # --- IMU aligned to this exact frame, not the latest sample ---
        sensor_data = sl.SensorsData()
        self.zed.get_sensors_data(sensor_data, sl.TIME_REFERENCE.IMAGE)
        imu = sensor_data.get_imu_data()
        lin = imu.get_linear_acceleration()
        ang = imu.get_angular_velocity()

        imu_msg = Imu()
        imu_msg.header = Header(stamp=stamp, frame_id='zed_imu')
        imu_msg.linear_acceleration.x = lin[0]
        imu_msg.linear_acceleration.y = lin[1]
        imu_msg.linear_acceleration.z = lin[2]
        imu_msg.angular_velocity.x = ang[0]
        imu_msg.angular_velocity.y = ang[1]
        imu_msg.angular_velocity.z = ang[2]
        self.imu_pub.publish(imu_msg)

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