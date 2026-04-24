#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

def yaw_to_quat(yaw: float):
    # z-only yaw quaternion
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class SerialOdom(Node):
    def __init__(self):
        super().__init__('ugv_serial_odom')

        # Params
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.05)      # meters
        self.declare_parameter('wheel_separation', 0.30)  # meters
        self.declare_parameter('ticks_per_rev', 1440)     # encoder CPR * gear etc
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.tpr = float(self.get_parameter('ticks_per_rev').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f"Reading ticks from {port} @ {baud}")

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_l = None
        self.prev_r = None

        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

    def loop(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if not line:
            return

        # Expect: T <left_ticks> <right_ticks> <dt_ms>
        parts = line.split()
        if len(parts) != 4 or parts[0] != 'T':
            return

        try:
            l = int(parts[1])
            r = int(parts[2])
            dt_ms = int(parts[3])
        except ValueError:
            return

        encoder_msg = Int32MultiArray()
        encoder_msg.data = [l, r]
        self.encoder_pub.publish(encoder_msg)

        if self.prev_l is None:
            self.prev_l, self.prev_r = l, r
            return

        dl = l - self.prev_l
        dr = r - self.prev_r
        self.prev_l, self.prev_r = l, r

        dt = max(dt_ms / 1000.0, 1e-3)

        # ticks -> distance
        meters_per_tick = (2.0 * math.pi * self.r) / self.tpr
        dL = dl * meters_per_tick
        dR = dr * meters_per_tick

        dS = (dR + dL) / 2.0
        dYaw = (dR - dL) / self.L

        # integrate
        self.yaw += dYaw
        self.x += dS * math.cos(self.yaw)
        self.y += dS * math.sin(self.yaw)

        vx = dS / dt
        wz = dYaw / dt

        now = self.get_clock().now().to_msg()

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # TF odom->base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = SerialOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
