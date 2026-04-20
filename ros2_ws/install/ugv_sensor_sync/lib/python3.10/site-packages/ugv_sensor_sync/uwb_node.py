#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import serial
import numpy as np
from collections import deque


class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        self.declare_parameter('port',                '/dev/ttyUSB1')
        self.declare_parameter('baud',                115200)
        self.declare_parameter('use_esp32_timestamp', False)

        self.port         = self.get_parameter('port').value
        self.baud         = int(self.get_parameter('baud').value)
        self.use_esp32_ts = self.get_parameter('use_esp32_timestamp').value

        self.pub = self.create_publisher(PointStamped, '/uwb/range', 10)

        self.pairs   = deque(maxlen=200)
        self.drift_a = 1.0
        self.drift_b = 0.0
        self.ser     = None  # opened lazily — ESP32 may not be plugged in yet

        # Try once at startup, but don't crash if it fails
        self._try_open_serial()

        # Main read loop — also handles reconnect if cable is plugged in later
        self.create_timer(0.02, self.read_uwb)   # 50 Hz poll
        self.get_logger().info(
            f'UWB node started (port={self.port}, '
            f'esp32_ts={self.use_esp32_ts}). '
            f'{"Connected." if self.ser else "Waiting for device..."}'
        )

    def _try_open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f'UWB serial opened: {self.port} @ {self.baud}')
        except serial.SerialException:
            self.ser = None   # silently wait — logged in read_uwb on first miss

    def read_uwb(self):
        # If not connected, try to reconnect every call (throttled by timer rate)
        if self.ser is None:
            self._try_open_serial()
            if self.ser is None:
                return   # still not plugged in, skip silently

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'UWB serial lost: {e} — will retry')
            self.ser = None
            return

        if not line:
            return

        t_rx_ns = self.get_clock().now().nanoseconds

        parts = line.split(',')
        try:
            if self.use_esp32_ts and len(parts) >= 3:
                t_esp32_us = int(parts[0])
                anchor_id  = int(parts[1])
                range_m    = float(parts[2])
                t_esp32_ns = t_esp32_us * 1000
                self.pairs.append((t_esp32_ns, t_rx_ns))
                self._fit_drift_model()
                stamp_ns = int(self.drift_a * t_esp32_ns + self.drift_b)
            else:
                anchor_id = int(parts[0])
                range_m   = float(parts[1])
                stamp_ns  = t_rx_ns
        except (ValueError, IndexError):
            self.get_logger().warn(f'Malformed UWB line: {line!r}')
            return

        msg = PointStamped()
        msg.header.stamp    = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        msg.header.frame_id = f'uwb_anchor_{anchor_id}'
        msg.point.x         = float(range_m)
        msg.point.y         = float(anchor_id)
        self.pub.publish(msg)

    def _fit_drift_model(self):
        if len(self.pairs) < 10:
            return
        xs = np.array([p[0] for p in self.pairs], dtype=np.float64)
        ys = np.array([p[1] for p in self.pairs], dtype=np.float64)
        xm, ym = xs.mean(), ys.mean()
        denom = np.dot(xs - xm, xs - xm)
        if denom == 0:
            return
        self.drift_a = np.dot(xs - xm, ys - ym) / denom
        self.drift_b = ym - self.drift_a * xm

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()