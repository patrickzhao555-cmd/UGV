import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.target_angle_deg = 0.0  
        self.window_degrees = 180.0     # Width of the cone

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
            
        self.publisher_ = self.create_publisher(Float32, '/front_distance', 10)
        self.get_logger().info(f'Node started. Looking at {self.target_angle_deg}° with a {self.window_degrees}° window.')

    def listener_callback(self, msg):
        points_per_radian = 1.0 / msg.angle_increment
        points_per_degree = points_per_radian * (math.pi / 180.0)
        center_index = int(self.target_angle_deg * points_per_degree)
        
        half_window_points = int((self.window_degrees / 2.0) * points_per_degree)
        start_idx = center_index - half_window_points
        end_idx = center_index + half_window_points

        num_readings = len(msg.ranges)
        sample_range = []
        for i in range(start_idx, end_idx):
            idx = i % num_readings 
            sample_range.append(msg.ranges[idx])

        valid_ranges = [r for r in sample_range if msg.range_min < r < msg.range_max]

        if valid_ranges:
            min_dist = min(valid_ranges)
            
            dist_msg = Float32()
            dist_msg.data = float(min_dist)
            self.publisher_.publish(dist_msg)
            
            self.get_logger().info(f'Closest obstacle in 180° front: {min_dist:.2f}m')
        else:
            self.get_logger().warn('180° field is completely clear!')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
