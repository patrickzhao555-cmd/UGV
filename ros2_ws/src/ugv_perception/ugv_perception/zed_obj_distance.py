import math
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

class ZedObjDistance(Node):
    def __init__(self):
        super().__init__('zed_obj_distance')
        self.sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.cb,
            10
        )

    def cb(self, msg):
        if not msg.objects:
            return

        # Print the closest object (by 3D magnitude)
        best = None
        best_d = 1e9
        for obj in msg.objects:
            x, y, z = obj.position
            d = math.sqrt(x*x + y*y + z*z)
            if d < best_d:
                best_d = d
                best = obj

        if best is None:
            return

        x, y, z = best.position
        self.get_logger().info(
            f"{best.label}/{best.sublabel} conf={best.confidence:.1f}% "
            f"d={best_d:.2f}m pos=({x:.2f},{y:.2f},{z:.2f}) frame={msg.header.frame_id}"
        )

def main():
    rclpy.init()
    node = ZedObjDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

