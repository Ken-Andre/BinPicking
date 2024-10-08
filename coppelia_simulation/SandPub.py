import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random
# import time

class SandBoxPositionPublisher(Node):
    def __init__(self):
        super().__init__('sandbox_position_publisher')
        self.publisher_ = self.create_publisher(Point, 'sandbox_positions', 10)
        self.timer = self.create_timer(8, self.publish_position)
        self.sandbox_dimensions = [0.60, 1.05, 0.35]  # Dimensions de la SandBox en mètres

    def publish_position(self):
        msg = Point()
        msg.x = random.uniform(0, self.sandbox_dimensions[0])
        msg.y = random.uniform(0, self.sandbox_dimensions[1])
        msg.z = random.uniform(0, self.sandbox_dimensions[2])
        if msg.z>0:
            msg.z = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}"')

def main(args=None):
    rclpy.init(args=args)
    sandbox_position_publisher = SandBoxPositionPublisher()
    rclpy.spin(sandbox_position_publisher)
    sandbox_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()