import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class SandBoxPositionPublisher(Node):
    def __init__(self):
        super().__init__('sandbox_position_publisher')
        self.publisher_ = self.create_publisher(Point, 'sandbox_positions', 10)
        self.timer = self.create_timer(8, self.publish_position)

        # List of cube positions
        self.cube_positions = [
            [-0.945, 0.470, 0.150],
            [-0.945, 0.000, 0.150],
            [-0.945, -0.470, 0.150],
            [-0.700, -0.470, 0.150],
            [-0.698, -0.025, 0.150],
            [-0.700, 0.450, 0.150],
            [-0.455, 0.470, 0.150],
            [-0.470, -0.025, 0.150]
        ]

    def publish_position(self):
        msg = Point()
        # Randomly select a cube position
        selected_position = random.choice(self.cube_positions)
        msg.x, msg.y, msg.z = selected_position

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.x}, {msg.y}, {msg.z}')

def main(args=None):
    rclpy.init(args=args)
    sandbox_position_publisher = SandBoxPositionPublisher()
    rclpy.spin(sandbox_position_publisher)
    sandbox_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
