import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random


class SandBoxPositionPublisher(Node):
    def __init__(self):
        super().__init__('sandbox_position_publisher')
        self.publisher_ = self.create_publisher(Point, 'sandbox_positions', 10)
        self.timer = self.create_timer(8, self.publish_position)

        # Ensure correct boundaries within the box, avoiding out-of-bounds positions
        self.sandbox_dimensions = [0.60, 1.05, 0.30]
        self.positions_abcd_cubes = [
            [0.05, 0.05, 0.1],  # Point A
            [0.55, 0.05, 0.1],  # Point B
            [0.55, 0.95, 0.1],  # Point C
            [0.05, 0.95, 0.1],  # Point D
            [0.3, 0.3, 0.1],  # Center
        ]

        self.safe_workspace = {
            'x_min': 0.1, 'x_max': 0.2,  # Set boundaries to stay inside the sandbox
            'y_min': 0.1, 'y_max': 0.2,
            'z_min': 0.1, 'z_max': 0.2
        }

    def publish_position(self):
        msg = Point()
        # Randomly select from the predefined points inside the sandbox
        selected_position = random.choice(self.positions_abcd_cubes)
        # msg.x, msg.y, msg.z = selected_position
        msg.x = random.uniform(self.safe_workspace['x_min'], self.safe_workspace['x_max'])
        msg.y = random.uniform(self.safe_workspace['y_min'], self.safe_workspace['y_max'])
        msg.z = random.uniform(self.safe_workspace['z_min'], self.safe_workspace['z_max'])

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.x}, {msg.y}, {msg.z}")


def main(args=None):
    rclpy.init(args=args)
    sandbox_position_publisher = SandBoxPositionPublisher()
    rclpy.spin(sandbox_position_publisher)
    sandbox_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
