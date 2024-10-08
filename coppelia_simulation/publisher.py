import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random

class CoordinatesPublisher(Node):

    def __init__(self):
        super().__init__('coordinates_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'movement_commands', 10)
        self.timer = self.create_timer(2, self.timer_callback)  # Publier toutes les 2 secondes

    def timer_callback(self):
        msg = Float64MultiArray()
        # Générer des coordonnées aléatoires (remplace ceci par tes prédictions de modèle)
        x = random.uniform(0.1, 0.5)
        y = random.uniform(0.1, 0.5)
        z = random.uniform(0.1, 0.3)
        msg.data = [x, y, z]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing coordinates: x={x}, y={y}, z={z}')


def main(args=None):
    rclpy.init(args=args)
    coordinates_publisher = CoordinatesPublisher()

    try:
        rclpy.spin(coordinates_publisher)
    except KeyboardInterrupt:
        pass

    coordinates_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
