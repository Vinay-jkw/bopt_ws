import rclpy
from rclpy.node import Node

class RandomObstacleNode(Node):

    def __init__(self):
        super().__init__('random_obstacle_node')

        self.get_logger().info(
            "Random obstacle node started"
        )

def main(args=None):

    rclpy.init(args=args)

    node = RandomObstacleNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()