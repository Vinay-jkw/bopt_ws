import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class BOPTController(Node):

    def __init__(self):
        super().__init__('bopt_controller')

        # Input from our high-level controller / keyboard
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Output to ros2_control traction controller
        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            '/traction_joint_controller/commands',
            10
        )

        self.max_wheel_velocity = 3.0

        self.get_logger().info('BOPT Controller started')

    def cmd_vel_callback(self, msg):

        # For now we only use linear.x
        wheel_velocity = msg.linear.x

        # Limit command
        wheel_velocity = max(
            -self.max_wheel_velocity,
            min(self.max_wheel_velocity, wheel_velocity)
        )

        command = Float64MultiArray()
        command.data = [wheel_velocity]

        self.traction_pub.publish(command)


def main(args=None):

    rclpy.init(args=args)

    node = BOPTController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()