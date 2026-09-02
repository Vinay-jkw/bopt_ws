import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class VelocitySteeringBridge(Node):
    """
    Bridges NMPC / Float64 velocity and steering_angle topics to /cmd_vel (Twist).
    """

    def __init__(self):
        super().__init__('velocity_steering_bridge')

        # Parameters
        self.declare_parameter('wheelbase', 1.542)
        self.declare_parameter('rate', 20.0)  # Hz

        self.wheelbase = self.get_parameter('wheelbase').value
        rate = self.get_parameter('rate').value

        # State
        self.velocity = 0.0
        self.steering_angle_deg = 0.0
        self.last_msg_time = self.get_clock().now()

        # Subscribers
        self.vel_sub = self.create_subscription(
            Float64,
            '/velocity',
            self.velocity_callback,
            10
        )
        self.steer_sub = self.create_subscription(
            Float64,
            '/steering_angle',
            self.steering_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer to periodically publish cmd_vel
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd_vel)
        self.get_logger().info('Velocity-Steering Bridge initialized (publishing to /cmd_vel).')

    def velocity_callback(self, msg: Float64):
        self.velocity = msg.data
        self.last_msg_time = self.get_clock().now()

    def steering_callback(self, msg: Float64):
        self.steering_angle_deg = msg.data
        self.last_msg_time = self.get_clock().now()

    def publish_cmd_vel(self):
        twist = Twist()
        delta_rad = math.radians(self.steering_angle_deg)

        twist.linear.x = float(self.velocity)
        if abs(self.velocity) > 1e-4:
            # Kinematic relation: steering = atan2(-wheelbase * yaw_rate, v)
            # => yaw_rate = - (v / wheelbase) * tan(delta_rad)
            twist.angular.z = float(- (self.velocity / self.wheelbase) * math.tan(delta_rad))
        else:
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySteeringBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
