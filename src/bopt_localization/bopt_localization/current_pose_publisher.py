import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CurrentPosePublisher(Node):

    def __init__(self):
        super().__init__('current_pose_publisher')

        # Declare parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_topic', 'current_pose')
        self.declare_parameter('publish_rate', 20.0)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher (relative topic name so it respects namespaces in multi-robot setups)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            10
        )

        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.05
        self.timer = self.create_timer(timer_period, self.publish_current_pose)

        self.get_logger().info(
            f"CurrentPosePublisher active: TF({self.map_frame} -> {self.base_frame}) -> topic '{self.pose_topic}' @ {publish_rate}Hz"
        )

    def publish_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z

        msg.pose.orientation.x = transform.transform.rotation.x
        msg.pose.orientation.y = transform.transform.rotation.y
        msg.pose.orientation.z = transform.transform.rotation.z
        msg.pose.orientation.w = transform.transform.rotation.w

        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
