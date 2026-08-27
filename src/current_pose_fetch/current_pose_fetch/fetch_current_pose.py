import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
import socket
from sys import stdout


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('fetch_current_pose')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.listener_callback,
            qos_profile=qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: PoseStamped):
        current_pose = msg.pose
        st=f"current_pose=({current_pose.position.x}, {current_pose.position.y}, {current_pose.orientation.z}, {current_pose.orientation.w})"
        print(st, file=stdout)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin_once(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
