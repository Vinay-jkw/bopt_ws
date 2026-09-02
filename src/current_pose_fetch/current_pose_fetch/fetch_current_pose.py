import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sys import stdout


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('fetch_current_pose')
        self.received = False

        # QoS for /current_pose (BEST_EFFORT & RELIABLE compatible)
        current_pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # QoS for /amcl_pose (TRANSIENT_LOCAL & RELIABLE)
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_current = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.listener_callback,
            qos_profile=current_pose_qos
        )

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            qos_profile=amcl_qos
        )

    def listener_callback(self, msg: PoseStamped):
        current_pose = msg.pose
        st = f"current_pose=({current_pose.position.x}, {current_pose.position.y}, {current_pose.orientation.z}, {current_pose.orientation.w})"
        print(st, file=stdout)
        stdout.flush()
        self.received = True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        current_pose = msg.pose.pose
        st = f"current_pose=({current_pose.position.x}, {current_pose.position.y}, {current_pose.orientation.z}, {current_pose.orientation.w})"
        print(st, file=stdout)
        stdout.flush()
        self.received = True


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        while rclpy.ok() and not minimal_subscriber.received:
            rclpy.spin_once(minimal_subscriber, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()