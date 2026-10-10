import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sys import stdout


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('fetch_current_pose')

        self.received = False

        # /current_pose
        current_pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # /amcl_pose
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # IMPORTANT:
        # No leading "/" -> follows node namespace
        self.sub_current = self.create_subscription(
            PoseStamped,
            'current_pose',
            self.listener_callback,
            qos_profile=current_pose_qos
        )

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            qos_profile=amcl_qos
        )

    def listener_callback(self, msg: PoseStamped):

        p = msg.pose

        print(
            f"[CURRENT_POSE] "
            f"x={p.position.x:.3f}, "
            f"y={p.position.y:.3f}, "
            f"qz={p.orientation.z:.6f}, "
            f"qw={p.orientation.w:.6f}",
            file=stdout
        )

        stdout.flush()
        self.received = True

    def amcl_callback(self, msg: PoseWithCovarianceStamped):

        p = msg.pose.pose

        print(
            f"[AMCL_POSE] "
            f"x={p.position.x:.3f}, "
            f"y={p.position.y:.3f}, "
            f"qz={p.orientation.z:.6f}, "
            f"qw={p.orientation.w:.6f}",
            file=stdout
        )

        stdout.flush()
        self.received = True


def main(args=None):

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        while rclpy.ok() and not minimal_subscriber.received:
            rclpy.spin_once(
                minimal_subscriber,
                timeout_sec=0.5
            )

    except KeyboardInterrupt:
        pass

    finally:
        minimal_subscriber.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()