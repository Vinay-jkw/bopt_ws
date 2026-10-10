
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time


class FetchPose(Node):
    def __init__(self):
        super().__init__('fetch_current_pose')

        self.current_count = 0
        self.amcl_count = 0

        current_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Match the C++ orientation subscriber's QoS
        amcl_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PoseStamped, 'current_pose',
            self.current_callback, current_qos
        )

        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.amcl_callback, amcl_qos
        )

    def current_callback(self, msg):
        self.current_count += 1
        p = msg.pose
        if self.current_count == 1:
            print(
                f'[CURRENT_POSE] x={p.position.x:.3f}, '
                f'y={p.position.y:.3f}, '
                f'qz={p.orientation.z:.6f}, '
                f'qw={p.orientation.w:.6f}',
                flush=True
            )

    def amcl_callback(self, msg):
        self.amcl_count += 1
        p = msg.pose.pose
        if self.amcl_count == 1:
            print(
                f'[AMCL_POSE] x={p.position.x:.3f}, '
                f'y={p.position.y:.3f}, '
                f'qz={p.orientation.z:.6f}, '
                f'qw={p.orientation.w:.6f}',
                flush=True
            )


def main(args=None):
    rclpy.init(args=args)
    node = FetchPose()

    try:
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        print(
            f'\nRESULT: current_pose messages={node.current_count}, '
            f'amcl_pose messages={node.amcl_count}',
            flush=True
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
