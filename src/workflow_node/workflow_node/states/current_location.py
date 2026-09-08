import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped

_MAX_SPIN_CYCLES = 100


class CurrentLocationNode(Node):
    """One-shot subscriber node that reads a single /current_pose message."""

    def __init__(self):
        super().__init__('current_location_node')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self._current_pose_callback, qos
        )
        self.current_pose = None
        self.success = False

    def _current_pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg.pose


def get_current_pose() -> list:
    """Spin a transient node until one /current_pose message arrives.

    Returns [x, y, qz, qw].

    Raises:
        RuntimeError: if no pose is received within the timeout window.
    """
    node = CurrentLocationNode()
    node.get_logger().info('Waiting for /current_pose...')
    try:
        spins = 0
        while rclpy.ok() and spins < _MAX_SPIN_CYCLES:
            rclpy.spin_once(node, timeout_sec=0.1)
            spins += 1
            if node.current_pose is not None:
                break

        if node.current_pose is None:
            raise RuntimeError(
                f"Timed out waiting for /current_pose — no message received "
                f"after {spins} spin cycles ({spins * 0.1:.1f}s). "
                "Check that the pose source is publishing."
            )

        pose = node.current_pose
        current_formatted_pose = [
            pose.position.x,
            pose.position.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        return current_formatted_pose

    except RuntimeError:
        raise
    except Exception as error:
        node.get_logger().error(f"Unexpected error in get_current_pose: {error}")
        traceback.print_exc()
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
