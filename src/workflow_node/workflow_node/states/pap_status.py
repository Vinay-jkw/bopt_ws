import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

_MAX_SPIN_CYCLES = 100


class PapNode(Node):
    """One-shot subscriber node for the Pallet-Already-Present field sensor."""

    def __init__(self):
        super().__init__('pallet_already_present_node')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(
            Bool, '/pap_field_status', self._pap_callback, qos
        )
        self.pap_status = None

    def _pap_callback(self, msg: Bool) -> None:
        self.pap_status = msg.data


def get_pap_status() -> bool:
    """Spin a transient node until one /pap_field_status message arrives.

    Returns the pallet-already-present boolean.

    Raises:
        RuntimeError: if no message is received within the timeout window.
    """
    node = PapNode()
    node.get_logger().info('Waiting for /pap_field_status...')
    try:
        spins = 0
        while rclpy.ok() and spins < _MAX_SPIN_CYCLES:
            rclpy.spin_once(node, timeout_sec=0.1)
            spins += 1
            if node.pap_status is not None:
                break

        if node.pap_status is None:
            raise RuntimeError(
                f"Timed out waiting for /pap_field_status — no message received "
                f"after {spins} spin cycles ({spins * 0.1:.1f}s). "
                "Check that the PAP sensor node is running."
            )

        return node.pap_status

    except RuntimeError:
        raise
    except Exception as error:
        node.get_logger().error(f"Unexpected error in get_pap_status: {error}")
        traceback.print_exc()
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
