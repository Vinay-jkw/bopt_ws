import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

_MAX_SPIN_CYCLES = 100


class CurrentOdotStateNode(Node):
    """One-shot subscriber that reads a single /byd/can_odot_data message."""

    def __init__(self):
        super().__init__('current_odot_state_node')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            String, '/byd/can_odot_data', self._odot_callback, qos
        )
        self.sensor_data = None
        self.back_sensor_byte = None
        self.front_sensor_byte = None

    def _odot_callback(self, msg: String) -> None:
        try:
            parts = msg.data.split(' ')
            if len(parts) < 2:
                self.get_logger().warning(
                    f"Unexpected odot message format: '{msg.data}'"
                )
                return
            sensor_data = parts[1]
            self.back_sensor_byte = sensor_data[0]
            self.front_sensor_byte = sensor_data[-1]
            self.sensor_data = parts
        except (IndexError, AttributeError) as error:
            self.get_logger().warning(f"Failed to parse odot message: {error}")
            traceback.print_exc()


def get_current_odot_state() -> tuple:
    """Spin a transient node until one /byd/can_odot_data message arrives.

    Returns (front_sensor_byte, back_sensor_byte).

    Raises:
        RuntimeError: if no message is received within the timeout window.
    """
    node = CurrentOdotStateNode()
    node.get_logger().info('Waiting for /byd/can_odot_data...')
    try:
        spins = 0
        while rclpy.ok() and spins < _MAX_SPIN_CYCLES:
            rclpy.spin_once(node, timeout_sec=0.1)
            spins += 1
            if node.sensor_data is not None:
                break

        if node.sensor_data is None:
            raise RuntimeError(
                f"Timed out waiting for /byd/can_odot_data — no message received "
                f"after {spins} spin cycles ({spins * 0.1:.1f}s). "
                "Check that the CAN odot node is running."
            )

        return node.front_sensor_byte, node.back_sensor_byte

    except RuntimeError:
        raise
    except Exception as error:
        node.get_logger().error(f"Unexpected error in get_current_odot_state: {error}")
        traceback.print_exc()
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


# if __name__ == '__main__':
#     import rclpy
#     rclpy.init()
#     data = get_current_odot_state()
#     print(data)
#     rclpy.shutdown()
    