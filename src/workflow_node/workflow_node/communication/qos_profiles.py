from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


def best_effort_qos(depth: int = 10) -> QoSProfile:
    """BEST_EFFORT QoS — used for pose and sensor topics."""
    qos = QoSProfile(depth=depth)
    qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
    return qos


def reliable_qos(depth: int = 10) -> QoSProfile:
    """RELIABLE QoS with KEEP_LAST history — used for command topics."""
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=depth)
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    return qos
