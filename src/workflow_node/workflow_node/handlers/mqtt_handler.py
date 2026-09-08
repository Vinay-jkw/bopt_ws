import traceback

from workflow_node.constants import (
    MQTT_TOPIC_ERROR_DETECTED,
    MQTT_TOPIC_ERROR_STATUS,
    MQTT_TOPIC_TASK_STATUS,
)


class MqttHandler:
    """Convenience wrapper for the workflow node's common MQTT publish patterns.

    All topic names are fixed constants; only the payload varies per call.
    The underlying mqttClient handles queuing and delivery guarantees.
    """

    def __init__(self, mqtt_node) -> None:
        self.mqtt = mqtt_node

    def publish_task_status(self, status: str) -> None:
        try:
            self.mqtt.publish2topic(MQTT_TOPIC_TASK_STATUS, status)
        except Exception as error:
            # No node logger here — use the mqtt_node's logger if available,
            # otherwise fall back to rclpy module logger
            from rclpy.logging import get_logger
            get_logger('mqtt_handler').error(
                f"Failed to publish task status '{status}': {error}"
            )
            traceback.print_exc()

    def publish_error_status(self, error: str) -> None:
        try:
            self.mqtt.publish2topic(MQTT_TOPIC_ERROR_STATUS, error)
        except Exception as exc:
            from rclpy.logging import get_logger
            get_logger('mqtt_handler').error(
                f"Failed to publish error status '{error}': {exc}"
            )
            traceback.print_exc()

    def publish_error_detected(self, error: str) -> None:
        try:
            self.mqtt.publish2topic(MQTT_TOPIC_ERROR_DETECTED, error)
        except Exception as exc:
            from rclpy.logging import get_logger
            get_logger('mqtt_handler').error(
                f"Failed to publish error_detected '{error}': {exc}"
            )
            traceback.print_exc()
