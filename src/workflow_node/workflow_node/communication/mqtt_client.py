import threading
import traceback
from collections import deque

from paho.mqtt import client as mqtt_client


class mqttClient(mqtt_client.Client):
    """Paho MQTT client with queued publish/subscribe until connection is ready.

    Failures are logged explicitly rather than silently dropped so that
    operator monitoring tools can detect communication problems.
    """

    def __init__(self, broker: str, port: int = 1883,
                 use_async_connect: bool = False) -> None:
        super().__init__(
            mqtt_client.CallbackAPIVersion.VERSION2,
            str(id(self)),
            clean_session=True,
            protocol=mqtt_client.MQTTv311,
        )
        self.broker = broker
        self.__subscriber_topics_callback: dict = {}
        self.__queued_subs: deque = deque()
        self.__queued_pubs: deque = deque()

        print(f"[mqtt_client] Connecting to broker: {broker}:{port}")
        try:
            if use_async_connect:
                self.connect_async(broker, port)
            else:
                self.connect(broker, port)
        except Exception as error:
            print(
                f"[mqtt_client] ERROR: Failed to connect to broker "
                f"'{broker}:{port}': {error}"
            )
            traceback.print_exc()
            raise

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print(f"[mqtt_client] Connected to MQTT broker '{self.broker}'.")
            try:
                que_subs, self.__queued_subs = self.__queued_subs, None
                que_pubs, self.__queued_pubs = self.__queued_pubs, None
                while len(que_subs):
                    self.subscribe2topic(*que_subs.pop())
                while len(que_pubs):
                    self.publish2topic(*que_pubs.pop())
            except Exception as error:
                print(
                    f"[mqtt_client] ERROR: Failed to flush queued ops "
                    f"after connect: {error}"
                )
                traceback.print_exc()
        else:
            print(
                f"[mqtt_client] ERROR: Connection to broker '{self.broker}' "
                f"failed with reason code {reason_code}."
            )

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        if reason_code != 0:
            print(
                f"[mqtt_client] WARNING: Unexpected disconnect from broker "
                f"'{self.broker}' (reason_code={reason_code}). "
                "Paho will attempt reconnection."
            )

    def on_message(self, client, userdata, _msg):
        try:
            msg = _msg.payload.decode('utf-8')
        except (UnicodeDecodeError, AttributeError) as error:
            print(
                f"[mqtt_client] WARNING: Could not decode payload on topic "
                f"'{_msg.topic}': {error}"
            )
            return

        div = msg.find('/')
        if div == -1:
            print(
                f"[mqtt_client] WARNING: Malformed payload on topic "
                f"'{_msg.topic}' — missing '/' separator: '{msg}'"
            )
            return

        topic = _msg.topic
        if topic not in self.__subscriber_topics_callback:
            print(
                f"[mqtt_client] WARNING: Received message on unregistered "
                f"topic '{topic}' — ignoring."
            )
            return

        def _dispatch():
            try:
                self.__subscriber_topics_callback[topic](msg[:div], msg[div + 1:])
            except Exception as error:
                print(
                    f"[mqtt_client] ERROR: Callback for topic '{topic}' "
                    f"raised an exception: {error}"
                )
                traceback.print_exc()

        threading.Thread(target=_dispatch, daemon=True).start()

    def subscribe2topic(self, topic: str, message_callback, qos: int = 0) -> None:
        try:
            if self.__queued_subs is None:
                if topic not in self.__subscriber_topics_callback:
                    result, mid = self.subscribe(topic, qos)
                    if result != mqtt_client.MQTT_ERR_SUCCESS:
                        print(
                            f"[mqtt_client] WARNING: subscribe() returned "
                            f"error code {result} for topic '{topic}'."
                        )
                self.__subscriber_topics_callback[topic] = message_callback
            else:
                self.__queued_subs.append((topic, message_callback, qos))
        except Exception as error:
            print(f"[mqtt_client] ERROR: subscribe2topic('{topic}') failed: {error}")
            traceback.print_exc()

    def publish2topic(self, topic: str, message: str, qos: int = 0,
                      ignore_result: bool = False) -> None:
        if self.__queued_pubs is not None:
            self.__queued_pubs.append((topic, message, qos, ignore_result))
            return

        try:
            payload = (
                self._client_host_ip
                if hasattr(self, '_client_host_ip')
                else self.broker
            ) + '/' + message
        except Exception as error:
            print(
                f"[mqtt_client] ERROR: Failed to build payload for topic "
                f"'{topic}': {error}"
            )
            return

        try:
            result = self.publish(topic, payload, qos)
            if ignore_result:
                return
            status = result[0]
            if status == 0:
                print(f"[mqtt_client] Sent `{payload}` to topic `{topic}`")
            else:
                print(
                    f"[mqtt_client] WARNING: publish() failed for topic "
                    f"'{topic}' (status={status}). Payload: `{payload}`"
                )
        except Exception as error:
            print(
                f"[mqtt_client] ERROR: Exception during publish to topic "
                f"'{topic}': {error}"
            )
            traceback.print_exc()
