from sys import argv

from collections import deque
import threading
from paho.mqtt import client as mqtt_client

class mqttClient(mqtt_client.Client):

    def __init__(self, broker, port=1883, use_async_connect = False) -> None:
        super().__init__(mqtt_client.CallbackAPIVersion.VERSION2, str(id(self)), clean_session=True, protocol=mqtt_client.MQTTv311)
        self.broker = broker
        if use_async_connect: self.connect_async(broker, port)
        else: self.connect(broker, port)
        self.__subscriber_topics_callback = dict()
        self.__queued_subs, self.__queued_pubs = deque(), deque()


    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected to MQTT Broker!")
            que_subs,  self.__queued_subs = self.__queued_subs, None
            que_pubs,  self.__queued_pubs = self.__queued_pubs, None
            while len(que_subs):
                self.subscribe2topic(*que_subs.pop())
            while len(que_pubs):
                self.publish2topic(*que_pubs.pop())
        else:
            print(f"Failed to connect, return code {reason_code}\n")

    def on_message(self, client, userdata, _msg):
        msg = _msg.payload.decode()
        div = msg.find('/')
        if div != -1:
            threading.Thread(target=lambda: self.__subscriber_topics_callback[_msg.topic](msg[:div], msg[div+1:])).start()

    def subscribe2topic(self, topic, message_callback, qos=0):
        if self.__queued_subs is None:
            if topic not in self.__subscriber_topics_callback:
                self.subscribe(topic, qos)
            self.__subscriber_topics_callback[topic] = message_callback
        else: self.__queued_subs.append((topic, message_callback, qos))

    def publish2topic(self, topic,  message, qos=0, ignore_result = False):
        if self.__queued_pubs is not None:
            self.__queued_pubs.append((topic, message, qos, ignore_result))
            return
        payload = (self._client_host_ip if hasattr(self, '_client_host_ip') else self.broker) + '/' + message
        while True:
            result = self.publish(topic, payload, qos)
            if ignore_result: break
            status = result[0]
            if status == 0:
                print(f"Send `{payload}` to topic `{topic}`")
                break
            else:
                print(f"Failed to send `{payload}` to topic `{topic}`")




