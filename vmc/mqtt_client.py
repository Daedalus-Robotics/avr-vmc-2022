import json
import time
from threading import Lock, Thread
from typing import Any, Callable

from loguru import logger
from paho.mqtt import client as mqtt

DEFAULT_QOS = 1


class MQTTClient:
    _instance = None

    @classmethod
    def get(cls, host: str = "localhost", port: int = 1883, retry: bool = True, status: Any = None) -> 'MQTTClient':
        if cls._instance is None:
            cls._instance = MQTTClient(host, port, retry, status)
        return cls._instance

    def __init__(self, host: str = "localhost", port: int = 1883, retry: bool = True, status: Any = None) -> None:
        self._instance = self

        self.host = host
        self.port = port
        self.retry = retry
        self.status = status

        self.client = mqtt.Client(protocol = mqtt.MQTTv311)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.topic_map: dict[str, tuple[Callable, bool, int, Any, Any, bool]] = {}
        self.topic_map_lock = Lock()
        self._connected = False

        self._last_flash = 0

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected()

    def connect(self) -> bool:
        if not self.is_connected:
            try:
                self.client.connect(host = self.host, port = self.port, keepalive = 60)
                self.client.loop_start()
            except ConnectionRefusedError:
                return False
        return True

    def disconnect(self) -> None:
        if self.is_connected:
            self.client.disconnect(mqtt.DISCONNECT)

    def _on_connect(self, _: mqtt.Client, __: Any, ___: dict, rc: int):
        self._connected = True
        logger.debug(f"Connected with result {rc}")
        if self.status is not None:
            self.status.update_status("mqtt", True)

        with self.topic_map_lock:
            for topic, info in self.topic_map.items():
                _, _, qos, options, properties, _ = info
                self.client.subscribe(topic, qos = qos, options = options, properties = properties)
                logger.success(f"Subscribed to: {topic}")

    def _on_disconnect(self, _: mqtt.Client, __: Any, rc: int, ___: dict = None):
        logger.debug(f"Disconnected with result {rc}")
        if self.status is not None:
            self.status.update_status("mqtt", False)
        if self.retry and rc is not mqtt.DISCONNECT:
            Thread(target = self._reconnect_loop).start()

    def _reconnect_loop(self) -> None:
        while not self.is_connected:
            self.connect()
            time.sleep(2)

    # def _on_message(self, _: mqtt.Client, __: Any, msg: mqtt.MQTTMessage):
    #     if msg.topic in self.topic_map:
    #         try:
    #             topic_tuple = self.topic_map[msg.topic]
    #             callback, is_json, _, _, _, use_args = topic_tuple
    #             payload: dict | bytes = msg.payload
    #             if is_json:
    #                 payload = json.loads(msg.payload)
    #             if use_args:
    #                 callback(payload)
    #             else:
    #                 callback()
    #         except (AttributeError, TypeError, json.JSONDecodeError) as e:
    #             logger.warning(f"Filed to run callback for topic {msg.topic}")
    #             logger.warning(e)

    def _on_message(self, _: mqtt.Client, __: Any, msg: mqtt.MQTTMessage):
        if msg.topic in self.topic_map:
            try:
                topic_tuple = self.topic_map[msg.topic]
                callback, is_json, _, _, _, use_args = topic_tuple
                if use_args:
                    payload: dict | bytes = msg.payload
                    if is_json:
                        payload = json.loads(msg.payload)
                    callback(payload)
                else:
                    callback()
                if self.status is not None:
                    ms = int(round(time.time() * 1000))
                    timesince = ms - self._last_flash
                    if timesince < 100:
                        return
                    self._last_flash = ms
                    self.status.led_event("mqtt", (255, 100, 0), 50)
            except (AttributeError, TypeError, json.JSONDecodeError) as e:
                logger.warning(f"Failed to run callback for topic {msg.topic}")
                logger.warning(e)

    def send_message(
            self, topic: str, payload: Any, qos: int = DEFAULT_QOS, retain: bool = False, properties: Any = None
    ) -> bool:
        serializable, json_payload = is_serializable(payload)
        if serializable:
            payload = json_payload
        try:
            if self.is_connected:
                self.client.publish(topic, payload, qos = qos, retain = retain, properties = properties)
            else:
                logger.warning("Can't publish. MQTT disconnected")
            return True
        except TypeError:
            return False

    def register_callback(
            self,
            topic: str,
            callback: Callable,
            is_json: bool = True,
            qos: int = DEFAULT_QOS,
            options: Any = None,
            properties: Any = None,
            use_args: bool = True
    ) -> bool:
        return_val = False
        if topic not in self.topic_map:
            with self.topic_map_lock:
                self.topic_map[topic] = (callback, is_json, qos, options, properties, use_args)
                return_val = True
                if self._connected:
                    self.client.subscribe(topic, qos = qos, options = options, properties = properties)
                    logger.success(f"Subscribed to: {topic}")
        return return_val

    def register_topic_map(self, topic_map: dict):
        for topic, callback in topic_map.items():
            self.register_callback(topic, callback)


def is_serializable(x: Any) -> (bool, str):
    try:
        json_string = json.dumps(x)
        return True, json_string
    except (TypeError, OverflowError):
        return False, None
