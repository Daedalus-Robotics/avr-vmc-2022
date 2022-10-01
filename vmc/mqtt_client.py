from __future__ import annotations

import json

import numpy as np
from loguru import logger
from paho.mqtt import client as mqtt


class MQTTClient:
    _instance = None

    @classmethod
    def get(cls) -> MQTTClient:
        if cls._instance is None:
            cls._instance = MQTTClient()
        return cls._instance

    def __init__(self, host: str = "localhost", port: int = 1883):
        self._instance = self

        self.host = host
        self.port = port

        self.client = mqtt.Client(protocol = mqtt.MQTTv311)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        self.topic_map = {}
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected()

    def connect(self) -> None:
        if not self.is_connected:
            self.client.connect(host = self.host, port = self.port, keepalive = 60)
            self.client.loop_start()

    def _on_connect(self, _: mqtt.Client, __: any, ___: dict, rc: int):
        self._connected = True
        logger.debug(f"Connected with result {rc}")

        for topic in self.topic_map.keys():
            self.client.subscribe(topic)
            logger.success(f"Subscribed to: {topic}")

    def _on_message(self, _: mqtt.Client, __: any, msg: mqtt.MQTTMessage):
        if msg.topic in self.topic_map:
            try:
                self.topic_map[msg.topic](json.loads(msg.payload))
            except (AttributeError, TypeError, json.JSONDecodeError) as e:
                logger.warning(f"Filed to run callback for topic { msg.topic }")
                logger.warning(e)

    def send_message(self, topic: str, payload: any) -> bool:
        serializable, json_payload = is_serializable(payload)
        if serializable:
            payload = json_payload
        try:
            self.client.publish(topic, payload)
            return True
        except TypeError:
            return False

    def register_callback(self, topic: any, callback: callable) -> bool:
        return_val = False
        if topic not in self.topic_map:
            self.topic_map[topic] = callback
            return_val = True
            if self._connected:
                self.client.subscribe(topic)
                logger.success(f"Subscribed to: {topic}")
        return return_val


def is_serializable(x) -> (bool, str):
    try:
        json_string = json.dumps(x)
        return True, json_string
    except (TypeError, OverflowError):
        return False, None
