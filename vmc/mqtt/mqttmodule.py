from bell.avr.mqtt.client import MQTTModule as BadMQTTModule
from deprecated.classic import deprecated
from loguru import logger


class MQTTModule(BadMQTTModule):
    @deprecated
    def __init__(self) -> None:
        super().__init__()
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883
