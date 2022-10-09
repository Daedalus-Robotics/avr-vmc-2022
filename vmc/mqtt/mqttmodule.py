from bell.avr.mqtt.client import MQTTModule as BadMQTTModule
from loguru import logger


class MQTTModule(BadMQTTModule):
    @DeprecationWarning
    def __init__(self):
        logger.warning("MQTTModule is deprecated, use MQTTClient instead")
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883
