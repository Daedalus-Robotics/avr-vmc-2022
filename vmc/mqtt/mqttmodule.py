from bell.avr.mqtt.client import MQTTModule as BadMQTTModule


class MQTTModule(BadMQTTModule):
    @DeprecationWarning
    def __init__(self):
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883
