from bell.avr.mqtt.client import MQTTModule as BadMQTTModule


class MQTTModule(BadMQTTModule):
    def __init__(self):
        super().__init__()
        self.mqtt_host = "localhost"
        self.mqtt_port = 1883
