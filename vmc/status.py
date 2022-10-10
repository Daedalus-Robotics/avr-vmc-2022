from typing import Any

from vmc.mqtt_client import MQTTClient


class Status:
    def __init__(self) -> None:
        self.client = MQTTClient.get()
        self.client.register_callback("avr/status/request_update", self.send_update, is_json = False, qos = 2)

        self.statuses: dict[str, bool] = {}

    def register_status(self, name: str, initial_value: bool = False) -> None:
        if name not in self.statuses:
            self.statuses[name] = initial_value

    def update_status(self, name: str, value: bool) -> None:
        self.statuses[name] = value
        self.send_update()

    def send_update(self, _: Any = None) -> None:
        self.client.send_message("avr/status/update", self.statuses, qos = 2, retain = True)
