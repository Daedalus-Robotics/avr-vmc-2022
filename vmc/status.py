from typing import Any, Callable

from vmc.mqtt_client import MQTTClient


class Status:
    def __init__(self) -> None:
        self.client = MQTTClient.get()
        self.client.register_callback("avr/status/request_update", self.send_update, is_json = False, qos = 2)

        self.statuses: dict[str, bool] = {}

    def register_status(self, name: str, initial_value: bool = False, restart_callback: Callable = None) -> None:
        if name not in self.statuses:
            self.statuses[name] = initial_value
            if restart_callback is not None:
                self.client.register_callback(
                        f"avr/status/restart/{name}",
                        restart_callback,
                        is_json = False,
                        use_args = False
                )

    def update_status(self, name: str, value: bool) -> None:
        if name in self.statuses:
            self.statuses[name] = value
            self.send_update()

    def send_update(self, _: Any = None) -> None:
        self.client.send_message("avr/status/update", self.statuses, qos = 2, retain = True)
