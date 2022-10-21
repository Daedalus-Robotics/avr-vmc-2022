from typing import Any, Callable

from vmc.mqtt_client import MQTTClient
from vmc.status_led import StatusLed, StatusStrip


class Status:
    def __init__(self, status_strip: StatusStrip) -> None:
        self.client = MQTTClient.get()
        self.client.register_callback("avr/status/request_update", self.send_update, is_json = False, qos = 2)

        self.status_strip = status_strip

        self.statuses: dict[str, bool] = {}
        self.status_leds: dict[str, StatusLed] = {}

    def register_status(self,
                        name: str,
                        initial_value: bool = False,
                        restart_callback: Callable = None,
                        led_num: int = -1) -> None:
        if name not in self.statuses:
            self.statuses[name] = initial_value
            if restart_callback is not None:
                self.client.register_callback(
                        f"avr/status/restart/{name}",
                        restart_callback,
                        is_json = False,
                        use_args = False
                )
            if led_num >= 0:
                self.status_leds[name] = self.status_strip.get_status_led(led_num)

    def update_status(self, name: str, value: bool) -> None:
        if name in self.statuses:
            self.statuses[name] = value
            self.send_update()

    def send_update(self, _: Any = None) -> None:
        self.client.send_message("avr/status/update", self.statuses, qos = 2, retain = True)
