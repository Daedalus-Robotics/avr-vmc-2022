from typing import Callable

from .mqtt_client import MQTTClient
from .status_led import StatusLed, StatusStrip

STOPPED_COLOR = (255, 0, 0)
RUNNING_COLOR = (0, 255, 0)


class Status:
    def __init__(self, status_strip: StatusStrip) -> None:
        self.client = MQTTClient.get()
        self.client.register_callback(
                "avr/status/request_update",
                self.send_update,
                qos=2,
                is_json=False,
                use_args=False
        )

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
                        is_json=False,
                        use_args=False
                )
            if led_num >= 0:
                self.status_leds[name] = self.status_strip.get_status_led(led_num)
                self.status_leds[name].set_color(RUNNING_COLOR if initial_value else STOPPED_COLOR)

    def add_restart_callback(self,
                             name: str,
                             restart_callback: Callable):
        if name in self.statuses:
            self.client.register_callback(
                    f"avr/status/restart/{name}",
                    restart_callback,
                    is_json=False,
                    use_args=False
            )

    def update_status(self, name: str, value: bool) -> None:
        if name in self.statuses:
            if value is not self.statuses[name]:
                self.statuses[name] = value
                if name in self.status_leds:
                    self.status_leds[name].set_color(RUNNING_COLOR if value else STOPPED_COLOR)
                self.send_update()

    def led_event(self, name: str, color: tuple[int, int, int], timeout: float):
        if name in self.status_leds:
            if timeout > 0:
                self.status_leds[name].flash_color(color, timeout)
            else:
                self.status_leds[name].set_color(color)

    def end_led_event(self, name: str):
        if name in self.status_leds and name in self.statuses:
            self.status_leds[name].set_color(RUNNING_COLOR if self.statuses[name] else STOPPED_COLOR)

    def send_update(self) -> None:
        self.client.send_message("avr/status/update", self.statuses, qos=2)
