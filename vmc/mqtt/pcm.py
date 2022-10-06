from bell.avr.mqtt.payloads import (
    AvrPcmFireLaserPayload,
    AvrPcmSetBaseColorPayload,
    AvrPcmSetLaserOffPayload,
    AvrPcmSetLaserOnPayload,
    AvrPcmSetServoAbsPayload,
    AvrPcmSetServoMaxPayload,
    AvrPcmSetServoMinPayload,
    AvrPcmSetServoOpenClosePayload,
    AvrPcmSetServoPctPayload,
    AvrPcmSetTempColorPayload,
)

from vmc.lib.pcc import PeripheralControlComputer
from vmc.mqtt_client import MQTTClient


class PeripheralControlModule:
    def __init__(self, pcc: PeripheralControlComputer):
        super().__init__()

        self.pcc = pcc

        self.client = MQTTClient.get()

        # MQTT topics
        self.topic_map = {
            "avr/pcm/set_base_color": self.set_base_color,
            "avr/pcm/set_temp_color": self.set_temp_color,
            "avr/pcm/set_servo_open_close": self.set_servo_open_close,
            "avr/pcm/set_servo_min": self.set_servo_min,
            "avr/pcm/set_servo_max": self.set_servo_max,
            "avr/pcm/fire_laser": self.fire_laser,
            "avr/pcm/set_laser_on": self.set_laser_on,
            "avr/pcm/set_laser_off": self.set_laser_off,
            "avr/pcm/set_servo_pct": self.set_servo_pct,
            "avr/pcm/set_servo_abs": self.set_servo_abs,
        }

        for topic, callback in self.topic_map.items():
            self.client.register_callback(topic, callback)

    def set_base_color(self, payload: AvrPcmSetBaseColorPayload) -> None:
        wrgb = payload["wrgb"]
        self.pcc.set_base_color(wrgb=list(wrgb))

    def set_temp_color(self, payload: AvrPcmSetTempColorPayload) -> None:
        wrgb = payload["wrgb"]
        time = payload.get("time", 0.5)  # default of 0.5 seconds
        self.pcc.set_temp_color(wrgb=list(wrgb), length =time)

    def set_servo_open_close(self, payload: AvrPcmSetServoOpenClosePayload) -> None:
        servo = payload["servo"]
        action = payload["action"]
        self.pcc.set_servo_open_close(servo, action)

    def set_servo_min(self, payload: AvrPcmSetServoMinPayload) -> None:
        servo = payload["servo"]
        min_pulse = payload["min_pulse"]
        self.pcc.set_servo_min(servo, min_pulse)

    def set_servo_max(self, payload: AvrPcmSetServoMaxPayload) -> None:
        servo = payload["servo"]
        max_pulse = payload["max_pulse"]
        self.pcc.set_servo_max(servo, max_pulse)

    def set_servo_pct(self, payload: AvrPcmSetServoPctPayload) -> None:
        servo = payload["servo"]
        percent = payload["percent"]
        self.pcc.set_servo_pct(servo, percent)

    def set_servo_abs(self, payload: AvrPcmSetServoAbsPayload) -> None:
        servo = payload["servo"]
        absolute = payload["absolute"]
        self.pcc.set_servo_abs(servo, absolute)

    def fire_laser(self, _: AvrPcmFireLaserPayload) -> None:
        self.pcc.fire_laser()

    def set_laser_on(self, _: AvrPcmSetLaserOnPayload) -> None:
        self.pcc.set_laser_on()

    def set_laser_off(self, _: AvrPcmSetLaserOffPayload) -> None:
        self.pcc.set_laser_off()


if __name__ == "__main__":
    pcm = PeripheralControlModule(PeripheralControlComputer())
    pcm.client.connect()
