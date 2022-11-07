import atexit
import ctypes
import os
import queue
import time
from queue import Queue
from struct import pack
from threading import Thread
from typing import Any, Callable, List, Literal, Optional, Union

from adafruit_platformdetect import Detector
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
from bell.avr.serial.client import SerialLoop
from loguru import logger
from serial import SerialException

from .mqtt_client import MQTTClient

PLATFORM = Detector()
DEFAULT_DEVICE_TYPE = "ttyACM" if True in (
    PLATFORM.board.any_jetson_board, PLATFORM.board.any_raspberry_pi
) else "tty.usbmodem"


class PeripheralControlComputer:
    def __init__(self, device_type: str = DEFAULT_DEVICE_TYPE) -> None:
        self.module: PeripheralControlModule | None = None
        self.dev_type = device_type

        self.dev = SerialLoop()
        self.dev.baudrate = 115200
        self.command_queue = Queue[bytes](10)

        self.PREAMBLE = (0x24, 0x50)

        self.HEADER_OUTGOING = (*self.PREAMBLE, 0x3C)
        self.HEADER_INCOMING = (*self.PREAMBLE, 0x3E)

        self.servo_min_values = {}
        self.servo_max_values = {}

        self.commands = {
            "SET_SERVO_OPEN_CLOSE": 0,
            "SET_SERVO_MIN": 1,
            "SET_SERVO_MAX": 2,
            "SET_SERVO_PCT": 3,
            "SET_SERVO_ABS": 4,
            "SET_BASE_COLOR": 5,
            "SET_TEMP_COLOR": 6,
            "FIRE_LASER": 7,
            "SET_LASER_ON": 8,
            "SET_LASER_OFF": 9,
            "RESET_AVR_PERIPH": 10,
            "CHECK_SERVO_CONTROLLER": 11,
            "SET_ONBOARD_BASE_COLOR": 12,
            "SET_ONBOARD_TEMP_COLOR": 13,
            "COLOR_WIPE": 14,
            "FIRE_LASER_COUNT": 15,
            "SET_SERVO_ABS_LIMIT": 16
        }

        self.on_state: Callable = lambda state: None

        self.shutdown: bool = True
        self.serial_error: bool = False
        self.port_thread: Thread | None = None
        atexit.register(self.__atexit__)

    def __atexit__(self) -> None:
        self.dev.close()

    def begin(self) -> None:
        if self.shutdown:
            self.shutdown = False
            self.port_thread = Thread(target = self._port_loop, daemon = True)
            self.port_thread.start()

    def end(self) -> None:
        if self.shutdown:
            self.shutdown = True
            self.port_thread.join(5)

    def begin_mqtt(self) -> None:
        if self.module is None:
            self.module = PeripheralControlModule(self)

    @property
    def is_connected(self) -> bool:
        return self.dev.is_open and os.path.exists(self.dev.port) and not self.serial_error

    def _port_loop(self) -> None:
        first_connection = True
        previously_connected = self.is_connected
        while not self.shutdown:
            if not self.is_connected:
                if previously_connected:
                    logger.info("PCC Disconnected")
                    self.on_state(False)
                self.dev.close()
                dev_list = os.listdir("/dev")
                connected = False
                for dev in dev_list:
                    if dev.find(DEFAULT_DEVICE_TYPE) != -1:
                        self.dev.close()
                        self.dev.port = f"/dev/{dev}"
                        try:
                            self.dev.open()
                            logger.info("PCC Connected")
                            self.serial_error = False
                            connected = True
                            self._send_queue()
                            self.on_state(True)
                            break
                        except SerialException as e:
                            logger.exception(e)
                            connected = False
                if not connected and (previously_connected or first_connection):
                    logger.warning("Cant connect to the PCC")
                previously_connected = connected
                first_connection = False
            if self.shutdown:
                break
            time.sleep(1)

    def _send(self, data: bytes, enable_queueing: bool = True) -> bool:
        if self.is_connected:
            try:
                self.dev.write(data)
                self._send_queue()
                return True
            except SerialException:
                self.serial_error = True
        else:
            if enable_queueing:
                try:
                    logger.debug("PCC not connected, adding to queue")
                    self.command_queue.put(data, block = False, timeout = 0)
                except queue.Full:
                    logger.warning("PCC write queue full")
                return False
            else:
                logger.debug("PPC not connected, queueing is disabled for this method")

    def _send_queue(self) -> None:
        if self.is_connected:
            count = 0
            while not self.command_queue.empty():
                try:
                    data = self.command_queue.get(block = False, timeout = 0)
                    self.dev.write(data)
                    count += 1
                except queue.Empty:
                    break
                except SerialException:
                    logger.warning("Failed to write to PCC")
                    self.serial_error = True
                    break
            if count > 0:
                logger.debug(f"Flushed {count} messages from command queue")

    def set_base_color(self, wrgb: list[int] | tuple[int, int, int, int]) -> None:
        command = self.commands["SET_BASE_COLOR"]
        wrgb = list(wrgb)

        # wrgb + code = 5
        if len(wrgb) != 4:
            wrgb = [0, 0, 0, 0]

        for i, color in enumerate(wrgb):
            if not isinstance(color, int) or color > 255 or color < 0:
                wrgb[i] = 0

        data = self._construct_payload(command, 1 + len(wrgb), wrgb)

        logger.debug(f"Setting base color: {data}")
        self._send(data)

    def set_temp_color(self, wrgb: list[int] | tuple[int, int, int, int], length: float = 0.5) -> None:
        command = self.commands["SET_TEMP_COLOR"]
        wrgb = list(wrgb)

        # wrgb + code = 5
        if len(wrgb) != 4:
            wrgb = [0, 0, 0, 0]

        for i, color in enumerate(wrgb):
            if not isinstance(color, int) or color > 255 or color < 0:
                wrgb[i] = 0

        time_bytes = self.list_pack("<f", length)
        data = self._construct_payload(
                command, 1 + len(wrgb) + len(time_bytes), wrgb + time_bytes
        )

        logger.debug(f"Setting temp color: {data}")
        self._send(data)

    def set_servo_open_close(self, servo: int, action: Literal["open", "close"]) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_OPEN_CLOSE"]
        data = []

        # 128 is inflection point, over 128 == open; under 128 == close

        if action == "close":
            data = [servo, 100]
            valid_command = True

        elif action == "open":
            data = [servo, 150]
            valid_command = True

        if not valid_command:
            return

        length = 3
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo open/close: {data}")
        self._send(data)

    def set_servo_min(self, servo: int, minimum: int) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_MIN"]
        data = []

        if isinstance(minimum, int):
            uint16_absolute = ctypes.c_uint16(minimum).value
            uint8_absolute_high = (uint16_absolute >> 8) & 0xFF
            uint8_absolute_low = uint16_absolute & 0xFF
            valid_command = True
            data = [servo, int(uint8_absolute_high), int(uint8_absolute_low)]

        if not valid_command:
            return

        length = 4
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo min: {data}")
        self._send(data)

    def set_servo_max(self, servo: int, maximum: int) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_MAX"]
        data = []

        if isinstance(maximum, int):
            uint16_absolute = ctypes.c_uint16(maximum).value
            uint8_absolute_high = (uint16_absolute >> 8) & 0xFF
            uint8_absolute_low = uint16_absolute & 0xFF
            valid_command = True
            data = [servo, int(uint8_absolute_high), int(uint8_absolute_low)]

        if not valid_command:
            return

        length = 4
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo max: {data}")
        self._send(data)

    def set_servo_pct(self, servo: int, pct: int) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_PCT"]
        data = []

        if isinstance(pct, (float, int)):
            if 100 >= pct >= 0:
                valid_command = True
                data = [servo, int(pct)]

        if not valid_command:
            return

        length = 3
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo percent: {data}")
        self._send(data)

    def set_servo_abs(self, servo: int, absolute: int) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_ABS"]
        data = []

        if isinstance(absolute, int):
            uint16_absolute = ctypes.c_uint16(absolute).value
            uint8_absolute_high = (uint16_absolute >> 8) & 0xFF
            uint8_absolute_low = uint16_absolute & 0xFF
            valid_command = True
            data = [servo, int(uint8_absolute_high), int(uint8_absolute_low)]

        if not valid_command:
            return

        length = 4
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo absolute: {data}")
        self._send(data)

    def disable_servo(self, servo: int):
        self.set_servo_abs(servo, 0)

    def fire_laser(self, count: int = 1, hold: bool = False) -> bool:
        if count == 1:
            command = self.commands["FIRE_LASER"]
            data = self._construct_payload(command, 1)

            logger.debug(f"Firing the laser: {data}")
            self._send(data)
            return True
        elif count > 1:
            command = self.commands["FIRE_LASER_COUNT"]
            data = self._construct_payload(command, 2, [count])

            logger.debug(f"Firing the laser {count} times: {data}")
            self._send(data)
            if hold:
                self.dev.timeout = float(count + 2)
                try:
                    self.dev.read_all()
                    if b"FLC" not in self.dev.read_until(b"\n"):
                        return False
                except TimeoutError:
                    return False
            return True

    def set_laser_on(self) -> None:
        command = self.commands["SET_LASER_ON"]

        length = 1
        data = self._construct_payload(command, length)

        logger.debug(f"Setting the laser on: {data}")
        self._send(data)

    def set_laser_off(self) -> None:
        command = self.commands["SET_LASER_OFF"]

        length = 1
        data = self._construct_payload(command, length)

        logger.debug(f"Setting the laser off: {data}")
        self._send(data)

    def reset(self) -> None:
        command = self.commands["RESET_AVR_PERIPH"]

        length = 1  # just the reset command
        data = self._construct_payload(command, length)

        logger.debug(f"Resetting the PCC: {data}")
        self._send(data, enable_queueing = False)

    def check_servo_controller(self) -> None:
        command = self.commands["CHECK_SERVO_CONTROLLER"]

        length = 1
        data = self._construct_payload(command, length)

        logger.debug(f"Checking servo controller: {data}")
        self._send(data)

    def set_onboard_base_color(self, rgb: list[int] | tuple[int, int, int, int]) -> None:
        command = self.commands["SET_ONBOARD_BASE_COLOR"]
        rgb = list(rgb)

        # rgb + code = 4
        if len(rgb) != 3:
            rgb = [0, 0, 0]

        for i, color in enumerate(rgb):
            if not isinstance(color, int) or color > 255 or color < 0:
                rgb[i] = 0

        data = self._construct_payload(command, 1 + len(rgb), rgb)

        logger.debug(f"Setting base color: {data}")
        self._send(data)

    def set_onboard_temp_color(self, rgb: list[int] | tuple[int, int, int, int], length: float = 0.5) -> None:
        command = self.commands["SET_ONBOARD_TEMP_COLOR"]
        rgb = list(rgb)

        # rgb + code = 4
        if len(rgb) != 3:
            rgb = [0, 0, 0]

        for i, color in enumerate(rgb):
            if not isinstance(color, int) or color > 255 or color < 0:
                rgb[i] = 0

        time_bytes = self.list_pack("<f", length)
        data = self._construct_payload(
                command, 1 + len(rgb) + len(time_bytes), rgb + time_bytes
        )

        logger.debug(f"Setting temp color: {data}")
        self._send(data)

    def color_wipe(self, delay: int):
        command = self.commands["COLOR_WIPE"]
        data = self._construct_payload(command, 2, [delay])
        self._send(data)

    def _construct_payload(self, code: int, size: int = 0, data: Optional[list] = None) -> bytes:
        # [$][P][>][LENGTH-HI][LENGTH-LOW][DATA][CRC]
        payload = bytes()

        if data is None:
            data = []

        new_data = (
            ("<3b", self.HEADER_OUTGOING),
            (">H", [size]),
            ("<B", [code]),
            ("<%dB" % len(data), data),
        )

        for section in new_data:
            payload += pack(section[0], *section[1])

        crc = self.calc_crc(payload, len(payload))

        payload += pack("<B", crc)

        return payload

    @staticmethod
    def list_pack(bit_format: Union[str, bytes], value: Any) -> List[int]:
        return list(pack(bit_format, value))

    @staticmethod
    def crc8_dvb_s2(crc: int, a: int) -> int:
        # https://stackoverflow.com/a/52997726
        crc ^= a
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) % 256 if crc & 0x80 else (crc << 1) % 256
        return crc

    @classmethod
    def calc_crc(cls, string: bytes, length: int) -> int:
        crc = 0
        for i in range(length):
            crc = cls.crc8_dvb_s2(crc, string[i])
        return crc


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
        self.pcc.set_base_color(wrgb = list(wrgb))

    def set_temp_color(self, payload: AvrPcmSetTempColorPayload) -> None:
        wrgb = payload["wrgb"]
        length = payload.get("time", 0.5)  # default of 0.5 seconds
        self.pcc.set_temp_color(wrgb = list(wrgb), length = length)

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
