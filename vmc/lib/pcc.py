import atexit
import ctypes
import os
import queue
import time
from queue import Queue
from struct import pack
from threading import Thread
from typing import Any, List, Literal, Optional, Union

from adafruit_platformdetect import Detector, Board
from bell.avr.serial.client import SerialLoop
from loguru import logger
from serial import SerialException

PLATFORM = Detector()
DEFAULT_DEVICE_TYPE = "something" if PLATFORM.board in (Board.any_jetson_board, Board.any_raspberry_pi) else "tty.usbmodem"


class PeripheralControlComputer:
    def __init__(self, device_type: str = DEFAULT_DEVICE_TYPE) -> None:
        self.dev_type = device_type

        self.dev = SerialLoop()
        self.dev.baudrate = 115200
        self.command_queue = Queue[bytes](10)

        self.PREAMBLE = (0x24, 0x50)

        self.HEADER_OUTGOING = (*self.PREAMBLE, 0x3C)
        self.HEADER_INCOMING = (*self.PREAMBLE, 0x3E)

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
        }

        self.shutdown: bool = False
        self.port_thread = Thread(target = self._port_loop, daemon = True).start()
        atexit.register(self.__atexit__)

    def __atexit__(self) -> None:
        self.dev.close()

    @property
    def is_connected(self) -> bool:
        return self.dev.is_open and os.path.exists(self.dev.port)

    def _port_loop(self) -> None:
        first_connection = True
        previously_connected = False
        while not self.shutdown:
            if not self.is_connected:
                if previously_connected:
                    logger.info("PCC Disconnected")
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
                            connected = True
                            self._send_queue()
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
            time.sleep(2)

    def _send(self, data: bytes) -> bool:
        if self.is_connected:
            try:
                self.dev.write(data)
                self._send_queue()
                return True
            except SerialException:
                pass
        try:
            logger.debug("PCC not connected, adding to queue")
            self.command_queue.put(data, block = False, timeout = 0)
        except queue.Full:
            logger.warning("PCC write queue full")
        return False

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
                    break
            if count > 0:
                logger.debug(f"Flushed {count} messages from command queue")

    def set_base_color(self, wrgb: List[int]) -> None:
        command = self.commands["SET_BASE_COLOR"]

        # wrgb + code = 5
        if len(wrgb) != 4:
            wrgb = [0, 0, 0, 0]

        for i, color in enumerate(wrgb):
            if not isinstance(color, int) or color > 255 or color < 0:
                wrgb[i] = 0

        data = self._construct_payload(command, 1 + len(wrgb), wrgb)

        logger.debug(f"Setting base color: {data}")
        self._send(data)

    def set_temp_color(self, wrgb: List[int], length: float = 0.5) -> None:
        command = self.commands["SET_TEMP_COLOR"]

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

    def set_servo_open_close(
        self, servo: int, action: Literal["open", "close"]
    ) -> None:
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

    def set_servo_min(self, servo: int, minimum: float) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_MIN"]
        data = []

        if isinstance(minimum, (float, int)):
            if 1000 > minimum > 0:
                valid_command = True
                data = [servo, minimum]

        if not valid_command:
            return

        length = 3
        data = self._construct_payload(command, length, data)

        logger.debug(f"Setting servo min: {data}")
        self._send(data)

    def set_servo_max(self, servo: int, maximum: float) -> None:
        valid_command = False

        command = self.commands["SET_SERVO_MAX"]
        data = []

        if isinstance(maximum, (float, int)):
            if 1000 > maximum > 0:
                valid_command = True
                data = [servo, maximum]

        if not valid_command:
            return

        length = 3
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

    def fire_laser(self) -> None:
        command = self.commands["FIRE_LASER"]

        length = 1
        data = self._construct_payload(command, length)

        logger.debug(f"Setting the laser on: {data}")
        self._send(data)

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
        self._send(data)

        self.dev.close()
        time.sleep(5)
        self.dev.open()

    def check_servo_controller(self) -> None:
        command = self.commands["CHECK_SERVO_CONTROLLER"]

        length = 1
        data = self._construct_payload(command, length)

        logger.debug(f"Checking servo controller: {data}")
        self._send(data)

    def _construct_payload(
        self, code: int, size: int = 0, data: Optional[list] = None
    ) -> bytes:
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
