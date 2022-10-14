import math
import socket
import time
from enum import Enum
from threading import Thread

import cv2
import numpy as np
from loguru import logger

from vmc import stream
from vmc.mqtt_client import MQTTClient

socket.setdefaulttimeout(0.5)
MAX_MESSAGE_SIZE = 9000


def limit(value: int, lower: int = None, upper: int = None):
    if value >= upper:
        value = upper
    elif value <= lower:
        value = lower
    return value


class CameraType(Enum):
    CSI = 0
    ZED_RIGHT = 1
    ZED_LEFT = 2
    ZED_DEPTH = 3


class FrameServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 9999, buffer_size: int = 65536, timeout: float = 1) -> None:
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.timeout = timeout

        self.cameras = {}
        self.current_camera = CameraType.CSI
        self.is_auto = False
        self.server_socket = None
        self.server_thread = None
        self.is_running = False

        self.client = MQTTClient.get()

        success, default_frame = stream.encode_frame(cv2.imread("./vmc/resources/no_camera.jpg"))
        self.cameras[CameraType.CSI] = default_frame
        self.cameras[CameraType.ZED_RIGHT] = default_frame
        self.cameras[CameraType.ZED_LEFT] = default_frame
        self.cameras[CameraType.ZED_DEPTH] = default_frame

        self.client.register_callback("avr/camera/restart", self.restart)
        self.client.register_callback("avr/camera/auto", self._set_auto)
        self.client.register_callback("avr/camera/select", self._set_camera)

    def update_frame(self,
                     frame: np.ndarray,
                     camera_type: CameraType,
                     compression_level: int,
                     height: int = None
                     ) -> bool:
        if height is None:
            height = frame.shape[0]
        # noinspection PyBroadException
        try:
            resized_frame = stream.image_resize(frame, height = height)
            success, encoded_frame = stream.encode_frame(
                    resized_frame,
                    (int(cv2.IMWRITE_JPEG_QUALITY), compression_level)
            )
            if success:
                self.cameras[camera_type] = encoded_frame
        except Exception:
            return False
        return success

    def _set_auto(self, state: bool | dict):
        if isinstance(state, bool):
            self.is_auto = state
        else:
            self.is_auto = state.get("enabled", False)

    def _set_camera(self, payload: dict):
        index: int = payload.get("index", 0)
        if index in self.cameras:
            self.set_camera(index)

    def set_camera(self, camera_type: CameraType | int, force: bool = False) -> None:
        if force or self.is_auto:
            self.current_camera = camera_type

    def start(self) -> None:
        self.is_running = True
        self.server_thread = Thread(target = self._server_loop, daemon = True)
        self.server_thread.start()

    def stop(self, hold: bool = False) -> None:
        self.is_running = False
        if hold and self.server_thread is not None:
            self.server_thread.join()

    def restart(self, _ = None) -> None:
        self.stop(True)
        self.start()

    def _get_frame(self) -> bytes:
        return self.cameras[self.current_camera]

    def _server_loop(self) -> None:
        logger.info(f"Server starting on port {self.port}")
        while self.is_running:
            logger.debug("Starting socket")
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.settimeout(self.timeout)

            client_addr = None
            while self.is_running:
                try:
                    msg, client_addr = self.server_socket.recvfrom(self.buffer_size)
                    if client_addr is not None:
                        logger.info(f"Client found")
                        logger.debug(f"Starting stream")
                        break
                except TimeoutError:
                    pass
            while self.is_running:
                try:
                    frame = self._get_frame()
                    message_count = math.ceil(len(frame) / MAX_MESSAGE_SIZE)
                    self.server_socket.sendto(message_count.to_bytes(5, 'big'), client_addr)
                    if message_count == 1:
                        self.server_socket.sendto(frame, client_addr)
                        check_alive = self.server_socket.recv(4)
                        if not check_alive == b"ping":
                            logger.debug("Ping was invalid")
                            break
                    else:
                        for i in range(message_count):
                            start = i * MAX_MESSAGE_SIZE
                            if i == len(frame) - 1:
                                self.server_socket.sendto(frame[start:], client_addr)
                            else:
                                end = (i + 1) * MAX_MESSAGE_SIZE
                                self.server_socket.sendto(frame[start:end], client_addr)
                            check_alive = self.server_socket.recv(4)
                            if not check_alive == b"ping":
                                logger.debug("Ping was invalid")
                                break
                except TimeoutError:
                    logger.debug("Socket timeout waiting for ping")
                    break
                except OSError as e:
                    logger.debug("Socket error")
                    logger.debug(e)
                    break
                time.sleep(1 / 30)
            logger.info("Closing socket")
            try:
                self.server_socket.shutdown(socket.SHUT_RDWR)
            except (OSError, TimeoutError):
                pass
            self.server_socket.close()
