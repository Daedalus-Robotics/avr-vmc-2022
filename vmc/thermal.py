import math
import time
from random import randint
from threading import Thread

import numpy as np
import cv2
from loguru import logger
from colour import Color

from utils import map, constrain
from vmc import stream
from vmc.mqtt_client import MQTTClient
from adafruit_platformdetect import Detector as PlatformDetector

platform_detector = PlatformDetector()
TESTING = not (platform_detector.board.any_raspberry_pi or platform_detector.board.any_jetson_board)
if not TESTING:
    import board
    import adafruit_amg88xx

POINTS = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
COLORDEPTH = 1024
COLORS = list(Color("indigo").range_to(Color("red"), COLORDEPTH))
COLORS = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in COLORS]
ENCODING_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]


client = MQTTClient.get()
client.connect()


class ThermalCamera:
    def __init__(self):
        self.bgr_frame = None
        self.hsv_frame = None

        if not TESTING:
            i2c = board.I2C()
            self.amg = adafruit_amg88xx.AMG88XX(i2c)
            logger.success("Connected to thermal camera!")

        Thread(target = self._update_loop).start()

    @property
    def pixels(self) -> np.ndarray:
        if not TESTING:
            return np.array(self.amg.pixels)
        else:
            pixels = np.zeros((8, 8, 1), np.uint8)
            for row_num in range(8):
                for pixel_num in range(8):
                    temp = randint(0, 80)
                    pixels[pixel_num, row_num] = temp
            return pixels

    def get_frame(self, color = False) -> np.ndarray:
        return self.bgr_frame if color else self.hsv_frame

    def update_frame(self, enable_interpolation = False) -> None:
        size = 240 if enable_interpolation else 8
        rgb_frame = np.zeros((size, size, 3), np.uint8)
        hsv_frame = np.zeros((size, size, 3), np.uint8)

        y = 0
        for row in self.pixels:
            x = 0
            for pixel in row:
                color_index = int(constrain(map(pixel, 0, 80, 0, COLORDEPTH - 1), 0, COLORDEPTH - 1))
                rgb_frame[x, y] = COLORS[color_index]

                value = int(constrain(map(pixel, 0, 80, 0, 255), 0, 255))
                hsv_frame[x, y] = (0, 0, value)

                x += 1
            y += 1

        self.bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        self.hsv_frame = hsv_frame

    def _update_loop(self):
        counter = 0
        while True:
            self.update_frame()
            if counter == 0:
                self._stream()
            counter = (counter + 1) % 3
            time.sleep(1/30)

    def _stream(self):
        if client.is_connected:
            frame = self.get_frame(color = True)
            encoded_frame = stream.encode_frame_uncompressed(frame)
            client.send_message("avr/thermal/reading", encoded_frame)


if __name__ == "__main__":
    thermal = ThermalCamera()

import test_camera
print(test_camera)
