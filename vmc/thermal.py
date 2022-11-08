import math
import time
from random import randint
from threading import Barrier, BrokenBarrierError, Thread
from typing import List

import cv2
import numpy as np
from adafruit_platformdetect import Detector as PlatformDetector
from colour import Color
from loguru import logger

from .mqtt_client import MQTTClient
from . import stream
from .utils import constrain, map

from scipy.interpolate import griddata

platform_detector = PlatformDetector()
TESTING = not (platform_detector.board.any_raspberry_pi or platform_detector.board.any_jetson_board)
if not TESTING:
    import board
    import adafruit_amg88xx

VIEW_SIZE = 30
CAMERA_SIZE = 8
TOTAL_CAMERA = CAMERA_SIZE * CAMERA_SIZE

POINTS = [(math.floor(ix / CAMERA_SIZE), (ix % CAMERA_SIZE)) for ix in range(TOTAL_CAMERA)]
GRID_X, GRID_Y = np.mgrid[0: CAMERA_SIZE - 1: TOTAL_CAMERA / 2j, 0: CAMERA_SIZE - 1: TOTAL_CAMERA / 2j]

COLORDEPTH = 1024
COLORS = list(Color("indigo").range_to(Color("red"), COLORDEPTH))
COLORS = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in COLORS]

ENCODING_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]


class ThermalCamera:
    def __init__(self) -> None:
        self.client = MQTTClient.get()

        self.bgr_frame = None
        self.hsv_frame = None

        lower_bound = np.array([0, 0, 200])
        upper_bound = np.array([0, 0, 255])
        self.detector = Detector(lower_bound, upper_bound, 2)

        if not TESTING:
            i2c = board.I2C()
            self.amg = adafruit_amg88xx.AMG88XX(i2c)
            logger.success("Connected to thermal camera!")
        else:
            self.testing_change_pos = 0
            self.testing_pos = (CAMERA_SIZE // 2, CAMERA_SIZE // 2)

        self.update_barrier = Barrier(2)

        Thread(target = self._update_loop, daemon = True).start()

    @property
    def pixels(self) -> np.ndarray:
        if not TESTING:
            pixel_list = self.amg.pixels
            int_pixel_list = []
            x = 0
            for row in pixel_list:
                y = 0
                int_row = []
                for pixel in row:
                    int_row.append(round(pixel))
                    y += 1
                int_pixel_list.append(int_row)
                x += 1
            pixels = np.array(int_pixel_list)

        else:
            x, y = self.testing_pos
            if self.testing_change_pos == 0:
                x, y = randint(x - 1, x + 1), randint(y - 1, y + 1)
                self.testing_pos = (x, y)
            self.testing_change_pos = (self.testing_change_pos + 1) % 8
            if x >= 7:
                x = CAMERA_SIZE // 2
            elif x < 1:
                x = CAMERA_SIZE // 2
            if y >= 7:
                y = CAMERA_SIZE // 2
            elif y < 1:
                y = CAMERA_SIZE // 2
            radius = 1
            pixels = np.zeros((CAMERA_SIZE, CAMERA_SIZE, 1), np.uint8)
            cv2.circle(pixels, (x, y), radius, 78, -1)
        return pixels

    def get_frame(self, color = False) -> np.ndarray:
        return self.bgr_frame if color else self.hsv_frame

    def update_frame(self) -> None:
        rgb_frame = np.zeros((VIEW_SIZE, VIEW_SIZE, 3), np.uint8)
        hsv_frame = np.zeros((VIEW_SIZE, VIEW_SIZE, 3), np.uint8)

        float_pixels_matrix = np.reshape(self.pixels, (CAMERA_SIZE, CAMERA_SIZE))
        float_pixels_matrix = np.rot90(float_pixels_matrix, 1)
        rotated_float_pixels = float_pixels_matrix.flatten()

        bicubic = griddata(
                POINTS,
                rotated_float_pixels,
                (GRID_X, GRID_Y),
                method = "cubic",
        )

        for ix, row in enumerate(bicubic):
            if ix >= VIEW_SIZE:
                break
            for iy, pixel in enumerate(row):
                if iy >= VIEW_SIZE:
                    break
                color_index = int(constrain(map(pixel, 0, 80, 0, COLORDEPTH - 1), 0, COLORDEPTH - 1))
                rgb_frame[ix, iy] = COLORS[color_index]

                value = int(constrain(map(pixel, 0, 80, 0, 255), 0, 255))
                hsv_frame[ix, iy] = (0, 0, value)

        self.bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        self.hsv_frame = hsv_frame

        self.detector.update(hsv_frame)

        try:
            self.update_barrier.wait(0)
        except BrokenBarrierError:
            self.update_barrier.reset()

    def _update_loop(self) -> None:
        while True:
            self.update_frame()
            self._stream()
            if TESTING:
                time.sleep(1 // 15)

    def _stream(self) -> None:
        if self.client.is_connected:
            frame = self.get_frame(color = True)
            frame = stream.image_resize(frame, 300)
            frame = self.detector.overlay(frame)
            success, encoded_frame = stream.encode_frame(frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if success:
                self.client.send_message("avr/raw/thermal/reading", encoded_frame)


class Detector:
    def __init__(self, lower_bound: np.ndarray, upper_bound: np.ndarray, kernel_size: int = 15):
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.kernel_size = kernel_size

        self.contours: List = []
        self.currently_detecting = False
        self.largest_contour = None
        self.main_detection_image_shape = (1, 1, 3)
        self.main_detection_center = (0, 0)
        self.main_detection_radius = 0
        self.previous_positions: List[tuple[int, int]] = []

    def update(self, hsv_frame: np.ndarray) -> None:
        mask = cv2.inRange(hsv_frame, self.lower_bound, self.upper_bound)

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        opened_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)

        _, thresh = cv2.threshold(opened_mask, 40, 255, cv2.THRESH_BINARY)
        self.contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        largest_area = 0
        self.largest_contour = None
        for contour in self.contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                self.largest_contour = contour

        moments = cv2.moments(self.largest_contour)

        self.currently_detecting = True if (len(self.contours) > 0) and (moments['m00'] != 0.0) else False
        if not self.currently_detecting:
            self.previous_positions = []

        if moments['m00'] != 0.0:
            center_x = round(moments['m10'] / moments['m00'])
            center_y = round(moments['m01'] / moments['m00'])

            self.main_detection_image_shape = hsv_frame.shape

            self.main_detection_center = (
                center_x,
                center_y
            )

            if len(self.previous_positions) > 3:
                self.previous_positions.pop(0)
            self.previous_positions.append(self.main_detection_center)

            x_cord = center_x
            y_cord = center_y
            for point in self.largest_contour:
                if point[0][1] > y_cord:
                    y_cord = point[0][1]
                if point[0][0] < x_cord:
                    x_cord = point[0][0]
            use_x_cord = (x_cord - center_x) > (y_cord - center_y)

            self.main_detection_radius = (x_cord if use_x_cord else y_cord) // 2
        time.sleep(1 / 5)

    def overlay(self, frame: np.ndarray, show_contours: bool = False) -> np.ndarray:
        ov_frame = frame.copy()

        position = self.stabilized_position

        if self.currently_detecting:
            detection_center_x = int(
                    map(
                            position[0],
                            0,
                            self.main_detection_image_shape[0],
                            0,
                            ov_frame.shape[0]
                    )
            )
            detection_center_y = int(
                    map(
                            position[1],
                            0,
                            self.main_detection_image_shape[1],
                            0,
                            ov_frame.shape[1]
                    )
            )
            radius = int(
                    map(
                            self.main_detection_radius,
                            0,
                            self.main_detection_image_shape[0],
                            0,
                            ov_frame.shape[0]
                    )
            )

            cv2.line(
                    ov_frame,
                    (detection_center_x, 0),
                    (detection_center_x, ov_frame.shape[1]),
                    (255, 255, 255),
                    2
            )
            cv2.line(
                    ov_frame,
                    (0, detection_center_y),
                    (ov_frame.shape[0], detection_center_y),
                    (255, 255, 255),
                    2
            )
            cv2.circle(
                    ov_frame,
                    (detection_center_x, detection_center_y),
                    radius,
                    (255, 0, 255),
                    3
            )
        if show_contours:
            cv2.drawContours(
                    ov_frame,
                    self.contours,
                    -1,
                    (255, 0, 255),
                    2
            )

        return ov_frame

    @property
    def stabilized_position(self) -> tuple[int, int] | None:
        if self.currently_detecting:
            x_sum = 0
            y_sum = 0
            for position in self.previous_positions:
                x_sum += position[0]
                y_sum += position[1]
            x_avg = x_sum // len(self.previous_positions)
            y_avg = y_sum // len(self.previous_positions)
            return x_avg, y_avg
        else:
            return None

if __name__ == "__main__":
    thermal = ThermalCamera()
