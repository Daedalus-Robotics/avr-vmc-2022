import time
from threading import Thread

import cv2

from vmc import frame_server
from vmc.mqtt_client import MQTTClient

cap = cv2.VideoCapture(4)
client = MQTTClient.get()
client.connect()

s: frame_server.FrameServer


def run():
    while True:
        try:
            success, frame = cap.read()
            if success:
                s.update_frame(frame, frame_server.CameraType.CSI, 90, 720)
            time.sleep(1/120)
        finally:
            pass


if __name__ == '__main__':
    s = frame_server.FrameServer()
    s.start()
    Thread(target = run, daemon = True).start()
