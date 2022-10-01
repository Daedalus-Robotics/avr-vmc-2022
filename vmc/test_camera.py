import time
from threading import Thread

import cv2

from vmc import frame_server, stream
from vmc.mqtt_client import MQTTClient

cap = cv2.VideoCapture(1)
client = MQTTClient.get()
client.connect()


def run():
    while True:
        try:
            _, frame = cap.read()
            frame = stream.image_resize(frame, height = 144)
            success, encoded_frame = stream.encode_frame(frame, (int(cv2.IMWRITE_JPEG_QUALITY), 35))
            if success:
                frame_server.update_csi(encoded_frame)
            time.sleep(1/60)
        finally:
            pass


frame_server.run()
Thread(target = run).start()
