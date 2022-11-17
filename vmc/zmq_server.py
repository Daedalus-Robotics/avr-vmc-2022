import json
import time
from threading import Barrier, BrokenBarrierError, Thread
from typing import Callable

import zmq
from loguru import logger

from .autonomy.autonomy import Autonomy
from .status import Status


class ZMQServer:
    def __init__(
            self,
            status: Status | None,
            autonomy: Autonomy,
            # pub_port: int = 5570,
            sub_port: int = 5580
    ) -> None:
        self.status = status
        self.autonomy = autonomy
        # self.pub_port = pub_port
        self.sub_port = sub_port

        self._last_flash = time.time()
        self.context = context = zmq.Context()

        # self.pub_socket = context.socket(zmq.PUB)
        # self.pub_socket.bind(f"tcp://*:{self.pub_port}")

        self.sub_socket = context.socket(zmq.SUB)
        self.sub_socket.bind(f"tcp://*:{self.sub_port}")

        self.topic_map: dict[str, Callable[[dict | list | str | bytes], None]] = {
            "gimbal_pos": self.autonomy.gimbal.zmq_pos,
            "gimbal_move": self.autonomy.gimbal.zmq_pos,
            "gimbal_disable": self.autonomy.gimbal.zmq_disable,
            "gimbal_auto": self.autonomy.gimbal.zmq_auto,
            "gimbal_fire": self.autonomy.gimbal.zmq_fire,
            "water_drop_kill": self.autonomy.water_drop.kill,
            "water_drop_try": self.autonomy.water_drop.try_drop
        }

        for topic in self.topic_map:
            self.sub_socket.subscribe(topic)

        self.sub_recv_thread = Thread(target = self._recv_loop, daemon = True)
        self.running = False
        self.running_barrier = Barrier(2)

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass

    def _recv_loop(self) -> None:
        if self.status is not None:
            self.status.update_status("zmq", True)
        try:
            while self.running:
                if self.sub_socket.poll(1000) != 0:
                    string = self.sub_socket.recv().decode()
                    message_split = string.split()
                    topic = message_split[0].lower().strip()
                    message_string = ""
                    for split in message_split[1:]:
                        message_string += split

                    try:
                        message = json.loads(message_string)
                    except json.JSONDecodeError:
                        message = message_string
                    logger.debug(f"Message received: \"{topic}: {message}\"")

                    callback = self.topic_map.get(topic, None)
                    if callback is not None:
                        ss = time.time()
                        timesince = ss - self._last_flash
                        if timesince >= 0.2:
                            self.status.led_event("zmq", (0, 150, 255), 0.1)
                            self._last_flash = ss
                        try:
                            callback(message)
                        finally:
                            pass
                    else:
                        logger.debug("Received bad topic. Skipping.")
        finally:
            try:
                self.running_barrier.wait(0)
            except BrokenBarrierError:
                pass
            if self.status is not None:
                self.status.update_status("zmq", False)

    def run(self) -> None:
        self.running = True
        self.sub_recv_thread.start()
