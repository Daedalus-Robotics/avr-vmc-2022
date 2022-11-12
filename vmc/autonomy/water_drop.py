from threading import Barrier, BrokenBarrierError

from loguru import logger

from ..mqtt_client import MQTTClient


class WaterDrop:
    def __init__(self) -> None:
        self.client = MQTTClient.get()

        self.is_dropping = False

        self.running = False
        self.running_barrier = Barrier(2)

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass

    async def run(self) -> None:
        logger.info("Water Drop started")
        self.running = True
        last_is_dropping = False
        while self.running:
            if self.is_dropping:
                if not last_is_dropping:
                    pass
                last_is_dropping = True
            else:
                if last_is_dropping:
                    pass
                last_is_dropping = False
        try:
            self.running_barrier.wait(0)
        except BrokenBarrierError:
            pass
