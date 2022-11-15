from threading import Barrier, BrokenBarrierError

from loguru import logger

from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer


class WaterDrop:
    def __init__(self, pcc: PeripheralControlComputer, servo_num: int) -> None:
        self.client = MQTTClient.get()

        self.pcc = pcc
        self.water_drop_servo = servo_num

        self.is_dropping = False

        self.running = False
        self.running_barrier = Barrier(2)

    @property
    def is_doing_stuff(self) -> bool:
        return self.is_dropping

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

    def set_water_drop(self, percent: int) -> None:
        if 0 <= percent <= 100:
            self.pcc.set_servo_pct(self.water_drop_servo, percent)

    def zmq_set(self, message: dict) -> None:
        try:
            percent = message.get("percent", 0)
            self.set_water_drop(percent)
        except AttributeError:
            logger.warning("Got invalid message for zmq_set")
