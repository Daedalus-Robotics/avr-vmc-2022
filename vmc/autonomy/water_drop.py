import time
from threading import Barrier, BrokenBarrierError

import mavsdk
from bell.avr.mqtt.payloads import AvrApriltagsVisibleTags
from loguru import logger

from ..apriltag.python.apriltag_processor import AprilTagModule
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer


class WaterDrop:
    def __init__(
            self,
            pcc: PeripheralControlComputer,
            apriltags: AprilTagModule,
            mavlink_system: mavsdk.System,
            servo_num: int
    ) -> None:
        self.client = MQTTClient.get()

        self.pcc = pcc
        self.apriltags = apriltags
        self.mavlink_system = mavlink_system
        self.water_drop_servo = servo_num

        self.is_dropping = False
        self.stop_autonomy = False
        self.dropping_tag: AvrApriltagsVisibleTags | None = None

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

    def kill(self, _: str = None) -> None:
        self.is_dropping = False

    def try_drop(self, message: dict) -> None:
        tag_id = message.get("tag", -1)
        recent_apriltags = self.apriltags.get_valid_apriltags(2)
        if tag_id in recent_apriltags:
            self.client.send_message(
                    "avr/gui/toast",
                    {
                        "text": f"Starting water drop on tag {tag_id}",
                        "timeout": 1
                    }
            )
            self.is_dropping = True
            self.dropping_tag = recent_apriltags[tag_id]
        else:
            self.client.send_message(
                    "avr/gui/toast",
                    {
                        "text": f"Tag {tag_id} not in sight!",
                        "timeout": 2
                    }
            )

    async def run(self) -> None:
        logger.info("Water Drop started")
        self.running = True
        while self.running:
            if self.is_dropping:
                try:
                    # while self.is_dropping:
                    #     pass
                    pos_world = self.dropping_tag = ["pos_world"]
                    n = pos_world["x"]
                    e = pos_world["y"]
                    d = pos_world["z"]
                    pos = mavsdk.system.offboard.PositionNedYaw(n, e, d, 0)

                    await self.mavlink_system.offboard.start()
                    await self.mavlink_system.offboard.set_position_ned(pos)
                    time.sleep(2)
                    await self.mavlink_system.offboard.stop()
                finally:
                    self.is_dropping = False
                    self.dropping_tag = None
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
