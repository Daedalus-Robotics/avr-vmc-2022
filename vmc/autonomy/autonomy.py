import asyncio
import time
from threading import Barrier, BrokenBarrierError, Thread

import mavsdk
from pymavlink import mavutil

from .gimbal import Gimbal
from .water_drop import WaterDrop
from ..apriltag.python.apriltag_processor import AprilTagModule
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer
from ..status import Status
from ..thermal import ThermalCamera
from ..vio.vio import VIOModule


class Autonomy:
    def __init__(
            self,
            mqtt_client: MQTTClient,
            status: Status,
            pcc: PeripheralControlComputer,
            thermal: ThermalCamera,
            vio: VIOModule,
            mavlink_system: mavsdk.System,
            mavutil_connection: mavutil.mavudp,
            apriltags: AprilTagModule
    ) -> None:
        self.mqtt_client = mqtt_client
        self.status = status
        self.pcc = pcc
        self.thermal = thermal
        self.vio = vio
        # self.zed = vio.camera.zed
        self.mavlink_system = mavlink_system
        self.mavutil_connection = mavutil_connection
        self.apriltags = apriltags

        self.gimbal = Gimbal(self.pcc, 2, 3, self.thermal)
        self.gimbal_thread = Thread(target=lambda: self.gimbal.run(), daemon=True)

        self.water_drop = WaterDrop(self.pcc, self.apriltags, self.mavlink_system, 1)
        self.water_drop_thread = Thread(target=lambda: asyncio.run(self.water_drop.run()), daemon=True)

        self.running = False
        self.running_barrier = Barrier(2)

        Thread(target=self._status_loop, daemon=True).start()

    def _status_loop(self) -> None:
        self.status.update_status("autonomy", False)
        while self.running:
            states = (
                self.gimbal_thread.is_alive(),
                self.water_drop_thread.is_alive()
            )
            self.status.update_status("autonomy", False not in states)
            is_doing_stuff = (
                self.gimbal.is_doing_stuff,
                self.water_drop.is_doing_stuff
            )
            if False not in is_doing_stuff:
                self.status.led_event("autonomy", (100, 0, 255), 0)
            else:
                self.status.end_led_event("autonomy")
            time.sleep(1)
        self.status.update_status("autonomy", False)
        try:
            self.running_barrier.wait(0)
        except BrokenBarrierError:
            pass

    def close(self) -> None:
        self.gimbal.close()
        self.water_drop.close()
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass

    async def run(self) -> None:
        self.gimbal_thread.start()
        self.water_drop_thread.start()
