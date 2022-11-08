import asyncio
from threading import Thread

import mavsdk
from pymavlink import mavutil
from pyzed import sl

from .gimbal import Gimbal
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer
from ..thermal import ThermalCamera


class Autonomy:
    def __init__(
            self,
            mqtt_client: MQTTClient,
            pcc: PeripheralControlComputer,
            thermal: ThermalCamera,
            zed_camera: sl.Camera,
            mavlink_system: mavsdk.System,
            mavutil_connection: mavutil.mavudp
    ) -> None:
        self.mqtt_client = mqtt_client
        self.pcc = pcc
        self.thermal = thermal
        self.zed_camera = zed_camera
        self.mavlink_system = mavlink_system
        self.mavutil_connection = mavutil_connection

        self.gimbal: Gimbal | None = None

    def close(self) -> None:
        self.gimbal.close()

    async def run(self) -> None:
        self.gimbal = Gimbal(self.pcc, 2, 3, self.thermal)
        Thread(target = lambda: self.gimbal.run(), daemon = True).start()
