import asyncio

import mavsdk
from pymavlink import mavutil

from vmc.mqtt.fcm.fcc_library import FlightControlComputer, PyMAVLinkAgent
from vmc.status import Status


class FlightControlModule:
    def __init__(self, mavlink_system: mavsdk.System, pymavlink_connection: mavutil.mavudp, status: Status) -> None:
        # create the FCC objects
        self.fcc = FlightControlComputer(mavlink_system, status)  # mavlink_system,
        self.gps_fcc = PyMAVLinkAgent(pymavlink_connection)  # pymavlink_connection

    async def run(self) -> None:
        await self.gps_fcc.run()

        await asyncio.gather(self.fcc.run())


if __name__ == "__main__":
    # noinspection PyTypeChecker
    fcm = FlightControlModule(None, None, Status())
    asyncio.run(fcm.run())
