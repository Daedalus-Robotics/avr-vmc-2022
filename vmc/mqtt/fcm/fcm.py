import asyncio

from vmc.mqtt.fcm.fcc_library import FlightControlComputer, PyMAVLinkAgent
from vmc.status import Status


class FlightControlModule:
    def __init__(self, status: Status) -> None:
        super().__init__()

        # create the FCC objects
        self.fcc = FlightControlComputer(status)
        self.gps_fcc = PyMAVLinkAgent()

    async def run(self) -> None:
        await self.gps_fcc.run()

        await asyncio.gather(self.fcc.run())


if __name__ == "__main__":
    fcm = FlightControlModule()
    asyncio.run(fcm.run())
