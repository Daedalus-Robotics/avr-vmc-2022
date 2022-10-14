import asyncio

from vmc.mqtt.fcm.fcc_library import FlightControlComputer, PyMAVLinkAgent


class FlightControlModule:
    def __init__(self) -> None:
        super().__init__()

        # create the FCC objects
        self.fcc = FlightControlComputer()
        self.gps_fcc = PyMAVLinkAgent()

    async def run(self) -> None:
        await self.gps_fcc.run()

        await asyncio.gather(self.fcc.run())


if __name__ == "__main__":
    fcm = FlightControlModule()
    asyncio.run(fcm.run())
