import asyncio
from threading import BrokenBarrierError

from vmc import utils
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.thermal import ThermalCamera

from loguru import logger

SERVO_RANGE = 180
CAMERA_FOV = 80
FULL_RANGE = SERVO_RANGE + CAMERA_FOV

X_LIMITS = (800, 1500)
X_CENTER = 50

Y_LIMITS = (500, 2500)
Y_CENTER = 50


class Gimbal:
    def __init__(self, pcc: PeripheralControlComputer, x_servo: int, y_servo: int, thermal: ThermalCamera) -> None:
        self.client: MQTTClient = MQTTClient.get()

        self.pcc = pcc
        self.x_servo = x_servo
        self.y_servo = y_servo
        self.thermal = thermal

        # self.pcc.set_servo_min(self.x_servo, X_LIMITS[0])
        # self.pcc.set_servo_max(self.x_servo, X_LIMITS[1])
        #
        # self.pcc.set_servo_min(self.y_servo, Y_LIMITS[0])
        # self.pcc.set_servo_max(self.y_servo, Y_LIMITS[1])

        self.last_x: int | None = None
        self.last_y: int | None = None

    async def disable(self) -> None:
        self.pcc.disable_servo(self.x_servo)
        self.pcc.disable_servo(self.y_servo)

    async def center(self) -> None:
        await self.set_pos(X_CENTER, Y_CENTER)

    async def set_x(self, x: int) -> None:
        self.pcc.set_servo_pct(self.x_servo, x)
        self.last_x = x

    async def set_y(self, y: int) -> None:
        self.pcc.set_servo_pct(self.y_servo, y)
        self.last_y = y

    async def set_pos(self, x: int, y: int) -> None:
        await self.set_x(x)
        await self.set_y(y)

    async def move_x(self, x: int):
        x = int(utils.constrain(utils.map(self.last_x + x, 0, SERVO_RANGE, 0, 100), 0, SERVO_RANGE))
        await self.set_x(x)

    async def move_y(self, y: int):
        y = int(utils.constrain(utils.map(self.last_y + y, 0, SERVO_RANGE, 0, 100), 0, SERVO_RANGE))
        await self.set_y(y)

    async def move(self, x: int, y: int):
        await self.move_x(x)
        await self.move_y(y)

    async def run(self) -> None:
        logger.info("Gimbal started")
        await self.center()
        last_pos = False
        while True:
            try:
                self.thermal.update_barrier.wait(5000)
            except BrokenBarrierError:
                await asyncio.sleep(2)
                continue
            if self.thermal.detector.currently_detecting:
                detection = self.thermal.detector.main_detection_center
                if detection == last_pos:
                    continue

                print("Detection: " + str(detection[0]))
                camera_x = int(utils.map(detection[0], 0, 30, 0, CAMERA_FOV) - 15)

                print("Move amount: " + str(camera_x))
                await self.move_x(camera_x)

                await asyncio.sleep(2)


if __name__ == '__main__':
    test_pcc = PeripheralControlComputer()
    test_pcc.begin()
    test_pcc.begin_mqtt()

    thermal_cam = ThermalCamera()

    gimbal = Gimbal(test_pcc, 2, 3, thermal_cam)

    gimbal.client.connect()
