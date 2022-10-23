from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer

X_LIMITS = (800, 1500)
X_CENTER = 50

Y_LIMITS = (500, 2500)
Y_CENTER = 50

class Gimbal:
    def __init__(self, pcc: PeripheralControlComputer, x_servo: int, y_servo: int) -> None:
        self.client: MQTTClient = MQTTClient.get()

        self.pcc = pcc
        self.x_servo = x_servo
        self.y_servo = y_servo

        self.pcc.set_servo_min(self.x_servo, X_LIMITS[0])
        self.pcc.set_servo_max(self.x_servo, X_LIMITS[1])

        self.pcc.set_servo_min(self.y_servo, Y_LIMITS[0])
        self.pcc.set_servo_max(self.y_servo, Y_LIMITS[1])

    async def disable(self) -> None:
        self.pcc.disable_servo(self.x_servo)
        self.pcc.disable_servo(self.y_servo)

    async def center(self) -> None:
        await self.set_pos(X_CENTER, Y_CENTER)

    async def set_x(self, x: int) -> None:
        self.pcc.set_servo_pct(self.x_servo, x)

    async def set_y(self, y: int) -> None:
        self.pcc.set_servo_pct(self.y_servo, y)

    async def set_pos(self, x: int, y: int) -> None:
        await self.set_x(x)
        await self.set_y(y)

if __name__ == '__main__':
    test_pcc = PeripheralControlComputer()
    test_pcc.begin()
    test_pcc.begin_mqtt()

    gimbal = Gimbal(test_pcc, 2, 3)

    gimbal.client.connect()
