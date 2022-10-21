from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer

X_LIMITS = (800, 1500)

class Gimbal:
    def __init__(self, pcc: PeripheralControlComputer, x_servo: int, y_servo: int) -> None:
        self.client: MQTTClient = MQTTClient.get()

        self.pcc = pcc
        self.x_servo = x_servo
        self.y_servo = y_servo

    async def set_x(self, x: int):
        self.pcc.set_servo_pct(self.x_servo, x)

    async def set_y(self, y: int):
        self.pcc.set_servo_pct(self.y_servo, y)

    async def set_pos(self, x: int, y: int):
        await self.set_x(x)
        await self.set_y(y)

if __name__ == '__main__':
    test_pcc = PeripheralControlComputer()
    test_pcc.begin()
    test_pcc.begin_mqtt()

    gimbal = Gimbal(test_pcc, 2, 3)

    gimbal.client.connect()
