import time
from threading import BrokenBarrierError

from .. import utils
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer
from ..thermal import ThermalCamera

from loguru import logger

SERVO_RANGE = 180
CAMERA_FOV = 80
FULL_RANGE = SERVO_RANGE + CAMERA_FOV

X_LIMITS = (500, 2500)
X_SOFT_LIMIT = (0, 180)
X_CENTER = 90

Y_LIMITS = (500, 2500)
Y_SOFT_LIMIT = (50, 180)
Y_CENTER = 90


class Gimbal:
    def __init__(self, pcc: PeripheralControlComputer, x_servo: int, y_servo: int, thermal: ThermalCamera) -> None:
        self.client: MQTTClient = MQTTClient.get()

        self.pcc = pcc
        self.x_servo = x_servo
        self.y_servo = y_servo
        self.thermal = thermal

        self.pcc.set_servo_min(self.x_servo, X_LIMITS[0])
        self.pcc.set_servo_max(self.x_servo, X_LIMITS[1])

        self.pcc.set_servo_min(self.y_servo, Y_LIMITS[0])
        self.pcc.set_servo_max(self.y_servo, Y_LIMITS[1])

        self.auto_aim_enabled = False
        self.do_single_aim = False
        self.last_x: int | None = None
        self.last_y: int | None = None

        self.client.register_callback("avr/gimbal/disable", lambda: self.disable(), is_json = False, use_args = False)
        self.client.register_callback("avr/gimbal/center", lambda: self.center(), is_json = False, use_args = False)
        self.client.register_callback(
                "avr/gimbal/pos",
                lambda payload: self.set_pos(payload.get("x", X_CENTER), payload.get("y", Y_CENTER)),
                is_json = True,
                use_args = True
        )
        self.client.register_callback(
                "avr/gimbal/move",
                lambda payload: self.move(payload.get("x", 0), payload.get("y", 0)),
                is_json = True,
                use_args = True
        )
        self.client.register_callback(
                "avr/gimbal/trigger_aim",
                lambda: self.trigger_auto_aim(),
                is_json = False,
                use_args = False
        )
        self.client.register_callback(
                "avr/gimbal/auto_aim",
                lambda payload: self.set_auto_aim(payload.get("enabled", False)),
                is_json = True,
                use_args = True
        )

    def disable(self) -> None:
        self.pcc.disable_servo(self.x_servo)
        self.pcc.disable_servo(self.y_servo)

    def center(self) -> None:
        self.set_pos(X_CENTER, Y_CENTER)

    def set_x(self, x: int) -> None:
        if X_SOFT_LIMIT[0] <= x <= X_SOFT_LIMIT[1]:
            self.last_x = x
            x = int(utils.map(x, 0, SERVO_RANGE, 0, 100))
            self.pcc.set_servo_pct(self.x_servo, x)

    def set_y(self, y: int) -> None:
        if Y_SOFT_LIMIT[0] <= y <= Y_SOFT_LIMIT[1]:
            self.last_y = y
            y = int(utils.map(y, 0, SERVO_RANGE, 0, 100))
            self.pcc.set_servo_pct(self.y_servo, y)

    def set_pos(self, x: int, y: int) -> None:
        self.set_x(x)
        self.set_y(y)

    def move_x(self, x: int):
        x = int(utils.constrain(self.last_x + x, 0, SERVO_RANGE))
        # print("last_x: " + str(self.last_x))
        # print("degrees: " + str(x))
        self.set_x(x)

    def move_y(self, y: int):
        y = int(utils.constrain(self.last_y + y, 0, SERVO_RANGE))
        # print("last_y: " + str(self.last_y))
        # print("degrees: " + str(y))
        self.set_y(y)

    def move(self, x: int, y: int):
        self.move_x(x)
        self.move_y(y)

    def trigger_auto_aim(self) -> None:
        self.do_single_aim = True

    def set_auto_aim(self, enabled: bool) -> None:
        self.auto_aim_enabled = enabled

    def run(self) -> None:
        logger.info("Gimbal started")
        self.center()
        aimed_last = None
        while True:
            if self.auto_aim_enabled or self.do_single_aim:
                self.do_single_aim = False
                aimed_last = True
                try:
                    self.thermal.update_barrier.wait(2000)
                except BrokenBarrierError:
                    time.sleep(1)
                    continue
                if self.thermal.detector.currently_detecting:
                    detection = self.thermal.detector.main_detection_center

                    # print("Detection: " + str(detection))
                    x_degrees = utils.map(detection[0], 0, 30, 0, CAMERA_FOV)
                    # print("1: " + str(x_degrees))
                    x_degrees = utils.map(x_degrees, 0, 80, -40, 40)
                    # print("2: " + str(x_degrees))
                    x_degrees = int(utils.constrain(x_degrees, -40, 40))
                    # print("3: " + str(x_degrees))
                    # x_degrees = utils.deadzone(int(x_degrees), 10)
                    x_degrees = utils.deadzone(x_degrees, 10)
                    x_degrees_half = int(x_degrees / 2)
                    # x_degrees_half = x_degrees
                    # camera_y = utils.deadzone(int(utils.map(detection[1], 0, 30, 0, CAMERA_FOV) - 15), 10)

                    y_degrees = utils.map(detection[1], 0, 30, 0, CAMERA_FOV)
                    y_degrees = utils.map(y_degrees, 0, 80, -40, 40)
                    y_degrees = int(utils.constrain(y_degrees, -40, 40))
                    # y_degrees = utils.deadzone(int(y_degrees), 10)
                    y_degrees = utils.deadzone(y_degrees, 10)
                    y_degrees_half = int(y_degrees / 2)
                    # y_degrees_half = y_degrees

                    # print("Move amount: " + str(x_degrees_half) + ", " + str(y_degrees_half))# + ", " + str(camera_y))
                    self.move(x_degrees_half, y_degrees_half)

                    time.sleep(1 / 2)
            else:
                if aimed_last:
                    self.center()
                aimed_last = False


if __name__ == '__main__':
    test_pcc = PeripheralControlComputer()
    test_pcc.begin()
    test_pcc.begin_mqtt()

    thermal_cam = ThermalCamera()

    gimbal = Gimbal(test_pcc, 2, 3, thermal_cam)

    gimbal.client.connect()

    gimbal.run()
