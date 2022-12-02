import time
from threading import Barrier, BrokenBarrierError

from loguru import logger

from .. import utils
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer
from ..thermal import ThermalCamera

SERVO_RANGE = 180
CAMERA_FOV = 80
FULL_RANGE = SERVO_RANGE + CAMERA_FOV

X_LIMITS = (500, 2500)
X_SOFT_LIMIT = (0, 180)
X_CENTER = 90

Y_LIMITS = (500, 2500)
Y_SOFT_LIMIT = (50, 180)
Y_CENTER = 90

X_OFFSET = 7
Y_OFFSET = -4


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

        self.client.register_callback("avr/gimbal/disable", lambda: self.disable(), is_json=False, use_args=False)
        self.client.register_callback("avr/gimbal/center", lambda: self.center(), is_json=False, use_args=False)
        self.client.register_callback(
                "avr/gimbal/pos",
                lambda payload: self.set_pos(payload.get("x", X_CENTER), payload.get("y", Y_CENTER)),
                is_json=True,
                use_args=True
        )
        self.client.register_callback(
                "avr/gimbal/move",
                lambda payload: self.move(payload.get("x", 0), payload.get("y", 0)),
                is_json=True,
                use_args=True
        )
        self.client.register_callback(
                "avr/gimbal/trigger_aim",
                lambda: self.trigger_auto_aim(),
                is_json=False,
                use_args=False
        )
        self.client.register_callback(
                "avr/gimbal/auto_aim",
                lambda payload: self.set_auto_aim(payload.get("enabled", False)),
                is_json=True,
                use_args=True
        )

        self.running = False
        self.running_barrier = Barrier(2)

    @property
    def is_doing_stuff(self) -> bool:
        return self.auto_aim_enabled

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass
        self.disable()

    def disable(self) -> None:
        self.pcc.disable_servo(self.x_servo)
        self.pcc.disable_servo(self.y_servo)

    def center(self) -> None:
        self.set_pos(X_CENTER, Y_CENTER)

    def send_update(self) -> None:
        self.client.send_message(
                "avr/gimbal/response_pos",
                {
                    "x": self.last_x,
                    "y": utils.map(self.last_y, 0, 180, Y_SOFT_LIMIT[0], Y_SOFT_LIMIT[1])
                }
        )

    def set_x(self, x: int) -> None:
        if X_SOFT_LIMIT[0] <= x <= X_SOFT_LIMIT[1]:
            self.last_x = x
            x = int(utils.map(x, 0, SERVO_RANGE, X_LIMITS[0], X_LIMITS[1]))
            self.pcc.set_servo_abs(self.x_servo, x)
        self.send_update()

    def set_y(self, y: int) -> None:
        if Y_SOFT_LIMIT[0] <= y <= Y_SOFT_LIMIT[1]:
            self.last_y = y
            y = int(utils.map(y, 0, SERVO_RANGE, Y_LIMITS[0], Y_LIMITS[1]))
            self.pcc.set_servo_abs(self.y_servo, y)
        self.send_update()

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

    def zmq_pos(self, message: dict):
        try:
            x = message.get("x", 0)
            y = message.get("y", 0)
            self.set_pos(x, y)
        except AttributeError:
            logger.warning("Got invalid message for zmq_pos")

    def zmq_move(self, message: dict):
        try:
            x = message.get("x", 0)
            y = message.get("y", 0)
            self.move(x, y)
        except AttributeError:
            logger.warning("Got invalid message for zmq_move")

    def zmq_disable(self, _: str):
        self.disable()

    def zmq_auto(self, message: dict):
        try:
            enabled = message.get("enabled", False)
            self.set_auto_aim(enabled)
        except AttributeError:
            logger.warning("Got invalid message for zmq_auto")

    def zmq_fire(self, _: str):
        self.pcc.fire_laser()

    def run(self) -> None:
        logger.info("Gimbal started")
        self.running = True
        self.center()
        aimed_last = None
        while self.running:
            if self.auto_aim_enabled or self.do_single_aim:
                if not aimed_last:
                    self.pcc.set_onboard_base_color((100, 150, 50, 0))
                self.do_single_aim = False
                aimed_last = True
                try:
                    self.thermal.update_barrier.wait(2)
                except BrokenBarrierError:
                    time.sleep(1)
                    logger.warning("Thermal camera is taking too long to respond")
                    continue
                if self.thermal.detector.currently_detecting:
                    pass
                    detection = self.thermal.detector.main_detection_center

                    # print("Detection: " + str(detection))
                    x_degrees = utils.map(detection[0], 0, 30, 0, CAMERA_FOV)
                    # print("1: " + str(x_degrees))
                    x_degrees = utils.map(x_degrees, 0, 80, -40, 40) + X_OFFSET
                    # print("2: " + str(x_degrees))
                    x_degrees = int(utils.constrain(x_degrees, -40, 40))
                    # print("3: " + str(x_degrees))
                    # x_degrees = utils.deadzone(int(x_degrees), 10)
                    x_degrees = utils.deadzone(x_degrees, 10)
                    x_degrees_half = int(x_degrees / 2)
                    # x_degrees_half = x_degrees
                    # camera_y = utils.deadzone(int(utils.map(detection[1], 0, 30, 0, CAMERA_FOV) - 15), 10)

                    y_degrees = utils.map(detection[1], 0, 30, 0, CAMERA_FOV)
                    y_degrees = utils.map(y_degrees, 0, 80, -40, 40) + Y_OFFSET
                    y_degrees = int(utils.constrain(y_degrees, -40, 40))
                    # y_degrees = utils.deadzone(int(y_degrees), 10)
                    y_degrees = utils.deadzone(y_degrees, 10)
                    y_degrees_half = int(y_degrees / 2)
                    # y_degrees_half = y_degrees

                    # print("Move amount: " + str(x_degrees_half) + ", " + str(y_degrees_half))# + ", " + str(camera_y))
                    self.move(x_degrees_half, y_degrees_half)
            else:
                if aimed_last:
                    self.pcc.set_onboard_base_color((0, 0, 0, 0))
                    self.center()
                aimed_last = False
            time.sleep(1 / 5)
            try:
                self.running_barrier.wait(0)
            except BrokenBarrierError:
                pass


if __name__ == '__main__':
    test_pcc = PeripheralControlComputer()
    test_pcc.begin()
    test_pcc.begin_mqtt()

    thermal_cam = ThermalCamera()

    gimbal = Gimbal(test_pcc, 2, 3, thermal_cam)

    gimbal.client.connect()

    gimbal.run()
