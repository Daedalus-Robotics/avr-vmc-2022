import json
import time
from threading import Barrier, BrokenBarrierError, Thread

import mavsdk
from bell.avr.mqtt.payloads import AvrApriltagsVisibleTags
from loguru import logger

from ..apriltag.python.apriltag_processor import AprilTagModule
from ..mqtt_client import MQTTClient
from ..pcc import PeripheralControlComputer


class WaterDrop:
    def __init__(
            self,
            pcc: PeripheralControlComputer,
            apriltags: AprilTagModule,
            mavlink_system: mavsdk.System,
            servo_num: int
    ) -> None:
        self.client = MQTTClient.get()

        self.pcc = pcc
        self.apriltags = apriltags
        self.mavlink_system = mavlink_system
        self.water_drop_servo = servo_num

        self.is_dropping = False
        self.stop_autonomy = False
        self.dropping_tag: AvrApriltagsVisibleTags | None = None

        self.running = False
        self.running_barrier = Barrier(2)

        # self.do_off_en = False
        # self.do_off_dis = False
        # self.do_go = False
        # self.do_go_cords = (0, 0, 0, 0)

        # self.client.register_callback("off_en", self.mqtt_off_en, False, False)
        # self.client.register_callback("off_dis", self.mqtt_off_dis, False, False)
        # self.client.register_callback("off_go", self.mqtt_go, True, True)
        self.temp_drop_tag = -1
        self.temp_drop_delay = 2

        self.client.register_callback("avr/autonomy/kill", self.kill, False, False)
        self.client.register_callback("avr/autonomy/set_drop_delay", self.set_drop_delay, True, True)
        self.client.register_callback("avr/autonomy/set_drop_tag", self.set_drop_tag, True, True)
        self.client.register_callback("avr/autonomy/set_auto_water_drop", self.set_auto_water_drop, True, True)

    def set_drop_delay(self, number: int | dict | str) -> None:
        if isinstance(number, str):
            number = json.loads(number)
        if isinstance(number, dict):
            number = number.get("num", None)
        if number is not None:
            self.temp_drop_delay = number
            self.client.send_message(
                    "avr/gui/toast",
                    {
                        "text": f"Drop delay set to {number} seconds",
                        "timeout": 2
                    }
            )

    def set_drop_tag(self, number: int | dict | str) -> None:
        if isinstance(number, str):
            number = json.loads(number)
        if isinstance(number, dict):
            number = number.get("id", None)
        if number is not None:
            self.temp_drop_tag = number
            self.client.send_message(
                    "avr/gui/toast",
                    {
                        "text": f"Set drop tag to {number}",
                        "timeout": 1
                    }
            )

    def set_auto_water_drop(self, message: bool | str | dict):
        if isinstance(message, str):
            message = json.loads(message)
        enabled = message.get("enabled", False) if isinstance(message, dict) else message
        self.is_dropping = enabled

    # def mqtt_off_en(self) -> None:
    #     self.do_off_en = True
    #
    # def mqtt_off_dis(self) -> None:
    #     self.do_off_dis = True
    #
    # def mqtt_go(self, message: dict) -> None:
    #     if not isinstance(message, dict):
    #         message = json.loads(message)
    #     n = message.get("n", 0)
    #     e = message.get("e", 0)
    #     d = message.get("d", 0)
    #     y = message.get("y", 0)
    #     self.do_go = True
    #     self.do_go_cords = (n, e, d, y)

    @property
    def is_doing_stuff(self) -> bool:
        return self.is_dropping

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass

    def kill(self, _: str = None) -> None:
        self.is_dropping = False

    def try_drop(self, message: dict) -> None:
        pass
        # if not isinstance(message, dict):
        #     message = json.loads(message)
        # tag_id = message.get("tag", -1)
        # recent_apriltags = self.apriltags.get_valid_apriltags(2)
        # if tag_id in recent_apriltags:
        #     self.client.send_message(
        #             "avr/gui/toast",
        #             {
        #                 "text": f"Starting water drop on tag {tag_id}",
        #                 "timeout": 1
        #             }
        #     )
        #     self.is_dropping = True
        #     self.dropping_tag = recent_apriltags[tag_id]
        # else:
        #     self.client.send_message(
        #             "avr/gui/toast",
        #             {
        #                 "text": f"Tag {tag_id} not in sight!",
        #                 "timeout": 2
        #             }
        #     )

    def run_blink_sequence(self) -> None:
        self.pcc.set_temp_color((255, 200, 255, 0), 0.2)
        time.sleep(0.3)
        self.pcc.set_temp_color((255, 200, 255, 0), 0.2)
        time.sleep(0.3)
        self.pcc.set_temp_color((255, 200, 255, 0), 0.2)

    def do_drop(self) -> None:
        self.set_water_drop(100)
        time.sleep(0.5)
        self.set_water_drop(80)
        time.sleep(0.2)
        self.set_water_drop(100)
        time.sleep(0.6)
        self.set_water_drop(0)

    async def run(self) -> None:
        logger.info("Water Drop started")
        self.running = True
        last_is_dropping = False
        while self.running:
            if self.is_dropping:
                if not last_is_dropping:
                    self.client.send_message(
                            "avr/autonomy/water_drop_state",
                            {
                                "state": "searching"
                            }
                    )
                    self.pcc.set_base_color((100, 0, 50, 200))
                last_is_dropping = True
                # if self.apriltags.closest_tag[0] is not None and time.time() - self.apriltags.closest_tag[1] < 5:
                time_offset = time.time() - self.apriltags.visible_detections[0]
                # print(f"Offset: {time_offset}")
                if time_offset < 5 and len(self.apriltags.visible_detections[1]) > 0:
                    if self.temp_drop_tag != -1:
                        if self.temp_drop_tag not in self.apriltags.visible_detections[1]:
                            continue
                    # tag_id = self.apriltags.closest_tag[0].get("id", -1)
                    tag_id = self.apriltags.visible_detections[1][0].get("id", -1)
                    # print(f"tag id: {tag_id}")
                    if tag_id == -1:
                        continue
                    Thread(target=self.run_blink_sequence, daemon=True).start()
                    logger.info(f"Locked onto tag {tag_id}")
                    self.client.send_message(
                            "avr/autonomy/water_drop_state",
                            {
                                "state": "locked"
                            }
                    )
                    self.client.send_message(
                            "avr/autonomy/water_drop_locked",
                            {
                                "tag": tag_id
                            }
                    )
                    self.client.send_message(
                            "avr/gui/toast",
                            {
                                "text": f"Locked onto tag {tag_id}",
                                "timeout": 2
                            }
                    )
                    time.sleep(0.1)
                    temp_start_time = time.time()
                    while self.is_dropping:
                        tag = self.apriltags.detections.get(tag_id, None)[0]
                        # if tag is None:
                        #     logger.debug(f"Tag {tag_id} not in view")
                        #     continue
                        # pos = tag.get("pos_rel", None)
                        # if pos is None:
                        #     logger.warning(f"Could not get a position for tag {tag_id}")
                        #     continue
                        # x = pos["x"]
                        # y = pos["y"]
                        # z = pos["z"]
                        # logger.info(f"Tracking tag {tag_id} at {pos}")
                        # if x < 0.5 and y < 0.5 and z < 0.5:
                        time_until_drop = int(self.temp_drop_delay - (time.time() - temp_start_time))
                        self.client.send_message(
                                "avr/autonomy/water_drop_countdown",
                                {
                                    "time": time_until_drop
                                }
                        )
                        # self.client.send_message(
                        #         "avr/gui/toast",
                        #         {
                        #             "text": f"Dropping in {time_until_drop}",
                        #             "timeout": 1
                        #         }
                        # )
                        if time.time() - temp_start_time > self.temp_drop_delay:
                            logger.info(f"Dropping on tag {tag_id}")
                            # self.client.send_message(
                            #         "avr/gui/toast",
                            #         {
                            #             "text": f"Dropping on tag {tag_id}",
                            #             "timeout": 2
                            #         }
                            # )
                            self.do_drop()
                            self.is_dropping = False
                        time.sleep(0.1)
                    time.sleep(1)
                    self.client.send_message(
                            "avr/autonomy/water_drop_state",
                            {
                                "state": "inactive"
                            }
                    )
                    self.client.send_message(
                            "avr/autonomy/water_drop_locked",
                            {
                                "tag": None
                            }
                    )
                    self.client.send_message(
                            "avr/autonomy/water_drop_countdown",
                            {
                                "time": None
                            }
                    )
            else:
                if last_is_dropping:
                    self.client.send_message(
                            "avr/autonomy/water_drop_state",
                            {
                                "state": "inactive"
                            }
                    )
                    self.pcc.set_base_color((0, 0, 0, 0))
                last_is_dropping = False
            time.sleep(1 / 10)
            # if self.do_off_en:
            #     logger.info("Enable offboard control")
            #     self.do_off_en = False
            #     await self.mavlink_system.offboard.start()
            # if self.do_off_dis:
            #     logger.info("Disable offboard control")
            #     self.do_off_dis = False
            #     await self.mavlink_system.offboard.stop()
            # if self.do_go:
            #     logger.info(f"Moving to pos: {self.do_go_cords}")
            #     self.do_go = False
            #     pos = mavsdk.system.offboard.PositionNedYaw(
            #             self.do_go_cords[0],
            #             self.do_go_cords[1],
            #             self.do_go_cords[2],
            #             self.do_go_cords[3],
            #     )
            #     await self.mavlink_system.offboard.set_position_ned(pos)
            # if self.is_dropping:
            #     try:
            #         # while self.is_dropping:
            #         #     pass
            #         pos_world = self.dropping_tag["pos_world"]
            #         n = pos_world["x"]
            #         e = pos_world["y"]
            #         d = pos_world["z"]
            #         pos = mavsdk.system.offboard.PositionNedYaw(n, e, d, 0)
            #
            #         await self.mavlink_system.offboard.start()
            #         await self.mavlink_system.offboard.set_position_ned(pos)
            #         time.sleep(2)
            #         await self.mavlink_system.offboard.stop()
            #     finally:
            #         self.is_dropping = False
            #         self.dropping_tag = None
        try:
            self.running_barrier.wait(0)
        except BrokenBarrierError:
            pass

    def set_water_drop(self, percent: int) -> None:
        if 0 <= percent <= 100:
            self.pcc.set_servo_pct(self.water_drop_servo, percent)

    def zmq_set(self, message: dict) -> None:
        try:
            percent = message.get("percent", 0)
            self.set_water_drop(percent)
        except AttributeError:
            logger.warning("Got invalid message for zmq_set")
