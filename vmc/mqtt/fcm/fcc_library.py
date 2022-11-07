import asyncio
import contextlib
import json
import math
import queue
import time
from typing import Any, Callable, List

import mavsdk
from bell.avr.mqtt.payloads import (
    AvrFcmAttitudeEulerPayload,
    AvrFcmBatteryPayload,
    AvrFcmEventsPayload,
    AvrFcmGpsInfoPayload,
    AvrFcmHilGpsStatsPayload,
    AvrFcmLocationGlobalPayload,
    AvrFcmLocationHomePayload,
    AvrFcmLocationLocalPayload,
    AvrFcmStatusPayload,
    AvrFcmVelocityPayload,
    AvrFusionHilGpsPayload,
)
from bell.avr.utils.decorators import async_try_except, try_except
from bell.avr.utils.timing import rate_limit
from deprecated.classic import deprecated
from loguru import logger
from mavsdk.action import ActionError
from mavsdk.offboard import VelocityBodyYawspeed, VelocityNedYaw
from pymavlink import mavutil

from vmc.mqtt.fcm.fcc_mision_api import MissionAPI
from vmc.mqtt_client import MQTTClient
from vmc.status import Status


class FCMMQTTModule:
    def __init__(self) -> None:
        self.client: MQTTClient | None = None

    @try_except()
    def _publish_event(self, name: str, payload: str = "") -> None:
        """
        Create and publish state machine event.
        """
        event = AvrFcmEventsPayload(
                name = name,
                payload = payload,
        )
        self.client.send_message("avr/fcm/events", event)


class DispatcherBusy(Exception):
    """
    Exception for when the action dispatcher is currently busy
    executing another action
    """


class DispatcherManager(FCMMQTTModule):
    def __init__(self) -> None:
        super().__init__()

        self.client = MQTTClient.get()

        self.currently_running_task = None
        self.timeout = 10

    async def schedule_task(self, task: Callable, payload: Any, name: str) -> None:
        """
        Schedule a task (async func) to be run by the dispatcher with the
        given payload. Task name is also required for printing.
        """
        logger.debug(f"Scheduling a task for '{name}'")
        # if the dispatcher is ok to take on a new task
        if (
                self.currently_running_task is not None
                and self.currently_running_task.done()
        ) or self.currently_running_task is None:
            await self.create_task(task, payload, name)
        else:
            raise DispatcherBusy

    async def create_task(self, task: Callable, payload: dict, name: str) -> None:
        """
        Create a task to be run.
        """
        self.currently_running_task = asyncio.create_task(
                self.task_waiter(task, payload, name)
        )

    async def task_waiter(self, task: Callable, payload: dict, name: str) -> None:
        """
        Execute a task with a timeout.
        """
        # noinspection PyBroadException
        try:
            await asyncio.wait_for(task(**payload), timeout = self.timeout)
            self._publish_event(f"request_{name}_completed_event")
            self.currently_running_task = None

        except asyncio.TimeoutError:
            # noinspection PyBroadException
            try:
                logger.warning(f"Task '{name}' timed out!")
                self._publish_event("action_timeout_event", name)
                self.currently_running_task = None

            except Exception:
                logger.exception("ERROR IN TIMEOUT HANDLER")

        except Exception:
            logger.exception("ERROR IN TASK WAITER")

    @try_except()
    def _publish_event(self, name: str, payload: str = "") -> None:
        """
        Create and publish state machine event.
        """
        event = AvrFcmEventsPayload(
                name = name,
                payload = payload,
        )
        self.client.send_message("avr/fcm/events", event)


class FlightControlComputer(FCMMQTTModule):
    def __init__(self, system: mavsdk.System, status: Status) -> None:
        super().__init__()

        self.client = MQTTClient.get()

        # mavlink stuff
        self.drone = system
        self.mission_api = MissionAPI(self.drone)

        self.status = status

        # queues
        self.action_queue = queue.Queue()
        self.offboard_ned_queue = queue.Queue()
        self.offboard_body_queue = queue.Queue()

        # current state of offboard mode, acts as a backup for PX4
        self.offboard_enabled = False

        # telemetry persistent variables
        self.in_air: bool = False
        self.is_armed: bool = False
        self.fcc_mode = "UNKNOWN"
        self.heading = 0.0

    @deprecated
    async def _connect(self) -> None:
        """
        Connect the Drone object.
        """
        logger.debug("Connecting to the FCC")

        # un-comment to show mavsdk server logging
        # import logging
        # logging.basicConfig(level=logging.DEBUG)

        # mavsdk does not support dns
        await self.drone.connect(system_address = "udp://0.0.0.0:14541")

        logger.success("Connected to the FCC")

    # noinspection PyMethodMayBeStatic
    async def async_queue_action(
            self, queue_: queue.Queue, action: Callable, frequency: int = 10
    ) -> None:
        """
        Creates a while loop that continuously tries to pull a dict from a queue
        and do something with it at a set frequency.

        The given function needs to accept a single argument of the protobuf object
        and be async.

        Setting the frequency to 0 will always run the action.
        """
        last_time = time.time()

        # this particular design will constantly get messages from the queue,
        # even if they are not used, just to try and process them as fast
        # as possible to prevent the queue from filling

        while True:
            # noinspection PyBroadException
            try:
                # get the next item from the queue
                data = queue_.get_nowait()
                # if the frequency is 0, or the time since our last run is greater
                # than the frequency, run
                if frequency == 0 or time.time() - last_time > (1 / frequency):
                    # call function
                    await action(data)
                    # reset timer
                    last_time = time.time()

            except queue.Empty:
                # if the queue was empty, just wait
                await asyncio.sleep(0.01)

            except Exception:
                logger.exception("Unexpected error in async_queue_action")

    async def run(self) -> asyncio.Future:
        """
        Run the Flight Control Computer module
        """
        # start our MQTT client

        # connect to the fcc
        await self._connect()

        # start the mission api MQTT client
        # self.mission_api.run_non_blocking()

        # start tasks
        return asyncio.gather(
                self.telemetry_tasks(),
                # uncomment the following lines to enable outside control
                # self.offboard_tasks(),
                # self.action_dispatcher(),
        )

    # region ###################  T E L E M E T R Y ###########################

    async def telemetry_tasks(self) -> asyncio.Future:
        """
        Gathers the telemetry tasks
        """
        return asyncio.gather(
                self.connected_status_telemetry(),
                # self.battery_telemetry(),
                # self.in_air_telemetry(),
                # self.is_armed_telemetry(),
                # self.flight_mode_telemetry(),
                # self.landed_state_telemetry(),
                # self.position_ned_telemetry(),
                # self.position_lla_telemetry(),
                # self.home_lla_telemetry(),
                # self.attitude_euler_telemetry(),
                # self.velocity_ned_telemetry(),
                # self.gps_info_telemetry(),
        )

    # @async_try_except()
    async def connected_status_telemetry(self) -> None:
        """
        Runs the connected_status telemetry loop
        """
        was_connected = False
        flip_time = time.time()
        debounce_time = 2

        logger.debug("connected_status loop started")
        async for connection_status in self.drone.core.connection_state():
            connected = connection_status.is_connected
            now = time.time()
            should_update = False

            # every time the state changes, record that time
            if connected != was_connected:
                should_update = True
                flip_time = time.time()

            # if the state has been steady for debounce_time
            if (now - flip_time > debounce_time) and should_update:
                self.status.update_status("fcc", connected)
                if connected:
                    self._publish_event("fcc_connected_event")
                else:
                    self._publish_event("fcc_disconnected_event")

            was_connected = connected

    @async_try_except()
    async def battery_telemetry(self) -> None:
        """
        Runs the battery telemetry loop
        """
        logger.debug("battery_telemetry loop started")
        async for battery in self.drone.telemetry.battery():
            update = AvrFcmBatteryPayload(
                    voltage = battery.voltage_v,
                    soc = battery.remaining_percent * 100.0,
            )

            self.client.send_message("avr/fcm/battery", update)

    @async_try_except()
    async def in_air_telemetry(self) -> None:
        """
        Runs the in_air telemetry loop
        """
        logger.debug("in_air loop started")
        async for in_air in self.drone.telemetry.in_air():
            self.in_air = in_air

    @async_try_except()
    async def is_armed_telemetry(self) -> None:
        """
        Runs the is_armed telemetry loop
        """
        was_armed = False
        logger.debug("is_armed loop started")
        async for armed in self.drone.telemetry.armed():

            # if the arming status is different from last time
            if armed != was_armed:
                if armed:
                    self._publish_event("fcc_armed_event")
                else:
                    self._publish_event("fcc_disarmed_event")
            was_armed = armed
            self.is_armed = armed

            update = AvrFcmStatusPayload(
                    armed = armed,
                    mode = str(self.fcc_mode),
            )

            self.client.send_message("avr/fcm/status", update)

    @async_try_except()
    async def landed_state_telemetry(self) -> None:
        """
        Runs the landed state loop, returns one of:
        IN_AIR,LANDING,ON_GROUND,TAKING_OFF, or UNKNOWN
        """
        previous_state = "UNKNOWN"

        async for state in self.drone.telemetry.landed_state():
            mode = str(state)
            # if we have a state change
            if mode != previous_state:
                if mode == "IN_AIR":
                    self._publish_event("landed_state_in_air_event")
                elif mode == "LANDING":
                    self._publish_event("landed_state_landing_event")
                elif mode == "ON_GROUND":
                    self._publish_event("landed_state_on_ground_event")
                elif mode == "TAKING_OFF":
                    self._publish_event("landed_state_taking_off_event")
                elif mode == "UNKNOWN":
                    self._publish_event("landed_state_unknown_event")
            previous_state = mode

    @async_try_except()
    async def flight_mode_telemetry(self) -> None:
        """
        Runs the flight_mode telemetry loop
        """
        # noinspection SpellCheckingInspection
        fcc_mode_map = {
            "UNKNOWN": "fcc_unknown_mode_event",
            "READY": "fcc_ready_mode_event",
            "TAKEOFF": "fcc_takeoff_mode_event",
            "HOLD": "fcc_hold_mode_event",
            "MISSION": "fcc_mission_mode_event",
            "RETURN_TO_LAUNCH": "fcc_rtl_mode_event",
            "LAND": "fcc_land_mode_event",
            "OFFBOARD": "fcc_offboard_mode_event",
            "FOLLOW_ME": "fcc_follow_mode_event",
            "MANUAL": "fcc_manual_mode_event",
            "ALTCTL": "fcc_alt_mode_event",
            "POSCTL": "fcc_pos_mode_event",
            "ACRO": "fcc_acro_mode_event",
            "STABILIZED": "fcc_stabilized_mode_event",
            "RATTITUDE": "fcc_rattitude_mode_event",
        }

        fcc_mode = "UNKNOWN"

        logger.debug("flight_mode_telemetry loop started")

        async for mode in self.drone.telemetry.flight_mode():

            update = AvrFcmStatusPayload(
                    mode = str(mode),
                    armed = self.is_armed,
            )

            self.client.send_message("avr/fcm/status", update)

            if mode != fcc_mode:
                if mode in fcc_mode_map:
                    self._publish_event(fcc_mode_map[str(mode)])
                else:
                    self._publish_event("fcc_mode_error_event")

            fcc_mode = mode
            self.fcc_mode = mode

    @async_try_except()
    async def position_ned_telemetry(self) -> None:
        """
        Runs the position_ned telemetry loop
        """
        logger.debug("position_ned telemetry loop started")
        async for position in self.drone.telemetry.position_velocity_ned():
            n = position.position.north_m
            e = position.position.east_m
            d = position.position.down_m

            update = AvrFcmLocationLocalPayload(dX = n, dY = e, dZ = d)

            self.client.send_message("avr/fcm/location/local", update)

    @async_try_except()
    async def position_lla_telemetry(self) -> None:
        """
        Runs the position_lla telemetry loop
        """
        logger.debug("position_lla telemetry loop started")
        async for position in self.drone.telemetry.position():
            update = AvrFcmLocationGlobalPayload(
                    lat = position.latitude_deg,
                    lon = position.longitude_deg,
                    alt = position.relative_altitude_m,
                    hdg = self.heading,
            )

            self.client.send_message("avr/fcm/location/global", update)

    @async_try_except()
    async def home_lla_telemetry(self) -> None:
        """
        Runs the home_lla telemetry loop
        """
        logger.debug("home_lla telemetry loop started")
        async for home_position in self.drone.telemetry.home():
            update = AvrFcmLocationHomePayload(
                    lat = home_position.latitude_deg,
                    lon = home_position.longitude_deg,
                    alt = home_position.relative_altitude_m,
            )

            self.client.send_message("avr/fcm/location/home", update)

    @async_try_except()
    async def attitude_euler_telemetry(self) -> None:
        """
        Runs the attitude_euler telemetry loop
        """

        logger.debug("attitude_euler telemetry loop started")
        async for attitude in self.drone.telemetry.attitude_euler():
            psi = attitude.roll_deg
            theta = attitude.pitch_deg
            phi = attitude.yaw_deg

            # do any necessary wrapping here
            update = AvrFcmAttitudeEulerPayload(
                    roll = psi,
                    pitch = theta,
                    yaw = phi,
            )

            heading = (2 * math.pi) + phi if phi < 0 else phi
            heading = math.degrees(heading)

            self.heading = heading

            # publish telemetry every tenth of a second
            rate_limit(
                    lambda: self.client.send_message("avr/fcm/attitude/euler", update),
                    frequency = 10,
            )

    @async_try_except()
    async def velocity_ned_telemetry(self) -> None:
        """
        Runs the velocity_ned telemetry loop
        """

        logger.debug("velocity_ned telemetry loop started")
        async for velocity in self.drone.telemetry.velocity_ned():
            update = AvrFcmVelocityPayload(
                    vX = velocity.north_m_s,
                    vY = velocity.east_m_s,
                    vZ = velocity.down_m_s,
            )

            self.client.send_message("avr/fcm/velocity", update)

    @async_try_except()
    async def gps_info_telemetry(self) -> None:
        """
        Runs the gps_info telemetry loop
        """
        logger.debug("gps_info telemetry loop started")
        async for gps_info in self.drone.telemetry.gps_info():
            update = AvrFcmGpsInfoPayload(
                    num_satellites = gps_info.num_satellites,
                    fix_type = str(gps_info.fix_type),
            )

            self.client.send_message("avr/fcm/gps_info", update)

    # endregion ###############################################################

    # region ################## D I S P A T C H E R  ##########################

    @async_try_except()
    async def action_dispatcher(self) -> None:
        logger.debug("action_dispatcher started")

        action_map = {
            "break": self.set_intentional_timeout,
            # "connect": self.connect,
            "arm": self.set_arm,
            "disarm": self.set_disarm,
            "kill": self.set_kill,
            "land": self.set_land,
            "reboot": self.set_reboot,
            "takeoff": self.set_takeoff,
            "offboard_start": self.offboard_start,
            "offboard_stop": self.offboard_stop,
            "upload_mission": self.upload_mission,
            "begin_mission": self.begin_mission,
            "pause_mission": self.pause_mission,
            "resume_mission": self.resume_mission,
        }

        dispatcher = DispatcherManager()

        while True:
            action = {}
            # noinspection PyBroadException
            try:
                action = self.action_queue.get_nowait()

                if action["payload"] == "":
                    action["payload"] = "{}"

                if action["name"] in action_map:
                    payload = json.loads(action["payload"])
                    await dispatcher.schedule_task(
                            action_map[action["name"]], payload, action["name"]
                    )
                else:
                    logger.warning(f"Unknown action: {action['name']}")

            except DispatcherBusy:
                logger.info("I'm busy running another task, try again later")
                self._publish_event("fcc_busy_event", payload = action["name"])

            except queue.Empty:
                await asyncio.sleep(0.1)

            except Exception:
                logger.exception("ERROR IN MAIN LOOP")

    async def simple_action_executor(
            self,
            action_fn: Callable,
            action_text: str,
    ) -> None:
        """
        Executes a given async action function, and publishes a success or failed
        state machine event given whether an `ActionError` was raised.
        """
        try:
            await action_fn()
            full_success_str = f"{action_text}_success_event"
            logger.info(f"Sending {full_success_str}")
            self._publish_event(full_success_str)

        except ActionError as e:
            full_fail_str = f"{action_text}_failed_event"
            logger.info(f"Sending {full_fail_str}")
            self._publish_event(full_fail_str)

            # noinspection PyProtectedMember
            if e._result.result_str == "CONNECTION_ERROR":
                asyncio.create_task(self._connect())

            raise e from e

    # endregion ###############################################################

    # region #####################  A C T I O N S #############################

    # noinspection PyUnusedLocal
    @async_try_except()
    async def set_intentional_timeout(self, **kwargs) -> None:
        """
        Sets a 20-second timeout.
        """
        with contextlib.suppress(asyncio.CancelledError):
            await asyncio.sleep(20)

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_arm(self, **kwargs) -> None:
        """
        Sets the drone to an armed state.
        """
        logger.info("Sending arm command")
        await self.simple_action_executor(self.drone.action.arm, "arm")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_disarm(self, **kwargs) -> None:
        """
        Sets the drone to a disarmed state.
        """
        logger.info("Sending disarm command")
        await self.simple_action_executor(self.drone.action.disarm, "disarm")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_kill(self, **kwargs) -> None:
        """
        Sets the drone to a kill state. This will forcefully shut off the drone
        regardless of being in the air or not.
        """
        logger.warning("Sending kill command")
        await self.simple_action_executor(self.drone.action.kill, "kill")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_land(self, **kwargs) -> None:
        """
        Commands the drone to land at the current position.
        """
        logger.info("Sending land command")
        await self.simple_action_executor(self.drone.action.land, "land_cmd")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_reboot(self, **kwargs) -> None:
        """
        Commands the drone computer to reboot.
        """
        logger.warning("Sending reboot command")
        await self.simple_action_executor(self.drone.action.reboot, "reboot")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def set_takeoff(self, takeoff_alt: float, **kwargs) -> None:
        """
        Commands the drone to take off to the given altitude.
        Will arm the drone if it is not already.
        """
        logger.info(f"Setting takeoff altitude to {takeoff_alt}")
        await self.drone.action.set_takeoff_altitude(takeoff_alt)
        await self.set_arm()
        logger.info("Sending takeoff command")
        await self.simple_action_executor(self.drone.action.takeoff, "takeoff")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def upload_mission(self, waypoints: List[dict], **kwargs) -> None:
        """
        Calls the mission api to upload a mission to the fcc.
        """
        logger.info("Starting mission upload process")
        await self.mission_api.build_and_upload(waypoints)

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def begin_mission(self, **kwargs) -> None:
        """
        Arms the drone and calls the mission api to start a mission.
        """
        logger.info("Arming the drone")
        await self.set_arm()
        # we shouldn't have to check the armed status because
        # the arm fn should raise an exception if it is unsuccessful
        logger.info("Starting the mission")
        await self.mission_api.start()
        if self.in_air:
            self._publish_event("mission_starting_from_air_event")

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def pause_mission(self, **kwargs) -> None:
        """
        Calls the mission api to pause a mission to the fcc.
        """
        logger.info("Starting mission upload process")
        await self.mission_api.pause()

    # noinspection PyUnusedLocal
    @async_try_except(reraise = True)
    async def resume_mission(self, **kwargs) -> None:
        """
        Calls the mission api to resume a mission to the fcc.
        """
        logger.info("Resuming Mission")
        await self.mission_api.resume()

    # endregion ###############################################################

    # region ##################### O F F B O A R D ############################

    async def offboard_tasks(self) -> asyncio.Future:
        """
        Gathers the offboard tasks
        """
        return asyncio.gather(self.offboard_ned(), self.offboard_body())

    # noinspection PyUnusedLocal
    async def offboard_start(self, **kwargs) -> None:
        """
        Starts offboard mode on the drone. Use with caution!
        """
        logger.info("Starting offboard mode")
        await self.drone.offboard.start()
        self.offboard_enabled = True

    # noinspection PyUnusedLocal
    async def offboard_stop(self, **kwargs) -> None:
        """
        Stops offboard mode on the drone.
        """
        logger.info("Stopping offboard mode")
        self.offboard_enabled = False
        await self.drone.offboard.stop()

    async def offboard_ned(self) -> None:
        """
        Feeds offboard NED data to the drone.
        """
        logger.debug("offboard_ned loop started")

        @async_try_except()
        async def process_offboard_ned(msg: dict) -> None:
            # if not currently in offboard mode, skip
            if not self.offboard_enabled:
                return

            north = msg["north"]
            east = msg["east"]
            down = msg["down"]
            yaw = msg["yaw"]
            await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(north, east, down, yaw)
            )

        await self.async_queue_action(
                self.offboard_ned_queue,
                process_offboard_ned,
                frequency = 20,
        )

    async def offboard_body(self) -> None:
        """
        Feeds offboard body data to the drone.
        """
        logger.debug("offboard_body loop started")

        @async_try_except()
        async def process_offboard_body(msg: dict) -> None:
            # if not currently in offboard mode, skip
            if not self.offboard_enabled:
                return

            forward = msg["forward"]
            right = msg["right"]
            down = msg["down"]
            yaw = msg["yaw"]
            # noinspection PyTypeChecker
            await self.drone.offboard.set_velocity_ned(
                    VelocityBodyYawspeed(forward, right, down, yaw)
            )

        await self.async_queue_action(
                self.offboard_body_queue,
                process_offboard_body,
                frequency = 20,
        )

    # endregion ###############################################################


class PyMAVLinkAgent:
    def __init__(self, connection: mavutil.mavudp) -> None:
        self.mavlink_connection: mavutil.mavudp = connection

        self.client = MQTTClient.get()
        # self.client.register_callback("avr/fusion/hil_gps", self.hilgps_msg_handler)

        self.num_frames = 0

    @try_except()
    async def run(self) -> None:
        """
        Set up a mavlink connection and kick off any tasks
        """

        # this NEEDS to be using UDP, TCP proved extremely unreliable
        # self.mavlink_connection = mavutil.mavlink_connection(
        #         "udpin:0.0.0.0:14542",
        #         source_system = 142,
        #         dialect = "bell"
        # )

        logger.debug("Waiting for Mavlink heartbeat")
        self.mavlink_connection.wait_heartbeat()
        logger.success("Mavlink heartbeat received")

    def reboot(self) -> None:
        self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                1, 1, 1, 0, 0, 0, 0, -1
        )

    def shutdown(self) -> None:
        self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                2, 2, 2, 0, 0, 0, 0, -1
        )

    @try_except(reraise = True)
    def hilgps_msg_handler(self, payload: AvrFusionHilGpsPayload) -> None:
        """
        Handle a HIL_GPS message.
        """
        msg = self.mavlink_connection.mav.hil_gps_heading_encode(  # type: ignore
                payload["time_usec"],
                payload["fix_type"],
                payload["lat"],
                payload["lon"],
                payload["alt"],
                payload["eph"],
                payload["epv"],
                payload["vel"],
                payload["vn"],
                payload["ve"],
                payload["vd"],
                payload["cog"],
                payload["satellites_visible"],
                payload["heading"],
        )
        # logger.debug(msg)
        self.mavlink_connection.mav.send(msg)  # type: ignore
        self.num_frames += 1

        # publish stats every second
        rate_limit(
                lambda: self.client.send_message("avr/fcm/hil_gps_stats",
                                                 AvrFcmHilGpsStatsPayload(num_frames = self.num_frames)),
                frequency = 1,
        )
