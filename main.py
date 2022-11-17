import argparse
import asyncio
import atexit
import os
import subprocess
import time
from threading import Thread

import mavsdk
from loguru import logger
from pymavlink import mavutil
from systemctl import Service, ServiceState

from vmc.apriltag.python.apriltag_processor import AprilTagModule
from vmc.autonomy.autonomy import Autonomy
from vmc.frame_server import FrameServer
from vmc.fcm.fcm import FlightControlModule
from vmc.fusion import FusionModule
from vmc.vio.vio import VIOModule
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.status_led import StatusStrip
from vmc.thermal import ThermalCamera
from vmc.zmq_server import ZMQServer

logger.level("SETUP", no = 50, color = "<magenta><bold><italic>", icon = "⚙️")

main_thread: Thread | None = None
# status_thread: Thread | None = None
has_started = False

mqtt_client = MQTTClient.get("localhost", 1883)
status_strip = StatusStrip(8)
status_strip.pixels.brightness = 0.5
status = Status(status_strip)
status.register_status("mqtt", False, None, led_num = 0)
mqtt_client.status = status

pcc: PeripheralControlComputer | None = None
zmq_server: ZMQServer | None = None
mavp2p: Service | None = None
thermal: ThermalCamera | None = None
frame_server: FrameServer | None = None
vio: VIOModule | None = None
mavlink_system: mavsdk.System | None = None
pymavlink_connection: mavutil.mavudp | None = None
fcm: FlightControlModule | None = None
fusion: FusionModule | None = None
apriltag: AprilTagModule | None = None
autonomy: Autonomy | None = None


def set_armed(state: bool | dict):
    if isinstance(state, dict):
        arm = state.get("arm", None)
    else:
        arm = state
    if arm is not None:
        if arm:
            pymavlink_connection.arducopter_arm()
        else:
            pymavlink_connection.arducopter_disarm()


@atexit.register
def stop() -> None:
    global has_started
    if has_started:
        if zmq_server is not None:
            logger.log("SETUP", "Stopping zmq server...")
            zmq_server.close()

        if autonomy is not None:
            logger.log("SETUP", "Stopping autonomy...")
            autonomy.close()

        if thermal is not None:
            logger.log("SETUP", "Stopping thermal camera...")
            thermal.close()

        if pcc is not None:
            logger.log("SETUP", "Stopping pcc...")
            pcc.color_wipe(10)
            pcc.end()
            status.update_status("pcc", False)

        if apriltag is not None:
            logger.log("SETUP", "Stopping apriltag...")
            apriltag.close()

        if fusion is not None:
            logger.log("SETUP", "Stopping fusion...")
            fusion.close()

        if vio is not None:
            logger.log("SETUP", "Stopping vio...")
            vio.close()

        if fcm is not None:
            logger.log("SETUP", "Stopping fcm...")
            fcm.fcc.close()

        if pymavlink_connection is not None:
            logger.log("SETUP", "Stopping pymavlink...")
            pymavlink_connection.close()
            status.update_status("fcc", False)

        if mavp2p is not None:
            status.update_status("mavp2p", False)

        status.update_status("vmc", False)
        status.update_status("mqtt", False)
        mqtt_client.disconnect()
        logger.log("SETUP", "Done stopping!")
        has_started = False


def restart_vmc() -> None:
    logger.info("Restarting vmc...")
    stop()
    if fcm is not None:
        fcm.gps_fcc.shutdown()
    time.sleep(2)
    subprocess.Popen(["sudo", "reboot"])


def shutdown_vmc() -> None:
    logger.info("Shutting down vmc...")
    stop()
    if fcm is not None:
        fcm.gps_fcc.shutdown()
    time.sleep(2)
    subprocess.Popen(["sudo", "shutdown", "now"])


async def main(start_modules: list[str]) -> None:
    global has_started
    global mqtt_client, status
    global zmq_server
    global pcc, thermal, frame_server
    global mavp2p
    global vio
    global mavlink_system, pymavlink_connection
    global fcm, fusion
    global apriltag
    global autonomy

    has_started = True
    mqtt_client.connect()

    status.register_status("vmc", True, restart_vmc)
    mqtt_client.register_callback(
            "avr/shutdown",
            shutdown_vmc,
            is_json = False,
            use_args = False
    )

    if "pcc" in start_modules:
        logger.log("SETUP", "Setting up pcc...")
        pcc = PeripheralControlComputer()
        status.register_status("pcc", pcc.is_connected, pcc.reset, 1)
        pcc.on_state = lambda state: status.update_status("pcc", state)

        logger.log("SETUP", "Starting pcc...")
        pcc.begin()
        pcc.begin_mqtt()

        # ToDo: move this to a reasonable spot
        pcc.set_servo_max(1, 1000)

    if ("mavsdk" in start_modules) or ("mavutil" in start_modules):
        logger.log("SETUP", "Setting up mavP2P...")
        mavp2p = Service("mavp2p.service")
        status.register_status("mavp2p", mavp2p.is_active, mavp2p.restart)
        mavp2p.on_state = lambda state: status.update_status("mavp2p", state)
        mavp2p_state = mavp2p.state
        logger.debug("MavP2P state: " + mavp2p_state.name)
        if not (mavp2p_state == ServiceState.ACTIVE):
            logger.log("SETUP", "MavP2P service is not active. Starting...")
            mavp2p.start()

    if "thermal" in start_modules:
        logger.log("SETUP", "Starting thermal camera...")
        thermal = ThermalCamera()

    if ("vio" in start_modules) or ("apriltag" in start_modules):
        logger.log("SETUP", "Setting up frame server...")
        frame_server = FrameServer()

        logger.log("SETUP", "Starting frame server...")
        frame_server.start()

    if "mavsdk" in start_modules:
        logger.log("SETUP", "Setting up connection to mavP2P using mavsdk...")
        mavlink_system = mavsdk.System(sysid = 141)
    if "mavutil" in start_modules:
        logger.log("SETUP", "Setting up connection to mavP2P using mavutil...")
        pymavlink_connection = mavutil.mavlink_connection(
                "udpin:0.0.0.0:14542",
                source_system = 142,
                dialect = "bell"
        )
        mqtt_client.register_callback("avr/arm", set_armed, is_json = True, use_args = True, qos = 2)

    if "fcm" in start_modules:
        logger.log("SETUP", "Setting up fcm...")
        status.register_status("fcc", False, None, 2)
        fcm = FlightControlModule(mavlink_system, pymavlink_connection, status)
        status.add_restart_callback("fcc", fcm.gps_fcc.reboot)

        logger.log("SETUP", "Starting fcm...")
        await fcm.run()

    if "vio" in start_modules:
        logger.log("SETUP", "Setting up vio...")
        status.register_status("vio", False, None, 3)
        vio = VIOModule(status, frame_server)

        logger.log("SETUP", "Starting vio...")
        vio.run()

    if "fusion" in start_modules:
        logger.log("SETUP", "Setting up fusion...")
        status.register_status("fusion", False, None, 4)
        fusion = FusionModule(status, vio, fcm)

        logger.log("SETUP", "Starting fusion...")
        Thread(target = fusion.run, daemon = True).start()

    if "apriltag" in start_modules:
        logger.log("SETUP", "Setting up apriltag...")
        status.register_status("apriltag", False, None, 5)
        apriltag = AprilTagModule(status)

        logger.log("SETUP", "Starting apriltag...")
        Thread(target = apriltag.run, daemon = True).start()

    if "autonomy" in start_modules:
        logger.log("SETUP", "Setting up autonomy...")
        status.register_status("autonomy", False, None, 6)
        autonomy = Autonomy(
                mqtt_client,
                status,
                pcc,
                thermal,
                vio,
                mavlink_system,
                pymavlink_connection,
                apriltag
        )

        logger.log("SETUP", "Starting autonomy...")
        await autonomy.run()

    if "autonomy" in start_modules:
        logger.log("SETUP", "Setting up zmq server...")
        status.register_status("zmq", False, None, 7)
        zmq_server = ZMQServer(status, autonomy)

        logger.log("SETUP", "Starting zmq server...")
        zmq_server.run()

    status.send_update()

    logger.log("SETUP", "Setup Done!")
    await asyncio.Future()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            epilog = "If none of these are set, the program will run everything."
    )
    parser.add_argument(
            "--pcc",
            action = "store_true",
            default = False,
            help = "Connect to the pcc"
    )
    parser.add_argument(
            "--thermal",
            action = "store_true",
            default = False,
            help = "Stream the thermal camera"
    )
    parser.add_argument(
            "--mavsdk",
            action = "store_true",
            default = False,
            help = "Connect to mavp2p using mavsdk (async)"
    )
    parser.add_argument(
            "--mavutil",
            action = "store_true",
            default = False,
            help = "Connect to mavp2p using mavutil in pymavlink"
    )
    parser.add_argument(
            "--fcm",
            action = "store_true",
            default = False,
            help = "Stream telemetry data and update local position. Requires both mavsdk and mavutil"
    )
    parser.add_argument(
            "--vio",
            action = "store_true",
            default = False,
            help = "Start the zed camera"
    )
    parser.add_argument(
            "--fusion",
            action = "store_true",
            default = False,
            help = "Stream the local position of the avr drone. Requires fcm and vio"
    )
    parser.add_argument(
            "--apriltag",
            action = "store_true",
            default = False,
            help = "Start detecting apriltags with the csi camera. Requires fusion"
    )
    parser.add_argument(
            "--autonomy",
            action = "store_true",
            default = False,
            help = "Start autonomy. Requires all other modules"
    )
    parser.add_argument(
            "--interpreter",
            action = "store_true",
            default = False,
            help = "Use this if you are running in an interpreter"
    )
    args = vars(parser.parse_args())
    is_interpreter = args.pop("interpreter") if "interpreter" in args else False
    is_interpreter = is_interpreter or bool(int(os.environ.get("IS_INTERPRETER", "0")))
    if not (True in args.values()):
        for k in args:
            args[k] = True
    if args.get("autonomy", False):
        args["pcc"] = True
        args["thermal"] = True
        args["apriltag"] = True
    if args.get("apriltag", False):
        args["fusion"] = True
    if args.get("fusion", False):
        args["fcm"] = True
        args["vio"] = True
    if args.get("fcm", False):
        args["mavsdk"] = True
        args["mavutil"] = True
    start_items = []
    for name, start in args.items():
        if start:
            start_items.append(name)

    main_thread = Thread(target = lambda: asyncio.run(main(start_items)), daemon = True)
    # status_thread = Thread(target = ensure_running, daemon = True)
    main_thread.start()
    # status_thread.start()

    if not is_interpreter:
        while True:
            pass
