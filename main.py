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

from vmc.autonomy.autonomy import Autonomy
from vmc.frame_server import FrameServer
from vmc.mqtt.fcm.fcm import FlightControlModule
from vmc.mqtt.fusion import FusionModule
from vmc.mqtt.vio.vio import VIOModule
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.status_led import StatusStrip
from vmc.thermal import ThermalCamera

logger.level("SETUP", no = 50, color = "<magenta><bold><italic>", icon = "⚙️")

main_thread: Thread | None = None
# status_thread: Thread | None = None

mqtt_client = MQTTClient.get("localhost", 1883, True)
status_strip = StatusStrip(8)
status = Status(status_strip)
status.register_status("mqtt", False, None, led_num = 0)
mqtt_client.status = status

pcc: PeripheralControlComputer | None = None
thermal: ThermalCamera | None = None
frame_server: FrameServer | None = None
vio: VIOModule | None = None
fcm: FlightControlModule | None = None
fusion: FusionModule | None = None
mavp2p: Service | None = None
mavlink_system: mavsdk.System | None = None
pymavlink_connection: mavutil.mavudp | None = None
autonomy: Autonomy | None = None


@atexit.register
def stop() -> None:
    pcc.color_wipe(10)
    pcc.end()
    status.update_status("pcc", False)

    mavp2p.stop()
    status.update_status("mavp2p", False)

    pymavlink_connection.close()
    status.update_status("fcc", False)

    status.update_status("vmc", False)
    mqtt_client.disconnect()


def restart_vmc() -> None:
    logger.info("Restarting vmc...")
    stop()
    time.sleep(2)
    subprocess.Popen(["sudo", "reboot"])


async def shutdown_vmc() -> None:
    logger.info("Shutting down vmc...")
    fcm.gps_fcc.shutdown()
    stop()
    time.sleep(2)
    subprocess.Popen(["sudo", "shutdown", "now"])
    await asyncio.Future()


async def main(start_modules: list[str]) -> None:
    global mqtt_client, status
    global pcc, thermal, frame_server, vio, fcm, fusion
    global mavp2p
    global mavlink_system, pymavlink_connection
    global autonomy
    mqtt_client.register_callback(
            "avr/shutdown",
            lambda: asyncio.run(shutdown_vmc()),
            is_json = False,
            use_args = False
    )

    pcc = PeripheralControlComputer()
    status.register_status("pcc", pcc.is_connected, pcc.reset, 1)
    pcc.on_state = lambda state: status.update_status("pcc", state)
    pcc.begin()
    pcc.begin_mqtt()

    if TESTING:
        status.register_status("mavp2p", True, lambda: None)
    else:
        mavp2p = Service("mavp2p.service")
        status.register_status("mavp2p", mavp2p.is_active, mavp2p.restart, 2)
        mavp2p.on_state = lambda state: status.update_status("mavp2p", state)

        if not (mavp2p.state == ServiceState.ACTIVE):
            mavp2p.start()

    status.register_status("vmc", True, restart_vmc)

    thermal = ThermalCamera()

    # No purpose without the csi cam working or the zed cam streaming
    # frame_server = FrameServer()
    # frame_server.start()

    if not TESTING:
        vio = VIOModule()
        Thread(target = vio.run, daemon = True).start()

        mavlink_system = mavsdk.System(sysid = 141)
        pymavlink_connection = mavutil.mavlink_connection(
                "udpin:0.0.0.0:14542",
                source_system = 142,
                dialect = "bell"
        )

        def set_armed(state: dict):
            arm = state.get("arm", None)
            if arm is not None:
                if arm:
                    pymavlink_connection.arducopter_arm()
                else:
                    pymavlink_connection.arducopter_disarm()

        mqtt_client.register_callback("avr/arm", set_armed, is_json = True, use_args = True, qos = 2)

        fcm = FlightControlModule(mavlink_system, pymavlink_connection, status)
        status.register_status("fcc", False, fcm.gps_fcc.reboot, 3)
        await fcm.run()

        fusion = FusionModule(vio, fcm)
        Thread(target = fusion.run).start()

    # ToDo: move this to a reasonable spot
    pcc.set_servo_max(1, 1000)

    autonomy = Autonomy(mqtt_client, pcc, thermal, vio.camera.zed, mavlink_system, pymavlink_connection)

    await autonomy.run()

    mqtt_client.connect()
    status.send_update()

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
            "--mavlink",
            action = "store_true",
            default = False,
            help = "Connect to mavp2p using mavsdk (async)"
    )
    parser.add_argument(
            "--mavutil",
            "--pymavlink",
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
            "--zed",
            action = "store_true",
            default = False,
            help = "Start the zed camera"
    )
    parser.add_argument(
            "--fusion",
            "--fuse",
            action = "store_true",
            default = False,
            help = "Stream the local position of the avr drone. Requires fcm and vio"
    )
    parser.add_argument(
            "-a",
            "--autonomy",
            action = "store_true",
            default = False,
            help = "Start autonomy. Requires all other modules"
    )
    parser.add_argument(
            "-i",
            "--interpreter",
            action = "store_true",
            default = False,
            help = "Use this if you are running in an interpreter"
    )
    args = vars(parser.parse_args())
    is_interpreter = args.pop("interpreter") if "interpreter" in args else False
    is_interpreter = is_interpreter or bool(int(os.environ.get("IS_INTERPRETER", "0")))
    if not (True in args.values()) or args.get("autonomy", False):
        for k in args:
            args[k] = True
    if args.get("fusion", False):
        if "fcm" in args:
            args["fcm"] = True
        if "vio" in args:
            args["vio"] = True
    if args.get("fcm", False):
        if "mavsdk" in args:
            args["mavsdk"] = True
        if "mavutil" in args:
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
