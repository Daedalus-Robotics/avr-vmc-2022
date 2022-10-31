import asyncio
import atexit
import subprocess
import sys
import time
from threading import Thread

import mavsdk
from adafruit_platformdetect import Detector
from systemctl import Service, ServiceState

from vmc.autonomy.autonomy import Autonomy
from vmc.frame_server import FrameServer
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.status_led import StatusStrip
from vmc.thermal import ThermalCamera

if (len(sys.argv) > 1) and (sys.argv[1] == "test"):
    TESTING = True
else:
    detector = Detector()
    TESTING = not (detector.board.any_jetson_board or detector.board.any_raspberry_pi)

if not TESTING:
    from pymavlink import mavutil
    from vmc.mqtt.fcm.fcm import FlightControlModule
    from vmc.mqtt.fusion import FusionModule
    from vmc.mqtt.vio.vio import VIOModule

main_thread: Thread | None = None
status_thread: Thread | None = None

mqtt_client = MQTTClient.get("localhost", 1883, True)
status_strip = StatusStrip(8)
status = Status(status_strip)

pcc: PeripheralControlComputer
thermal: ThermalCamera
frame_server: FrameServer
if not TESTING:
    vio: VIOModule
    fcm: FlightControlModule
    fusion: FusionModule
    mavp2p: Service
    mavlink_system: mavsdk.System
    pymavlink_connection: mavutil.mavudp
    autonomy: Autonomy


@atexit.register
def stop() -> None:
    pcc.color_wipe(10)
    pcc.end()
    status.update_status("pcc", False)

    if not TESTING:
        mavp2p.stop()
    status.update_status("mavp2p", False)

    if not TESTING:
        fcm.gps_fcc.shutdown()
        pymavlink_connection.close()
        status.update_status("fcc", False)

    status.update_status("vmc", False)
    mqtt_client.disconnect()


def restart_vmc() -> None:
    stop()
    time.sleep(2)
    subprocess.Popen(["sudo", "reboot"])


async def shutdown_vmc() -> None:
    if not TESTING:
        stop()
        time.sleep(2)
        subprocess.Popen(["sudo", "shutdown", "now"])
        await asyncio.Future()


async def main() -> None:
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
    status.register_status("pcc", pcc.is_connected, pcc.reset, 0)
    pcc.on_state = lambda state: status.update_status("pcc", state)
    pcc.begin()
    pcc.begin_mqtt()

    if TESTING:
        status.register_status("mavp2p", True, lambda: None)
    else:
        mavp2p = Service("mavp2p.service")
        status.register_status("mavp2p", mavp2p.is_active, mavp2p.restart, 1)
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
        fcm = FlightControlModule(mavlink_system, pymavlink_connection, status)
        status.register_status("fcc", False, fcm.gps_fcc.reboot, 2)
        await fcm.run()

        fusion = FusionModule()
        Thread(target = fusion.run).start()

    autonomy = Autonomy(mqtt_client, pcc, thermal, vio.camera.zed, mavlink_system, pymavlink_connection)

    await autonomy.run()

    mqtt_client.connect()
    status.send_update()

    await asyncio.Future()


if __name__ == '__main__':
    main_thread = Thread(target = lambda: asyncio.run(main()), daemon = True)
    # status_thread = Thread(target = ensure_running, daemon = True)
    main_thread.start()
    # status_thread.start()
