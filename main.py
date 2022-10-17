import asyncio
import atexit
import subprocess
import time
from threading import Thread

import mavsdk
from adafruit_platformdetect import Detector
from pymavlink import mavutil
from systemctl import Service

from vmc.frame_server import FrameServer
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.thermal import ThermalCamera

detector = Detector()
TESTING = not (detector.board.any_jetson_board or detector.board.any_raspberry_pi)

if not TESTING:
    from vmc.mqtt.fcm.fcm import FlightControlModule
    from vmc.mqtt.fusion import FusionModule
    from vmc.mqtt.vio.vio import VIOModule

mqtt_client = MQTTClient.get("localhost", 1883, True)
status = Status()

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

test_thing = None


@atexit.register
def stop() -> None:
    pcc.color_wipe(60)
    pcc.end()
    status.update_status("pcc", False)

    if not TESTING:
        mavp2p.stop()
    status.update_status("mavp2p", False)

    if not TESTING:
        fcm.gps_fcc.shutdown()
        fcm.gps_fcc.mavlink_connection.close()
        status.update_status("fcc", False)

    status.update_status("vmc", False)
    mqtt_client.disconnect()


def restart_vmc() -> None:
    stop()
    time.sleep(2)
    subprocess.Popen(["sudo", "reboot"])


def shutdown_vmc() -> None:
    stop()
    time.sleep(2)
    subprocess.Popen(["sudo", "shutdown", "now"])


def test_rc_channels(print_stuff = True) -> None:
    global test_thing
    connection: mavutil.mavudp = fcm.gps_fcc.mavlink_connection

    first = True
    while True:
        v = connection.recv_match(type = "RC_CHANNELS", blocking = True)
        if print_stuff:
            if first:
                print(type(v))
            first = False
            print(v)
        test_thing = v
        print(v.chan8_raw >= 1060)
        time.sleep(0.1)

async def main() -> None:
    global mqtt_client, status
    global pcc, thermal, frame_server, vio, fcm, fusion
    global mavp2p
    global mavlink_system, pymavlink_connection
    mqtt_client.register_callback("avr/shutdown", shutdown_vmc, is_json = False, use_args = False)

    pcc = PeripheralControlComputer()
    status.register_status("pcc", pcc.is_connected, pcc.reset)
    pcc.on_state = lambda state: status.update_status("pcc", state)
    pcc.begin()
    pcc.begin_mqtt()

    if TESTING:
        status.register_status("mavp2p", True, lambda: None)
    else:
        mavp2p = Service("mavp2p.service")
        status.register_status("mavp2p", mavp2p.is_active, mavp2p.restart)
        mavp2p.on_state = lambda state: status.update_status("mavp2p", state)

    status.register_status("vmc", True, restart_vmc)

    thermal = ThermalCamera()

    # No purpose without the csi cam working or the zed cam streaming
    frame_server = FrameServer()
    frame_server.start()

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
        status.register_status("fcc", False, fcm.gps_fcc.reboot)
        await fcm.run()
        # s = mavsdk.System(sysid = 141)
        # s.connect(system_address = "udp://0.0.0.0:14541")

        fusion = FusionModule()
        Thread(target = fusion.run).start()

    mqtt_client.connect()
    status.send_update()

    await asyncio.Future()


if __name__ == '__main__':
    Thread(target = lambda: asyncio.run(main()), daemon = False).start()
