import asyncio
import atexit
import subprocess
import time
from threading import Thread

import mavsdk
from adafruit_platformdetect import Detector
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


@atexit.register
def stop() -> None:
    pcc.color_wipe(60)
    pcc.end()
    status.register_status("pcc", False)

    if not TESTING:
        mavp2p.stop()
    status.register_status("mavp2p", False)

    status.update_status("vmc", False)
    mqtt_client.disconnect()


def restart_vmc() -> None:
    stop()
    time.sleep(5)
    subprocess.Popen(["sudo", "reboot"])

async def main() -> None:
    global mqtt_client, status, pcc, thermal, frame_server, vio, fcm, fusion, mavp2p
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

        fcm = FlightControlModule(status)
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
