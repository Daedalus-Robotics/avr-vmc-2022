import atexit
import subprocess
import time
from threading import Thread

from adafruit_platformdetect import Detector
from systemctl import Service

# from vmc.frame_server import FrameServer
from vmc.mqtt.fcm.fcm import FlightControlModule
from vmc.mqtt.fusion import FusionModule
from vmc.mqtt.vio.vio import VIOModule
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.thermal import ThermalCamera

detector = Detector()
TESTING = not (detector.board.any_jetson_board or detector.board.any_raspberry_pi)

mqtt_client = MQTTClient.get("localhost", 1883, True)
status = Status()

pcc: PeripheralControlComputer
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


if __name__ == '__main__':
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
    # frame_server = FrameServer()
    # frame_server.start()

    vio = VIOModule()
    Thread(target = vio.run, daemon = True).start()

    fcm = FlightControlModule()
    fcm.run()

    fusion = FusionModule()
    fusion.run()

    mqtt_client.connect()
    status.send_update()
