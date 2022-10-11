import subprocess

from adafruit_platformdetect import Detector
from systemctl import Service

from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status

detector = Detector()
TESTING = not (detector.board.any_jetson_board or detector.board.any_raspberry_pi)

mqtt_client = MQTTClient.get("localhost", 1883, True)
status = Status()

pcc: PeripheralControlComputer
mavp2p: Service


def restart_vmc() -> None:
    pcc.color_wipe(60)
    pcc.end()
    status.register_status("pcc", False)

    if not TESTING:
        mavp2p.stop()
    status.register_status("mavp2p", False)

    status.update_status("vmc", False)
    mqtt_client.disconnect()
    if TESTING:
        subprocess.Popen(["echo", "The vmc would normally restart now"])
    else:
        subprocess.Popen(["sleep", "8", ";sudo", "reboot"])


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

    mqtt_client.connect()
    status.send_update()
