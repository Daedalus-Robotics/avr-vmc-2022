from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status

mqtt_client = MQTTClient.get("localhost", 1883, True)
status = Status()

pcc: PeripheralControlComputer

if __name__ == '__main__':
    pcc = PeripheralControlComputer()
    status.register_status("pcc", pcc.is_connected, pcc.reset)
    pcc.on_state = lambda state: status.update_status("pcc", state)
    pcc.begin()
    pcc.begin_mqtt()

    mqtt_client.connect()
    status.send_update()
