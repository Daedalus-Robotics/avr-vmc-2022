import argparse
import asyncio
import atexit
import datetime
import json
import os
import subprocess
import time
from threading import Thread

import mavsdk
from loguru import logger
from mavsdk.telemetry import FixType
from pymavlink import mavutil
from systemctl import Service, ServiceState

from vmc.apriltag.python.apriltag_processor import AprilTagModule
from vmc.autonomy.autonomy import Autonomy
from vmc.fcm.fcm import FlightControlModule
from vmc.frame_server import FrameServer
from vmc.fusion import FusionModule
from vmc.mqtt_client import MQTTClient
from vmc.pcc import PeripheralControlComputer
from vmc.status import Status
from vmc.status_led import StatusStrip
from vmc.thermal import ThermalCamera
from vmc.vio.vio import VIOModule
from vmc.zmq_server import ZMQServer

logger.level("SETUP", no=50, color="<magenta><bold><italic>", icon="⚙️")
logger.level("TEST", no=52, color="<green><bold><italic>", icon="🎚️")
logger.level("TEST_FAILED", no=52, color="<red><bold><italic>", icon="🎚️")

main_thread: Thread | None = None
# status_thread: Thread | None = None
has_started = False

mqtt_client = MQTTClient.get("localhost", 1883)
status_strip = StatusStrip(8)
status_strip.pixels.brightness = 0.5
status = Status(status_strip)
status.register_status("mqtt", False, None, led_num=0)
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
            mavp2p.stop()
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


def run_test() -> None:
    Thread(target=test, daemon=True).start()


def test_mqtt(message: dict) -> None:
    try:
        long = message.get("long", False)
        test(long)
    finally:
        pass


def test(long: bool) -> None:
    pcc_working = None
    mavp2p_working = None
    thermal_working = None
    frame_working = None
    pymavlink_working = None
    fcm_working = None
    vio_working = None
    fusion_working = None
    apriltag_working = None
    autonomy_working = None

    thermal_functional = None
    fusion_functional = None
    apriltag_functional = None

    fusion_fix = None
    if pcc is not None:
        logger.log("TEST", "Testing PCC...")
        pcc_working = pcc.is_connected
        if pcc_working:
            logger.log("TEST", "PCC is running")
            test_pcc()
        else:
            logger.log("TEST_FAILED", "PCC not running")
    if mavp2p is not None:
        logger.log("TEST", "Testing mavp2p...")
        mavp2p_working = mavp2p.is_active
        if mavp2p_working:
            logger.log("TEST", "Mavp2p is running")
        else:
            logger.log("TEST_FAILED", "Mavp2p is not running")
    if thermal is not None:
        logger.log("TEST", "Testing thermal cam...")
        thermal_working = thermal.amg.temperature is not None and thermal.running
        if thermal_working:
            logger.log("TEST", "Thermal cam running")
            if long:
                thermal_functional = test_thermal()
        else:
            logger.log("TEST_FAILED", "Thermal cam not running")
    if frame_server is not None:
        logger.log("TEST", "Testing frame server...")
        frame_working = frame_server.is_running
        if frame_working:
            logger.log("TEST", "Frame server running")
        else:
            logger.log("TEST_FAILED", "Frame server not running")
    if pymavlink_connection is not None:
        logger.log("TEST", "Testing PyMAVLink...")
        heartbeat = pymavlink_connection.wait_heartbeat(True, 5)
        pymavlink_working = heartbeat is not None
        if pymavlink_working:
            logger.log("TEST", "PyMAVLink working")
        else:
            logger.log("TEST_FAILED", "PyMAVLink not working")
    if fcm is not None:
        logger.log("TEST", "Testing flight controller and mavsdk system...")
        fcm_working = not fcm.fcc.telemetry_tasks_future.done()
        if fcm_working:
            logger.log("TEST", "Flight controller and mavsdk system are working")
        else:
            logger.log("TEST_FAILED", "Flight controller and mavsdk system are not working")
    if vio is not None:
        logger.log("TEST", "Testing VIO...")
        vio_working = False
        if vio.running and vio.camera.zed.is_opened():
            vio_working = True
            logger.log("TEST", "VIO and zed camera working")
        if not vio_working:
            logger.log("TEST_FAILED", "VIO not working")
    if fusion is not None:
        logger.log("TEST", "Testing fusion...")
        fusion_working = fusion.running
        if fusion.running:
            pos_state = fcm.fcc.position_state
            satellite_num, gps_fix = pos_state
            fusion_fix = gps_fix
            if satellite_num > 0 and gps_fix == FixType.FIX_3D:
                fusion_functional = True
                logger.log("TEST", "Fusion is working")
            else:
                fusion_functional = False
        if not fusion_working:
            logger.log("TEST_FAILED", "Fusion not working")
    if apriltag is not None:
        logger.log("TEST", "Testing apriltag...")
        apriltag_working = apriltag.process is not None and apriltag.process.poll() is None
        if apriltag_working:
            logger.log("TEST", "Apriltag is working")
            if long:
                apriltag_functional = test_apriltag()
                if apriltag_functional:
                    logger.log("TEST", "Apriltag is functional")
                else:
                    logger.log("TEST_FAILED", "Apriltag is not functional")
        else:
            logger.log("TEST_FAILED", "Apriltag not working")
    if autonomy is not None:
        logger.log("TEST", "Testing autonomy...")
        autonomy_states = (
            autonomy.running,
            autonomy.gimbal_thread.is_alive(),
            autonomy.water_drop_thread.is_alive()
        )
        autonomy_working = False not in autonomy_states
        if autonomy_working:
            logger.log("TEST", "Autonomy is working")
            test_autonomy()
        else:
            logger.log("TEST_FAILED", "Autonomy not working")
    logger.log("TEST", "Testing done, generating report...")
    test_print_report(
            pcc_working,
            mavp2p_working,
            thermal_working,
            frame_working,
            pymavlink_working,
            fcm_working,
            vio_working,
            fusion_working,
            apriltag_working,
            autonomy_working,
            thermal_functional,
            fusion_functional,
            apriltag_functional,
            fusion_fix
    )


def test_pcc() -> None:
    logger.log("TEST", "Testing pcc functions...")
    logger.log("TEST", "Testing laser...")
    pcc.fire_laser(5, False)
    logger.log("TEST", "Testing leds...")
    pcc.color_wipe(100)
    time.sleep(2)


def test_thermal() -> bool:
    logger.log("TEST", "Testing hotspot detection...")
    logger.log("TEST", "Waiting for hotspot to be detected")
    functional = False
    start_time = time.time()
    while time.time() < start_time + 30:
        if thermal.detector.currently_detecting:
            logger.log("TEST", "Found hotspot")
            functional = True
            break
    return functional


def test_apriltag() -> bool:
    logger.log("TEST", "Testing apriltag functions...")
    logger.log("TEST", "Waiting for tag 24 to be detected")
    functional = False
    start_time = time.time()
    while time.time() < start_time + 30:
        if 24 in apriltag.detections:
            detect_time = apriltag.detections[24][1]
            if detect_time > start_time:
                logger.log("TEST", "Found apriltag 24")
                functional = True
                break
    return functional


def test_autonomy() -> None:
    logger.log("TEST", "Testing autonomy functions...")
    try:
        gimbal_test_data = json.load(open("vmc/resources/gimbal_test.json", 'r'))
        logger.log("TEST", "Testing gimbal...")
        for point in gimbal_test_data:
            timeout = point.get("time", 0)
            x = point.get("x", None)
            y = point.get("y", None)
            if None not in (x, y):
                time.sleep(timeout)
                autonomy.gimbal.set_pos(x, y)
    except (OSError, json.JSONDecodeError):
        logger.log("TEST_FAILED", "Can't read gimbal test data")
    logger.log("TEST", "Testing water dropper in 5 seconds, pick the drone up")
    time.sleep(5)
    autonomy.water_drop.set_water_drop(100)
    time.sleep(1.2)
    autonomy.water_drop.set_water_drop(0)


def test_print_report(
        pcc_working: bool,
        mavp2p_working: bool,
        thermal_working: bool,
        frame_working: bool,
        pymavlink_working: bool,
        fcm_working: bool,
        vio_working: bool,
        fusion_working: bool,
        apriltag_working: bool,
        autonomy_working: bool,
        thermal_functional: bool,
        fusion_functional: bool,
        apriltag_functional: bool,
        fusion_fix: FixType
) -> None:
    report_list = ["-" * 10]
    if pcc_working is not None:
        pcc_report = f"PCC:\n  Running: {pcc_working}"
        report_list.append(pcc_report)
    if mavp2p_working is not None:
        mavp2p_report = f"MavP2P:\n  Running: {mavp2p_working}"
        report_list.append(mavp2p_report)
    if thermal_working is not None:
        thermal_report = "Thermal:"
        thermal_report += f"\n  Running: {thermal_working}"
        if thermal_functional is not None:
            thermal_report += f"\n  Detector: {'Functional' if thermal_functional else 'Non-Functional'}"
        report_list.append(thermal_report)
    if frame_working is not None:
        frame_report = f"Frame Server:\n  Running: {frame_working}"
        report_list.append(frame_report)
    if fcm_working is not None:
        fcm_report = "Flight Controller:"
        fcm_report += f"\n  MAVLink:"
        fcm_report += f"\n    Connection: {'Connected' if pymavlink_working else 'Disconnected'}"
        fcm_report += f"\n    Telemetry Tasks: {'Running' if fcm_working else 'Stopped'}"
        report_list.append(fcm_report)
    elif pymavlink_working is not None:
        pymavlink_report = f"PyMavlink:\n  Running: {pymavlink_working}"
        report_list.append(pymavlink_report)
    if vio_working is not None:
        vio_report = f"VIO:\n  Running: {vio_working}"
        report_list.append(vio_report)
    if fusion_working is not None:
        fusion_report = "Fusion:"
        fusion_report += f"\n  Running: {fusion_working}"
        fusion_report += f"\n  GPS Fix: {fusion_fix}"
        fusion_report += f"\n  Position Hold: {'Functional' if fusion_functional else 'Non-Functional'}"
        report_list.append(fusion_report)
    if apriltag_working is not None:
        apriltag_report = "Apriltag:"
        apriltag_report += f"\n  Running: {apriltag_working}"
        if apriltag_functional is not None:
            apriltag_report += f"\n  Detector: {'Functional' if apriltag_functional else 'Non-Functional'}"
        report_list.append(apriltag_report)
    if autonomy_working is not None:
        autonomy_report = f"Autonomy:\n  Running: {autonomy_working}"
        report_list.append(autonomy_report)
    report_list.append("-" * 10)

    # timestamp = datetime.datetime.utcnow().strftime("%Y-%m-%d-%H:%M")
    # report_file = open(f"~/vmc-report-{timestamp}", "w")
    for report in report_list:
        print(report)
        # report_file.write(report)


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
            is_json=False,
            use_args=False
    )
    mqtt_client.register_callback(
            "avr/test",
            test_mqtt,
            is_json=True,
            use_args=True
    )

    if "pcc" in start_modules:
        logger.log("SETUP", "Setting up pcc...")
        pcc = PeripheralControlComputer()
        status.register_status("pcc", pcc.is_connected, pcc.reset, 1)
        pcc.on_state = lambda state: status.update_status("pcc", state)

        logger.log("SETUP", "Starting pcc...")
        pcc.begin()
        pcc.begin_mqtt()
        pcc.set_base_color((255, 255, 255, 255))
        pcc.set_base_color((255, 0, 0, 0))

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
        mavlink_system = mavsdk.System(sysid=141)
    if "mavutil" in start_modules:
        logger.log("SETUP", "Setting up connection to mavP2P using mavutil...")
        pymavlink_connection = mavutil.mavlink_connection(
                "udpin:0.0.0.0:14542",
                source_system=142,
                dialect="bell"
        )
        mqtt_client.register_callback("avr/arm", set_armed, is_json=True, use_args=True, qos=2)

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
        Thread(target=fusion.run, daemon=True).start()

    if "apriltag" in start_modules:
        logger.log("SETUP", "Setting up apriltag...")
        status.register_status("apriltag", False, None, 5)
        apriltag = AprilTagModule(status)

        logger.log("SETUP", "Starting apriltag...")
        Thread(target=apriltag.run, daemon=True).start()

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
            epilog="If none of these are set, the program will run everything."
    )
    parser.add_argument(
            "--pcc",
            action="store_true",
            default=False,
            help="Connect to the pcc"
    )
    parser.add_argument(
            "--thermal",
            action="store_true",
            default=False,
            help="Stream the thermal camera"
    )
    parser.add_argument(
            "--mavsdk",
            action="store_true",
            default=False,
            help="Connect to mavp2p using mavsdk (async)"
    )
    parser.add_argument(
            "--mavutil",
            action="store_true",
            default=False,
            help="Connect to mavp2p using mavutil in pymavlink"
    )
    parser.add_argument(
            "--fcm",
            action="store_true",
            default=False,
            help="Stream telemetry data and update local position. Requires both mavsdk and mavutil"
    )
    parser.add_argument(
            "--vio",
            action="store_true",
            default=False,
            help="Start the zed camera"
    )
    parser.add_argument(
            "--fusion",
            action="store_true",
            default=False,
            help="Stream the local position of the avr drone. Requires fcm and vio"
    )
    parser.add_argument(
            "--apriltag",
            action="store_true",
            default=False,
            help="Start detecting apriltags with the csi camera. Requires fusion"
    )
    parser.add_argument(
            "--autonomy",
            action="store_true",
            default=False,
            help="Start autonomy. Requires all other modules"
    )
    parser.add_argument(
            "--interpreter",
            action="store_true",
            default=False,
            help="Use this if you are running in an interpreter"
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

    main_thread = Thread(target=lambda: asyncio.run(main(start_items)), daemon=True)
    # status_thread = Thread(target = ensure_running, daemon = True)
    main_thread.start()
    # status_thread.start()

    if not is_interpreter:
        while True:
            pass
