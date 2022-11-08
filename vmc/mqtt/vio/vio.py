import math
import time
from threading import Barrier, BrokenBarrierError, Thread
from typing import Tuple

import numpy as np
from bell.avr.mqtt.payloads import (
    AvrVioConfidencePayload,
    AvrVioHeadingPayload,
    AvrVioOrientationEulPayload,
    AvrVioPositionNedPayload,
    AvrVioResyncPayload,
    AvrVioVelocityNedPayload,
)
from bell.avr.utils.decorators import run_forever, try_except
from loguru import logger

from vmc.frame_server import CameraType, FrameServer
from vmc.mqtt_client import MQTTClient
from .vio_library import CameraCoordinateTransformation
from .zed_library import ZEDCamera


class VIOModule:
    def __init__(self, frame_server: FrameServer) -> None:
        self.client = MQTTClient.get()

        self.frame_server = frame_server

        # settings
        self.init_sync = False
        self.continuous_sync = True
        self.CAM_UPDATE_FREQ = 10

        # connected libraries
        self.camera = ZEDCamera()
        self.coord_trans = CameraCoordinateTransformation()

        # mqtt
        self.client.register_callback("avr/vio/resync", self.handle_resync)

        self.position_ned = None
        self.orientation_eul = None
        self.vio_heading = None
        self.velocity_ned = None
        self.vio_confidence = None

        self.position_ned_func = lambda thing: None
        self.orientation_eul_func = lambda thing: None
        self.vio_heading_func = lambda thing: None
        self.velocity_ned_func = lambda thing: None
        self.vio_confidence_func = lambda thing: None

        self.running = False
        self.running_barrier1 = Barrier(2)
        self.running_barrier2 = Barrier(2)

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier1.wait(2)
        except BrokenBarrierError:
            pass
        try:
            self.running_barrier2.wait(2)
        except BrokenBarrierError:
            pass
        self.camera.zed.close()

    def handle_resync(self, payload: AvrVioResyncPayload) -> None:
        # whenever new data is published to the ZEDCamera resync topic, we need to compute a new correction
        # to compensate for sensor drift over time.
        if not self.init_sync or self.continuous_sync:
            heading_ref = payload["heading"]
            self.coord_trans.sync(
                    heading_ref, {"n": payload["n"], "e": payload["e"], "d": payload["d"]}
            )
            self.init_sync = True

    @try_except(reraise = False)
    def publish_updates(
            self,
            ned_pos: Tuple[float, float, float],
            ned_vel: Tuple[float, float, float],
            rpy: Tuple[float, float, float],
            tracker_confidence: float,
    ) -> None:
        if np.isnan(ned_pos).any():
            raise ValueError("ZEDCamera has NaNs for position")

        # send position update
        n = float(ned_pos[0])
        e = float(ned_pos[1])
        d = float(ned_pos[2])
        ned_update = AvrVioPositionNedPayload(n = n, e = e, d = d)  # cm

        # noinspection PyTypeChecker
        # self.send_message("avr/vio/position/ned", ned_update)
        self.position_ned = ned_update
        self.position_ned_func(self.position_ned)

        if np.isnan(rpy).any():
            raise ValueError("Camera has NaNs for orientation")

        # send orientation update
        eul_update = AvrVioOrientationEulPayload(psi = rpy[0], theta = rpy[1], phi = rpy[2])
        # noinspection PyTypeChecker
        # self.send_message("avr/vio/orientation/eul", eul_update)
        self.orientation_eul = eul_update
        self.orientation_eul_func(self.orientation_eul)

        # send heading update
        heading = rpy[2]
        # correct for negative heading
        if heading < 0:
            heading += 2 * math.pi
        heading = np.rad2deg(heading)
        heading_update = AvrVioHeadingPayload(degrees = heading)
        # noinspection PyTypeChecker
        # self.send_message("avr/vio/heading", heading_update)
        self.vio_heading = heading_update
        self.vio_heading_func(self.vio_heading)
        # coord_trans.heading = rpy[2]

        if np.isnan(ned_vel).any():
            raise ValueError("Camera has NaNs for velocity")

        # send velocity update
        vel_update = AvrVioVelocityNedPayload(n = ned_vel[0], e = ned_vel[1], d = ned_vel[2])
        # noinspection PyTypeChecker
        # self.send_message("avr/vio/velocity/ned", vel_update)
        self.velocity_ned = vel_update
        self.velocity_ned_func(self.velocity_ned)

        confidence_update = AvrVioConfidencePayload(
                tracker = tracker_confidence,
        )
        # noinspection PyTypeChecker
        # self.send_message("avr/vio/confidence", confidence_update)
        self.vio_confidence = confidence_update
        self.vio_confidence_func(self.vio_confidence)

    @try_except(reraise = False)
    def process_camera_data(self) -> None:
        while self.running:
            data = self.camera.get_pipe_data()

            if data is None:
                logger.debug("Waiting on camera data")
                return

            # collect data from the sensor and transform it into "global" NED frame
            ned_pos, ned_vel, rpy, = self.coord_trans.transform_trackcamera_to_global_ned(data)

            self.publish_updates(
                    ned_pos,
                    ned_vel,
                    rpy,
                    data["tracker_confidence"],
            )
            time.sleep(1 / 10)
            try:
                self.running_barrier1.wait(0)
            except BrokenBarrierError:
                pass

    @try_except(reraise = False)
    def update_frames(self) -> None:
        while self.running:
            success, right, left, depth = self.camera.get_frames()
            if success:
                self.frame_server.update_frame(right, CameraType.ZED_RIGHT, 60, 480)
                self.frame_server.update_frame(left, CameraType.ZED_LEFT, 60, 480)
                self.frame_server.update_frame(depth, CameraType.ZED_DEPTH, 60, 480)
            time.sleep(1 / 10)
            try:
                self.running_barrier2.wait(0)
            except BrokenBarrierError:
                pass

    def run(self) -> None:
        self.running = True

        # set up the tracking camera
        logger.debug("Setting up camera connection")
        self.camera.setup()

        # begin processing data
        Thread(target = self.update_frames, daemon = True).start()
        Thread(target = self.process_camera_data, daemon = True).start()


if __name__ == "__main__":
    f = FrameServer()
    f.start()
    vio = VIOModule(f)
    vio.run()
    while True:
        pass
