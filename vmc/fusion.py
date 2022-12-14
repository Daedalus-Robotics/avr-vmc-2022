import math
import time
from threading import Barrier, BrokenBarrierError

import numpy as np
import pymap3d
from bell.avr.mqtt.payloads import (
    AvrApriltagsSelectedPayload,
    AvrFusionAttitudeEulerPayload,
    AvrFusionAttitudeHeadingPayload,
    AvrFusionAttitudeQuatPayload,
    AvrFusionClimbratePayload,
    AvrFusionCoursePayload,
    AvrFusionGeoPayload,
    AvrFusionGroundspeedPayload,
    AvrFusionHilGpsPayload,
    AvrFusionPositionNedPayload,
    AvrFusionVelocityNedPayload,
    AvrVioHeadingPayload,
    AvrVioOrientationEulPayload,
    AvrVioOrientationQuatPayload,
    AvrVioPositionNedPayload,
    AvrVioResyncPayload,
    AvrVioVelocityNedPayload,
)
from bell.avr.utils.decorators import try_except
from loguru import logger

from .fcm.fcm import FlightControlModule
from .mqtt_client import MQTTClient
from .status import Status
from .vio.vio import VIOModule

MSG_INTERVAL = 1


class FusionModule:
    def __init__(self, status: Status, vio: VIOModule, fcm: FlightControlModule) -> None:
        self.client = MQTTClient.get()

        self.fusion_attitude_heading = None
        self.last_attitude_heading = 0
        self.fusion_attitude_euler = None
        self.last_attitude_euler = 0
        self.fusion_attitude_quat = None
        self.last_attitude_quat = 0
        self.fusion_climbrate = None
        self.last_climbrate = 0
        self.fusion_course = None
        self.last_course = 0
        self.fusion_groundspeed = None
        self.last_groundspeed = 0
        self.fusion_vel_ned = None
        self.last_vel_ned = 0
        self.fusion_pos_ned = None
        self.last_pos_ned = 0
        self.fusion_geo = None
        self.last_geo = 0
        self.vio_init = None
        # noinspection SpellCheckingInspection
        self.config = {
            # Bell HQ VIP helipad
            # https://www.google.com/maps/place/32%C2%B048'30.8%22N+97%C2%B009'22.8%22W
            "origin": {"lat": 32.808549, "lon": -97.156345, "alt": 161.5},
            "hil_gps_constants": {
                "fix_type": 3,
                "eph": 20,
                "epv": 5,
                "satellites_visible": 13,
            },
            "COURSE_THRESHOLD": 10,
            "POS_DETLA_THRESHOLD": 10,
            "POS_D_THRESHOLD": 30,
            "HEADING_DELTA_THRESHOLD": 5,
            "AT_DERIV_THRESHOLD": 10,
        }

        self.status = status
        self.vio = vio
        self.fcm = fcm

        self.topic_map = {
            # "avr/vio/position/ned": self.fuse_pos,
            # "avr/vio/orientation/eul": self.fuse_att_euler,
            # "avr/vio/heading": self.fuse_att_heading,
            # "avr/vio/velocity/ned": self.fuse_vel,
            # "avr/fusion/position/ned": self.local_to_geo,

            # uncomment to re-enable position re-syncing
            # currently not well enough tested/reliable to be competition ready
            # "avr/apriltags/selected": self.on_apriltag_message
        }

        self.vio.position_ned_func = self.fuse_pos
        self.vio.orientation_eul_func = self.fuse_att_euler
        self.vio.vio_heading_func = self.fuse_att_heading
        self.vio.velocity_ned_func = self.fuse_vel
        self.vio.position_ned_func = self.local_to_geo

        # on_apriltag storage
        self.norm = None
        self.last_pos = [0, 0, 0]
        self.deriv = [0, 0, 0]
        self.last_apriltag = time.time()

        self.running = True
        self.running_barrier = Barrier(2)

    def close(self) -> None:
        self.running = False
        try:
            self.running_barrier.wait(2)
        except BrokenBarrierError:
            pass

    @try_except(reraise=True)
    def local_to_geo(self, payload: AvrFusionPositionNedPayload) -> None:
        """
        Callback for the fusion/pos topic. This method calculates the
        geodetic location from an NED position and origin and publishes it.
        """
        lla = pymap3d.enu2geodetic(
                float(payload["e"]) / 100,  # type: ignore # East   | Y
                float(payload["n"]) / 100,  # type: ignore # North  | X
                -1 * float(payload["d"]) / 100,  # type: ignore # Up     | Z
                self.config["origin"]["lat"],  # Origin lat
                self.config["origin"]["lon"],  # Origin lon
                self.config["origin"]["alt"],  # Origin alt
                deg=True,
        )

        geo_update = AvrFusionGeoPayload(
                lat=float(lla[0]), lon=float(lla[1]), alt=float(lla[2])
        )

        ss = time.time()
        timesince = ss - self.last_geo
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/geo", geo_update)
            self.last_geo = ss
        self.fusion_geo = geo_update

    @try_except(reraise=True)
    def fuse_pos(self, payload: AvrVioPositionNedPayload) -> None:
        """
        Callback for receiving pos data in NED reference frame from VIO and
        publishes into a fusion/pos topic.

        Avr doesn't have sophisticated fusion yet, so this just re-routes the
        message onto the fusion topic.
        """

        pos_update = AvrFusionPositionNedPayload(
                n=payload["n"], e=payload["e"], d=payload["d"]
        )

        ss = time.time()
        timesince = ss - self.last_pos_ned
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/position/ned", pos_update)
            self.last_pos_ned = ss
        self.fusion_pos_ned = pos_update

    @try_except(reraise=True)
    def fuse_vel(self, payload: AvrVioVelocityNedPayload) -> None:
        """
        Callback for receiving vel data in NED reference frame from VIO and
        publishes into a fusion/vel topic.

        Avr doesn't have sophisticated fusion yet, so this just re-routes the
        message onto the fusion topic.
        """
        # record that VIO has initialized
        self.vio_init = True

        # forward ned velocity message
        vmc_vel_update = AvrFusionVelocityNedPayload(
                Vn=payload["n"], Ve=payload["e"], Vd=payload["d"]
        )

        ss = time.time()
        timesince = ss - self.last_vel_ned
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/velocity/ned", vmc_vel_update)
            self.last_vel_ned = ss
        self.fusion_vel_ned = vmc_vel_update

        # logger.debug("avr/fusion/velocity/ned message sent")

        # compute groundspeed
        gs = np.linalg.norm([payload["n"], payload["e"]])
        groundspeed_update = AvrFusionGroundspeedPayload(groundspeed=float(gs))

        ss = time.time()
        timesince = ss - self.last_groundspeed
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/groundspeed", groundspeed_update)
            self.last_groundspeed = ss
        self.fusion_groundspeed = groundspeed_update

        # arctan gets real noisy when the values get small, so we just lock course
        # to heading when we aren't really moving
        if gs >= self.config["COURSE_THRESHOLD"]:
            course = math.atan2(payload["e"], payload["n"])
            # wrap [-pi, pi] to [0, 360]
            if course < 0:
                course += 2 * math.pi

            # rad to deg
            course = math.degrees(course)
            course_update = AvrFusionCoursePayload(course=course)

            ss = time.time()
            timesince = ss - self.last_course
            if timesince >= MSG_INTERVAL:
                # noinspection PyTypeChecker
                self.client.send_message("avr/fusion/course", course_update)
            self.last_course = ss
            self.fusion_course = course_update

        m_per_s_2_ft_per_min = 196.85
        climb_rate_update = AvrFusionClimbratePayload(
                climb_rate_fps=-1 * payload["d"] * m_per_s_2_ft_per_min
        )

        ss = time.time()
        timesince = ss - self.last_climbrate
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/climbrate", climb_rate_update)
            self.last_climbrate = ss
        self.fusion_climbrate = climb_rate_update

    @try_except(reraise=True)
    def fuse_att_quat(self, payload: AvrVioOrientationQuatPayload) -> None:
        """
        Callback for receiving quaternion att data in NED reference frame
        from vio and publishes into a fusion/att/quat topic.

        Avr doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        quat_update = AvrFusionAttitudeQuatPayload(
                w=payload["w"], x=payload["x"], y=payload["y"], z=payload["z"]
        )

        ss = time.time()
        timesince = ss - self.last_attitude_quat
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/attitude/quat", quat_update)
            self.last_attitude_quat = ss
        self.fusion_attitude_quat = quat_update

    @try_except(reraise=True)
    def fuse_att_euler(self, payload: AvrVioOrientationEulPayload) -> None:
        """
        Callback for receiving euler att data in NED reference frame from VIO and
        publishes into a fusion/att/euler topic.

        Avr doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        euler_update = AvrFusionAttitudeEulerPayload(
                psi=payload["psi"], theta=payload["theta"], phi=payload["phi"]
        )

        ss = time.time()
        timesince = ss - self.last_attitude_euler
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/attitude/euler", euler_update)
            self.last_attitude_euler = ss
        self.fusion_attitude_euler = euler_update

    @try_except(reraise=True)
    def fuse_att_heading(self, payload: AvrVioHeadingPayload) -> None:
        """
        Callback for receiving heading att data in NED reference frame from VIO and
        publishes into a fusion/att/heading topic.

        Avr doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        heading_update = AvrFusionAttitudeHeadingPayload(heading=payload["degrees"])

        ss = time.time()
        timesince = ss - self.last_attitude_heading
        if timesince >= MSG_INTERVAL:
            # noinspection PyTypeChecker
            self.client.send_message("avr/fusion/attitude/heading", heading_update)
            self.last_attitude_heading = ss
        self.fusion_attitude_heading = heading_update

        # if the groundspeed is below the threshold, we lock the course to the heading

        # if "avr/fusion/groundspeed" not in self.message_cache:
        if self.fusion_groundspeed is None:
            logger.debug("Empty groundspeed in fuse att heading")

        elif (
                # self.message_cache["avr/fusion/groundspeed"]
                self.fusion_groundspeed["groundspeed"] < self.config["COURSE_THRESHOLD"]
        ):
            # self.message_cache["avr/fusion/course"] =
            self.fusion_course = AvrFusionCoursePayload(
                    course=payload["degrees"]
            )

    @try_except(reraise=False)
    def assemble_hil_gps_message(self) -> None:
        """
        This code takes the pos data from fusion and formats it into a special
        message that is exactly what the FCC needs to generate the hil_gps message
        (with heading)
        """
        self.status.update_status("fusion", True)
        while self.running:
            # if "avr/fusion/geo" not in self.message_cache:
            if self.fusion_geo is None:
                logger.debug("Waiting for avr/fusion/geo to be populated")
                time.sleep(1)
                continue

            # geodetic = self.message_cache["avr/fusion/geo"]
            geodetic = self.fusion_geo
            lat = int(geodetic["lat"] * 10000000)  # convert to int32 format
            lon = int(geodetic["lon"] * 10000000)  # convert to int32 format

            # if lat / lon is 0, that means the ned -> lla conversion hasn't run yet,
            # don't send that data to FCC
            if lat == 0 or lon == 0:
                continue

            # if "avr/fusion/velocity/ned" not in self.message_cache:
            if self.fusion_vel_ned is None:
                logger.debug("Waiting for avr/fusion/velocity/ned to be populated")
                time.sleep(1)
                continue
            # elif self.message_cache["avr/fusion/velocity/ned"]["Vn"] is None:
            elif self.fusion_vel_ned["Vn"] is None:
                logger.debug("avr/fusion/velocity/ned/vn message cache is empty")
                time.sleep(1)
                continue

            crs = 0
            # if "avr/fusion/course" in self.message_cache:
            if self.fusion_course is not None:
                # if self.message_cache["avr/fusion/course"]["course"] is not None:
                #     crs = int(self.message_cache["avr/fusion/course"]["course"])
                if self.fusion_course["course"] is not None:
                    crs = int(self.fusion_course["course"])
            else:
                logger.debug("Waiting for avr/fusion/course message to be populated")
                time.sleep(1)
                continue

            gs = 0
            # if "avr/fusion/groundspeed" in self.message_cache:
            if self.fusion_groundspeed is not None:
                # if self.message_cache["avr/fusion/groundspeed"]["groundspeed"] is not None:
                #     gs = int(self.message_cache["avr/fusion/groundspeed"]["groundspeed"])
                if self.fusion_groundspeed["groundspeed"] is not None:
                    gs = int(self.fusion_groundspeed["groundspeed"])
            else:
                logger.debug("avr/fusion/groundspeed message cache is empty")
                time.sleep(1)
                continue

            # if "avr/fusion/attitude/heading" in self.message_cache:
            if self.fusion_attitude_heading is not None:
                heading = int(
                        # self.message_cache["avr/fusion/attitude/heading"]["heading"] * 100
                        self.fusion_attitude_heading["heading"] * 100
                )
            else:
                logger.debug("Waiting for avr/fusion/attitude/heading to be populated")
                time.sleep(1)
                continue

            hil_gps_update = AvrFusionHilGpsPayload(
                    time_usec=int(time.time() * 1000000),
                    fix_type=int(self.config["hil_gps_constants"]["fix_type"]),  # 3 - 3D fix
                    lat=lat,
                    lon=lon,
                    alt=int(
                            # self.message_cache["avr/fusion/geo"]["alt"] * 1000
                            self.fusion_geo["alt"] * 1000
                    ),  # convert m to mm
                    eph=int(self.config["hil_gps_constants"]["eph"]),  # cm
                    epv=int(self.config["hil_gps_constants"]["epv"]),  # cm
                    vel=gs,
                    # vn = int(self.message_cache["avr/fusion/velocity/ned"]["Vn"]),
                    # ve = int(self.message_cache["avr/fusion/velocity/ned"]["Ve"]),
                    # vd = int(self.message_cache["avr/fusion/velocity/ned"]["Vd"]),
                    vn=int(self.fusion_vel_ned["Vn"]),
                    ve=int(self.fusion_vel_ned["Ve"]),
                    vd=int(self.fusion_vel_ned["Vd"]),
                    cog=int(crs * 100),
                    satellites_visible=int(
                            self.config["hil_gps_constants"]["satellites_visible"]
                    ),
                    heading=heading,
            )
            # self.client.send_message("avr/fusion/hil_gps", hil_gps_update)
            self.fcm.gps_fcc.hilgps_msg_handler(hil_gps_update)
            time.sleep(1 / 10)
        self.status.update_status("fusion", False)
        try:
            self.running_barrier.wait(0)
        except BrokenBarrierError:
            pass

    @try_except(reraise=True)
    def on_apriltag_message(self, msg: AvrApriltagsSelectedPayload) -> None:
        if (
                # "avr/fusion/position/ned" not in self.message_cache
                # or "avr/fusion/attitude/heading" not in self.message_cache
                self.fusion_pos_ned is None or self.fusion_attitude_heading is None
        ):
            logger.debug(
                    "Waiting for avr/fusion/position/ned and avr/fusion/attitude/heading to be populated"
            )
            return

        now = time.time()

        # pull ned and heading from cache
        # cam_ned = self.message_cache["avr/fusion/position/ned"]
        # cam_heading = self.message_cache["avr/fusion/attitude/heading"]["heading"]
        cam_ned = self.fusion_pos_ned
        cam_heading = self.fusion_attitude_heading["heading"]

        # get april tag ned and heading
        at_ned = msg["pos"]
        at_heading = msg["heading"]

        # compute differences
        n_dist = abs(at_ned["n"] - cam_ned["n"])
        e_dist = abs(at_ned["e"] - cam_ned["e"])
        d_dist = abs(at_ned["d"] - cam_ned["d"])

        norm = np.linalg.norm([n_dist, e_dist, d_dist])

        heading_delta = abs(at_heading - cam_heading)
        if heading_delta > 180:
            heading_delta = 360 - heading_delta

        for idx, val in enumerate(at_ned.keys()):
            # noinspection PyTypedDict
            self.deriv[idx] = (at_ned[val] - self.last_pos[idx]) / (
                    now - self.last_apriltag
            )
            # noinspection PyTypedDict
            self.last_pos[idx] = at_ned[val]

        deriv_norm = np.linalg.norm(self.deriv)

        if (
                self.norm > self.config["POS_DETLA_THRESHOLD"]
                or abs(heading_delta) > self.config["HEADING_DELTA_THRESHOLD"]
        ) and deriv_norm < self.config["AT_DERIV_THRESHOLD"]:
            logger.debug(f"Resync Triggered! Delta={norm}")

            if d_dist > self.config["POS_D_THRESHOLD"]:
                # don't resync Z if del_d is too great,
                # reject AT readings that are extraneous
                at_ned["d"] = cam_ned["d"]

            resync = AvrVioResyncPayload(
                    n=at_ned["n"],
                    e=at_ned["e"],
                    d=at_ned["d"],
                    heading=at_heading,
            )
            # noinspection PyTypeChecker
            self.client.send_message("avr/vio/resync", resync)

        self.last_apriltag = now

    def run(self) -> None:
        # noinspection PyBroadException
        try:
            self.assemble_hil_gps_message()
        except Exception:
            logger.exception("Issue while assembling hil message")
