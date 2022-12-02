import math
import os
import subprocess
import time
import warnings
from threading import Thread
from typing import Any, List, Optional, Tuple

import numpy as np
import transforms3d as t3d
from bell.avr.mqtt.payloads import (
    AvrApriltagsRawPayload,
    AvrApriltagsRawTags,
    AvrApriltagsSelectedPayload,
    AvrApriltagsVisiblePayload,
    AvrApriltagsVisibleTags,
    AvrApriltagsVisibleTagsPosWorld,
)

from ...mqtt_client import MQTTClient
from ...status import Status

warnings.simplefilter("ignore", np.RankWarning)


class AprilTagModule:
    def __init__(self, status: Status) -> None:
        self.client = MQTTClient.get()

        self.status = status

        self.process: subprocess.Popen | None = None

        self.config: dict = {
            "cam": {
                "pos": [0, 0, 8.5],  # cm from FC forward, right, down
                "rpy": [
                    0,
                    0,
                    math.pi / 2,
                ],  # cam x = body -y; cam y = body x, cam z = body z
            },
            "tag_truth": {"0": {"rpy": [0, 0, 0], "xyz": [0, 0, 0]}},
        }

        # dict to hold transformation matrices
        self.tm = {}
        # setup transformation matrices
        self.setup_transforms()

        self.client.register_callback("avr/apriltags/raw", self.on_apriltag_message)

        self.detections = {}

        self.visible_detections = (time.time(), {})
        self.closest_tag = (None, -1)

    def close(self) -> None:
        self.process.terminate()
        time.sleep(1)  # wait for apriltag to be terminated

    def get_valid_apriltags(self, cutoff: float) -> dict[int, Any]:
        valid_apriltags = {}
        for tid, t in self.detections:
            tag, dtime = t
            time_offset = time.time() - dtime
            if time_offset > cutoff:
                valid_apriltags[tid] = tag
        return valid_apriltags

    def setup_transforms(self) -> None:
        cam_rpy = self.config["cam"]["rpy"]

        # rotation matrix
        rmat = t3d.euler.euler2mat(
                cam_rpy[0],
                cam_rpy[1],
                cam_rpy[2],
                axes="rxyz",
        )

        h_cam_aero_body = t3d.affines.compose(self.config["cam"]["pos"], rmat, [1, 1, 1])
        h_aero_body_cam = np.linalg.inv(h_cam_aero_body)

        self.tm["H_aeroBody_cam"] = h_aero_body_cam

        for tag, tag_data in self.config["tag_truth"].items():
            name = f"tag_{tag}"
            rmat = t3d.euler.euler2mat(
                    tag_data["rpy"][0], tag_data["rpy"][1], tag_data["rpy"][2], axes="rxyz"
            )
            tag_tf = t3d.affines.compose(tag_data["xyz"], rmat, [1, 1, 1])

            h_to_from = f"H_{name}_aeroRef"
            self.tm[h_to_from] = tag_tf

            h_to_from = f"H_{name}_cam"
            self.tm[h_to_from] = np.eye(4)

    def on_apriltag_message(self, payload: AvrApriltagsRawPayload) -> None:
        tag_list: List[AvrApriltagsVisibleTags] = []

        min_dist = 1000000
        closest_tag = None

        for index, tag in enumerate(payload["tags"]):
            (
                id_,
                horizontal_distance,
                vertical_distance,
                angle,
                pos_world,
                pos_rel,
                heading,
            ) = self.handle_tag(tag)

            # weird special case (this shouldn't really happen though?)
            if id_ is None:
                continue

            tag = AvrApriltagsVisibleTags(
                    id=id_,
                    horizontal_dist=horizontal_distance,
                    vertical_dist=vertical_distance,
                    angle_to_tag=angle,
                    heading=heading,
                    pos_rel={
                        "x": pos_rel[0],
                        "y": pos_rel[1],
                        "z": pos_rel[2],
                    },
                    pos_world={
                        "x": None,
                        "y": None,
                        "z": None,
                    },
            )
            # print(f"Rel: {pos_rel}")

            # add some more info if we had the truth data for the tag
            if pos_world is not None and pos_world.any():
                tag["pos_world"] = AvrApriltagsVisibleTagsPosWorld(
                        x=pos_world[0],
                        y=pos_world[1],
                        z=pos_world[2],
                )
                if horizontal_distance < min_dist:
                    min_dist = horizontal_distance
                    closest_tag = index

            tag_list.append(tag)
            self.detections[id_] = (tag, time.time())
        self.visible_detections = (time.time(), tag_list)
        # print("Tag: " + str(tag_list))

        self.client.send_message(
                "avr/apriltags/visible", AvrApriltagsVisiblePayload(tags=tag_list)
        )

        if closest_tag is not None:
            closest_tag_dict = self.visible_detections[closest_tag]
            self.closest_tag = (closest_tag_dict, time.time())

            pos_world = tag_list[closest_tag]["pos_world"]

            # this shouldn't happen
            assert pos_world["x"] is not None
            assert pos_world["y"] is not None
            assert pos_world["z"] is not None

            apriltag_position = AvrApriltagsSelectedPayload(
                    tag_id=tag_list[closest_tag]["id"],
                    pos={
                        "n": pos_world["x"],
                        "e": pos_world["y"],
                        "d": pos_world["z"],
                    },
                    heading=tag_list[closest_tag]["heading"],
            )

            self.client.send_message("avr/apriltags/selected", apriltag_position)

    @staticmethod
    def angle_to_tag(pos: Tuple[float, float, float]) -> float:
        deg = math.degrees(
                math.atan2(pos[1], pos[0])
        )  # TODO - i think plus pi/2 bc this is respect to +x

        if deg < 0.0:
            deg += 360.0

        return deg

    def world_angle_to_tag(
            self, pos: Tuple[float, float, float], tag_id: int
    ) -> Optional[float]:
        """
        returns the angle with respect to "north" in the "world frame"
        """
        if str(tag_id) not in self.config["tag_truth"].keys():
            return

        del_x = self.config["tag_truth"][str(tag_id)]["xyz"][0] - pos[0]
        del_y = self.config["tag_truth"][str(tag_id)]["xyz"][1] - pos[1]
        deg = math.degrees(
                math.atan2(del_y, del_x)
        )  # TODO - i think plus pi/2 bc this is respect to +x

        if deg < 0.0:
            deg += 360.0

        return deg

    @staticmethod
    def h_inv(h: np.ndarray) -> np.ndarray:
        """
        A method to efficiently compute the inverse of a homogeneous transformation
        matrix. Reference: http://vr.cs.uiuc.edu/node81.html
        """

        t, r, z, s = t3d.affines.decompose44(h)

        r_t = np.transpose(r)
        h_rot = t3d.affines.compose(
                [0, 0, 0],
                r_t,
                [1, 1, 1],
        )
        h_tran = t3d.affines.compose(
                [-1 * t[0], -1 * t[1], -1 * t[2]],
                np.eye(3),
                [1, 1, 1],
        )

        return h_rot.dot(h_tran)

    def handle_tag(
            self, tag: AvrApriltagsRawTags
    ) -> Tuple[
        int,
        float,
        float,
        float,
        Optional[np.ndarray],
        Tuple[float, float, float],
        float,
    ]:
        """
        Calculates the distance, position, and heading of the drone in NED frame
        based on the tag detections.
        """

        tag_id = tag["id"]

        tag_rot = np.asarray(tag["rotation"])
        rpy = t3d.euler.mat2euler(tag_rot)
        r = t3d.euler.euler2mat(0, 0, rpy[2], axes="rxyz")
        h_tag_cam = t3d.affines.compose(
                [tag["pos"]["x"] * 100, tag["pos"]["y"] * 100, tag["pos"]["z"] * 100],
                r,
                [1, 1, 1],
        )
        t, r, z, s = t3d.affines.decompose44(h_tag_cam)

        name = f"tag_{tag_id}"
        h_to_from = f"H_{name}_cam"
        self.tm[h_to_from] = h_tag_cam

        # H_cam_tag = np.linalg.inv(H_tag_cam)
        h_cam_tag = self.h_inv(h_tag_cam)

        h_aerobody_tag = h_cam_tag.dot(self.tm["H_aeroBody_cam"])

        t2, r2, z2, s2 = t3d.affines.decompose44(h_aerobody_tag)
        rpy = t3d.euler.mat2euler(r2)
        pos_rel: Tuple[float, float, float] = t2  # type: ignore

        horizontal_distance: float = np.linalg.norm([pos_rel[0], pos_rel[1]])  # type: ignore
        vertical_distance = abs(pos_rel[2])

        heading = rpy[2]
        if heading < 0:
            heading += 2 * math.pi

        heading: float = np.rad2deg(heading)
        angle = self.angle_to_tag(pos_rel)  # type: ignore

        # if we have a location definition for the visible tag
        if str(tag["id"]) in self.config["tag_truth"].keys():
            h_cam_aero_ref = self.tm[f"H_{name}_aeroRef"].dot(h_cam_tag)
            h_aero_body_aero_ref = h_cam_aero_ref.dot(self.tm["H_aeroBody_cam"])

            pos_world, r, z, s = t3d.affines.decompose44(h_aero_body_aero_ref)

            return (
                tag_id,
                horizontal_distance,
                vertical_distance,
                angle,
                pos_world,
                pos_rel,
                heading,
            )
        else:
            return (
                tag_id,
                horizontal_distance,
                vertical_distance,
                angle,
                None,
                pos_rel,
                heading,
            )

    def _status_loop(self) -> None:
        self.status.update_status("apriltag", True)
        while self.process.poll() is None:
            pass
        self.status.update_status("apriltag", False)

    def run(self) -> None:
        dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../c/avrapriltags")
        os.system(f"chmod a+x {dir_path}")
        self.process = subprocess.Popen(dir_path)
        Thread(target=self._status_loop, daemon=True).start()
