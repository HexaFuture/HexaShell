"""URAI's pictures as HexaShell's: one ``/api/harness/observe`` packet (lossless RGB-D as npy in base64,
the camera's K and pose, the arms, their collision capsules) as a :class:`~hexashell.vision.rgbd.DepthView`."""

from __future__ import annotations

import base64
import io
from dataclasses import dataclass

import numpy as np

from ...vision.cameras import Camera
from ...vision.rgbd import DepthView
from .capsules import Capsule, capsule_hit, capsule_window, pixel_rays

#: HexaShell's camera names -> URAI's
CAMERA_TO_URAI = {"head": "head", "left_wrist": "hand_left", "right_wrist": "hand_right"}
URAI_TO_CAMERA = {remote: name for name, remote in CAMERA_TO_URAI.items()}

#: a pixel whose measured depth is within this (m) of where its ray meets an arm's capsule shows the robot;
#: what is in front of the arm keeps its own depth and is not the robot's
ROBOT_SURFACE_TOLERANCE_M = 0.01


def robot_pixels(depth: np.ndarray, k: np.ndarray, t_world_camera: np.ndarray, capsules: dict) -> np.ndarray:
    """The pixels that show the robot: the capsules' depth along each ray, against the measured depth."""
    height, width = depth.shape
    origin = t_world_camera[:3, 3]
    directions = pixel_rays(k, width, height) @ t_world_camera[:3, :3].T
    t_camera_world = np.linalg.inv(t_world_camera)
    robot_depth = np.full(depth.size, np.inf)
    for arm_capsules in capsules.values():
        for data in arm_capsules:
            cap = Capsule(np.asarray(data["a"], float), np.asarray(data["b"], float), data["radius_m"])
            indices = capsule_window(cap, k, t_camera_world, width, height)
            if indices is None:
                continue
            distances, _, hits = capsule_hit(np.broadcast_to(origin, (len(indices), 3)), directions[indices], cap)
            visible = hits & (distances < robot_depth[indices])
            robot_depth[indices[visible]] = distances[visible]
    robot_depth = robot_depth.reshape(depth.shape)
    return np.isfinite(depth) & np.isfinite(robot_depth) & (np.abs(depth - robot_depth) <= ROBOT_SURFACE_TOLERANCE_M)


@dataclass
class Observation:
    id: str
    timestamp: float
    view: DepthView
    arms: dict

    @classmethod
    def from_packet(cls, packet: dict, *, step: int) -> Observation:
        name = URAI_TO_CAMERA[packet["camera"]]
        rgb = np.load(io.BytesIO(base64.b64decode(packet["rgb"]["data"])), allow_pickle=False)
        depth = np.load(io.BytesIO(base64.b64decode(packet["depth"]["data"])), allow_pickle=False)
        k = np.asarray(packet["k"], float)
        t_world_camera = np.asarray(packet["t_world_camera"], float)
        height, width = depth.shape
        camera = Camera.from_cv(name, k, t_world_camera, width, height)
        robot = robot_pixels(depth, k, t_world_camera, packet["capsules"])
        view = DepthView(name, rgb, depth, camera, robot, step=step, frame_id=packet["id"])
        return cls(packet["id"], packet["timestamp"], view, packet["arms"])
