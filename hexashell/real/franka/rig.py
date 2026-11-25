"""What one FR3 cell is: read from a JSON file, nothing about a real cell is built in.

    {"name": "Franka FR3", "robot_ip": "<the robot's FCI address>", "table_z": 0.0,
     "t_world_base": [[...4x4...]], "tool_length_m": 0.1034, "tip_past_tcp_m": 0.009, "realtime": true, "dynamics_factor": 0.1,
     "gripper_speed_m_s": 0.05, "grasp_force_n": 20.0, "max_speed_m_s": 0.25, "plan_camera": "external",
     "cameras": {"external": {"serial": "<RealSense serial>", "width": 640, "height": 480, "fps": 30,
                              "t_world_camera": [[...4x4...]]},
                 "wrist": {"serial": "<RealSense serial>", "t_flange_camera": [[...4x4...]]}}}

``name``, ``robot_ip``, ``table_z``, ``tool_length_m``, ``tip_past_tcp_m``, ``plan_camera`` and ``cameras`` are
required, each camera with a serial and
one pose: ``t_world_camera`` for a fixed one, ``t_flange_camera`` (fr3_link8 to the colour optical frame, in OpenCV's
axes) for one on the hand.  ``t_world_base`` defaults to the identity.  ``table_z`` is measured, so it has no default.
``tool_length_m`` is the flange to the gripping centre and ``tip_past_tcp_m`` the centre to the finger tips: both
depend on the fingers fitted, and a default would let the table check pass a finger that hits the table.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .arm import Fr3


@dataclass(frozen=True)
class CameraMount:
    name: str
    serial: str
    width: int = 640
    height: int = 480
    fps: int = 30
    t_world_camera: np.ndarray | None = None
    t_flange_camera: np.ndarray | None = None
    temporal_filter_alpha: float | None = None

    def pose(self, arm: Fr3, q) -> np.ndarray:
        """The camera's world pose with the joints at ``q``."""
        if self.t_world_camera is not None:
            return self.t_world_camera
        return arm.flange(q) @ self.t_flange_camera


@dataclass(frozen=True)
class Rig:
    name: str
    robot_ip: str
    table_z: float
    tool_length_m: float
    tip_past_tcp_m: float
    plan_camera: str
    cameras: dict = field(default_factory=dict)
    t_world_base: np.ndarray = field(default_factory=lambda: np.eye(4))
    realtime: bool = True
    dynamics_factor: float = 0.1
    gripper_speed_m_s: float = 0.05
    grasp_force_n: float = 20.0
    max_speed_m_s: float = 0.25

    def arm(self) -> Fr3:
        return Fr3(self.t_world_base, table_z=self.table_z, tool_length_m=self.tool_length_m,
                   tip_past_tcp_m=self.tip_past_tcp_m)


def _pose(value, what: str) -> np.ndarray:
    pose = np.asarray(value, float)
    if pose.shape != (4, 4):
        raise ValueError(f"{what} must be a 4x4 matrix")
    return pose


def _mount(name: str, data: dict) -> CameraMount:
    poses = [key for key in ("t_world_camera", "t_flange_camera") if key in data]
    if len(poses) != 1:
        raise ValueError(f"camera {name}: give exactly one of t_world_camera and t_flange_camera")
    fields = {key: value for key, value in data.items() if key not in poses}
    return CameraMount(name=name, **fields, **{poses[0]: _pose(data[poses[0]], f"camera {name}: {poses[0]}")})


def rig_from_dict(data: dict) -> Rig:
    data = dict(data)
    cameras = {name: _mount(name, item) for name, item in data.pop("cameras").items()}
    if "t_world_base" in data:
        data["t_world_base"] = _pose(data["t_world_base"], "t_world_base")
    rig = Rig(cameras=cameras, **data)
    if rig.plan_camera not in cameras:
        raise ValueError(f"plan_camera {rig.plan_camera!r} is not one of the cameras {sorted(cameras)}")
    return rig


def load_rig(path: str | Path) -> Rig:
    return rig_from_dict(json.loads(Path(path).read_text()))
