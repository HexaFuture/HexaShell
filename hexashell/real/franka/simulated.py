"""A kinematic FR3 with the driver's and the cameras' interfaces, for tests and rehearsals without the robot, as
URAI has for its arms.  It answers with the real kinematics; it has no dynamics, and nothing but the table and one
block in the scene.  Fingers that close round the block hold it, and it falls onto the table when they open.

``--sim`` marks the service's backend mode "sim", so that nothing takes it for a real robot."""

from __future__ import annotations

import threading
import time

import numpy as np

from ..urai.capsules import Capsule, capsule_hit, pixel_rays
from .arm import HAND_OPENING_M, READY_Q, TIP_PAST_TCP_M, TOOL_LENGTH_M, Fr3
from .cameras import Picture
from .planner import TimedPath
from .rig import CameraMount, Rig

BLOCK_EDGE_M = 0.04
#: The block is held when the TCP is this close to its centre as the fingers close
GRASP_REACH_M = 0.02
TABLE_RGB, BLOCK_RGB, ARM_RGB = (150, 140, 125), (200, 30, 30), (235, 235, 235)


def simulated_rig() -> Rig:
    overhead = np.eye(4)
    overhead[:3, :3] = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    overhead[:3, 3] = (0.45, 0.0, 1.1)
    wrist = np.eye(4)
    wrist[:3, 3] = (0.05, 0.0, 0.03)
    return Rig(name="Simulated Franka FR3", robot_ip="", table_z=0.0, tool_length_m=TOOL_LENGTH_M,
               tip_past_tcp_m=TIP_PAST_TCP_M, plan_camera="external", cameras={
        "external": CameraMount("external", "sim-external", t_world_camera=overhead),
        "wrist": CameraMount("wrist", "sim-wrist", t_flange_camera=wrist)})


class SimulatedDriver:
    def __init__(self, arm: Fr3, *, block_xy=(0.45, 0.12), time_scale: float = 1.0):
        self.name = "Simulated Franka FR3"
        self.mode = "sim"
        self.arm = arm
        self.q = np.array(READY_Q, float)
        self.max_width_m = HAND_OPENING_M
        self.width_m = HAND_OPENING_M
        self.time_scale = float(time_scale)
        self.block = np.array([block_xy[0], block_xy[1], arm.table_z + BLOCK_EDGE_M / 2])
        self.held_offset: np.ndarray | None = None
        self.fault: str | None = None
        #: the next path ends in a reflex this far through it (for the tests of the fault path)
        self.fail_at: float | None = None
        self.lock = threading.Lock()

    def joints(self) -> np.ndarray:
        with self.lock:
            return self.q.copy()

    def gripper_width_m(self) -> float:
        return self.width_m

    def health(self) -> dict:
        return {"robot_mode": "Reflex" if self.fault else "Idle", "has_errors": self.fault is not None,
                "errors": self.fault or "", "control_command_success_rate": 1.0}

    def _set(self, q: np.ndarray) -> None:
        with self.lock:
            self.q = q
            if self.held_offset is not None:
                self.block = self.arm.tcp(q)[:3, 3] + self.held_offset

    def follow(self, path: TimedPath, cancel: threading.Event) -> None:
        from .driver import RobotFault

        if self.fault:
            raise RobotFault(self.fault)
        for i in range(1, len(path.q)):
            if cancel.is_set():
                return
            if self.fail_at is not None and path.t[i] >= self.fail_at * path.duration_s:
                self.fault, self.fail_at = "cartesian_reflex", None
                raise RobotFault(self.fault)
            time.sleep((path.t[i] - path.t[i - 1]) * self.time_scale)
            self._set(path.q[i].copy())

    def grip(self, width_m: float, cancel: threading.Event) -> None:
        closing = width_m < self.width_m
        tcp = self.arm.tcp(self.q)
        if closing and self.held_offset is None and np.linalg.norm(tcp[:3, 3] - self.block) < GRASP_REACH_M \
                and width_m < BLOCK_EDGE_M < self.width_m:
            self.held_offset = self.block - tcp[:3, 3]
            self.width_m = BLOCK_EDGE_M
            return
        if self.held_offset is not None and width_m > BLOCK_EDGE_M:
            self.held_offset = None
            self.block = self.block.copy()
            self.block[2] = self.arm.table_z + BLOCK_EDGE_M / 2
        elif self.held_offset is not None:
            return
        self.width_m = float(width_m)

    def recover(self) -> bool:
        self.fault = None
        return True

    def stop(self) -> None:
        pass


class SimulatedCamera:
    """Pictures ray-cast from the table, the block and the arm's capsules."""

    FOCAL = 460.0

    def __init__(self, mount: CameraMount, driver: SimulatedDriver):
        self.mount, self.driver = mount, driver

    def capture(self) -> Picture:
        width, height = self.mount.width, self.mount.height
        k = np.array([[self.FOCAL, 0.0, (width - 1) / 2], [0.0, self.FOCAL * 1.01, (height - 1) / 2], [0.0, 0.0, 1.0]])
        q = self.driver.joints()
        arm = self.driver.arm
        pose = self.mount.pose(arm, q)
        rays = pixel_rays(k, width, height) @ pose[:3, :3].T
        origin = pose[:3, 3]
        depth = np.full(rays.shape[0], np.inf)
        rgb = np.zeros((rays.shape[0], 3), np.uint8)
        with np.errstate(divide="ignore", invalid="ignore"):
            to_table = (arm.table_z - origin[2]) / rays[:, 2]
        hit = np.isfinite(to_table) & (to_table > 0.0)
        depth[hit], rgb[hit] = to_table[hit], TABLE_RGB
        self._box(origin, rays, depth, rgb)
        for capsule in arm.capsules(q):
            cap = Capsule(capsule.a, capsule.b, capsule.radius_m)
            distance, _, hits = capsule_hit(np.broadcast_to(origin, rays.shape), rays, cap)
            nearer = hits & (distance < depth)
            depth[nearer], rgb[nearer] = distance[nearer], ARM_RGB
        depth[~np.isfinite(depth)] = np.nan
        return Picture(rgb.reshape(height, width, 3), depth.reshape(height, width).astype(np.float32), k, time.time())

    def _box(self, origin, rays, depth, rgb) -> None:
        half = BLOCK_EDGE_M / 2
        low, high = self.driver.block - half, self.driver.block + half
        with np.errstate(divide="ignore", invalid="ignore"):
            near = (low - origin) / rays
            far = (high - origin) / rays
        enter = np.nanmax(np.minimum(near, far), axis=1)
        leave = np.nanmin(np.maximum(near, far), axis=1)
        hit = (enter <= leave) & (enter > 0.0) & (enter < depth)
        depth[hit], rgb[hit] = enter[hit], BLOCK_RGB


__all__ = ["SimulatedCamera", "SimulatedDriver", "simulated_rig", "BLOCK_EDGE_M"]
