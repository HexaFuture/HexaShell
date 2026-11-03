"""A fake LIBERO table for the bench tests: a table with a red box, the agentview and wrist cameras,
an arm on a fixed base that gets where it is sent at once (counting the steps it would take).

It reuses the machinery of :mod:`fakes_casa` (rendering, straight-line motions, the gripper, the
wire round trip, the colour SAM3) with LIBERO's frame: origin at the arm's base plate, x toward the
table's middle, y left, z up; the table top at z = 0.  Success: the box lifted 8 cm off the table.
"""

from __future__ import annotations

import math

import numpy as np

import fakes_casa as C
from hexashell.benches.libero import protocol as P
from hexashell.benches.libero.host import CAMS, check_cam
from hexashell.benches.libero.session import LiberoSession
from hexashell.benches.pose import rotation_of

TABLE = C.COUNTER
TABLE_BOX = (np.array([0.15, -0.6, -0.8]), np.array([1.0, 0.6, 0.0]))
TABLE_TOP = 0.0
BOX_SIZE = C.BOX_SIZE
BOX_START = np.array([0.55, -0.05, TABLE_TOP + BOX_SIZE[2] / 2])
SHOULDER = np.array([0.0, 0.0, 0.33])
REACH = 0.85
START_TCP = (np.array([0.452, 0.0, 0.261]), (86.8, 180.0, 180.0))  # as LIBERO: down, tilted 3 degrees back
AGENTVIEW = ((1.15, 0.0, 0.55), (0.40, 0.0, 0.0))
TASK = "libero_object_swap_t0"


class FakeTable(C.FakeKitchen):
    """The simulator side: LIBERO's facts, no base to drive, a table instead of a counter."""

    op_drive = None

    def reset(self) -> None:
        super().reset()
        p, angles = START_TCP
        self.tcp_base[:3, :3] = rotation_of(*angles)
        self.tcp_base[:3, 3] = p
        self.start_tcp_base = self.tcp_base.copy()
        self.box = BOX_START.copy()

    def shoulder(self) -> np.ndarray:
        return SHOULDER.copy()

    def proprio(self) -> dict:
        return {"tcp": self.tcp(), "width": float(self.width), "joints": np.zeros(7)}

    def row(self) -> list[float]:
        tcp = self.tcp()
        return [self.steps, *tcp[:3, 3], *C.rotvec(tcp[:3, :3]), self.width]

    def _tick(self, rows: list) -> bool:
        self.steps += 1
        if self.held is not None:
            self.box = self.tcp()[:3, 3] + self.held
            if self.box[2] - BOX_SIZE[2] / 2 >= TABLE_TOP + 0.08:
                self.success = True
        rows.append(self.row())
        return not self.over

    def handle(self, op: str, args: dict) -> dict:
        if op != "open":
            return super().handle(op, args)
        self.ops.append(op)
        self.reset()
        suite, index = P.parse_task(args["task"])
        return {"facts": {"task": args["task"], "suite": suite, "task_index": index, "seed": args["seed"],
                          "init_state": P.init_index(args["seed"]), "instruction": "pick up the red box",
                          "task_label": "pick_up_the_red_box", "robot": "MountedPanda", "horizon": self.horizon,
                          "step_s": P.STEP_S, "settle_steps": P.SETTLE_STEPS},
                "start": self.proprio(), "seconds": 0.01, "level": self.level}

    def _reply(self, result: dict, rows: list) -> dict:
        log = np.asarray(rows, float) if rows else np.zeros((0, 8))
        return {"result": result, "proprio": self.proprio(), "log": log}

    def op_observe(self, cams=None, robot_mask=True) -> dict:
        out = {"step": self.steps, "images": {}, "depth": {}, "robot": {}, "cameras": {}}
        focal = 0.5 * self.size / math.tan(math.radians(C.FOVY) / 2)
        boxes = [(*TABLE_BOX, TABLE), (self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2, C.BOX)]
        tcp = self.tcp()
        for cam in cams or CAMS:
            if cam == "wrist":
                pose = np.eye(4)
                pose[:3, 0], pose[:3, 1], pose[:3, 2] = tcp[:3, 1], -tcp[:3, 2], -tcp[:3, 0]
                pose[:3, 3] = tcp[:3, 3] - 0.10 * tcp[:3, 0]
                spheres = []
            else:
                pose = C.look_at(*AGENTVIEW)
                spheres = [(tcp[:3, 3], C.HAND_R, C.HAND)]
            rgb, depth, ids = C.render(pose, focal, self.size, boxes, spheres)
            out["images"][cam], out["depth"][cam] = rgb, depth
            out["robot"][cam] = ids == C.HAND
            out["cameras"][cam] = {"pose": pose, "focal": focal, "width": self.size, "height": self.size}
        out["proprio"] = self.proprio()
        return {"obs": out}

    def _check(self, pose: np.ndarray) -> dict:
        p = pose[:3, 3]
        dist = float(np.linalg.norm(p - SHOULDER))
        if p[2] < -0.05:
            return {"ok": False, "why": "below the floor limit", "err_m": 0.0, "err_deg": 0.0}
        if dist > REACH:
            return {"ok": False, "why": f"{dist:.2f} m from the shoulder (the arm reaches {REACH} m)",
                    "err_m": dist - REACH, "err_deg": 0.0}
        return {"ok": True, "err_m": 0.0005, "err_deg": 0.1, "limit_margin": 0.6, "joints": np.zeros(7)}

    def _run(self, poses: list[np.ndarray]) -> tuple[list, bool]:
        """Follow poses of the hand, one per step; stops at the table top (blocked)."""
        rows, blocked = [], False
        for pose in poses:
            if self.over:
                break
            if self.held is None and pose[0, 3] > TABLE_BOX[0][0] and pose[2, 3] < TABLE_TOP + 0.01:
                blocked = True
                break
            self.tcp_base = pose.copy()
            if not self._tick(rows):
                break
        return rows, blocked

    def _set_grip(self, opening: float) -> None:
        held = self.held is not None
        super()._set_grip(opening)
        if held and self.held is None:  # let go: the box drops onto the table
            self.box[2] = TABLE_TOP + BOX_SIZE[2] / 2


class FakeSam3(C.FakeSam3):
    WORDS = {"red": C.BOX, "table": TABLE}


class FakeLiberoSession(LiberoSession):
    """A LiberoSession whose server is the fake table and whose SAM3 is the colour one."""

    def __init__(self, task: str = TASK, seed: int = 0, *, size: int = 256, sam3: bool = True,
                 table: FakeTable | None = None):
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused", size=size)
        self.kitchen = table or FakeTable(size=size)
        self.sam3 = FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = C.FakeLink(self.kitchen)

    def _scene(self, settings):
        if self.sam3 is None:
            return None
        from hexashell.vision.rgbd import DepthScene

        return DepthScene("fake", cams=list(CAMS), check_cam=check_cam, log=self.rec.event, client=self.sam3)
