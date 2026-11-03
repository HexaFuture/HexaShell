"""A fake RoboTwin 2.0 table for the bench tests: RoboTwin's two arms (the chains of the RoboTwin rig;
a step takes them to their joint targets, the fingers move at most 0.45 of their range per step), a
table with a red box, the head camera and the two wrist cameras with depth and the robot's pixels (or
without depth, with the robot's parts), and a colour-based SAM3.  The table top has a pattern that the
pictures without depth are matched by.  Success: the box lifted 8 cm off the table.

:class:`FakeTwin` answers the operations of :mod:`hexashell.benches.robotwin.server` with replies of
the same shape, through :class:`fakes_casa.FakeLink` (the wire encoding), so the episode side
(session, host, motion, perception, the cells' robot) runs as it does against the real server.
World frame as in RoboTwin: x to the robot's right, y away from it, z up; the table top at 0.728.
"""

from __future__ import annotations

import math

import numpy as np

import fakes_casa as C
from hexashell.benches.robotwin import protocol as P
from hexashell.benches.robotwin.host import closing_yaw
from hexashell.benches.robotwin.rig import ROBOTWIN
from hexashell.benches.robotwin.session import RoboTwinSession
from hexashell.motion.chain import arm_chain, slide
from hexashell.vision.cameras import check_cam

ARMS = ("left", "right")
TASK = "adjust_bottle"  # any task of the list: the fake has its own scene
SEED = 7  # a development seed: the instruction is the fake's own
TABLE = C.COUNTER
TABLE_TOP = 0.728
TABLE_BOX = (np.array([-0.6, -0.35, 0.0]), np.array([0.6, 0.35, TABLE_TOP]))
BOX_SIZE = np.array([0.04, 0.06, 0.05])  # narrow along x: the fingers close along x (yaw 0)
BOX_START = np.array([-0.15, 0.0, TABLE_TOP + BOX_SIZE[2] / 2])
#: RoboTwin's cameras: 320 x 240, focal 358.64 px; the head camera's pose, the wrist cameras' mount on link6
WIDTH, HEIGHT, FOCAL = 320, 240, 358.6422
HEAD = np.array([[1.0, 0.0, 0.0, -0.032], [0.0, 0.8, -0.6, -0.45], [0.0, 0.6, 0.8, 1.35], [0.0, 0.0, 0.0, 1.0]])
WRIST_MOUNT = np.array([[0.0, 0.38942, -0.92106, 0.07], [-1.0, 0.0, 0.0, 0.032], [0.0, 0.92106, 0.38942, 0.065],
                        [0.0, 0.0, 0.0, 1.0]])
GRIP_RATE = 0.45
PADS_TOUCH = 0.1818  # the reading with nothing between the fingers
HAND_R = 0.03
#: the table top's pattern: square cells PATTERN_M wide, each brighter or darker by up to PATTERN_GAIN
PATTERN_M = 0.008
PATTERN_GAIN = 20
#: ids the hands' parts are drawn with to tell them apart (PART_ID + their P.ROBOT_PARTS number)
PART_ID = 100


def part(name: str) -> int:
    return P.ROBOT_PARTS.index(name) + 1


def patterned(rgb: np.ndarray, depth: np.ndarray, ids: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """The picture with the table top's pattern, fixed on the table (each cell the same in every camera)."""
    h, w = ids.shape
    ys, xs = np.mgrid[0:h, 0:w].astype(float)
    local = np.stack([(xs - (w - 1) / 2.0) / FOCAL, -(ys - (h - 1) / 2.0) / FOCAL, -np.ones_like(xs)], -1)
    world = pose[:3, 3] + depth[..., None] * (local @ pose[:3, :3].T)
    cx, cy = (np.floor(world[..., k] / PATTERN_M).astype(np.int64) for k in (0, 1))
    shade = (cx * 73856093 ^ cy * 19349663) % (2 * PATTERN_GAIN + 1) - PATTERN_GAIN
    out = rgb.astype(int)
    table = ids == TABLE
    out[table] += shade[table][:, None]
    return np.clip(out, 0, 255).astype(np.uint8)


def reading_for(width: float) -> float:
    """The gripper reading with the pads ``width`` apart (RoboTwin's finger travel)."""
    return (width + 0.02) / 0.11


class FakeTwin:
    """The simulator side: state, operations, status."""

    def __init__(self, horizon: int = 600):
        self.horizon = horizon
        self.closed = False
        self.ops: list[str] = []
        self.observed: list[list[str]] = []  # the cameras each observe asked for
        self.chains = {arm: arm_chain(arm, ROBOTWIN) for arm in ARMS}

    # -- state -----------------------------------------------------------------------------------
    def reset(self) -> None:
        self.steps = 0
        self.success = False
        self.q = {arm: np.zeros(6) for arm in ARMS}  # RoboTwin's arms start at zero joint angles
        self.grip = {arm: 1.0 for arm in ARMS}
        self.command = {arm: 1.0 for arm in ARMS}
        self.box = BOX_START.copy()
        self.held: tuple[str, np.ndarray] | None = None  # (arm, box centre in the flange frame)
        self.blocked = 0
        self.extra: list[np.ndarray] = []  # centres of more red boxes, only drawn (for a test)

    @property
    def over(self) -> bool:
        return self.success or self.steps >= self.horizon

    def status(self) -> dict:
        return {"step": self.steps, "horizon": self.horizon, "success": self.success, "over": self.over}

    def flange(self, arm: str) -> np.ndarray:
        return self.chains[arm].flange(self.q[arm])

    def grasp_point(self, arm: str) -> np.ndarray:
        return slide(self.flange(arm), ROBOTWIN.grasp_reach)[:3, 3]

    def proprio(self) -> dict:
        return {arm: {"joints": self.q[arm].copy(), "flange": self.flange(arm), "gripper": float(self.grip[arm]),
                      "target": self.q[arm].copy(), "grip_command": float(self.command[arm])} for arm in ARMS}

    # -- one step --------------------------------------------------------------------------------
    def _into_table(self, arm: str, q) -> bool:
        tip = slide(self.chains[arm].flange(q), ROBOTWIN.tip_reach)[:3, 3]
        lo, hi = TABLE_BOX
        return bool(lo[0] <= tip[0] <= hi[0] and lo[1] <= tip[1] <= hi[1] and tip[2] < TABLE_TOP)

    def _between_fingers(self, arm: str) -> bool:
        pose = slide(self.flange(arm), ROBOTWIN.grasp_reach)
        p, closing = pose[:3, 3], pose[:3, 1]
        off = np.abs(p - self.box)
        inside = off[0] <= BOX_SIZE[0] / 2 + 0.01 and off[1] <= BOX_SIZE[1] / 2 + 0.005 \
            and -BOX_SIZE[2] / 2 + 0.005 <= p[2] - self.box[2] <= BOX_SIZE[2] / 2 + 0.01
        return bool(inside and abs(closing[0]) > math.cos(math.radians(25.0)))

    def _step(self, action: np.ndarray) -> None:
        for i, arm in enumerate(ARMS):
            q = action[7 * i:7 * i + 6]
            if self._into_table(arm, q):
                self.blocked += 1  # stays where it is, as an arm pressed on the table
            else:
                self.q[arm] = q.copy()
            self.command[arm] = float(action[7 * i + 6])
        for arm in ARMS:
            goal = self.command[arm]
            g = self.grip[arm] + float(np.clip(goal - self.grip[arm], -GRIP_RATE, GRIP_RATE))
            holding = self.held is not None and self.held[0] == arm
            if goal < self.grip[arm]:  # closing
                width_g = reading_for(BOX_SIZE[0])
                if holding or (self.held is None and self._between_fingers(arm) and g <= width_g):
                    g = max(g, width_g)
                    if not holding:
                        inv = np.linalg.inv(self.flange(arm))
                        self.held = (arm, (inv @ np.r_[self.box, 1.0])[:3])
                g = max(g, PADS_TOUCH)
            elif holding and g > reading_for(BOX_SIZE[0]) + 0.05:  # let go: the box drops onto the table
                self.held = None
                self.box[2] = TABLE_TOP + BOX_SIZE[2] / 2
            self.grip[arm] = g
        if self.held is not None:
            arm, local = self.held
            self.box = (self.flange(arm) @ np.r_[local, 1.0])[:3]
            if self.box[2] - BOX_SIZE[2] / 2 >= TABLE_TOP + 0.08:
                self.success = True
        self.steps += 1

    # -- operations ------------------------------------------------------------------------------
    def handle(self, op: str, args: dict) -> dict:
        self.ops.append(op)
        if op == "ping":
            return {"pid": 0}
        if op == "open":
            self.reset()
            task, seed = P.check_task(args["task"]), int(args["seed"])
            try:
                P.check_seed(task, seed)
            except ValueError as exc:
                raise C.Refusal(str(exc)) from None
            facts = {"task": task, "seed": seed, "instruction": "pick up the red box",
                     "instruction_from": f"the task's {P.instruction_kind(seed)} descriptions",
                     "step_limit": self.horizon, "setup_s": 0.01, "cameras": list(P.CAMERAS)}
            return {"facts": facts, "start": self.proprio(), "seconds": 0.01}
        if op == "close":
            self.closed = True
            return {}
        if not hasattr(self, "steps"):
            raise C.Refusal("no episode: send open first")
        if op == "act":
            a = np.asarray(args["action"], float).reshape(-1)
            if a.size != 14 or not np.all(np.isfinite(a)):
                raise C.Refusal("an action is 14 finite numbers")
            a[6], a[13] = min(max(a[6], 0.0), 1.0), min(max(a[13], 0.0), 1.0)
            if not self.over:
                self._step(a)
            return {"proprio": self.proprio()}
        if op == "state":
            return {"proprio": self.proprio()}
        if op == "observe":
            return {"obs": self.observe(args.get("cams"), bool(args.get("robot_mask", True)),
                                        bool(args.get("depth", True)), bool(args.get("parts", False)))}
        raise C.Refusal(f"unknown operation {op!r}")

    def observe(self, cams=None, robot_mask: bool = True, depth: bool = True, parts: bool = False) -> dict:
        names = list(P.CAMERAS) if not cams else [str(c) for c in cams]
        self.observed.append(names)
        for name in names:
            if name not in P.CAMERAS:
                raise C.Refusal(f"no camera {name!r}")
        boxes = [(*TABLE_BOX, TABLE), (self.box - BOX_SIZE / 2, self.box + BOX_SIZE / 2, C.BOX)]
        boxes += [(c - BOX_SIZE / 2, c + BOX_SIZE / 2, C.BOX) for c in self.extra]
        out = {"images": {}, "depth": {}, "robot": {}, "cameras": {}, "step": self.steps, "render_s": 0.0}
        if parts:
            out["parts"] = {}
        for name in names:
            own = name.split("_")[0] if name != "head" else None
            pose = HEAD if own is None else self.flange(own) @ WRIST_MOUNT
            hands = [(self.grasp_point(arm), part(f"{arm} fingers")) for arm in ARMS if arm != own]
            hands += [(self.flange(arm)[:3, 3], part(f"{arm} arm")) for arm in ARMS if arm != own]
            rgb, z, ids = C.render(pose, FOCAL, (WIDTH, HEIGHT), boxes, [(c, HAND_R, C.HAND) for c, _ in hands])
            out["images"][name] = patterned(rgb, z, ids, pose)
            if depth:
                out["depth"][name] = z
            if robot_mask:
                out["robot"][name] = ids == C.HAND
            if parts:
                _, _, tagged = C.render(pose, FOCAL, (WIDTH, HEIGHT), boxes,
                                        [(c, HAND_R, PART_ID + k) for c, k in hands])
                out["parts"][name] = np.where(tagged > PART_ID, tagged - PART_ID, 0).astype(np.uint8)
            out["cameras"][name] = {"focal": FOCAL, "focal_y": FOCAL, "center": (WIDTH / 2, HEIGHT / 2),
                                    "pose": pose, "width": WIDTH, "height": HEIGHT}
        return out


class FakeSam3(C.FakeSam3):
    WORDS = {"red": C.BOX, "table": TABLE, "desk": TABLE}

    @staticmethod
    def classes(image: np.ndarray) -> np.ndarray:
        out = C.FakeSam3.classes(image)
        near = np.all(np.abs(np.asarray(image, int) - C.COLOURS[TABLE]) <= PATTERN_GAIN, axis=-1)
        out[near & (out == 0)] = TABLE  # the table top with its pattern
        return out


class FakeTwinSession(RoboTwinSession):
    """A RoboTwinSession whose server is the fake table and whose SAM3 is the colour one."""

    def __init__(self, task: str = TASK, seed: int = SEED, *, sam3: bool = True, twin: FakeTwin | None = None):
        super().__init__({"task_name": task, "eval_seed": seed}, python="unused")
        self.kitchen = twin or FakeTwin()
        self.sam3 = FakeSam3() if sam3 else None

    def start_server(self, log_path) -> None:
        self.server = C.FakeLink(self.kitchen)

    def _scene(self, settings):
        if self.sam3 is None:
            return None
        if not self.depth:
            from hexashell.benches.robotwin.scene import TwinScene

            return TwinScene("fake", log=self.rec.event, client=self.sam3)
        from hexashell.vision.rgbd import DepthScene

        return DepthScene("fake", cams=list(P.CAMERAS), check_cam=check_cam, log=self.rec.event, client=self.sam3,
                          closing_yaw=closing_yaw)
