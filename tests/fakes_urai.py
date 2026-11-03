"""A URAI service in process, for the tests of hexashell/real/urai: its harness seam (``GET /api/harness/model``,
``POST /api/harness/observe``) and the motion calls (``PUT /api/draft``, ``POST /api/preview``,
``POST /api/execute``, ``GET /api/state``) answered through httpx's MockTransport, so that the real
:class:`~hexashell.real.urai.client.UraiClient` talks to it.

Nothing is planned or simulated: two TCPs over a table at z = 0 with a red 40 mm block.  A goto ends where
it was sent (plus ``miss``); its path is the straight line, or bulges up by ``bend`` for ``motion="swing"``;
an execution takes the plan's duration (plus ``overhead_s``) at once.  Fingers that close round the block
hold it, and it falls back onto the table when they open.  The pictures are ray-cast from a head camera
above the table and one camera on each wrist, all with fx != fy.
"""

from __future__ import annotations

import base64
import copy
import io
import json
import math

import httpx
import numpy as np

from hexashell.real.urai.client import UraiClient

URL = "http://urai.test"
TABLE_Z = 0.0
EDGE_M = 0.04  # the block's edge
BLOCK_XY = (0.27, -0.12)
MAX_OPENING_MM = 70.0
REACH_M = 0.7  # farther than this from its base, an arm has no plan
BASES = {"left": (0.0, 0.0, 0.0), "right": (0.0, -0.6, 0.0)}
#: rpy_deg of a tool pointing down whose fingers close along world x (HexaShell: pitch 90, yaw 0, roll 0)
DOWN_RPY = [180.0, 0.0, 180.0]
START_XYZ = {"left": [0.25, 0.1, 0.25], "right": [0.25, -0.35, 0.25]}
TOLERANCE = {"position_mm": 3.0, "orientation_deg": 2.0}
WIDTH, HEIGHT = 200, 150
HEAD_K = [[240.0, 0.0, 99.5], [0.0, 220.0, 74.5], [0.0, 0.0, 1.0]]
WRIST_K = [[130.0, 0.0, 99.5], [0.0, 120.0, 74.5], [0.0, 0.0, 1.0]]
HEAD_AT = (0.25, -0.12, 0.7)
#: a wrist camera, from its TCP (world axes; it looks straight down)
WRIST_FROM_TCP = (-0.04, 0.0, 0.1)
TABLE_RGB, BLOCK_RGB = (150, 150, 150), (200, 30, 30)
#: the collision capsule of each gripper, up from its TCP
GRIPPER_M, GRIPPER_RADIUS_M = 0.12, 0.03


class Conflict(Exception):
    """URAI's 409: it will not do that (``detail`` says why)."""


def encode(array: np.ndarray) -> dict:
    stream = io.BytesIO()
    np.save(stream, array, allow_pickle=False)
    return {"encoding": "npy-base64", "data": base64.b64encode(stream.getvalue()).decode("ascii")}


def looking_down(at) -> np.ndarray:
    """t_world_camera of a camera at ``at`` looking straight down (CV axes: image x along world x)."""
    pose = np.diag([1.0, -1.0, -1.0, 1.0])
    pose[:3, 3] = at
    return pose


def render(k, pose: np.ndarray, boxes) -> tuple[np.ndarray, np.ndarray]:
    """RGB and depth (along the optical axis, float32; NaN where nothing is) of the table and ``boxes``
    ((low corner, high corner) each)."""
    k = np.asarray(k, float)
    us, vs = np.meshgrid(np.arange(WIDTH, dtype=float), np.arange(HEIGHT, dtype=float))
    rays = np.stack([(us - k[0, 2]) / k[0, 0], (vs - k[1, 2]) / k[1, 1], np.ones_like(us)], -1).reshape(-1, 3)
    d, o = rays @ pose[:3, :3].T, pose[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[:, 2] < 0, (TABLE_Z - o[2]) / d[:, 2], np.inf)
        rgb = np.empty((len(rays), 3), np.uint8)
        rgb[:] = TABLE_RGB
        for low, high in boxes:
            t1, t2 = (np.asarray(low) - o) / d, (np.asarray(high) - o) / d
            near, far = np.nanmax(np.minimum(t1, t2), axis=1), np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (near <= far) & (near > 0) & (near < t)
            t[hit] = near[hit]
            rgb[hit] = BLOCK_RGB
    depth = np.where(np.isfinite(t), t, np.nan).astype(np.float32)
    return rgb.reshape(HEIGHT, WIDTH, 3), depth.reshape(HEIGHT, WIDTH)


class FakeSam3:
    """SAM3 on these pictures: the block is what is red; words find it when they say red (or 红), a point or
    a box finds it when it is on it."""

    def __init__(self):
        self.requests: list[dict] = []

    def health(self) -> dict:
        return {"ok": True}

    def close(self) -> None:
        pass

    def segment(self, image, *, text=None, points=None, labels=None, box=None, **kw) -> list:
        from hexashell.vision.sam3_client import Segment

        self.requests.append({"text": text, "points": points, "box": box})
        rgb = np.asarray(image).astype(int)
        red = (rgb[..., 0] > 2 * rgb[..., 1]) & (rgb[..., 0] > 2 * rgb[..., 2])
        if text is not None:
            wanted = "red" in text.lower() or "红" in text
        elif points:
            x, y = (int(round(v)) for v in points[0])
            wanted = bool(red[y, x])
        else:
            x0, y0, x1, y1 = (int(round(v)) for v in box)
            wanted = bool(red[y0:y1 + 1, x0:x1 + 1].any())
        if not wanted or not red.any():
            return []
        ys, xs = np.nonzero(red)
        return [Segment(red, 0.9, (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())), int(red.sum()))]


class FakeUrai:
    def __init__(self):
        self.arms = {arm: self._state(arm, xyz, DOWN_RPY, MAX_OPENING_MM) for arm, xyz in START_XYZ.items()}
        #: every pose an arm has had, by its joints (what home goes back to)
        self.poses = {arm: {tuple(s["joints_deg"]): (s["xyz"], s["rpy_deg"])} for arm, s in self.arms.items()}
        self.block = np.array([*BLOCK_XY, TABLE_Z + EDGE_M / 2])  # centre
        self.held_by: str | None = None
        self.grip_offset = np.zeros(3)
        self.frames = 0
        self.head_id: str | None = None
        self.revision = 0
        self.draft: dict | None = None
        self.preview: dict | None = None
        self.plans = 0
        self.execution = {"state": "idle", "progress": 0.0, "elapsed_s": 0.0}
        self._ending: dict | None = None
        self._polls = 0
        #: what to make of the next plans and executions
        self.no_line = False  # straight lines are refused
        self.bend = 0.0  # how far a swing bulges up from the line (m; below, when negative)
        self.miss = (0.0, 0.0, 0.0)  # where a goto ends, from its goal
        self.fail: str | None = None  # the next execution: "error", "empty_grasp" or "cancelled"
        self.overhead_s = 0.004
        self.running_polls = 1  # states read "running" after an execute
        #: the requests (method, path, query, body, token), the drafts and the plans run
        self.requests: list[dict] = []
        self.drafts: list[dict] = []
        self.executed: list[dict] = []

    def client(self, log=None, token: str | None = None) -> UraiClient:
        """The real client, with this service behind its httpx client."""
        client = UraiClient(URL, token, log=log or (lambda event: None))
        headers = client.http.headers
        client.http.close()
        client.http = httpx.Client(base_url=URL, headers=headers, transport=httpx.MockTransport(self.handle))
        return client

    def handle(self, request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content) if request.content else None
        self.requests.append({"method": request.method, "path": request.url.path, "query": dict(request.url.params),
                              "body": body, "token": request.headers.get("authorization")})
        answer = {("GET", "/api/harness/model"): self._model, ("POST", "/api/harness/observe"): self._observe,
                  ("GET", "/api/state"): self._status, ("PUT", "/api/draft"): self._draft,
                  ("POST", "/api/preview"): self._preview, ("POST", "/api/execute"): self._execute,
                  }.get((request.method, request.url.path))
        if answer is None:
            return httpx.Response(404, json={"detail": "Not Found"})
        try:
            return httpx.Response(200, json=answer(body))
        except Conflict as exc:
            return httpx.Response(409, json={"detail": str(exc)})

    # -- the robot ---------------------------------------------------------------------------------
    def _state(self, arm: str, xyz, rpy, gripper_mm: float) -> dict:
        dx, dy, dz = np.subtract(xyz, BASES[arm])
        joints = [math.degrees(math.atan2(dy, dx)), 100.0 * dz, 100.0 * math.hypot(dx, dy), *rpy]
        return {"xyz": [float(v) for v in xyz], "rpy_deg": [float(v) for v in rpy],
                "joints_deg": [round(float(v), 6) for v in joints], "gripper_mm": float(gripper_mm),
                "enabled": True, "feedback_age_s": 0.0, "ctrl_mode": "CAN"}

    def _move(self, arm: str, xyz, rpy) -> None:
        state = self._state(arm, xyz, rpy, self.arms[arm]["gripper_mm"])
        self.arms[arm] = state
        self.poses[arm][tuple(state["joints_deg"])] = (state["xyz"], state["rpy_deg"])
        if self.held_by == arm:
            self.block = np.asarray(xyz, float) + self.grip_offset

    def _grip(self, arm: str, opening_mm: float) -> None:
        tcp = np.asarray(self.arms[arm]["xyz"])
        width = float(opening_mm)
        if opening_mm < EDGE_M * 1000:
            around = (math.hypot(*(tcp[:2] - self.block[:2])) < 0.015
                      and abs(tcp[2] - self.block[2]) <= EDGE_M / 2)
            if self.held_by == arm or (self.held_by is None and around):
                if self.held_by is None:
                    self.held_by, self.grip_offset = arm, self.block - tcp
                width = EDGE_M * 1000
        elif self.held_by == arm:
            self.held_by = None
            self.block[2] = TABLE_Z + EDGE_M / 2
        self.arms[arm]["gripper_mm"] = width

    def tcp_camera(self, arm: str) -> np.ndarray:
        return looking_down(np.add(self.arms[arm]["xyz"], WRIST_FROM_TCP))

    # -- the service -------------------------------------------------------------------------------
    def _model(self, body) -> dict:
        return {"name": "fake-piper", "arm_names": list(self.arms), "table_z": TABLE_Z,
                "cameras": ["head", "hand_left", "hand_right"],
                "arms": {arm: {"t_world_base": [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z],
                                                [0.0, 0.0, 0.0, 1.0]],
                               "tcp_offset_mm": 142.5, "table_z": TABLE_Z, "max_opening_mm": MAX_OPENING_MM}
                         for arm, (x, y, z) in BASES.items()}}

    def _observe(self, body) -> dict:
        camera = body["camera"]
        self.frames += 1
        frame_id = f"frame-{self.frames}"
        if camera == "head":
            self.head_id = frame_id
            k, pose = HEAD_K, looking_down(HEAD_AT)
        else:
            k, pose = WRIST_K, self.tcp_camera({"hand_left": "left", "hand_right": "right"}[camera])
        half = EDGE_M / 2
        rgb, depth = render(k, pose, [(self.block - half, self.block + half)])
        capsules = {arm: [{"name": "gripper", "a": s["xyz"], "b": np.add(s["xyz"], (0, 0, GRIPPER_M)).tolist(),
                           "radius_m": GRIPPER_RADIUS_M}] for arm, s in self.arms.items()}
        return {"id": frame_id, "timestamp": float(self.frames), "camera": camera, "rgb": encode(rgb),
                "depth": encode(depth), "k": k, "t_world_camera": pose.tolist(), "arms": copy.deepcopy(self.arms),
                "capsules": capsules}

    def _status(self, body) -> dict:
        if self._ending is not None:
            if self._polls > 0:
                self._polls -= 1
            else:
                self.execution, self._ending = self._ending, None
        return {"arms": copy.deepcopy(self.arms), "execution": copy.deepcopy(self.execution)}

    def _draft(self, body) -> dict:
        if body["observation_id"] != self.head_id:
            raise Conflict(f"Observation {body['observation_id']} is not the current one; observe again")
        self.revision += 1
        self.draft = body
        self.drafts.append(copy.deepcopy(body))
        return {"revision": self.revision, "draft": body}

    def _preview(self, body) -> dict:
        if self.draft is None or body.get("expected_revision") != self.revision:
            raise Conflict("草稿版本已变化")
        self.plans += 1
        plan = {"id": f"plan-{self.plans}", "revision": self.revision, "observation_id": self.draft["observation_id"],
                "start": copy.deepcopy(self.arms), "arms": {}, "duration_s": 0.0,
                "pose_tolerance": dict(TOLERANCE), "completion": self.draft["completion"],
                "planning_only": body["planning_only"], "input_draft": copy.deepcopy(self.draft), "notes": []}
        for arm, spec in self.draft["arms"].items():
            now = np.asarray(self.arms[arm]["xyz"])
            if spec["primitive"] == "goto":
                goal = np.asarray(spec["xyz"], float)
                if np.linalg.norm(goal - BASES[arm]) > REACH_M:
                    raise Conflict(f"{arm}: 目标超出工作空间")
                if spec["motion"] == "line" and self.no_line:
                    raise Conflict(f"{arm}: 直线路径与桌面碰撞")
                s = np.linspace(0.0, 1.0, 11)
                path = now + s[:, None] * (goal - now)
                if spec["motion"] == "swing":
                    path[:, 2] += self.bend * np.sin(np.pi * s)
                duration = max(float(np.linalg.norm(goal - now)) / spec["speed_m_s"], 0.12)
            elif spec["primitive"] == "home":
                path, duration = np.array([now, self.poses[arm][tuple(spec["joints_deg"])][0]]), 0.6
            else:  # gripper, wait
                path, duration = np.array([now, now]), float(spec["duration_s"])
            plan["arms"][arm] = {"xyz": path.tolist()}
            plan["duration_s"] = max(plan["duration_s"], duration)
        self.preview = plan
        return plan

    def _execute(self, body) -> dict:
        plan = self.preview
        if plan is None or body["preview_id"] != plan["id"] or plan["observation_id"] != self.head_id:
            raise Conflict("Missing or stale preview; generate a new preview")
        self.preview = None
        self.executed.append(plan)
        fail, self.fail = self.fail, None
        spent = plan["duration_s"] + self.overhead_s
        ending = {"state": "completed", "progress": 1.0, "elapsed_s": spent, "id": plan["id"],
                  "execution_time_s": spent, "result": {"completed": True}}
        if fail == "error":
            ending = {"state": "error", "progress": 0.0, "elapsed_s": spent, "id": plan["id"],
                      "execution_time_s": spent, "error": "关节 3 跟踪误差超限"}
        elif fail == "cancelled":
            ending = {"state": "cancelled", "progress": 0.5, "elapsed_s": spent / 2, "id": plan["id"],
                      "execution_time_s": spent / 2}
        else:
            for arm, spec in plan["input_draft"]["arms"].items():
                if spec["primitive"] == "goto":
                    if "opening_mm" in spec:  # sent as the motion starts
                        self._grip(arm, spec["opening_mm"])
                    self._move(arm, np.add(spec["xyz"], self.miss), spec["rpy_deg"])
                elif spec["primitive"] == "home":
                    self._move(arm, *self.poses[arm][tuple(spec["joints_deg"])])
                elif spec["primitive"] == "gripper":
                    self._grip(arm, spec["opening_mm"])
            if fail == "empty_grasp":
                ending = {"state": "idle", "stage": "ready_after_empty_grasp", "progress": 0.0, "id": plan["id"],
                          "elapsed_s": spent, "execution_time_s": spent, "last_error": "夹爪闭合后未检测到物体",
                          "result": {"completed": False, "outcome": "empty_grasp", "ready_for_next": True}}
        self._ending, self._polls = ending, self.running_polls
        self.execution = {"state": "running", "progress": 0.0, "elapsed_s": 0.0, "id": plan["id"]}
        return copy.deepcopy(self.execution)
