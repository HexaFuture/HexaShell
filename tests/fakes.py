"""Stand-ins for the simulator and the model, so the whole runtime can run without either.

``FakeTable`` renders boxes on a table into the three cameras with the same camera models the
tools use, follows joint targets with a little lag (like RoboDojo's position control), and moves
the gripper at most a fifth of its range per action.  ``drive`` plays RoboDojo's client loop
against a :class:`~hexashell.xpl.adapter.PolicyHost`.  ``ScriptedGateway`` answers the episode's
requests from a list of prepared replies.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass

import cv2
import numpy as np

from hexashell.geometry import homogeneous, rot_z
from hexashell.motion.chain import arm_chain
from hexashell.runtime.llm import Reply
from hexashell.state import STATUS_KEY
from hexashell.vision.cameras import HEAD_FOCAL, HEAD_POSE, WRIST_FOCAL, Camera, wrist_pose
from hexashell.world import ARMS, TABLE_Z


@dataclass
class Box:
    label: str
    center: tuple[float, float, float]  # middle of the box (world)
    size: tuple[float, float, float]  # x, y, z extent before turning
    yaw_deg: float = 0.0
    color: tuple[int, int, int] = (200, 40, 40)

    def corners(self) -> np.ndarray:
        sx, sy, sz = (v / 2.0 for v in self.size)
        local = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
        pose = homogeneous(rot_z(np.radians(self.yaw_deg)), self.center)
        return local @ pose[:3, :3].T + pose[:3, 3]


def pose_to_xyzwxyz(pose: np.ndarray) -> list[float]:
    """RoboDojo's [x, y, z, qw, qx, qy, qz] (largest-component method, safe near half turns)."""
    r = pose[:3, :3]
    t = np.trace(r)
    if t > max(r[0, 0], r[1, 1], r[2, 2]):
        s = 2.0 * np.sqrt(1.0 + t)
        w, x, y, z = s / 4, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] >= r[1, 1] and r[0, 0] >= r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, s / 4, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] >= r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, s / 4, (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4
    return [float(v) for v in (*pose[:3, 3], w, x, y, z)]


class FakeTable:
    def __init__(self, boxes: list[Box], *, instruction="Pick up the red block.", task="fake_task",
                 step_limit=400, lag=0.7, start_q=None):
        self.boxes = boxes
        self.instruction = instruction
        self.task = task
        self.step_limit = step_limit
        self.lag = lag
        self.chains = {arm: arm_chain(arm) for arm in ARMS}
        self.q = {arm: np.array(start_q[arm] if start_q else np.zeros(6), float) for arm in ARMS}
        self.grip = {arm: 1.0 for arm in ARMS}
        self.step = 0
        self.success = False

    # -- simulation ------------------------------------------------------------------------------
    def apply(self, action: dict) -> None:
        for arm in ARMS:
            target = np.asarray(action[f"{arm}_arm_joint_state"], float)
            self.q[arm] = self.q[arm] + self.lag * (target - self.q[arm])
            goal = float(action[f"{arm}_ee_joint_state"][0])
            self.grip[arm] += float(np.clip(goal - self.grip[arm], -0.2, 0.2))
        self.step += 1

    def over(self) -> bool:
        return self.success or self.step >= self.step_limit

    # -- observation -----------------------------------------------------------------------------
    def cameras(self) -> dict[str, Camera]:
        cams = {"head": Camera("head", HEAD_FOCAL, HEAD_POSE.copy())}
        for arm in ARMS:
            cams[f"{arm}_wrist"] = Camera(f"{arm}_wrist", WRIST_FOCAL, wrist_pose(self.chains[arm].flange(self.q[arm])))
        return cams

    def render(self, cam: Camera) -> np.ndarray:
        img = np.full((cam.height, cam.width, 3), (90, 110, 130), np.uint8)  # wall / background
        # the table top as a polygon clipped to points in front of the camera
        table = np.array([[x, y, TABLE_Z] for x, y in ((-0.7, -0.6), (0.7, -0.6), (0.7, 0.5), (-0.7, 0.5))])
        self._fill(img, cam, table, (170, 150, 120))
        far_first = sorted(self.boxes, key=lambda b: -np.linalg.norm(np.asarray(b.center) - cam.position))
        for box in far_first:
            self._fill(img, cam, box.corners(), box.color, hull=True)
        return img

    def _fill(self, img, cam, points, color, hull=False):
        px, depth = cam.project(points)
        if np.any(depth <= 0.01):
            if hull:
                return
            # crude clip: drop points behind the camera
            px = px[depth > 0.01]
            if len(px) < 3:
                return
        pts = np.round(px).astype(np.int32)
        if hull:
            pts = cv2.convexHull(pts)
        cv2.fillPoly(img, [pts.reshape(-1, 1, 2)], color)

    def obs(self) -> dict:
        state = {}
        for arm in ARMS:
            state[f"{arm}_arm_joint_state"] = self.q[arm].tolist()
            state[f"{arm}_ee_pose"] = pose_to_xyzwxyz(self.chains[arm].flange(self.q[arm]))
            state[f"{arm}_ee_joint_state"] = [self.grip[arm]]
        cams = self.cameras()
        vision = {f"cam_{name}": {"color": self.render(cam)} for name, cam in cams.items()}
        status = {"step": self.step, "step_limit": self.step_limit, "task_name": self.task, "eval_seed": 0,
                  "layout": 0, "pictures_behind": 0}
        return {"state": state, "vision": vision, "instruction": self.instruction,
                "additional_info": {STATUS_KEY: status}}

    def verdict(self) -> dict:
        return {"step": self.step, "step_limit": self.step_limit, "success": self.success,
                "progress": 100.0 if self.success else 0.0, "task_name": self.task}


def drive(table: FakeTable, host, timeout_s: float = 600.0) -> dict:
    """RoboDojo's client loop: observation, action (asking again while the answer is WAIT), until
    the episode ends or the agent stops; then the verdict."""
    import time

    end = time.monotonic() + timeout_s
    host.reset()
    while not table.over() and time.monotonic() < end:
        host.update_obs(table.obs())
        answer = host.get_action()
        while isinstance(answer, list) and len(answer) == 1 and answer[0].get(STATUS_KEY) == "wait":
            answer = host.get_action()
        if not answer:
            break
        for action in answer:
            table.apply(action)
            if table.over():
                break
    verdict = table.verdict()
    host.on_trial_end(verdict)
    return verdict


def python_call(code: str, call_id: str | None = None) -> dict:
    return {"id": call_id, "type": "function", "function": {"name": "python", "arguments": json.dumps({"code": code})}}


def finish_call(status="stuck", summary="test") -> dict:
    return {"id": None, "type": "function",
            "function": {"name": "finish", "arguments": json.dumps({"status": status, "summary": summary})}}


class ScriptedGateway:
    """Answers each request with the next prepared reply (a list of tool calls, or text)."""

    script: list = []
    seen: list = []

    def __init__(self, *args, log=None, **kwargs):
        self.log = log or (lambda e: None)
        self.turn = 0
        self.lock = threading.Lock()

    def settings(self) -> dict:
        return {"model": "scripted"}

    def ask(self, messages, tools, *, deadline=None, stop=None) -> Reply:
        with self.lock:
            type(self).seen.append(messages)
            item = type(self).script[self.turn] if self.turn < len(type(self).script) else [finish_call()]
            self.turn += 1
        if isinstance(item, str):
            return Reply(text=item, calls=[], finish_reason="stop", usage={"prompt": 10, "completion": 5})
        reasoning = None
        if isinstance(item, tuple):  # (reasoning, calls)
            reasoning, item = item
        calls = [dict(c, id=c.get("id") or f"call_{self.turn}_{i}") for i, c in enumerate(item)]
        return Reply(text="", calls=calls, finish_reason="tool_calls", usage={"prompt": 100, "completion": 20},
                     reasoning=reasoning)

    def close(self) -> None:
        pass
