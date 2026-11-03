"""A RoboCasa episode: the simulator server, the robot calls, the task card and the verdict.

The server (:mod:`.server`) runs in the RoboCasa Python environment; :class:`CasaSession` starts it,
opens the episode (task, seed -> a kitchen of the split) and answers the cells' robot calls
through :class:`.host.CasaHost`.  Records: ``server.log`` (the server's output) next to the usual
files; ``states.jsonl`` has one line per control step.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from ..pose import shown_angles
from ..session import ArmSession
from . import protocol as P
from .host import FINGER_STROKE_M, CasaHost

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "robocasa"
CAMERA_WORDS = {"left": "high up behind the base on its left, looking ahead and down",
                "right": "high up behind the base on its right, looking ahead and down",
                "wrist": "on the hand, looking where the fingers point"}


def skill(task: str) -> str | None:
    path = SKILLS / f"{task}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


class CasaSession(ArmSession):
    bench = "robocasa"
    proxy = "hexashell.benches.robocasa.proxy"
    server_module = "hexashell.benches.robocasa.server"
    host_class = CasaHost

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None, size: int = P.IMAGE_SIZE):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=size)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])
        self.split = P.split_of(info.get("split"))
        P.task_set(self.task)

    def open_args(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "split": self.split}

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.task}_s{self.seed}"

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "layout": self.facts.get("layout_id")}

    def card(self, settings) -> tuple[str, dict]:
        f, host = self.facts, self.host
        horizon = int(f.get("horizon") or P.HORIZON[self.task])
        tcp = host.start_tcp
        p, y, r = (round(v) + 0 for v in shown_angles(tcp[:3, :3]))  # + 0: no "-0"
        xyz = ", ".join(f"{v:.3f}" for v in tcp[:3, 3])
        opening = max(0.0, min(1.0, float(host.proprio["width"]) / FINGER_STROKE_M))
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {f.get('instruction')!r}",
                 f"Task: {self.task} (RoboCasa)",
                 f"Step budget: {horizon} steps ({horizon * P.STEP_S:.0f} s of simulated time) for the whole episode "
                 f"(used so far: {self.step or 0}). The episode ends by itself the moment the task is done.",
                 f"Start pose of the hand (robot.home() returns to it): grasp point ({xyz}), pitch {p} yaw {y} "
                 f"roll {r}, gripper opening {opening:.2f}.",
                 "The base starts at (0, 0) heading 0: the start frame has its origin on the floor under the base, "
                 "x forward, y to the left, z up."]
        shown = [c for c in self.cams if c in self.first]
        words = "; ".join(f"{c} ({CAMERA_WORDS[c]})" for c in shown)
        lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "skill": False}
        if settings.skills:
            text = skill(self.task)
            if text:
                lines += ["", "## Notes for this task", "", text]
                used["skill"] = True
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        f = self.facts
        seed_set = P.seed_set(self.seed) if self.split == "pretrain50" else None
        return {"task": self.task, "seed": self.seed, "split": self.split, "task_set": P.task_set(self.task),
                "seed_set": seed_set, "layout_id": f.get("layout_id"), "style_id": f.get("style_id"),
                "instruction": self.instruction}

    # -- records ---------------------------------------------------------------------------------
    def row_extra(self, rest: np.ndarray) -> dict[str, Any]:
        return {"base": [round(float(rest[0]), 5), round(float(rest[1]), 5), round(math.degrees(float(rest[2])), 3)]}
