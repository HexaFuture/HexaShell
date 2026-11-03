"""A LIBERO-PRO episode: the simulator server, the robot calls, the task card and the verdict.

The server (:mod:`.server`) runs in the LIBERO-PRO Python environment; :class:`LiberoSession` starts
it, opens the episode (task ``<suite>_t<i>``, seed -> initial state) and answers the cells' robot
calls through :class:`.host.LiberoHost`.  Records: ``server.log`` (the server's output) next to the
usual files; ``states.jsonl`` has one line per control step.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ..pose import shown_angles
from ..session import ArmSession
from . import protocol as P
from .host import FINGER_STROKE_M, LiberoHost

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "libero"
CAMERA_WORDS = {"agentview": "across the workspace, facing the robot and looking down at the table in front of it",
                "wrist": "on the hand, looking where the fingers point"}


def skill(task: str) -> str | None:
    path = SKILLS / f"{task}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


class LiberoSession(ArmSession):
    bench = "libero"
    proxy = "hexashell.benches.libero.proxy"
    server_module = "hexashell.benches.libero.server"
    host_class = LiberoHost

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None, size: int = P.IMAGE_SIZE):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=size)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])
        self.suite, self.index = P.parse_task(self.task)

    def open_args(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed}

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.task}_s{self.seed}"

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "suite": self.suite}

    def card(self, settings) -> tuple[str, dict]:
        f, host = self.facts, self.host
        horizon = int(f.get("horizon") or P.official_steps(self.task))
        tcp = host.start_tcp
        p, y, r = (round(v) + 0 for v in shown_angles(tcp[:3, :3]))  # + 0: no "-0"
        xyz = ", ".join(f"{v:.3f}" for v in tcp[:3, 3])
        opening = max(0.0, min(1.0, float(host.proprio["width"]) / FINGER_STROKE_M))
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {f.get('instruction')!r}",
                 f"Task: {self.task} (LIBERO-PRO, task {self.index} of {self.suite})",
                 f"Step budget: {horizon} steps ({horizon * P.STEP_S:.0f} s of simulated time) for the whole episode "
                 f"(used so far: {self.step or 0}). The episode ends by itself the moment the task is done.",
                 f"Start pose of the hand (robot.home() returns to it): grasp point ({xyz}), pitch {p} yaw {y} "
                 f"roll {r}, gripper opening {opening:.2f}.",
                 "The arm frame has its origin at the arm's base plate, x forward (toward the workspace), y to the "
                 "left, z up; the robot does not move."]
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
        return {"task": self.task, "seed": self.seed, "suite": self.suite, "task_index": self.index,
                "init_state": f.get("init_state"), "robot": f.get("robot"), "instruction": self.instruction}
