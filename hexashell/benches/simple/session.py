"""A SIMPLE episode: the simulator server, the robot calls, the task card and the verdict.

The server (:mod:`.server`) runs in SIMPLE's Python environment (bench-simple); :class:`SimpleSession` starts
it, opens the episode (task, seed -> a fresh learning or validation layout, or for a test seed the official
evaluation episode, which the run must allow: protocol.OFFICIAL_TEST_VARIABLE=1) and answers the cells' robot
calls through :class:`.host.SimpleHost`.  The episode runs to protocol.RELAXED_STEPS (45 s) and records
when the task first succeeded, so a result says both whether it succeeded within the official limit and within
the 45 s.  Records: ``server.log`` (the server's output) next to the usual
files; ``states.jsonl`` has one line per control step.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ..session import ArmSession
from . import protocol as P
from .host import SimpleHost

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "simple"


#: what the task card says of each legs' controller (protocol.Task.controller)
CONTROLLER_WORDS = {
    "amo": "AMO, which keeps the feet in place well while the body bends; standing upright the pelvis goes no "
           "lower than about 0.54 m, a little lower when leaning",
    "decoupled_wbc": "GR00T's whole-body controller, which walks well; right after a walk, lean rather than squat "
                     "deep (a deep squat then shifts the robot 20-30 cm back)",
}


def skill(task: str) -> str | None:
    path = SKILLS / f"{task}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


def _xyz(v) -> str:
    return "(" + ", ".join(f"{float(a):.3f}" for a in v) + ")"


class SimpleSession(ArmSession):
    bench = "simple"
    proxy = "hexashell.benches.simple.proxy"
    server_module = "hexashell.benches.simple.server"
    host_class = SimpleHost
    #: the first open starts Isaac Sim (about six minutes on a busy machine) and builds the scene
    server_timeout_s = 1800.0

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None, renderer: str | None = None):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=P.GAZE_WIDTH)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])
        P.check_task(self.task)
        #: a test seed (an official evaluation episode) is opened only when the run allows it
        self.official_test = P.split_of(self.seed) == "test" and P.official_test_allowed()
        P.check_seed(self.seed, official_test=self.official_test)
        #: who draws the pictures (None: the server's default, $SIMPLE_RENDERER or Isaac Sim)
        self.renderer = renderer

    def open_args(self) -> dict[str, Any]:
        args = {"task": self.task, "seed": self.seed, "renderer": self.renderer}
        if self.official_test:
            args["official_test"] = True
        return args

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.task}_s{self.seed}"

    def labels(self) -> dict[str, Any]:
        labels = {"task": self.task, "seed": self.seed, "layout": P.split_of(self.seed)}
        if self.official_test:
            level, index = P.official_episode(self.seed)
            labels["official_episode"] = f"{level} episode {index}"
        return labels

    def card(self, settings) -> tuple[str, dict]:
        f, host = self.facts, self.host
        start = host.start
        horizon = int(f.get("relaxed_steps") or P.RELAXED_STEPS)
        official = int(f.get("official_steps") or P.TASKS[self.task].official_steps)
        palms = start["palms"]
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {f.get('instruction')!r}",
                 f"Task: {self.task} (SIMPLE, Unitree G1 humanoid)",
                 f"Step budget: {horizon} steps ({horizon * P.STEP_S:.0f} s of simulated time; one step is "
                 f"{P.STEP_S:g} s) for the whole episode (used so far: {self.step or 0}). The benchmark's official "
                 f"limit is {official} steps ({official * P.STEP_S:.0f} s): a success within it counts in full, a "
                 "later one still counts as done in time for this run. The episode ends by itself the moment the "
                 "task is done.",
                 f"Legs' controller: {CONTROLLER_WORDS.get(str(f.get('controller')), f.get('controller'))}.",
                 "The robot stands at the start frame's origin: origin on the ground under the pelvis, x forward, y "
                 "to its left, z up; the frame stays fixed while it walks.",
                 f"Start: pelvis {start['pelvis_height']:.3f} m high, torso pitch {start['torso_pitch']:.0f} yaw "
                 f"{start['torso_yaw']:.0f}; left palm at {_xyz(palms['left']['point'])}, right palm at "
                 f"{_xyz(palms['right']['point'])}; fingers open; gaze yaw {start['gaze'][0]:.0f} pitch "
                 f"{start['gaze'][1]:.0f}."]
        shown = [c for c in self.cams if c in self.first]
        if shown:
            lines.append("Picture 1 below: the gaze camera (on the head, turned by robot.gaze), at the start; it stays "
                         "in the conversation.")
        used = {"task": self.task, "skill": False}
        if settings.skills:
            text = skill(self.task)
            if text:
                lines += ["", "## Notes for this task", "", text]
                used["skill"] = True
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        s = self.status
        return {"task": self.task, "seed": self.seed, "split": P.split_of(self.seed), "instruction": self.instruction,
                "first_success_step": s.get("first_success_step"), "official_steps": s.get("official_steps"),
                "official_success": bool(s.get("official_success")), "renderer": self.facts.get("renderer"),
                "controller": self.facts.get("controller")}

    # -- records ---------------------------------------------------------------------------------
    def _rows(self, rows: np.ndarray) -> None:
        """server.LOG_COLUMNS per step: step, pelvis xyz, heading, pelvis height, torso pitch and yaw, left and
        right palm points (start frame)."""
        for row in np.asarray(rows, float):
            self.rec.state({"step": int(row[0]), "pelvis": [round(float(v), 4) for v in row[1:4]],
                            "heading": round(float(row[4]), 2), "pelvis_height": round(float(row[5]), 4),
                            "torso_pitch": round(float(row[6]), 2), "torso_yaw": round(float(row[7]), 2),
                            "left_palm": [round(float(v), 4) for v in row[8:11]],
                            "right_palm": [round(float(v), 4) for v in row[11:14]]})
