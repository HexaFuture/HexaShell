"""A LIBERO-Pro 2.0 episode: the simulator server, the robot calls, the task card and the verdict.

As LIBERO-PRO's (:mod:`hexashell.benches.libero.session`), with the perturbation case kept from the model: the
card names the suite, never the case, and so does the task the cells can read; the record folder's name hides
the case behind a hash (:func:`.protocol.record_name`).  The instruction is the one the benchmark gives now: a
dynamic language perturbation changes it during the episode, and the cell after the change says so
(:meth:`LiberoPro2Session.cell_note`).  Records: ``server.log`` (the server's output) next to the usual files;
``states.jsonl`` has one line per control step; the opening's event has the perturbation plan and the official
fingerprints (never shown to the model).

The episode starts from the initial state its split gives the seed (:func:`.protocol.init_state_of`).  Pictures
come without depth unless the setting ``depth`` is on (the official environment renders none): perception is then
RoboDojo's on the colour pictures (:class:`.scene.LiberoScene`), the prompts' ``rgb`` variant explains it, and the
card gives the table top the nominal model has (:func:`.sim.nominal_table`).  Without depth the episode starts
with the camera check (:mod:`.calib`; CAMERA_CHECK_ENV "0" turns it off): when the pictures do not show the robot
where the cameras' models and the readings put it, the hand moves for it and the models are fitted again, and the
card says so; else nothing changes.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from ..libero.session import CAMERA_WORDS
from ..pose import shown_angles
from ..session import ArmSession
from . import protocol as P
from .calib import calibrate
from .host import FINGER_STROKE_M, LiberoPro2Host
from .scene import untrusted_words

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "liberopro2"
#: the camera check at the start (:mod:`.calib`) is made unless this environment variable is "0", "off" or "no"
CAMERA_CHECK_ENV = "HEXASHELL_LIBEROPRO2_CAMERA_CHECK"


def skill(name: str) -> str | None:
    path = SKILLS / f"{name}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


class LiberoPro2Session(ArmSession):
    bench = "liberopro2"
    proxy = "hexashell.benches.liberopro2.proxy"
    server_module = "hexashell.benches.liberopro2.server"
    host_class = LiberoPro2Host

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None, size: int = P.IMAGE_SIZE):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=size)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])
        self.suite, self.index, self.case = P.parse_task(self.task)
        #: the instruction the model was last given
        self.told: str | None = None
        #: whether the cameras render depth and perception reads it (the setting ``depth``, off by default)
        self.depth = False
        #: what the camera check at the start found (:func:`.calib.calibrate`; None: not made, or it failed)
        self.camera_check: dict | None = None

    def open_args(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "init_state": P.init_state_of(self.task, self.seed),
                "depth": self.depth}

    def prompt_variant(self) -> str | None:
        return None if self.depth else "rgb"

    def open(self, settings, rec, attach) -> None:
        self.depth = bool(settings.depth)
        super().open(settings, rec, attach)
        if self.depth or self.host.scene is None \
                or os.environ.get(CAMERA_CHECK_ENV, "1").strip().lower() in ("0", "off", "no", "false"):
            return
        self.camera_check = found = calibrate(self.host, rec.event)
        if found and found.get("need"):
            self.host.recalibrated(found)
        if (found and found.get("need")) or self.step:
            self.first = self.host.pictures()

    def camera_line(self) -> str | None:
        """The card's line on the camera check, when it changed something."""
        found = self.camera_check
        if not found or not found.get("need"):
            return None
        move = found.get("move") or {}
        said = []
        if move.get("moved"):
            xyz = ", ".join(f"{v:.3f}" for v in self.host._tcp()[:3, 3])
            said.append(f"the hand moved for it ({move.get('steps', 0)} steps) to grasp point ({xyz}), where the "
                        "episode goes on")
        if found.get("fixes"):
            said.append("the cameras' models were fitted again to the pictures; the pictures and all positions are "
                        "through the fitted models")
        else:
            said.append("the cameras' models stand")
        if found.get("wrist") == "untrusted":
            said.append(untrusted_words("wrist"))
        return ("Camera check at the start (the pictures against the robot the readings show): "
                + "; ".join(said) + ".")

    def _scene(self, settings):
        if self.depth or not settings.sam3_url:
            return super()._scene(settings)
        try:
            from .scene import LiberoScene

            return LiberoScene(settings.sam3_url, log=self.rec.event)
        except Exception as exc:  # noqa: BLE001 - run without perception, and say so
            self.errors.append(f"perception unavailable: {exc}")
            self.rec.event({"kind": "perception_unavailable", "error": str(exc)})
            return None

    @property
    def instruction(self) -> str | None:
        return self.status.get("instruction") or self.facts.get("instruction")

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return P.record_name(self.task, self.seed)

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "suite": self.suite, "case": self.case}

    def card(self, settings) -> tuple[str, dict]:
        f, host = self.facts, self.host
        horizon = int(f.get("horizon") or P.max_steps(self.task))
        tcp = host.start_tcp
        p, y, r = (round(v) + 0 for v in shown_angles(tcp[:3, :3]))  # + 0: no "-0"
        xyz = ", ".join(f"{v:.3f}" for v in tcp[:3, 3])
        opening = max(0.0, min(1.0, float(host.proprio["width"]) / FINGER_STROKE_M))
        self.told = self.instruction
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {self.told!r}",
                 f"Task: a {self.suite} task of LIBERO-Pro 2.0",
                 f"Step budget: {horizon} steps ({horizon * P.STEP_S:.0f} s of simulated time) for the whole episode "
                 f"(used so far: {self.step or 0}). The episode ends by itself the moment the task is done.",
                 f"Start pose of the hand (robot.home() returns to it): grasp point ({xyz}), pitch {p} yaw {y} "
                 f"roll {r}, gripper opening {opening:.2f}.",
                 "The arm frame has its origin at the arm's base plate, x forward (toward the workspace), y to the "
                 "left, z up; the robot does not move."]
        table = f.get("table") or {}
        if not self.depth and table:
            what = "The floor the objects lie on" if table.get("floor") else "The table top the objects stand on"
            lines.append(f"{what}: z = {float(table['z']):.3f} (the robot's model of the scene).")
        line = self.camera_line()
        if line:
            lines.append(line)
        shown = [c for c in self.cams if c in self.first]
        words = "; ".join(f"{c} ({CAMERA_WORDS[c]})" for c in shown)
        lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "skill": False, "notes": [], "depth": self.depth,
                "table_z": table.get("z") if table else None}
        if settings.skills:
            # learning, validation and test share the base tasks (protocol), so a base task may have its own note
            base = f"{self.suite}_t{P.parse_task(self.task)[1]}"
            for name, title in (("common", "Notes"), (self.suite, f"Notes for {self.suite}"),
                                (base, "Notes for this task")):
                text = skill(name)
                if text:
                    lines += ["", f"## {title}", "", text]
                    used["notes"].append(name)
            used["skill"] = bool(used["notes"])
        return "\n".join(lines) + "\n", used

    def cell_note(self) -> str | None:
        now = self.instruction
        if self.told is None or not now or now == self.told:
            return None
        self.told = now
        return f"The benchmark changed the instruction; from now on it is {now!r} (authoritative)."

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        f = self.facts
        return {"task": self.task, "seed": self.seed, "suite": self.suite, "task_index": self.index,
                "case": self.case, "kind": P.kind(self.case), "category": P.category(self.case),
                "split": P.split_of(self.task, self.seed), "init_state": f.get("init_state"),
                "depth": self.depth, "table_z": (f.get("table") or {}).get("z"), "camera_check": self._check_brief(),
                "robot": f.get("robot"), "instruction": f.get("instruction"), "instruction_final": self.instruction}

    def _check_brief(self) -> dict | None:
        found = self.camera_check
        if found is None:
            return None
        out = {"need": bool(found.get("need")), "gain": found.get("gain")}
        if found.get("need"):
            out.update(agentview=found.get("agentview"), wrist=found.get("wrist"),
                       steps=(found.get("move") or {}).get("steps"), fixes=sorted(found.get("fixes") or {}))
        return out
