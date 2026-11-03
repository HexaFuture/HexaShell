"""A RoboTwin 2.0 episode: the simulator server, the robot calls, the task card and the verdict.

The server (:mod:`.server`) runs in RoboTwin's Python environment; :class:`RoboTwinSession` starts it,
opens the episode (task, seed) and answers the cells' robot calls through :class:`.host.TwinHost`
(RoboDojo's robot API).  With the setting ``depth`` off, perception works without the depth pictures
(:mod:`.scene`) and the system prompt has the ``rgb`` variant's parts.  Records: ``server.log`` (the
server's output) next to the usual files;
``states.jsonl`` has one line per step.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ...motion.driver import display_angles
from ...motion.ik import speed_named
from ...runtime.session import Attach
from ...vision.cameras import check_cam
from ..armhost import describe
from ..session import BenchSession
from . import protocol as P
from .host import TwinHost, closing_yaw

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "robotwin"
CAMERA_WORDS = {"head": "above and behind the arms, looking down at the table",
                "left_wrist": "on the left hand, looking where its fingers point",
                "right_wrist": "on the right hand, looking where its fingers point"}


def skill(task: str) -> str | None:
    path = SKILLS / f"{task}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


class RoboTwinSession(BenchSession):
    bench = "robotwin"
    proxy = "hexashell.benches.robotwin.proxy"
    server_module = "hexashell.benches.robotwin.server"
    #: the first ray-traced picture of a process can take ten minutes on a busy machine
    server_timeout_s = 1800.0

    def __init__(self, info: dict[str, Any], *, python: str, env_file: str | None = None,
                 egl_device: str | None = None):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device)
        self.task = P.check_task(str(info["task_name"]))
        self.seed = int(info["eval_seed"])
        self.rec = None
        self.host: TwinHost | None = None
        self.facts: dict[str, Any] = {}
        self.first: dict[str, np.ndarray] = {}
        #: whether perception reads the depth pictures (the setting ``depth``, off by default)
        self.depth = False

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.task}_s{self.seed}"

    def prompt_variant(self) -> str | None:
        return None if self.depth else "rgb"

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed}

    # -- lifetime ------------------------------------------------------------------------------
    def open(self, settings, rec, attach: Attach) -> None:
        self.rec = rec
        self.depth = bool(settings.depth)
        self.start_server(Path(rec.dir) / "server.log")
        opened = self.call("open", task=self.task, seed=self.seed)
        self.facts = dict(opened["facts"])
        rec.event({"kind": "sim_open", **{k: v for k, v in opened.items() if k not in ("ok", "status", "start")},
                   **self.server_facts()})
        self._log_state(0, opened["start"], None)
        scene = self._scene(settings)
        self.host = TwinHost(self, opened, scene=scene, log=rec.event, attach=attach, on_step=self._log_state,
                             speed=speed_named(settings.motion), depth=self.depth)
        self.perception = scene is not None
        self.first = self.host.pictures()

    def _scene(self, settings):
        if not settings.sam3_url:
            return None
        try:
            if not self.depth:
                from .scene import TwinScene

                return TwinScene(settings.sam3_url, log=self.rec.event)
            from ...vision.rgbd import DepthScene

            return DepthScene(settings.sam3_url, cams=list(P.CAMERAS), check_cam=check_cam, log=self.rec.event,
                              closing_yaw=closing_yaw)
        except Exception as exc:  # noqa: BLE001 - run without perception, and say so
            self.errors.append(f"perception unavailable: {exc}")
            self.rec.event({"kind": "perception_unavailable", "error": str(exc)})
            return None

    def close(self) -> None:
        if self.host is not None and self.host.scene is not None:
            try:
                self.host.scene.close()
            except Exception:  # noqa: BLE001
                pass
        super().close()

    # -- during the episode ----------------------------------------------------------------------
    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        return self.host.dispatch(name, args, kwargs)

    @property
    def instruction(self) -> str | None:
        return self.facts.get("instruction")

    def over_reason(self) -> str:
        return describe(self.status, finished=self.finished, failure=self.failure)

    def over_text(self) -> str:
        return f"The episode is over: {self.over_reason()}."

    def frames(self) -> dict[str, np.ndarray]:
        """The pictures of this step already rendered, and the head camera's in any case."""
        if self.failure or self.host is None:
            return {}
        try:
            views = self.host.cached_views()
            if "head" not in views:
                views.update(self.host.views(["head"]))
            return {cam: v.image for cam, v in views.items()}
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            self.rec.event({"kind": "frames_failed", "error": str(exc)})
            return {}

    def start_pictures(self) -> list[tuple[str, np.ndarray]]:
        return [(cam, self.first[cam]) for cam in P.CAMERAS if cam in self.first]

    def card(self, settings) -> tuple[str, dict]:
        host = self.host
        seen = "head picture" if self.depth else "pictures"
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {self.instruction!r}",
                 f"Task: {self.task} (RoboTwin 2.0)",
                 f"Step budget: {self.step_limit or P.step_limit(self.task)} steps for the whole episode (used so far: "
                 f"{self.step or 0}). The episode ends by itself the moment the task is done.",
                 (f"Table top: z = {host.table_z:.3f} (measured where its far edge shows in the head picture at the "
                  "start; good to about 6 mm)." if host.table_by == "the far edge" else
                  f"Table top: z = {host.table_z:.3f} (measured in the {seen} at the start)."
                  if host.table_measured else
                  f"Table top: about z = {host.table_z:.3f} (it could not be measured in the {seen}; it "
                  "may be up to 3 cm lower)."),
                 "Start pose of each arm (robot.home() returns to it):"]
        for arm in host.rig.arms:
            pose = host.driver.start_pose[arm]
            p, y, r = (round(v) + 0 for v in display_angles(pose[:3, :3]))  # + 0: no "-0"
            xyz = ", ".join(f"{v:.3f}" for v in pose[:3, 3])
            lines.append(f"- {arm}: grasp point ({xyz}), pitch {p} yaw {y} roll {r}, gripper "
                         f"{host.start_grippers[arm]:.2f}")
        shown = [c for c in P.CAMERAS if c in self.first]
        words = "; ".join(f"{c} ({CAMERA_WORDS[c]})" for c in shown)
        lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "skill": False, "table_z": host.table_z, "table_measured": host.table_measured,
                "depth": self.depth}
        if settings.skills:
            text = skill(self.task)
            if text:
                lines += ["", "## Notes for this task", "", text]
                used["skill"] = True
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        f = self.facts
        host = self.host
        return {"task": self.task, "seed": self.seed, "instruction": self.instruction,
                "instruction_from": f.get("instruction_from"),
                "table_z": None if host is None else host.table_z, "depth": self.depth,
                "setup_s": f.get("setup_s")}

    # -- records ---------------------------------------------------------------------------------
    def _log_state(self, seq: int, proprio: dict, action) -> None:
        record: dict[str, Any] = {"seq": seq, "step": self.step}
        for arm in ("left", "right"):
            a = proprio[arm]
            flange = np.asarray(a["flange"], float)
            record[arm] = {"joints": [round(float(v), 5) for v in np.asarray(a["joints"], float)],
                           "flange_xyz": [round(float(v), 5) for v in flange[:3, 3]],
                           "gripper": round(float(a["gripper"]), 4)}
        if action is not None:
            record["action"] = [round(float(v), 5) for v in np.asarray(action, float)]
        self.rec.state(record)
