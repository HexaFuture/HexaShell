"""A RoboColiseum episode: one episode of a Genie Sim benchmark run, played through :mod:`.link`.

The benchmark (``app.py`` of genie_sim v3.5.0) runs in its own process and asks our WebSocket server
for actions (:class:`.link.Server`); every episode it starts becomes an :class:`.link.EpisodeLink`,
and :class:`ColiseumSession` is what :class:`~hexashell.runtime.episode.Episode` needs of it: the robot
calls (:class:`.host.G2Host`), the task card, the start pictures and the verdict, which the benchmark
writes to its evaluate_ret file when the episode ends.

What the model is told: the instruction and the benchmark's stage texts
(``segments``) as they come, the base task's name (a robust episode's perturbation is not named), the
start pose of both hands; the scoring mechanism is in the system prompt, without its numbers (no step
allowances, no thresholds).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from ...runtime.session import Attach, Session
from . import obs as O
from . import protocol as P
from .host import G2Host, check_cam, describe_verdict

ROOT = Path(__file__).resolve().parents[3]
SKILLS = ROOT / "skills" / "robocoliseum1"
#: the longest the end of an episode may take once the model is done: the benchmark plays held steps
#: until a stage runs out of its steps (sorting_packages_continuous allows thousands), then writes its
#: verdict
VERDICT_CAP_S = 3600.0


def skill(name: str) -> str | None:
    """``skills/robocoliseum1/<name>.md`` (a task, a board or ``common``), or None."""
    path = SKILLS / f"{name}.md"
    return path.read_text(encoding="utf-8").strip() if path.exists() else None


def note_names(board: str, task: str) -> list[str]:
    """The notes a card gets, in order: ``common``, the board's, the task's.  A robust config plays an
    instruction-board task, so it gets that board's note and its base task's (never one naming the perturbation)."""
    return ["common", "instruction" if board == "robust" else board, task]


def _gripper_words(opening: float, holding: bool | None) -> str:
    if holding:
        return "closed on something it holds at the start"
    return f"open {opening:.2f}" if opening >= 0.05 else "closed"


class ColiseumSession(Session):
    bench = "robocoliseum1"
    proxy = "hexashell.benches.robocoliseum1.proxy"

    def __init__(self, link, info: dict[str, Any]):
        facts = P.parse_config(info["config"])
        super().__init__({**info, "task_name": facts["task"], "board": facts["board"]})
        self.link = link
        self.facts = facts
        self.config = facts["config"]
        self.task = facts["task"]
        self.layout = info.get("layout")
        self.seed = info.get("eval_seed")
        #: the k-th episode of this benchmark run (0 = the first)
        self.k = int(info.get("episode", link.index))
        self.rec = None
        self.host: G2Host | None = None
        self.first: dict[str, np.ndarray] = {}

    # -- names -----------------------------------------------------------------------------------
    def record_name(self) -> str | None:
        return f"{self.config}_L{self.layout}_s{self.seed}_e{self.k}"

    def labels(self) -> dict[str, Any]:
        return {"task": self.task, "seed": self.seed, "layout": self.layout, "config": self.config,
                "episode": self.k}

    # -- lifetime ------------------------------------------------------------------------------
    def open(self, settings, rec, attach: Attach) -> None:
        self.rec = rec
        first = self.link.first
        if not settings.depth:
            self.errors.append("depth off is not supported on RoboColiseum: perception reads the depth pictures")
        scene = self._scene(settings)
        self.host = G2Host(self.link, task_name=self.task, scene=scene, log=rec.event, attach=attach, rec=rec,
                           seed=int(self.seed or 0) * 1000 + self.k)
        self.perception = scene is not None
        self.first = {cam: img for cam, img in first.images.items() if cam in O.CAMERAS}
        rec.event({"kind": "episode_open", "config": self.config, "layout": self.layout, "seed": self.seed,
                   "episode": self.k, "instruction": first.prompt, "segments": first.segments,
                   "first": O.summary(first), "start": self.host.driver.start_info()})

    def _scene(self, settings):
        if not settings.sam3_url:
            return None
        try:
            from ...vision.rgbd import DepthScene, closing_yaw_robocasa

            # closing_yaw_robocasa: a yaw in (-90, 90], at yaw 0 the fingers close along world y - ours
            return DepthScene(settings.sam3_url, cams=list(O.CAMERAS), check_cam=check_cam, log=self.rec.event,
                              closing_yaw=closing_yaw_robocasa)
        except Exception as exc:  # noqa: BLE001 - run without perception, and say so
            self.errors.append(f"perception unavailable: {exc}")
            self.rec.event({"kind": "perception_unavailable", "error": str(exc)})
            return None

    def conclude(self, timeout_s: float) -> dict | None:
        """Hold still until the benchmark ends the episode, then its verdict (``timeout_s`` is the shared
        runner's minute: the benchmark may play many held steps first, so the cap is VERDICT_CAP_S)."""
        self.link.conclude()
        v = self.link.wait_verdict(VERDICT_CAP_S)
        if v is None:
            return None
        v = dict(v)
        v.setdefault("step", v.get("dispatches"))
        return v

    def close(self) -> None:
        if self.host is not None and self.host.scene is not None:
            try:
                self.host.scene.close()
            except Exception:  # noqa: BLE001
                pass

    # -- during the episode ----------------------------------------------------------------------
    @property
    def over(self) -> bool:
        return self.link.over

    @property
    def verdict(self) -> dict | None:
        return self.link.verdict

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        return self.host.dispatch(name, args, kwargs)

    @property
    def step(self) -> int | None:
        return None  # the benchmark's step allowances are per stage and not told

    @property
    def step_limit(self) -> int | None:
        return None

    @property
    def instruction(self) -> str | None:
        return self.link.prompt

    def frames(self) -> dict[str, np.ndarray]:
        try:
            return {cam: img for cam, img in self.link.latest().images.items() if cam in O.CAMERAS}
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            if self.rec is not None:
                self.rec.event({"kind": "frames_failed", "error": str(exc)})
            return {}

    def over_text(self) -> str:
        reason = self.host.over_reason() if self.host is not None else describe_verdict(self.link.verdict)
        return f"The episode is over: {reason}."

    def start_pictures(self) -> list[tuple[str, np.ndarray]]:
        return [(cam, self.first[cam]) for cam in O.CAMERAS if cam in self.first]

    def card(self, settings) -> tuple[str, dict]:
        first = self.link.first
        driver = self.host.driver
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {self.instruction!r}",
                 f"Task: {self.task}"]
        stages = [str(s).strip() for s in (first.segments or []) if str(s).strip()]
        if stages and stages != [str(self.instruction).strip()]:
            lines.append("The benchmark's own description of the stages, judged in this order:")
            lines += [f"{i}. {s}" for i, s in enumerate(stages, 1)]
        lines.append("Start pose of each hand (robot.home() returns to it):")
        for arm, (xyz, angles) in driver.start_info().items():
            p, y, r = (round(v) + 0 for v in angles)  # + 0: no "-0"
            grip = _gripper_words(driver.start_grippers[arm], driver.holding(arm))
            lines.append(f"- {arm}: grasp point ({', '.join(f'{v:.3f}' for v in xyz)}), pitch {p} yaw {y} roll {r}, "
                         f"gripper {grip}")
        shown = [c for c in O.CAMERAS if c in self.first]
        words = "; ".join(f"{c} ({P.CAMERA_WORDS[c]})" for c in shown)
        lines.append(f"Pictures 1-{len(shown)} below: {words}, at the start; they stay in the conversation.")
        used = {"task": self.task, "config": self.config, "layout": self.layout, "seed": self.seed, "skill": False,
                "notes": [], "stages": len(stages)}
        if settings.skills:
            titles = ("Notes", "Notes for tasks of this kind", "Notes for this task")
            for name, title in zip(note_names(self.facts["board"], self.task), titles):
                text = skill(name)
                if text:
                    lines += ["", f"## {title}", "", text]
                    used["notes"].append(name)
            used["skill"] = bool(used["notes"])
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        first = self.link.first
        return {"task": self.task, "config": self.config, "board": self.facts["board"], "kind": self.facts["kind"],
                "layout": self.layout, "seed": self.seed, "episode": self.k, "instruction": self.instruction,
                "segments": first.segments, "result_index": self.link.result_index,
                "score": (verdict or {}).get("progress")}

    def report(self, finish: dict | None) -> dict[str, Any]:
        host = self.host
        out: dict[str, Any] = {"link": dict(self.link.stats)}
        if host is not None:
            out.update({"rows_sent": host.driver.rows, "motions": host.motions, "catches": dict(host.driver.catches)})
        return out

    def timing_note(self) -> str:
        return ("the simulation waits while you think (a long pause plays a held step now and then, nothing more); "
                "every motion uses steps, and each stage of the task has its own allowance of steps.")
