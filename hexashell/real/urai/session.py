"""One episode on a robot run by URAI (:class:`UraiSession`), and the command that runs one (:func:`main`).

There is no judge: the episode ends when the model calls finish (whose statuses add ``done``), at the
step budget or at a limit, and result.json keeps the model's report (``agent_report``) with
``success`` null.  The robot does not stand still while the model thinks, so the episode ends at the
latest one budget after it opens.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from ...runtime.session import Attach, Session
from . import texts
from .client import UraiClient
from .host import UraiHost


class UraiSession(Session):
    bench = "urai"
    proxy = "hexashell.real.urai.proxy"
    deadline_budgets = 1

    def __init__(self, url: str, instruction: str, *, task_name: str = "task", token: str | None = None,
                 step_limit: int | None = None, speed_m_s: float = 0.12):
        super().__init__({"task_name": task_name})
        self.url, self.token = url, token
        self._instruction, self._step_limit = instruction, step_limit
        self.speed_m_s = speed_m_s
        self.rec = None
        self.client: UraiClient | None = None
        self.host: UraiHost | None = None

    def record_name(self) -> str:
        return f"urai-{self.info['task_name']}"

    # -- lifetime ------------------------------------------------------------------------------
    def open(self, settings, rec, attach: Attach) -> None:
        self.rec = rec
        self.client = UraiClient(self.url, self.token, log=rec.event)
        self.host = UraiHost(self.client, self.instruction, task_name=self.info["task_name"],
                             step_limit=self.step_limit, speed_m_s=self.speed_m_s,
                             log=rec.event, attach=attach, record_state=rec.state)
        rec.state({"step": 0, "arms": self.host.arms})

    def conclude(self, timeout_s: float) -> dict:
        return self.verdict

    def close(self) -> None:
        if self.client is not None:
            self.client.close()

    # -- during the episode ----------------------------------------------------------------------
    @property
    def over(self) -> bool:
        return self.host is not None and self.host.over

    @property
    def verdict(self) -> dict:
        return {"success": None, "step": self.step, "step_limit": self.step_limit}

    def dispatch(self, name: str, args: tuple, kwargs: dict) -> Any:
        return self.host.dispatch(name, args, kwargs)

    def card(self, settings) -> tuple[str, dict]:
        model = self.host.model
        card = texts.CARD.format(instruction=self.instruction, name=model["name"],
                                 model=json.dumps(model, ensure_ascii=False), task=self.host.task())
        return card, {"task": self.info["task_name"], "skill": False}

    def start_pictures(self) -> list[tuple[str, np.ndarray]]:
        return [(cam, obs.view.image) for cam, obs in self.host.observations.items()]

    @property
    def step(self) -> int | None:
        return None if self.host is None else self.host.step

    @property
    def step_limit(self) -> int | None:
        return self._step_limit

    @property
    def instruction(self) -> str:
        return self._instruction

    def frames(self) -> dict[str, np.ndarray]:
        try:
            return {cam: view.image for cam, view in self.host.refresh(list(self.host.observations)).items()}
        except Exception as exc:  # noqa: BLE001 - a record, not worth the episode
            self.rec.event({"kind": "frames_failed", "error": str(exc)})
            return {}

    def over_text(self) -> str:
        return texts.OVER.format(step=self.step, limit=self.step_limit)

    def report(self, finish: dict | None) -> dict[str, Any]:
        return {"agent_report": finish, "evaluation": {"source": "none"}, "success": None}

    # -- what the model is told ------------------------------------------------------------------
    def tools(self) -> list[dict]:
        tools = copy.deepcopy(super().tools())
        finish = next(t["function"] for t in tools if t["function"]["name"] == "finish")
        finish["description"] = texts.FINISH
        status = finish["parameters"]["properties"]["status"]
        status["enum"] = ["done", *status["enum"]]
        status["description"] = texts.FINISH_STATUS
        finish["parameters"]["properties"]["summary"]["description"] = (
            "Report in English what was achieved, what was verified, and what remains incomplete or uncertain.")
        return tools

    def nudge_text(self) -> str:
        return texts.NUDGE

    def finished_text(self) -> str:
        return texts.FINISHED

    def timing_note(self) -> str:
        return texts.TIMING


def main(argv: list[str] | None = None) -> int:
    from ...runtime.episode import Episode, Settings

    ap = argparse.ArgumentParser(prog="python -m hexashell.real.urai", description="Run a HexaShell session through URAI HTTP")
    ap.add_argument("--url", default=os.environ.get("URAI_URL"), help="URAI service URL; can also be set with URAI_URL")
    ap.add_argument("--task", required=True, help="Task instruction; use English for English records")
    ap.add_argument("--name", default="task", help="Task name used in records")
    ap.add_argument("--script", default="", help="Python script with cells separated by # %%%%; omit to use the model")
    ap.add_argument("--config", help="JSON configuration file for Settings")
    ap.add_argument("--results", default="runs")
    ap.add_argument("--step-limit", type=int)
    ap.add_argument("--speed", type=float, default=0.12, help="Planning speed for goto/home in metres per second")
    args = ap.parse_args(argv)
    if not args.url:
        ap.error("Provide the URAI service URL with --url or URAI_URL")
    config = json.loads(Path(args.config).read_text()) if args.config else {}
    if args.script:
        config["script"] = args.script
    settings = Settings.from_config(config)
    session = UraiSession(args.url, args.task, task_name=args.name, token=os.environ.get("URAI_TOKEN"),
                          step_limit=args.step_limit, speed_m_s=args.speed)
    episode = Episode(session, settings, Path(args.results))
    episode.run()
    print(episode.rec.dir)
    return 1 if episode.end in ("error", "gateway_error") else 0
