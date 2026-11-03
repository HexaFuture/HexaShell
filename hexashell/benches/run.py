"""One benchmark episode: ``python -m hexashell.benches.run --bench robocasa --task CloseFridge --seed 0``
(or ``--bench libero --task libero_goal_swap_t3 --seed 0``, ``--bench robotwin --task adjust_bottle --seed 100000``).

Agent settings: the benchmark's limits (replies, time) as defaults, then ``config/benches.yml``
(or ``--config``), then ``HEXASHELL_<NAME>`` environment variables, then ``--set name=value``.
Machine settings: ``--python``, the benchmark's own Python (default ``$HEXASHELL_<BENCH>_PYTHON``);
``--env-file``, a file of ``export`` lines its server needs (``$HEXASHELL_<BENCH>_ENV``); ``--egl``,
the GPU that renders (``$HEXASHELL_EGL_DEVICE``); ``--sim-socket``, a warm simulator server to play on
(:mod:`hexashell.benches.daemon`) instead of starting one.  The record goes under ``--out``.
RoboCasa's kitchens: ``--split`` (``$HEXASHELL_ROBOCASA_SPLIT``), ``pretrain50`` (the default) or
``pretrain``, RoboCasa's own evaluation split (:mod:`.robocasa.protocol`).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import fields
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BENCHES = ("robocasa", "libero", "robotwin")


def bench_defaults(bench: str, task: str) -> dict:
    if bench == "robocasa":
        from .robocasa import protocol as P

        return {"max_turns": P.REPLIES[P.task_set(task)], "budget_s": P.EPISODE_S, "ladder": False}
    if bench == "libero":
        from .libero import protocol as P

        P.parse_task(task)
        return {"max_turns": P.REPLIES, "budget_s": P.EPISODE_S, "ladder": False}
    if bench == "robotwin":
        from .robotwin import protocol as P

        P.check_task(task)
        return {"max_turns": P.REPLIES, "budget_s": P.EPISODE_S, "ladder": False}
    raise ValueError(f"unknown benchmark {bench!r}")


def make_session(bench: str, task: str, seed: int, *, python: str, env_file: str | None, egl: str | None,
                 split: str | None = None):
    if split and bench != "robocasa":
        raise ValueError(f"--split is RoboCasa's; {bench} has one test set")
    if bench == "robocasa":
        from .robocasa.session import CasaSession

        return CasaSession({"task_name": task, "eval_seed": seed, "split": split}, python=python, env_file=env_file,
                           egl_device=egl)
    if bench == "libero":
        from .libero.session import LiberoSession

        return LiberoSession({"task_name": task, "eval_seed": seed}, python=python, env_file=env_file,
                             egl_device=egl)
    if bench == "robotwin":
        from .robotwin.session import RoboTwinSession

        return RoboTwinSession({"task_name": task, "eval_seed": seed}, python=python, env_file=env_file,
                               egl_device=egl)
    raise ValueError(f"unknown benchmark {bench!r}")


def load_config(path: str | None) -> dict:
    if not path or not Path(path).exists():
        return {}
    import yaml

    data = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"{path}: not a mapping of settings")
    return data


def settings_for(bench: str, task: str, config: str | None, sets: list[str]):
    from ..runtime.episode import Settings

    cfg = {**bench_defaults(bench, task), **load_config(config)}
    settings = Settings.from_config(cfg)
    names = {f.name for f in fields(Settings)}
    for item in sets:
        name, sep, value = item.partition("=")
        if not sep or name not in names:
            raise SystemExit(f"--set {item!r}: NAME=VALUE with NAME one of {', '.join(sorted(names))}")
        kind = type(getattr(settings, name))
        if kind is bool:
            setattr(settings, name, value.strip().lower() in ("1", "true", "yes", "on"))
        else:
            setattr(settings, name, kind(value))
    return settings


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.run", description=__doc__.splitlines()[0])
    ap.add_argument("--bench", required=True, choices=BENCHES)
    ap.add_argument("--task", required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", default=str(ROOT / "results" / "benches"))
    ap.add_argument("--config", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--python", default=None, help="the benchmark's Python")
    ap.add_argument("--env-file", default=None)
    ap.add_argument("--egl", default=None, help="GPU index for rendering")
    ap.add_argument("--sim-socket", default=None,
                    help="play on the warm simulator server at this unix socket (python -m hexashell.benches.daemon)")
    ap.add_argument("--split", default=None,
                    help="RoboCasa's kitchens: pretrain50 (default) or pretrain ($HEXASHELL_ROBOCASA_SPLIT)")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    args = ap.parse_args(argv)

    key = args.bench.upper()
    python = args.python or os.environ.get(f"HEXASHELL_{key}_PYTHON")
    if not python:
        ap.error(f"--python or $HEXASHELL_{key}_PYTHON: the {args.bench} environment's Python")
    env_file = args.env_file or os.environ.get(f"HEXASHELL_{key}_ENV") or None
    egl = args.egl if args.egl is not None else os.environ.get("HEXASHELL_EGL_DEVICE")
    settings = settings_for(args.bench, args.task, args.config, args.set)

    from ..runtime.episode import Episode

    split = args.split or os.environ.get(f"HEXASHELL_{key}_SPLIT") or None
    session = make_session(args.bench, args.task, args.seed, python=python, env_file=env_file, egl=egl, split=split)
    session.sim_socket = args.sim_socket or None
    episode = Episode(session, settings, Path(args.out))
    episode.run()
    result = episode.rec.dir / "result.json"
    summary = {"record": str(episode.rec.dir)}
    if result.exists():
        data = json.loads(result.read_text())
        summary.update({k: data.get(k) for k in ("task", "seed", "success", "step", "step_limit", "end", "turns",
                                                 "errors")})
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
