#!/usr/bin/env python3
"""scan_robotwin_seeds.py -- the RoboTwin 2.0 seeds the expert solves, found the way RoboTwin's evaluation finds them.

RoboTwin's evaluation walks a task's seeds upward in one process: build the scene (``setup_demo``),
let the expert play it (``play_once``), and keep the seed only if every plan of the expert succeeded
and the success check passed; an unstable scene (UnStableError) or any other error skips the seed.
A kept seed's scene is then built again for the policy, and SAPIEN's cache is emptied after every
5th kept seed.  This script does the same with the settings of our episodes (``demo_randomized``,
eval mode, :func:`hexashell.benches.robotwin.sim.task_args`), their scene fixes
(:func:`~hexashell.benches.robotwin.sim.fix_task_module` and, before every build,
:func:`~hexashell.benches.robotwin.sim.reset_clutter_area`) and cuRobo, the planner RoboTwin's
expert uses, so the first five evaluation seeds it keeps should be those of RPent's list.

One task and one range per process, as RoboTwin walks them (without ``reset_clutter_area`` the outcome
of a seed would depend on what the process built before it); a line per seed tried, then a summary
line, in ``<out>/<task>.<range>.jsonl``:

    <bench-robotwin python> scripts/setup/scan_robotwin_seeds.py one TASK START WANT OUT [--gpu N] [--tries N]

A task whose seeds the expert rarely solves (put_object_cabinet: the five of RPent's list lie in 174 seeds)
can be scanned in parts, in parallel: the part from ``START`` (any seed after the range's first) goes to
``<out>/<task>.<range>+<offset>.jsonl``; ``list`` joins the parts in seed order and checks that they
leave no gap.

All tasks, both ranges (development seeds from 0, evaluation seeds from 100000), a few processes per
GPU; a (task, range) whose file has its summary line is skipped, and one that another process is
still writing is left to it, so the command can be run again, also while an earlier run goes on:

    <bench-robotwin python> scripts/setup/scan_robotwin_seeds.py all OUT --gpus 0,2,3 [--per-gpu 3] [--want 20]

Then the seed list of the protocol (``hexashell/benches/robotwin/seeds.json``): per task the first 10
development seeds kept for learning, the next 10 for validation, and the first 15 evaluation seeds kept
for the test, as RoboTwin's evaluation takes the first seeds its expert solves:

    <bench-robotwin python> scripts/setup/scan_robotwin_seeds.py list OUT

A kept seed whose scene could not be built again for the policy (``rebuild_error``: RoboTwin's
evaluation would stop there) is left out of the list.  ``list`` joins a range's parts until its list is
full and names the tasks still short, whose range is then scanned on in a part from the seed after its
last one; it also notes the tasks whose first kept evaluation seeds are not the five of RPent's list.

The scans need ``$HEXASHELL_ROBOTWIN_ENV`` sourced (assets, Vulkan, EGL).
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

RANGES = {"dev": 0, "eval": 100000}
#: seeds tried at most per (task, range), and seconds at most per seed
MAX_TRIES = 200
SEED_S = 900
CLEAR_CACHE_EVERY = 5


class Timeout(Exception):
    pass


def range_name(start: int) -> str:
    return "eval" if start >= RANGES["eval"] else "dev"


def scan_path(out: Path, task: str, start: int) -> Path:
    name = range_name(start)
    offset = start - RANGES[name]
    return out / (f"{task}.{name}.jsonl" if offset == 0 else f"{task}.{name}+{offset}.jsonl")


def scan(task: str, start: int, want: int, out: Path, tries: int = MAX_TRIES) -> None:
    from hexashell.benches.robotwin import sim

    sim.assets_root()
    args = sim.task_args(task, str(out / "scratch"))
    args["planner_backend"] = "curobo"
    module = importlib.import_module(f"robotwin.envs.{task}")
    sim.fix_task_module(task, module)
    env = getattr(module, task)()

    def timeout(*_):
        raise Timeout(f"over {SEED_S} s")

    signal.signal(signal.SIGALRM, timeout)
    path = scan_path(out, task, start)
    found: list[int] = []
    t_all = time.time()
    with open(path, "w", encoding="utf-8") as f:
        for seed in range(start, start + tries):
            if len(found) >= want:
                break
            t0 = time.time()
            rec: dict = {"seed": seed}
            signal.alarm(SEED_S)
            try:
                sim.reset_clutter_area(env, task)
                with sim.quiet():
                    env.setup_demo(now_ep_num=seed, seed=seed, **args)
                    env.play_once()
                rec["plan_success"] = bool(env.plan_success)
                rec["check_success"] = bool(env.check_success())
                rec["ok"] = rec["plan_success"] and rec["check_success"]
            except Exception as exc:  # noqa: BLE001 - RoboTwin's evaluation skips the seed on any error
                rec.update(ok=False, error=f"{type(exc).__name__}: {exc}"[:300])
            finally:
                signal.alarm(0)
                try:
                    with sim.quiet():
                        env.close_env()
                except Exception:  # noqa: BLE001
                    pass
            if rec["ok"]:
                found.append(seed)
                try:  # the scene built again for the policy, as the evaluation does
                    sim.reset_clutter_area(env, task)
                    with sim.quiet():
                        env.setup_demo(now_ep_num=seed, seed=seed, **args)
                except Exception as exc:  # noqa: BLE001
                    rec["rebuild_error"] = f"{type(exc).__name__}: {exc}"[:300]
                with sim.quiet():
                    env.close_env(clear_cache=len(found) % CLEAR_CACHE_EVERY == 0)
            rec["s"] = round(time.time() - t0, 1)
            f.write(json.dumps(rec) + "\n")
            f.flush()
        f.write(json.dumps({"task": task, "start": start, "want": want, "tries": tries, "found": found, "done": True,
                            "complete": len(found) >= want, "s": round(time.time() - t_all, 1)}) + "\n")


def done(path: Path) -> bool:
    try:
        last = path.read_text(encoding="utf-8").strip().splitlines()[-1]
        return bool(json.loads(last).get("done"))
    except (OSError, IndexError, ValueError):
        return False


def busy(path: Path) -> bool:
    """Another process is scanning it: the file was written to within the time a seed may take."""
    try:
        return not done(path) and time.time() - path.stat().st_mtime < SEED_S + 60
    except OSError:
        return False


def scan_all(out: Path, gpus: list[str], per_gpu: int, want: int) -> int:
    from hexashell.benches.robotwin import protocol as P

    jobs = [(task, start) for start in RANGES.values() for task in P.tasks()
            if not done(scan_path(out, task, start))]
    slots = [g for g in gpus for _ in range(per_gpu)]
    running: dict = {}  # slot index -> (Popen, task, start)
    failed = 0
    logs = out / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} scans on {len(slots)} slots", flush=True)
    while jobs or running:
        for i, (proc, task, start) in list(running.items()):
            if proc.poll() is not None:
                del running[i]
                ok = proc.returncode == 0 and done(scan_path(out, task, start))
                failed += not ok
                print(f"{time.strftime('%H:%M:%S')} {task} {range_name(start)}: "
                      f"{'done' if ok else f'FAILED (exit {proc.returncode})'}", flush=True)
        for i in range(len(slots)):
            free = [j for j in jobs if not busy(scan_path(out, *j))]
            if i in running or not free:
                continue
            jobs.remove(free[0])
            task, start = free[0]
            if done(scan_path(out, task, start)):  # finished by another process meanwhile
                continue
            env = dict(os.environ, CUDA_DEVICE_ORDER="PCI_BUS_ID", CUDA_VISIBLE_DEVICES=slots[i])
            log = open(logs / f"{task}.{range_name(start)}.log", "w", encoding="utf-8")
            proc = subprocess.Popen([sys.executable, __file__, "one", task, str(start), str(want), str(out)],
                                    env=env, stdout=log, stderr=subprocess.STDOUT)
            log.close()
            running[i] = (proc, task, start)
        time.sleep(5)
    return 1 if failed else 0


def make_list(out: Path) -> int:
    from hexashell.benches.robotwin import protocol as P

    tasks, notes = {}, []
    want = {"dev": P.PART_SEEDS["learn"] + P.PART_SEEDS["validation"], "eval": P.PART_SEEDS["test"]}
    for task in P.tasks():
        published = [seed for t, seed in P.published_episodes() if t == task]
        found = {}
        for name, first in RANGES.items():
            found[name], expect = [], first  # the parts in seed order, each starting where the last one stopped
            parts = sorted(out.glob(f"{task}.{name}*.jsonl"),
                           key=lambda p: int(p.name.split("+")[1].split(".")[0]) if "+" in p.name else 0)
            for path in parts:
                if len(found[name]) >= want[name]:
                    break
                if not done(path):
                    print(f"{path}: not scanned to the end", file=sys.stderr)
                    return 2
                lines = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
                if lines[0]["seed"] != expect:
                    print(f"{path}: starts at {lines[0]['seed']}, the scan before it stopped before {expect}",
                          file=sys.stderr)
                    return 2
                found[name] += [r["seed"] for r in lines[:-1] if r.get("ok") and "rebuild_error" not in r]
                expect = lines[-2]["seed"] + 1
        if found["eval"][:len(published)] != published:
            notes.append(f"{task}: the scan kept {found['eval'][:len(published)]} first, RPent's list has {published}")
        if len(found["dev"]) < want["dev"] or len(found["eval"]) < want["eval"]:
            notes.append(f"{task}: only {len(found['dev'])} development and {len(found['eval'])} test seeds")
        learn = P.PART_SEEDS["learn"]
        tasks[task] = {"learn": found["dev"][:learn], "validation": found["dev"][learn:want["dev"]],
                       "test": found["eval"][:want["eval"]]}
    P.SEED_LIST.write_text(json.dumps({
        "made": f"{time.strftime('%Y-%m-%d')} by scripts/setup/scan_robotwin_seeds.py: demo_randomized, eval mode, "
                "the cuRobo expert; a seed is kept when the expert's plans all succeed and the success check passes; "
                "the test seeds are the first kept from 100000",
        "notes": notes, "tasks": tasks}, indent=1) + "\n", encoding="utf-8")
    print(f"{P.SEED_LIST}: {len(tasks)} tasks")
    for note in notes:
        print(" ", note)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="mode", required=True)
    one = sub.add_parser("one")
    one.add_argument("task")
    one.add_argument("start", type=int)
    one.add_argument("want", type=int)
    one.add_argument("out", type=Path)
    one.add_argument("--gpu", help="nvidia-smi index (default: CUDA_VISIBLE_DEVICES as set)")
    one.add_argument("--tries", type=int, default=MAX_TRIES, help=f"seeds tried at most ({MAX_TRIES})")
    every = sub.add_parser("all")
    every.add_argument("out", type=Path)
    every.add_argument("--gpus", required=True, help="nvidia-smi indices, e.g. 0,2,3")
    every.add_argument("--per-gpu", type=int, default=3)
    every.add_argument("--want", type=int, default=20)
    listing = sub.add_parser("list")
    listing.add_argument("out", type=Path)
    a = ap.parse_args()
    if a.mode == "list":
        return make_list(a.out)
    a.out.mkdir(parents=True, exist_ok=True)
    if a.mode == "all":
        return scan_all(a.out, a.gpus.split(","), a.per_gpu, a.want)
    if a.gpu is not None:
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
        os.environ["CUDA_VISIBLE_DEVICES"] = a.gpu
    scan(a.task, a.start, a.want, a.out, a.tries)
    return 0


if __name__ == "__main__":
    sys.exit(main())
