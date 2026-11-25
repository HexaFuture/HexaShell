"""One RoboColiseum Season II run: ``python -m hexashell.benches.robocoliseum.run --config g2op_crsemb_hang_cups_on_cup_tree --instance 0``.

One benchmark run (``geniesim benchmark run`` in our container of the GPU, :mod:`.sim`; a scored wok_flip
layout: ``run-layouts``) plays one case of one config: one episode, played by HexaShell through :mod:`.link`
(:class:`.session.ColiseumSession` in an :class:`~hexashell.runtime.episode.Episode`), with its own record
``<config>[_<archive>][_seed<seed>]_i<instance>_e0_<time>``.

Split guard: the case must belong to ``--part`` (default ``learn``; :func:`.protocol.case_of`); a validation
or test case runs only when that part is named.  Our own wok_flip layouts and Rubik's cube scenes are named
by ``--seed`` (and the instance: the layout 0 of the seed, the scene index in our archive).

Native runtimes (Newton, libuipc, MuJoCo) end a run when a reply is late (:data:`.protocol.REPLY_WAIT_S`): a
request still unanswered close to that limit gets a hold row (``--hold-s``, by default the limit itself).

Agent settings: the board's limits (replies, minutes) and depth on, then ``config/benches.yml``
(``--settings``), then ``HEXASHELL_<NAME>``, then ``--set NAME=VALUE``.  The run's folder (``--run-dir``):
the benchmark's output ``bench.log``, the wire log ``link.jsonl``, our events ``run.jsonl``, a copy of the
result file and ``run.json``, this run's summary; the benchmark's own result root is
``protocol.SOURCE/output/hexashell/<run id>`` (:func:`.sim.result_root`).
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import shutil
import signal
import socket
import sys
import threading
import time
from dataclasses import fields
from pathlib import Path

from . import protocol as P
from . import sim
from .link import Server

ROOT = Path(__file__).resolve().parents[3]
#: how long the benchmark may take to connect (the container's Isaac start and the scene's load; a cold Kit
#: cache takes much longer)
START_S = 900.0
COLD_START_S = 2700.0
#: after the benchmark closed the connection with the episode over: how long its process may take to exit
EXIT_GRACE_S = 180.0
#: ... by runtime: a libuipc run first lets its stream scorer finish the episode's frames (the benchmark allows
#: it its drain timeout, 3600 s); a scorer that shares its CPU with another run's can still be at it 180 s after the
#: episode, and stopped there the episode gets no score
EXIT_GRACE_RUNTIME_S = {"libuipc": 3660.0}


def exit_grace_s(runtime: str) -> float:
    return EXIT_GRACE_RUNTIME_S.get(runtime, EXIT_GRACE_S)
#: ROS 2 domain ids of runs on one machine (each run holds a lock on its own; ids reach 232 at most)
DOMAINS = range(101, 231)
LOCKS = Path("/tmp") / f"hexashell-robocoliseum-{os.getuid()}"
#: replies of the model and wall-clock seconds per episode, by board (table tasks of the learning runs: median 25-31
#: replies, at most 80, about 19 s each; a WBC task chains drives, crouches and several picks and places - the
#: door took 48 replies, the cartons are four picks, a cart push and four places - so it gets twice as many)
REPLIES = {"finemanip": 80, "crsemb": 80, "wbc": 160, "dexmanip": 80}
EPISODE_S = {"finemanip": 5400.0, "crsemb": 5400.0, "wbc": 7200.0, "dexmanip": 5400.0}


def limits(board: str) -> dict:
    return {"max_turns": REPLIES[board], "budget_s": EPISODE_S[board], "ladder": False}


def settings_for(board: str, path: str | None, sets: list[str]):
    from ...runtime.episode import Settings
    from ..run import load_config

    settings = Settings.from_config({**limits(board), "depth": True, **load_config(path)})
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


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def take_domain() -> tuple[int, int]:
    """A ROS 2 domain id no other run of ours on this machine holds (the containers share the host's network:
    two simulators on one domain would hear each other's robot); the lock goes with the process."""
    LOCKS.mkdir(mode=0o700, exist_ok=True)
    for d in DOMAINS:
        fd = os.open(LOCKS / f"ros_domain_{d}.lock", os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            continue
        return d, fd
    raise SystemExit(f"no free ROS domain id in {DOMAINS.start}..{DOMAINS.stop - 1}")


def stop_run(proc, name: str, run_id: str, log) -> int | None:
    """End the run: TERM, then KILL, to its process group inside the container, then the docker client; then,
    also when the benchmark had exited by itself, whatever the run started that is still alive in the container
    (:func:`sim.end_leftovers`)."""
    rc = _stop_group(proc, name, run_id, log)
    try:
        left = sim.end_leftovers(name, run_id)
    except Exception as exc:  # noqa: BLE001 - the run's results matter more than its clean-up
        log({"kind": "sim_leftovers", "error": f"{type(exc).__name__}: {exc}"})
    else:
        if left["term"]:
            log({"kind": "sim_leftovers", **left})
    return rc


def _stop_group(proc, name: str, run_id: str, log) -> int | None:
    if proc.poll() is not None:
        return proc.returncode
    for sig, wait in (("TERM", 40.0), ("KILL", 20.0)):
        sent = sim.stop(name, run_id, sig)
        log({"kind": "sim_signal", "signal": sig, "sent": sent})
        try:
            return proc.wait(timeout=wait)
        except Exception:  # noqa: BLE001 - next signal
            continue
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        return proc.wait(timeout=10)
    except Exception:  # noqa: BLE001
        return proc.poll()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum.run",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, help="a Season II config (protocol.TASKS)")
    ap.add_argument("--instance", type=int, required=True)
    ap.add_argument("--seed", type=int, default=None,
                    help="our own wok_flip layout or Rubik's cube scene order (protocol.cases); none: official")
    ap.add_argument("--part", choices=P.PARTS, default="learn", help="the split part the instance must belong to")
    ap.add_argument("--gpu", default=os.environ.get("HEXASHELL_GPU", "0"))
    ap.add_argument("--port", type=int, default=0, help="the WebSocket port (0: a free one)")
    ap.add_argument("--out", default=str(ROOT / "results" / "benches" / "robocoliseum"), help="records")
    ap.add_argument("--run-dir", default=None, help="default: <out>/runs/<config>_i<instance>_<time>")
    ap.add_argument("--settings", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--hold-s", type=float, default=None,
                    help="seconds a request waits for the model before a hold row (default: Isaac no limit, the "
                         "local benchmark waits as long as it takes; native runtimes their reply limit)")
    ap.add_argument("--start-s", type=float, default=None, help=f"default {START_S:.0f}, {COLD_START_S:.0f} when cold")
    ap.add_argument("--max-h", type=float, default=24.0, help="the whole run's cap")
    args = ap.parse_args(argv)

    try:
        t = P.task(args.config)
    except P.TaskError as exc:
        ap.error(str(exc))
    try:
        case = P.case_of(t.config, args.instance, args.part, args.seed)
    except P.TaskError as exc:
        ap.error(str(exc))
    part = args.part
    #: the benchmark's limit on one reply (native runtimes; Isaac: none locally)
    reply_limit = P.REPLY_WAIT_S.get(t.runtime)
    hold_s = args.hold_s if args.hold_s is not None else reply_limit
    settings = settings_for(t.board, args.settings, args.set)
    out = Path(args.out).resolve()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    run_id = f"{case.name}_{stamp}"
    run_dir = Path(args.run_dir or out / "runs" / run_id).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    port = args.port or free_port()
    domain, lock_fd = take_domain()

    events = open(run_dir / "run.jsonl", "a", encoding="utf-8")
    events_lock = threading.Lock()

    def log(record: dict) -> None:
        with events_lock:
            events.write(json.dumps({"t": round(time.time(), 3), **record}, ensure_ascii=False, default=str) + "\n")
            events.flush()

    from ...runtime.episode import Episode
    from .session import ColiseumSession

    played: list[dict] = []

    def play(link) -> None:
        info = {"config": t.config, "instance": args.instance, "episode": link.index, "part": part,
                "seed": case.seed, "archive": case.archive, "run_dir": str(run_dir), "run_id": run_id}
        log({"kind": "episode_begin", "episode": link.index, "prompt": link.prompt, "robot": link.robot})
        episode = Episode(ColiseumSession(link, info), settings, out)
        try:
            episode.run()
        finally:
            entry = {"episode": link.index, "record": str(episode.rec.dir)}
            result = episode.rec.dir / "result.json"
            if result.exists():
                data = json.loads(result.read_text())
                entry.update({k: data.get(k) for k in ("instruction", "success", "progress", "score", "valid",
                                                       "dispatches", "end", "turns", "errors")})
            played.append(entry)
            log({"kind": "episode_end", **entry})

    _, results_dir = sim.result_root(run_id)
    name = sim.container_for(args.gpu)
    sim.ensure(name, args.gpu, run_dir / "container.log")
    cold = sim.cold()
    watchdog = sim.WATCHDOG_S
    # Newton compiles its Warp kernels on a first start (about 9.5 min without the shared cache)
    start_s = args.start_s or (COLD_START_S if cold or t.runtime == "newton" else START_S)
    server = Server("127.0.0.1", port, on_episode=play, results_dir=results_dir, hold_s=hold_s,
                    reply_limit_s=reply_limit, runtime=t.runtime, log_path=run_dir / "link.jsonl",
                    smooth_depth=t.board == "wbc")
    server.start()
    log({"kind": "start", "config": t.config, "board": t.board, "robot": t.robot, "instance": args.instance,
         "case": case.key, "runtime": t.runtime, "hold_s": hold_s, "smooth_depth": server.smooth_depth, "part": part, "gpu": args.gpu, "container": name, "port": port, "ros_domain": domain, "run_id": run_id,
         "cold": cold, "model": settings.model, "reasoning_effort": settings.reasoning_effort,
         "skills": settings.skills, "script": settings.script})

    def on_term(signum, _frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)  # once: the clean-up below runs to its end
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_term)
    proc = sim.launch(name, case, port, run_id, domain, run_dir / "bench.log", watchdog_s=watchdog)
    t0 = time.monotonic()
    end = "sim_exit"
    rc = None
    grace_s = exit_grace_s(t.runtime)
    try:
        quiet_since = None
        while proc.poll() is None:
            now = time.monotonic()
            if not server.connected_once.is_set() and now - t0 > start_s:
                end = "no_connection"
                break
            if now - t0 > args.max_h * 3600:
                end = "run_cap"
                break
            done = server.connected_once.is_set() and server.disconnected.is_set() and all(
                ep.over for ep in server.episodes)
            quiet_since = (quiet_since or now) if done else None
            if quiet_since is not None and now - quiet_since > grace_s:
                end = "sim_exit_hung"
                break
            time.sleep(2.0)
    except KeyboardInterrupt as exc:
        end = f"interrupted ({exc})"
    finally:
        rc = stop_run(proc, name, run_id, log)
        log({"kind": "sim_end", "end": end, "returncode": rc, "minutes": round((time.monotonic() - t0) / 60, 1)})
        for ep in server.episodes:
            if not ep.over:
                ep.abandon("the benchmark process ended")
        for th in list(server.threads):
            th.join(timeout=900)
        server.stop()
        os.close(lock_fd)

    files = sim.results(run_id, case)
    scored = [f for f in files if sim.scored(f)]
    for f in files:
        try:
            shutil.copy2(f, run_dir / f.name)
        except OSError as exc:
            log({"kind": "copy_failed", "file": str(f), "error": str(exc)})
    played.sort(key=lambda e: e["episode"])
    summary = {"config": t.config, "instance": args.instance, "case": case.key, "seed": case.seed,
               "archive": case.archive, "part": part, "run_id": run_id, "run_dir": str(run_dir),
               "results_dir": str(results_dir), "result_files": [f.name for f in files],
               "scored_files": [f.name for f in scored], "end": end,
               "returncode": rc, "episodes": len(server.episodes), "results": played,
               "minutes": round((time.monotonic() - t0) / 60, 1)}
    (run_dir / "run.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n")
    events.close()
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if end in ("sim_exit", "sim_exit_hung") and played and scored else 1


if __name__ == "__main__":
    sys.exit(main())
