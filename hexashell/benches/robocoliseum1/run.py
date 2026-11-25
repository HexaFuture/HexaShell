"""One RoboColiseum run: ``python -m hexashell.benches.robocoliseum1.run --config g2op_if_pick_block_color --layout 0``.

One simulator process (genie_sim's ``app.py``, :mod:`.sim`) plays one layout of one config.  A layout is
played once per instruction it has (on every board), so a run has one or several episodes, one after
another on the same connection; each is played by HexaShell through
:mod:`.link` (:class:`.session.ColiseumSession` in an :class:`~hexashell.runtime.episode.Episode`) and gets
its own record ``<config>_L<layout>_s<seed>_e<k>_<time>``.

Split A guard: the layout must belong to ``--part`` (default ``learn``); a validation or test layout runs
only when that part is named.

Agent settings: the board's limits (replies, minutes) and depth on, then ``config/benches.yml``
(``--settings``), then ``HEXASHELL_<NAME>``, then ``--set NAME=VALUE``.  Machine paths: see :mod:`.sim`.
The simulator's working folder (``--run-dir``): its log ``bench.log``, the wire log ``link.jsonl``, its
results under ``root/output/benchmark``, ``judge.jsonl`` (the VLM-judged configs: :mod:`.judge`) and
``run.json``, this run's summary.
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
from .link import HOLD_S, Server

ROOT = Path(__file__).resolve().parents[3]
#: how long the benchmark may take to connect (Isaac Sim's start, the scene's load)
START_S = 900.0
#: after the benchmark closed the connection with every episode over: how long app.py may take to exit
EXIT_GRACE_S = 180.0
#: ROS 2 domain ids of runs on one machine (each run holds a lock on its own; ids reach 232 at most)
DOMAINS = range(101, 231)
LOCKS = Path("/tmp") / f"hexashell-robocoliseum1-{os.getuid()}"


def settings_for(board: str, path: str | None, sets: list[str]):
    from ...runtime.episode import Settings
    from ..run import load_config

    settings = Settings.from_config({**P.limits(board), "depth": True, **load_config(path)})
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
    """A ROS 2 domain id no other run on this machine holds (two simulators on one domain would hear each
    other's robot); the lock goes with the process."""
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


def stop_sim(proc, log) -> int | None:
    """End the simulator's process group: TERM, then KILL."""
    if proc.poll() is not None:
        return proc.returncode
    for sig, wait in ((signal.SIGTERM, 30.0), (signal.SIGKILL, 15.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        log({"kind": "sim_signal", "signal": sig.name})
        try:
            return proc.wait(timeout=wait)
        except Exception:  # noqa: BLE001 - next signal
            continue
    return proc.poll()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.run",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, help="g2op_... (a file of the benchmark's config/)")
    ap.add_argument("--layout", type=int, required=True)
    ap.add_argument("--part", choices=P.PARTS, default="learn", help="the split part the layout must belong to")
    ap.add_argument("--seed", type=int, default=None, help="default: the robust board's own seed outside the test, "
                                                           "else the official one")
    ap.add_argument("--gpu", default=os.environ.get("HEXASHELL_GPU", "0"))
    ap.add_argument("--port", type=int, default=0, help="the WebSocket port (0: a free one)")
    ap.add_argument("--out", default=str(ROOT / "results" / "benches" / "robocoliseum1"), help="records")
    ap.add_argument("--run-dir", default=None, help="default: <out>/runs/<config>_L<layout>_s<seed>_<time>")
    ap.add_argument("--settings", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--hold-s", type=float, default=HOLD_S)
    ap.add_argument("--start-s", type=float, default=START_S)
    ap.add_argument("--max-h", type=float, default=24.0, help="the whole run's cap")
    ap.add_argument("--keep-caches", action="store_true", help="keep the run's Kit / warp folders")
    args = ap.parse_args(argv)

    facts = P.parse_config(args.config)
    config, board = facts["config"], facts["board"]
    if not 0 <= args.layout < facts["layouts"]:
        ap.error(f"{config} has layouts 0..{facts['layouts'] - 1}")
    if P.part_of(args.layout) != args.part:
        ap.error(f"layout {args.layout} is a {P.part_of(args.layout)} layout, not {args.part} "
                 f"(--part {P.part_of(args.layout)} to run it)")
    seed = args.seed if args.seed is not None else P.seed_for(board, args.part)
    settings = settings_for(board, args.settings, args.set)
    out = Path(args.out).resolve()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    run_dir = Path(args.run_dir or out / "runs" / f"{config}_L{args.layout}_s{seed}_{stamp}").resolve()
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
        info = {"config": config, "layout": args.layout, "eval_seed": seed, "episode": link.index,
                "part": args.part, "run_dir": str(run_dir)}
        log({"kind": "episode_begin", "episode": link.index, "prompt": link.prompt})
        episode = Episode(ColiseumSession(link, info), settings, out)
        try:
            episode.run()
        finally:
            entry = {"episode": link.index, "record": str(episode.rec.dir)}
            result = episode.rec.dir / "result.json"
            if result.exists():
                data = json.loads(result.read_text())
                entry.update({k: data.get(k) for k in ("instruction", "success", "progress", "score", "end", "turns", "errors")})
            played.append(entry)
            log({"kind": "episode_end", **entry})

    server = Server("127.0.0.1", port, on_episode=play, results_dir=run_dir / "root" / "output" / "benchmark",
                    hold_s=args.hold_s, log_path=run_dir / "link.jsonl")
    server.start()
    overrides = [f"--benchmark.infer_host=127.0.0.1:{port}", f"--benchmark.instance_ids={args.layout}",
                 f"--benchmark.seed={seed}"]
    log({"kind": "start", "config": config, "board": board, "layout": args.layout, "part": args.part, "seed": seed,
         "gpu": args.gpu, "port": port, "ros_domain": domain, "overrides": overrides, "model": settings.model,
         "reasoning_effort": settings.reasoning_effort, "skills": settings.skills, "script": settings.script})

    def on_term(signum, _frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)  # once: the clean-up below runs to its end
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_term)
    judge = None
    if facts["sub_task"] in P.VLM_JUDGED:
        from .judge import Judge
        judge = Judge(settings, log_path=run_dir / "judge.jsonl").start()
        log({"kind": "judge", "url": judge.url, "model": judge.model, "effort": judge.effort})
    proc = sim.launch(run_dir, args.gpu, config, overrides, domain=domain, judge_url=judge and judge.url)
    t0 = time.monotonic()
    end = "sim_exit"
    rc = None
    try:
        quiet_since = None
        while proc.poll() is None:
            now = time.monotonic()
            if not server.connected_once.is_set() and now - t0 > args.start_s:
                end = "no_connection"
                break
            if now - t0 > args.max_h * 3600:
                end = "run_cap"
                break
            done = server.connected_once.is_set() and server.disconnected.is_set() and all(
                ep.over for ep in server.episodes)
            quiet_since = (quiet_since or now) if done else None
            if quiet_since is not None and now - quiet_since > EXIT_GRACE_S:
                end = "sim_exit_hung"
                break
            time.sleep(2.0)
    except KeyboardInterrupt as exc:
        end = f"interrupted ({exc})"
    finally:
        rc = stop_sim(proc, log)
        if judge is not None:
            judge.stop()
            log({"kind": "judge_end", "requests": judge.requests, "failures": judge.failures})
        log({"kind": "sim_end", "end": end, "returncode": rc, "minutes": round((time.monotonic() - t0) / 60, 1)})
        for ep in server.episodes:
            if not ep.over:
                ep.abandon("the benchmark process ended")
        for t in list(server.threads):
            t.join(timeout=900)
        server.stop()
        os.close(lock_fd)
        if not args.keep_caches:
            for sub in ("kit", "warp", "documents"):
                shutil.rmtree(run_dir / sub, ignore_errors=True)

    played.sort(key=lambda e: e["episode"])
    summary = {"config": config, "layout": args.layout, "seed": seed, "part": args.part, "run_dir": str(run_dir),
               "end": end, "returncode": rc, "episodes": len(server.episodes), "results": played,
               "minutes": round((time.monotonic() - t0) / 60, 1)}
    (run_dir / "run.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n")
    events.close()
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if end in ("sim_exit", "sim_exit_hung") and played else 1


if __name__ == "__main__":
    sys.exit(main())
