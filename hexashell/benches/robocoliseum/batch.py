"""A batch of RoboColiseum Season II runs over several machines.

    python -m hexashell.benches.robocoliseum.batch plan <batch dir> --part learn [validation ..] [--boards ..]
                                                       [--robots ..] [--tasks ..] [--configs ..] [--repeats N]
                                                       [--arms sk ns [--arms-where-noted]]
    python -m hexashell.benches.robocoliseum.batch run <batch dir> --hosts <hosts.json>
    python -m hexashell.benches.robocoliseum.batch status <batch dir>
    python -m hexashell.benches.robocoliseum.batch report <batch dir>

A unit is one run (:mod:`.run`): one instance of one config, one episode.  ``plan`` lists the units of a part
of our split (:func:`.protocol.cases`; Isaac tasks only, robots with robot calls only) in ``units.json``, by
board (:data:`PRIORITY`), then config and instance; ``--arms sk ns`` plans every unit once per arm (``sk``
with the skills, ``ns`` without: the unit's own env sets HEXASHELL_SKILLS over the host's), side by side, so a
control and its treatment run in one batch at the same time (``--arms-where-noted``: only the tasks that get a
note).  ``run`` plays them on the machines of the hosts file and is the only writer of ``ctl/state.json``;
stopped and started again, it picks up where it was.  A file ``ctl/STOP`` lets running units end and starts no
new ones.  The hosts file is read again every round, so machines and slots can change while it runs.
``report`` writes ``episodes.tsv`` (one row per unit's last attempt) and ``summary.tsv``.

Each attempt of a unit has its folder ``units/<id>/a<n>/``: ``run/`` (the run folder), the episode's record,
``run.out``, ``pid`` and ``rc``.  The batch folder has the same absolute path on every machine; a remote
machine's attempt folder is copied back when the attempt ends.  An attempt without a result file (the
simulator failed, the machine lost it) is tried again, up to ``--attempts`` in all.  A run uses the container
of its GPU (``hexashell-geniesim-gpu<N>``, started when missing: :func:`.sim.ensure`), so the machines need
the image and the assets (``protocol.IMAGE``, ``protocol.ASSETS``).

hosts file: a JSON list of {"name", "ssh" (user@address or an ssh alias; "" for this machine), "ssh_key"
(optional), "code" (the checkout there), "python", "gpus" {index: units at once},
"need_mb" (free GPU memory to start one), "max" (units at once on the machine), "runtime_max" {runtime: units
of that runtime at once on the machine} (optional; e.g. {"libuipc": 1}: the benchmark pins the stream scorer of
every libuipc run to the machine's highest CPU, and two at once starve both), "env" {NAME: VALUE} (e.g.
HEXASHELL_SAM3_URL)}.  A queued unit whose runtime is at its cap waits; the next one in the order may start.  A
unit's own "code" (a checkout at the same path on every machine), when units.json gives one, runs it instead of the
host's: two versions of the skills as two arms of one batch.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

from . import protocol as P

OK_ENDS = ("sim_exit", "sim_exit_hung")
ROUND_S = 30.0
#: seconds between two starts on one machine (a simulator's start is heavy)
LAUNCH_GAP_S = 90.0
#: a unit started this recently may not hold all its GPU memory yet; it is counted as SIM_MB more
LOADING_S = 420.0
SIM_MB = 9000
#: a unit's cap: a cold start (45 min), the episode (90 min) and the end, with room
UNIT_TIMEOUT_S = 4 * 3600
#: order of the boards in a plan
PRIORITY = {b: k for k, b in enumerate(P.BOARDS)}
#: arms a unit can be planned in: the env its run gets on top of the host's
ARMS = {"sk": {"HEXASHELL_SKILLS": "1"}, "ns": {"HEXASHELL_SKILLS": "0"}}
NO_PROXY = ("HTTPS_PROXY", "HTTP_PROXY", "https_proxy", "http_proxy", "ALL_PROXY", "all_proxy")


def playable(robot: str) -> bool:
    """Whether the bench has robot calls for this robot (:data:`.session.ROBOT_WORDS`)."""
    from .session import ROBOT_WORDS

    return robot in ROBOT_WORDS


def has_note(t: P.Task) -> bool:
    """Whether a card of this task gets any note (:func:`.session.note_names`), i.e. whether the arms differ."""
    from .session import note_names, notes_folder, skill

    folder = notes_folder(t)
    return any(skill(name, folder) is not None for name, _ in note_names(t))


def plan_units(parts, boards=P.BOARDS, robots=None, tasks=None, configs=None, repeats: int = 1, arms=None,
               arms_where_noted: bool = False) -> list[dict]:
    """The units of one part or several: the tasks of ``boards`` (``robots``, ``tasks``: only these robots and
    base tasks; ``configs``: only these configs) on robots the bench plays, one unit per case (our own wok_flip
    and Rubik's cube cases carry their seed and archive); ``repeats``: each unit
    that many times (the k-th repeat's id ends in ``_r<k>``); ``arms``: each unit once per arm (:data:`ARMS`;
    the id ends in ``_<arm>``; ``arms_where_noted``: a task that gets no note once, in the first arm)."""
    parts = (parts,) if isinstance(parts, str) else tuple(parts)
    if repeats > 1 and "test" in parts:
        raise SystemExit("the test episodes are played once each")
    units, skipped = [], Counter()
    for part in parts:
        for config in P.configs(boards, robots):
            t = P.task(config)
            if (tasks and t.task not in tasks) or (configs and config not in configs):
                continue
            if t.runtime not in P.RUNTIMES:
                skipped[f"{t.runtime} runtime"] += 1
                continue
            if not playable(t.robot):
                skipped[f"robot {t.robot}"] += 1
                continue
            unit_arms = arms
            if arms and arms_where_noted and not has_note(t):
                unit_arms = arms[:1]
            for case in P.cases(config, part):
                for k in range(1, repeats + 1):
                    unit = {"id": case.name + (f"_r{k}" if k > 1 else ""), "config": config,
                            "instance": case.instance, "part": part, "board": t.board, "robot": t.robot,
                            "task": t.task, "repeat": k, "runtime": t.runtime,
                            **({"seed": case.seed} if case.seed is not None else {}),
                            **({"archive": case.archive} if case.archive else {})}
                    if not unit_arms:
                        units.append(unit)
                    for arm in unit_arms or ():
                        units.append({**unit, "id": f"{unit['id']}_{arm}", "arm": arm, "env": dict(ARMS[arm])})
    if configs:
        unknown = set(configs) - set(P.TASKS)
        if unknown:
            raise SystemExit(f"not Season II configs: {sorted(unknown)}")
    if skipped:
        print("not planned (configs x parts): " + ", ".join(f"{k} {n}" for k, n in sorted(skipped.items())),
              file=sys.stderr)
    if not units:
        raise SystemExit("no units: nothing matches")
    units.sort(key=lambda u: (PRIORITY[u["board"]], u["repeat"], u["config"], u["instance"], u.get("seed") or 0,
                              u.get("arm", "")))
    return units


def write_json(path: Path, data) -> None:
    tmp = path.with_name(path.name + ".new")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n")
    os.replace(tmp, path)


def _no_proxy_env() -> dict:
    return {k: v for k, v in os.environ.items() if k not in NO_PROXY}


class Host:
    def __init__(self, spec: dict):
        self.update(spec)
        self.last_launch = 0.0
        self.polled = False
        self.gpu_mb: dict[str, tuple[int, int]] = {}  # index -> (used, total)

    def update(self, spec: dict) -> None:
        self.spec = spec
        self.name = spec["name"]
        self.ssh = spec.get("ssh") or ""
        self.code = spec["code"]
        self.python = spec["python"]
        self.gpus = {str(k): int(v) for k, v in spec.get("gpus", {}).items()}
        self.need_mb = int(spec.get("need_mb", 12000))
        self.max = int(spec.get("max", sum(self.gpus.values())))
        self.runtime_max = {str(k): int(v) for k, v in spec.get("runtime_max", {}).items()}
        self.env = dict(spec.get("env", {}))
        control = Path("/tmp") / f"hexashell-robocoliseum-ssh-{os.getuid()}"
        control.mkdir(mode=0o700, exist_ok=True)
        # commands share one master connection, started on its own (a master started by a command whose
        # output is captured would keep that output open while it lives)
        base = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=15", "-o", "ServerAliveInterval=30",
                "-o", f"ControlPath={control}/ssh-%C"]
        if spec.get("ssh_key"):
            base = ["-i", os.path.expanduser(spec["ssh_key"]), "-o", "IdentitiesOnly=yes", *base]
        self.ssh_opts = [*base, "-o", "ControlMaster=no"]
        self.master_opts = [*base, "-o", "ControlMaster=yes", "-o", "ControlPersist=1800"]

    def ensure_master(self) -> None:
        if not self.ssh:
            return
        quiet = dict(stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     env=_no_proxy_env())
        try:
            if subprocess.run(["ssh", *self.ssh_opts, "-O", "check", self.ssh], timeout=20, **quiet).returncode != 0:
                subprocess.run(["ssh", *self.master_opts, "-fN", self.ssh], timeout=60, **quiet)
        except subprocess.TimeoutExpired:
            pass

    def sh(self, script: str, timeout: float = 120.0) -> subprocess.CompletedProcess:
        cmd = ["ssh", *self.ssh_opts, self.ssh, "bash -s"] if self.ssh else ["bash", "-s"]
        return subprocess.run(cmd, input=script, capture_output=True, text=True, timeout=timeout,
                              env=_no_proxy_env())

    def launch_script(self, unit: dict, gpu: str, adir: str) -> str:
        env = " ".join(f"{k}={shlex.quote(str(v))}" for k, v in {**self.env, **unit.get("env", {})}.items())
        unset = " ".join(f"-u {k}" for k in ("DISPLAY", "XAUTHORITY", *NO_PROXY))
        inner = (f"env {unset} PYTHONPATH=. {env} timeout {UNIT_TIMEOUT_S} {shlex.quote(self.python)} "
                 f"-m hexashell.benches.robocoliseum.run --config {shlex.quote(unit['config'])} "
                 f"--instance {int(unit['instance'])} --part {shlex.quote(unit['part'])} --gpu {int(gpu)} "
                 + (f"--seed {int(unit['seed'])} " if unit.get("seed") is not None else "") +
                 f"--out {shlex.quote(adir)} --run-dir {shlex.quote(adir + '/run')} "
                 f"> {shlex.quote(adir + '/run.out')} 2>&1; echo $? > {shlex.quote(adir + '/rc')}")
        code = unit.get("code") or self.code
        return (f"set -e\nmkdir -p {shlex.quote(adir)}\ncd {shlex.quote(code)}\n"
                f"setsid nohup bash -c {shlex.quote(inner)} > /dev/null 2>&1 < /dev/null &\n"
                f"echo $! > {shlex.quote(adir + '/pid')}\necho $!\n")

    def poll_script(self, adirs: list[str]) -> str:
        lines = []
        for a in adirs:
            q = shlex.quote(a)
            lines.append(f'if [ -f {q}/rc ]; then echo "R {a} $(cat {q}/rc)"; '
                         f'elif [ -f {q}/pid ] && kill -0 "$(cat {q}/pid)" 2>/dev/null; then echo "A {a}"; '
                         f'else echo "G {a}"; fi')
        lines.append("nvidia-smi --query-gpu=index,memory.used,memory.total --format=csv,noheader,nounits "
                     "| sed 's/^/M /'")
        return "\n".join(lines) + "\n"

    def fetch(self, adir: str) -> bool:
        """Copy a remote attempt folder back (this machine's are in place)."""
        if not self.ssh:
            return True
        Path(adir).mkdir(parents=True, exist_ok=True)
        cmd = ["rsync", "-a", "-e", " ".join(["ssh", *map(shlex.quote, self.ssh_opts)]), f"{self.ssh}:{adir}/",
               f"{adir}/"]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800, env=_no_proxy_env())
        return r.returncode == 0


class Batch:
    def __init__(self, root: Path):
        self.root = Path(os.path.abspath(root))  # not resolved: the same path names it on every machine
        self.ctl = self.root / "ctl"
        self.ctl.mkdir(parents=True, exist_ok=True)
        self.units = {u["id"]: u for u in json.loads((self.root / "units.json").read_text())}
        self.order = list(self.units)
        path = self.ctl / "state.json"
        self.state: dict[str, dict] = json.loads(path.read_text()) if path.exists() else {}
        for uid in self.order:
            self.state.setdefault(uid, {"status": "queued", "attempts": []})

    def save(self) -> None:
        write_json(self.ctl / "state.json", self.state)

    def log(self, text: str) -> None:
        line = f"{time.strftime('%m-%d %H:%M:%S')} {text}"
        with open(self.ctl / "ctl.log", "a", encoding="utf-8") as f:
            f.write(line + "\n")
        print(line, flush=True)

    def adir(self, uid: str, n: int) -> str:
        return str(self.root / "units" / uid / f"a{n}")

    def judge(self, uid: str, attempt: dict, rc: int | None) -> tuple[bool, str]:
        """An attempt counts when the benchmark ended on its own and wrote its result file, and our episode
        ended with a result (its score is the benchmark's, whatever it is)."""
        run = Path(self.adir(uid, attempt["n"])) / "run" / "run.json"
        if not run.exists():
            return False, f"rc {rc}, no run.json"
        data = json.loads(run.read_text())
        played = data.get("results") or []
        # a result file can hold no score: a cloth_hanger run went to its end, its scorer was stopped while
        # it still caught up, and the file the benchmark left had no details (run.json from before scored_files
        # was written: the result files count)
        scored = data.get("scored_files", data.get("result_files")) or []
        if data.get("end") not in OK_ENDS or not scored or not played:
            return False, (f"rc {rc}, end {data.get('end')}, result files {len(data.get('result_files') or [])}, "
                           f"scored {len(scored)}, episodes {len(played)}")
        first = played[0]
        if first.get("valid") is False:
            return False, f"the benchmark marked the episode invalid (score {first.get('score')})"
        if first.get("valid") is None and first.get("score") is None:
            return False, "no verdict for the episode"
        return True, f"score {first.get('score')} progress {first.get('progress')} dispatches {first.get('dispatches')}"

    def finish(self, host: Host, uid: str, rc: int | None, attempts_max: int) -> None:
        st = self.state[uid]
        attempt = st["attempts"][-1]
        adir = self.adir(uid, attempt["n"])
        fetched = host.fetch(adir)
        ok, why = self.judge(uid, attempt, rc)
        attempt.update(t1=round(time.time(), 1), rc=rc, ok=ok, why=why, fetched=fetched)
        if ok:
            st["status"] = "done"
        elif len(st["attempts"]) < attempts_max:
            st["status"] = "queued"
        else:
            st["status"] = "failed"
        self.save()
        self.log(f"{'done' if ok else 'FAIL'} {uid} a{attempt['n']} on {host.name}:{attempt['gpu']} ({why})"
                 f"{'' if fetched else ' [copy back failed]'} -> {st['status']}")

    def running_on(self, host: str) -> list[str]:
        return [uid for uid in self.order if self.state[uid]["status"] == "running"
                and self.state[uid]["attempts"][-1]["host"] == host]

    def next_queued(self, host: Host, running: list[str]) -> str | None:
        """The first queued unit, in the order, whose runtime is not at its cap on this machine (``runtime_max``)."""
        busy = Counter(self.units[u].get("runtime") for u in running)
        for uid in self.order:
            if self.state[uid]["status"] != "queued":
                continue
            runtime = self.units[uid].get("runtime")
            cap = host.runtime_max.get(runtime) if runtime is not None else None
            if cap is None or busy[runtime] < cap:
                return uid
        return None

    def run(self, hosts_path: Path, attempts_max: int) -> int:
        hosts: dict[str, Host] = {}
        noted: set[str] = set()
        self.log(f"start: {len(self.order)} units, hosts from {hosts_path}")
        while True:
            specs = json.loads(hosts_path.read_text())
            for spec in specs:
                if spec["name"] in hosts:
                    hosts[spec["name"]].update(spec)
                else:
                    hosts[spec["name"]] = Host(spec)
            for uid in self.order:  # units running on a machine no longer listed stay polled through it
                st = self.state[uid]
                if st["status"] == "running" and st["attempts"][-1]["host"] not in hosts and uid not in noted:
                    noted.add(uid)
                    self.log(f"note: {uid} runs on {st['attempts'][-1]['host']}, not in the hosts file")
            now = time.time()
            for host in hosts.values():
                host.ensure_master()
                running = self.running_on(host.name)
                try:
                    r = host.sh(host.poll_script([self.adir(u, self.state[u]["attempts"][-1]["n"]) for u in running]))
                except subprocess.TimeoutExpired:
                    host.polled = False
                    self.log(f"poll {host.name}: timed out")
                    continue
                if r.returncode != 0:
                    host.polled = False
                    self.log(f"poll {host.name}: rc {r.returncode} {r.stderr.strip()[:200]}")
                    continue
                host.polled = True
                host.gpu_mb = {}
                for line in r.stdout.splitlines():
                    parts = line.split()
                    if not parts:
                        continue
                    if parts[0] == "M" and len(parts) >= 4:
                        idx, used, total = (p.strip(",") for p in parts[1:4])
                        host.gpu_mb[idx] = (int(used), int(total))
                    elif parts[0] in ("R", "G") and len(parts) >= 2:
                        uid = Path(parts[1]).parent.name
                        rc = int(parts[2]) if parts[0] == "R" and len(parts) >= 3 and parts[2].lstrip("-").isdigit() else None
                        if uid in self.state and self.state[uid]["status"] == "running":
                            self.finish(host, uid, rc, attempts_max)
            stop = (self.ctl / "STOP").exists()
            if not stop:
                for host in hosts.values():
                    self.maybe_launch(host, now)
            left = [u for u in self.order if self.state[u]["status"] in ("queued", "running")]
            if not left or (stop and not any(self.state[u]["status"] == "running" for u in self.order)):
                self.log("all units ended" if not left else "stopped (ctl/STOP), nothing running")
                return 0
            time.sleep(ROUND_S)

    def maybe_launch(self, host: Host, now: float) -> None:
        if not host.polled or now - host.last_launch < LAUNCH_GAP_S:
            return
        running = self.running_on(host.name)
        if len(running) >= host.max:
            return
        uid = self.next_queued(host, running)
        if uid is None:
            return
        best, best_free = None, None
        for gpu, cap in host.gpus.items():
            if gpu not in host.gpu_mb:
                continue
            ours = [u for u in running if self.state[u]["attempts"][-1]["gpu"] == gpu]
            if len(ours) >= cap:
                continue
            loading = sum(1 for u in ours if now - self.state[u]["attempts"][-1]["t0"] < LOADING_S)
            used, total = host.gpu_mb[gpu]
            free = total - used - loading * SIM_MB
            if free >= host.need_mb and (best_free is None or free > best_free):
                best, best_free = gpu, free
        if best is None:
            return
        st = self.state[uid]
        n = len(st["attempts"]) + 1
        adir = self.adir(uid, n)
        try:
            r = host.sh(host.launch_script(self.units[uid], best, adir))
        except subprocess.TimeoutExpired:
            self.log(f"launch {uid} on {host.name}: timed out")
            return
        host.last_launch = now
        if r.returncode != 0 or not r.stdout.strip().isdigit():
            self.log(f"launch {uid} on {host.name}:{best} failed: rc {r.returncode} {r.stderr.strip()[:200]}")
            return
        st["attempts"].append({"n": n, "host": host.name, "gpu": best, "pid": int(r.stdout.strip()),
                               "t0": round(now, 1)})
        st["status"] = "running"
        self.save()
        self.log(f"start {uid} a{n} on {host.name}:{best} (free {best_free} MiB, "
                 f"{len(running) + 1} running there)")

    def status(self) -> str:
        def group(u):  # the board and robot, and the arm when the batch has arms
            unit = self.units[u]
            return f"{unit['board']} {unit['robot']}" + (f" {unit['arm']}" if unit.get("arm") else "")

        c = Counter((group(u), self.state[u]["status"]) for u in self.order)
        groups = sorted({g for g, _ in c}, key=lambda g: (PRIORITY[g.split()[0]], g))
        out = []
        for g in groups:
            out.append(f"{g:24s} " + " ".join(f"{s} {c[(g, s)]}" for s in ("done", "running", "queued", "failed")
                                                if c[(g, s)]))
        hosts = Counter(self.state[u]["attempts"][-1]["host"] for u in self.order
                        if self.state[u]["status"] == "running")
        out.append("running: " + ", ".join(f"{h} {n}" for h, n in sorted(hosts.items())))
        return "\n".join(out)

    def report(self) -> Path:
        """``episodes.tsv``: one row per ended unit's last attempt (from its record's result.json), and
        ``summary.tsv``: per board, robot and task (and arm), episodes, successes, mean score and progress."""
        cols = ["unit", "arm", "board", "robot", "task", "config", "instance", "part", "status", "attempt", "success",
                "score", "progress", "valid", "dispatches", "stages", "end", "turns", "seconds", "prompt_tokens",
                "errors", "record"]
        rows, per = ["\t".join(cols)], {}
        for uid in self.order:
            st = self.state[uid]
            if not st["attempts"] or st["status"] == "running":
                continue
            a = st["attempts"][-1]
            unit = self.units[uid]
            # a record is <config>[_<variant>]_i<NN>_e<k>-<time>: the native rubik and wok units name a variant
            for rec in sorted(Path(self.adir(uid, a["n"])).glob(f"{unit['config']}_*")):
                res = rec / "result.json"
                if not res.exists():
                    continue
                d = json.loads(res.read_text())
                v = d.get("verdict") or {}
                stages = " ".join(f"{x.get('name')}:{x.get('score')}/{x.get('count')}of{x.get('baseline')}"
                                  for x in v.get("stages") or [])
                arm = unit.get("arm", "")
                row = [uid, arm, d.get("board"), d.get("robot"), d.get("task"), d.get("config"), d.get("instance"),
                       d.get("part"), st["status"], a["n"], d.get("success"), d.get("score"), d.get("progress"),
                       d.get("valid"), d.get("dispatches"), stages, d.get("end"), d.get("turns"), d.get("seconds"),
                       (d.get("tokens") or {}).get("prompt"), len(d.get("errors") or []), rec.name]
                rows.append("\t".join("" if x is None else str(x) for x in row))
                key = (arm, d.get("board"), d.get("robot"), d.get("task"))
                n, ok, score, prog = per.get(key, (0, 0, 0.0, 0.0))
                per[key] = (n + 1, ok + bool(d.get("success")), score + float(d.get("score") or 0.0),
                            prog + float(d.get("progress") or 0.0))
        path = self.root / "episodes.tsv"
        path.write_text("\n".join(rows) + "\n")
        arms = sorted({k[0] for k in per})
        lead = "arm\t" if arms not in ([], [""]) else ""
        lines = [lead + "board\trobot\ttask\tepisodes\tsuccesses\tmean_score\tmean_progress"]

        def line(arm, board, robot, task, n, ok, score, prog):
            lines.append((f"{arm}\t" if lead else "") + f"{board}\t{robot}\t{task}\t{n}\t{ok}\t{score / n:.3f}\t"
                         f"{prog / n:.3f}")

        for (arm, board, robot, task), vals in sorted(per.items(), key=lambda kv: (
                PRIORITY.get(kv[0][1], 9), kv[0][2], kv[0][3], kv[0][0])):
            line(arm, board, robot, task, *vals)
        for arm in arms:
            for board in sorted({k[1] for k in per if k[0] == arm}, key=lambda b: PRIORITY.get(b, 9)):
                vals = [v for k, v in per.items() if k[0] == arm and k[1] == board]
                line(arm, board, "ALL", "ALL", *(sum(v[i] for v in vals) for i in range(4)))
        (self.root / "summary.tsv").write_text("\n".join(lines) + "\n")
        return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum.batch",
                                 description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("dir")
    p.add_argument("--part", nargs="+", choices=P.PARTS, required=True)
    p.add_argument("--boards", nargs="+", default=list(P.BOARDS), choices=P.BOARDS)
    p.add_argument("--robots", nargs="+", default=None, choices=sorted(P.ROBOTS), help="only these robots")
    p.add_argument("--tasks", nargs="+", default=None, help="only these base tasks (e.g. hang_cups_on_cup_tree)")
    p.add_argument("--configs", nargs="+", default=None, help="only these configs")
    p.add_argument("--repeats", type=int, default=1, help="each unit this many times (learning, validation)")
    p.add_argument("--arms", nargs="+", default=None, choices=sorted(ARMS),
                   help="each unit once per arm: sk = skills on, ns = skills off (a control in the same batch)")
    p.add_argument("--arms-where-noted", action="store_true",
                   help="with --arms: tasks that get no note (the same prompt in every arm) once, in the first arm")
    r = sub.add_parser("run")
    r.add_argument("dir")
    r.add_argument("--hosts", required=True)
    r.add_argument("--attempts", type=int, default=2)
    s = sub.add_parser("status")
    s.add_argument("dir")
    t = sub.add_parser("report")
    t.add_argument("dir")
    args = ap.parse_args(argv)
    root = Path(args.dir)

    if args.cmd == "plan":
        root.mkdir(parents=True, exist_ok=True)
        if (root / "units.json").exists():
            raise SystemExit(f"{root / 'units.json'} exists: a batch is planned once")
        if args.arms_where_noted and not args.arms:
            raise SystemExit("--arms-where-noted needs --arms")
        units = plan_units(args.part, args.boards, args.robots, args.tasks, args.configs, args.repeats, args.arms,
                           args.arms_where_noted)
        write_json(root / "units.json", units)
        c = Counter(f"{u['board']} {u['robot']}" for u in units)
        print(", ".join(f"{k} {n}" for k, n in c.items()), f"- {len(units)} units")
        return 0
    batch = Batch(root)
    if args.cmd == "run":
        return batch.run(Path(args.hosts), args.attempts)
    if args.cmd == "status":
        print(batch.status())
        return 0
    print(batch.report())
    return 0


if __name__ == "__main__":
    sys.exit(main())
