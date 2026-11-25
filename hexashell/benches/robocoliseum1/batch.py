"""A batch of RoboColiseum runs over several machines.

    python -m hexashell.benches.robocoliseum1.batch plan <batch dir> --part learn [validation ..] [--boards ..] [--tasks ..]
                                                        [--repeats N] [--arms sk ns [--arms-where-noted]] [--official]
    python -m hexashell.benches.robocoliseum1.batch run <batch dir> --hosts <hosts.json>
    python -m hexashell.benches.robocoliseum1.batch status <batch dir>
    python -m hexashell.benches.robocoliseum1.batch report <batch dir>

A unit is one run (:mod:`.run`: one layout of one config, all its instructions).  ``plan`` lists the units of
a part in ``units.json`` (the robust board: in learning and validation one layout per (task, perturbation),
:func:`.protocol.robust_layout`; in the test the official draw's test layouts, :func:`.protocol.official_draw`),
longest first; ``--official`` plans what the official run plays in those parts instead (the robust board:
the official draw's layouts there, with its seed).  ``--arms sk ns`` plans every unit once per arm (``sk``
with the skills, ``ns`` without: the unit's own env sets HEXASHELL_SKILLS over the host's), side by side in
the order, so a control and its treatment run in one batch at the same time (``--arms-where-noted``: only
the tasks that have a skill).  ``run`` plays them on the machines of the hosts file and is the only writer of
``ctl/state.json``; stopped and started again, it picks up where it was.  A file ``ctl/STOP`` lets running
units end and starts no new ones.  The hosts file is read again every round, so machines and slots can change
while it runs.  ``report`` writes ``episodes.tsv`` (one row per episode of each unit's last attempt).

Each attempt of a unit has its folder ``units/<id>/a<n>/``: ``run/`` (the run folder), the records of its
episodes, ``run.out``, ``pid`` and ``rc``.  The batch folder has the same absolute path on every machine;
a remote machine's attempt folder is copied back when the attempt ends, without the cameras' videos and the
run's links into the source tree.  An attempt that did not play all its episodes (the simulator failed, the
machine lost it) is tried again, up to ``--attempts`` in all.

hosts file: a JSON list of {"name", "ssh" (user@address; "" for this machine), "ssh_key"
(optional), "code" (the checkout there), "python", "gpus" {index: units at once}, "need_mb" (free GPU memory to start
one), "max" (units at once on the machine), "env" {NAME: VALUE} (e.g. HEXASHELL_SAM3_URL)}.  A unit's own
"code" (a checkout at the same path on every machine), when units.json gives one, runs it instead of the
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
from pathlib import Path

from . import protocol as P

OK_ENDS = ("sim_exit", "sim_exit_hung")
ROUND_S = 30.0
#: seconds between two starts on one machine (a simulator's start is heavy)
LAUNCH_GAP_S = 60.0
#: a unit started this recently may not hold all its GPU memory yet; it is counted as SIM_MB more
LOADING_S = 420.0
SIM_MB = 9000
UNIT_TIMEOUT_S = 30000
#: order of the boards in a plan (longest episodes first)
PRIORITY = {"manip": 0, "instruction": 1, "robust": 2, "spatial": 3}
#: arms a unit can be planned in: the env its run gets on top of the host's
ARMS = {"sk": {"HEXASHELL_SKILLS": "1"}, "ns": {"HEXASHELL_SKILLS": "0"}}
NO_PROXY = ("HTTPS_PROXY", "HTTP_PROXY", "https_proxy", "http_proxy", "ALL_PROXY", "all_proxy")
SYNC_EXCLUDES = ("*.webm", "run/root/source/", "run/xdg/", "run/bin/", "run/kit/", "run/warp/", "run/documents/")


def episodes_of(config: str, layout: int) -> int:
    """How many episodes a run of this layout plays (its instructions; counted, never read out)."""
    sub = P.parse_config(config)["sub_task"]
    data = json.loads((P.NATIVE_CONFIG / "llm_task" / sub / str(layout) / "instructions.json").read_text())
    return len(data["instructions"])


def has_note(board: str, task: str) -> bool:
    """Whether a card of this task gets any note (:func:`.session.note_names`), i.e. whether the arms differ."""
    from .session import note_names, skill

    return any(skill(name) is not None for name in note_names(board, task))


def plan_units(parts, boards, tasks=None, repeats: int = 1, arms=None, official: bool = False,
               arms_where_noted: bool = False) -> list[dict]:
    """The units of one part or several; ``tasks``: only these base tasks; ``repeats``: each unit that many
    times (the k-th repeat's id ends in ``_r<k>``, same layout and seed); ``arms``: each unit once per arm
    (:data:`ARMS`; the id ends in ``_<arm>``); ``arms_where_noted``: only the units whose task gets a note
    (:func:`has_note`) once per arm, the others, whose prompt is the same in every arm, once in the first arm;
    ``official``: the episodes the official run plays in these parts - the robust board's official draw
    (:func:`.protocol.official_draw`, seed 1) instead of our own layout and seed outside the test (the unit
    carries ``seed``); the other boards play every layout with the official seed in every part anyway."""
    parts = (parts,) if isinstance(parts, str) else tuple(parts)
    if repeats > 1 and ("test" in parts or official):
        raise SystemExit("the test and the official episodes are played once each")
    units = []
    for part in parts:
        for config in P.configs(boards):
            if tasks and P.parse_config(config)["task"] not in tasks:
                continue
            board = P.board_of(config)
            seed = None
            if board == "robust":
                if part == "test" or official:
                    layouts = [k for k in P.official_draw(config) if P.part_of(k) == part]
                    seed = P.OFFICIAL_SEED if part != "test" else None
                else:
                    layouts = [P.robust_layout(config, part)]
            else:
                layouts = P.layouts(config, part)
            unit_arms = arms
            if arms and arms_where_noted and not has_note(board, P.parse_config(config)["task"]):
                unit_arms = arms[:1]
            for layout in layouts:
                for k in range(1, repeats + 1):
                    unit = {"id": f"{config}_L{layout}" + (f"_r{k}" if k > 1 else ""), "config": config,
                            "layout": layout, "part": part, "board": board, "repeat": k,
                            "episodes": episodes_of(config, layout)}
                    if seed is not None:
                        unit["seed"] = seed
                    if not unit_arms:
                        units.append(unit)
                    for arm in unit_arms or ():
                        units.append({**unit, "id": f"{unit['id']}_{arm}", "arm": arm, "env": dict(ARMS[arm])})
    if tasks and not units:
        raise SystemExit(f"no units: tasks {tasks} are not on boards {list(boards)}")
    units.sort(key=lambda u: (PRIORITY[u["board"]], u["repeat"], -u["episodes"], u["id"]))
    return units


def write_json(path: Path, data) -> None:
    tmp = path.with_name(path.name + ".new")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n")
    os.replace(tmp, path)


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
        self.need_mb = int(spec.get("need_mb", 10000))
        self.max = int(spec.get("max", sum(self.gpus.values())))
        self.env = dict(spec.get("env", {}))
        control = Path("/tmp") / f"hexashell-robocoliseum1-{os.getuid()}"
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
        env = {k: v for k, v in os.environ.items() if k not in NO_PROXY}
        quiet = dict(stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
        try:
            if subprocess.run(["ssh", *self.ssh_opts, "-O", "check", self.ssh], timeout=20, **quiet).returncode != 0:
                subprocess.run(["ssh", *self.master_opts, "-fN", self.ssh], timeout=60, **quiet)
        except subprocess.TimeoutExpired:
            pass

    def sh(self, script: str, timeout: float = 120.0) -> subprocess.CompletedProcess:
        cmd = ["ssh", *self.ssh_opts, self.ssh, "bash -s"] if self.ssh else ["bash", "-s"]
        env = {k: v for k, v in os.environ.items() if k not in NO_PROXY}
        return subprocess.run(cmd, input=script, capture_output=True, text=True, timeout=timeout, env=env)

    def launch_script(self, unit: dict, gpu: str, adir: str) -> str:
        env = " ".join(f"{k}={shlex.quote(str(v))}" for k, v in {**self.env, **unit.get("env", {})}.items())
        unset = " ".join(f"-u {k}" for k in ("DISPLAY", "XAUTHORITY", *NO_PROXY))
        inner = (f"env {unset} PYTHONPATH=. {env} timeout {UNIT_TIMEOUT_S} {shlex.quote(self.python)} "
                 f"-m hexashell.benches.robocoliseum1.run --config {unit['config']} --layout {unit['layout']} "
                 f"--part {unit['part']} " + (f"--seed {int(unit['seed'])} " if "seed" in unit else "")
                 + f"--gpu {gpu} --out {shlex.quote(adir)} --run-dir {shlex.quote(adir + '/run')} "
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
        cmd = ["rsync", "-a", *[f"--exclude={e}" for e in SYNC_EXCLUDES],
               "-e", " ".join(["ssh", *map(shlex.quote, self.ssh_opts)]), f"{self.ssh}:{adir}/", f"{adir}/"]
        env = {k: v for k, v in os.environ.items() if k not in NO_PROXY}
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=1800, env=env)
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
        run = Path(self.adir(uid, attempt["n"])) / "run" / "run.json"
        if not run.exists():
            return False, f"rc {rc}, no run.json"
        data = json.loads(run.read_text())
        played = len(data.get("results") or [])
        want = self.units[uid]["episodes"]
        if data.get("end") not in OK_ENDS or played < want:
            return False, f"rc {rc}, end {data.get('end')}, played {played}/{want}"
        return True, f"played {played}/{want}"

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
        queued = [u for u in self.order if self.state[u]["status"] == "queued"]
        if not queued:
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
        uid = queued[0]
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
        from collections import Counter
        def group(u):  # the board, and the arm when the batch has arms
            arm = self.units[u].get("arm")
            return self.units[u]["board"] + (f" {arm}" if arm else "")

        c = Counter((group(u), self.state[u]["status"]) for u in self.order)
        groups = sorted({g for g, _ in c}, key=lambda g: (PRIORITY[g.split()[0]], g))
        out = []
        for g in groups:
            out.append(f"{g:12s} " + " ".join(f"{s} {c[(g, s)]}" for s in ("done", "running", "queued", "failed")
                                                if c[(g, s)]))
        hosts = Counter(self.state[u]["attempts"][-1]["host"] for u in self.order
                        if self.state[u]["status"] == "running")
        out.append("running: " + ", ".join(f"{h} {n}" for h, n in sorted(hosts.items())))
        return "\n".join(out)

    def report(self) -> Path:
        """``episodes.tsv``: one row per episode of each ended unit's last attempt (from the records'
        result.json), and ``summary.tsv``: per board and task, successes and mean progress."""
        cols = ["unit", "arm", "board", "task", "kind", "layout", "status", "attempt", "episode", "success", "progress",
                "stages", "stage", "step", "end", "turns", "seconds", "prompt_tokens", "errors", "record"]
        rows, per = ["\t".join(cols)], {}
        for uid in self.order:
            st = self.state[uid]
            if not st["attempts"] or st["status"] == "running":
                continue
            a = st["attempts"][-1]
            for rec in sorted(Path(self.adir(uid, a["n"])).glob("g2op_*")):
                res = rec / "result.json"
                if not res.exists():
                    continue
                d = json.loads(res.read_text())
                v = d.get("verdict") or {}
                subs = " ".join(f"{x.get('name')}:{x.get('score')}" for x in v.get("sub_steps") or [])
                arm = self.units[uid].get("arm", "")
                row = [uid, arm, d.get("board"), d.get("task"), d.get("kind") or "", d.get("layout"), st["status"], a["n"],
                       d.get("episode"), d.get("success"), d.get("progress"), subs, v.get("stage"), d.get("step"),
                       d.get("end"), d.get("turns"), d.get("seconds"), (d.get("tokens") or {}).get("prompt"),
                       len(d.get("errors") or []), rec.name]
                rows.append("\t".join("" if x is None else str(x) for x in row))
                key = (arm, d.get("board"), d.get("task"))
                n, ok, prog = per.get(key, (0, 0, 0.0))
                per[key] = (n + 1, ok + bool(d.get("success")), prog + float(d.get("progress") or 0.0))
        path = self.root / "episodes.tsv"
        path.write_text("\n".join(rows) + "\n")
        arms = sorted({a for a, _, _ in per})
        head = "board\ttask\tepisodes\tsuccesses\tmean_progress"
        lines = [("arm\t" if arms != [""] else "") + head]

        def line(arm, board, task, n, ok, prog):
            lines.append((f"{arm}\t" if arms != [""] else "") + f"{board}\t{task}\t{n}\t{ok}\t{prog / n:.3f}")

        for (arm, board, task), (n, ok, prog) in sorted(per.items(), key=lambda kv: (
                PRIORITY.get(kv[0][1], 9), kv[0][2], kv[0][0])):
            line(arm, board, task, n, ok, prog)
        for arm in arms:
            for board in sorted({b for a, b, _ in per if a == arm}, key=lambda b: PRIORITY.get(b, 9)):
                vals = [v for k, v in per.items() if k[0] == arm and k[1] == board]
                line(arm, board, "ALL", sum(v[0] for v in vals), sum(v[1] for v in vals), sum(v[2] for v in vals))
        (self.root / "summary.tsv").write_text("\n".join(lines) + "\n")
        return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.batch",
                                 description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan")
    p.add_argument("dir")
    p.add_argument("--part", nargs="+", choices=P.PARTS, required=True)
    p.add_argument("--official", action="store_true",
                   help="the official run's episodes of these parts (robust: the official draw and seed)")
    p.add_argument("--boards", nargs="+", default=list(P.BOARDS), choices=P.BOARDS)
    p.add_argument("--tasks", nargs="+", default=None, help="only these base tasks (e.g. bimanual_hold_pot)")
    p.add_argument("--repeats", type=int, default=1, help="each unit this many times (learning, validation)")
    p.add_argument("--arms", nargs="+", default=None, choices=sorted(ARMS),
                   help="each unit once per arm: sk = skills on, ns = skills off (a control in the same batch)")
    p.add_argument("--arms-where-noted", action="store_true",
                   help="with --arms: tasks without a skill (the same prompt in every arm) once, in the first arm")
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
        units = plan_units(args.part, args.boards, args.tasks, args.repeats, args.arms, args.official,
                           args.arms_where_noted)
        write_json(root / "units.json", units)
        from collections import Counter
        u, e = Counter(x["board"] for x in units), Counter()
        for x in units:
            e[x["board"]] += x["episodes"]
        print(" ".join(f"{b} {u[b]} units / {e[b]} episodes" for b in u), f"- {len(units)} units, {sum(e.values())} episodes")
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
