#!/usr/bin/env python3
"""Work through a list of benchmark episodes on warm simulator servers.

    python3 scripts/bench_pool.py NAME SLOTS [MAX_RUNNING]

The same list, records, logs and events as scripts/bench_batch.sh (see there): logs/NAME.todo, one
episode per line ("BENCH TASK SEED [ATTEMPT]"), records under results/NAME/, each launch's output in
logs/NAME/<tag>.log, one line per launch and exit in logs/NAME.events; the same retries (ATTEMPTS);
logs/NAME.max and logs/NAME.pause work the same way.

The difference: an episode plays on a slot, a GPU with a warm simulator server on it
(python -m hexashell.benches.daemon), which plays one episode after another without starting the
simulator again (minutes per episode: imports, the first scene, the first pictures).  SLOTS lists a
GPU per slot (4,4,5,5: two slots on GPU 4 and two on GPU 5).  A slot's server starts when the slot
first needs one, for the benchmark of the episode it takes; it is started again after RECYCLE
episodes, after an episode whose simulator failed or that left no result, and for another
benchmark; an episode that ended in an error is also played again on a fresh server.  A free slot takes an episode of the task it played last when there is one (LIBERO and
RoboCasa keep the task's environment), else of the same benchmark, else the first.  A server starts
only on a GPU with NEED MiB free, and only on a GPU that draws: before its first server, and before
the next after one failed there, hexashell.benches.gpucheck renders on it in LIBERO-PRO's environment
(else RoboCasa's; RoboTwin renders on the same graphics engine); a GPU it finds unfit (one that
times out on graphics, NVIDIA Xid 109) gets no episodes and no servers until a check GPU_CHECK
seconds later finds it fit.  Each server's log: logs/NAME/server-<slot>-<n>.log.

Settings (environment): RECYCLE (20), ATTEMPTS (3), SPACING (20 s between starts), NEED (3000),
IDLE (1800 s: a server without an episode for that long ends), GPU_CHECK (1800 s: how long a GPU's
check holds; 0: no checks), SETS (--set words for every episode,
as for bench_batch.sh), SAM3_URLS (GPU=URL,...: an episode on a listed GPU uses that SAM3 service,
so that each GPU's episodes can have their own), GATEWAY_HOLD (300 s).  Machine settings:
$HEXASHELL_ENV or config/env.sh, read by bash first.
An episode that the model's gateway ended (gateway_error) is listed again without using up one of
its ATTEMPTS (at most GATEWAY_RELISTS times), and no episode starts for GATEWAY_HOLD seconds after
it: an outage of the gateway then costs neither attempts nor a stream of episodes started into it.
Stopping the runner (SIGTERM, Ctrl-C) stops its episodes and servers and lists their episodes again.
"""

from __future__ import annotations

import fcntl
import json
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]

if os.environ.get("HEXASHELL_POOL_ENV") != "1":  # read the machine settings with bash, then run again
    env_file = os.environ.get("HEXASHELL_ENV") or str(HERE / "config" / "env.sh")
    if not os.path.isfile(env_file):
        sys.exit("no machine settings: copy config/env.example.sh to config/env.sh (or set HEXASHELL_ENV)")
    os.execvp("bash", ["bash", "-c", 'source "$1" || exit 2; export HEXASHELL_POOL_ENV=1; '
                       ': "${HEXASHELL_PYTHON:?set HEXASHELL_PYTHON in $1}"; exec "$HEXASHELL_PYTHON" "${@:2}"',
                       "bench_pool", env_file, str(Path(__file__).resolve()), *sys.argv[1:]])

sys.path.insert(0, str(HERE))
from hexashell.benches.session import server_env  # noqa: E402

RECYCLE = int(os.environ.get("RECYCLE", "20"))
ATTEMPTS = int(os.environ.get("ATTEMPTS", "3"))
SPACING = float(os.environ.get("SPACING", "20"))
NEED = int(os.environ.get("NEED", "3000"))
IDLE = float(os.environ.get("IDLE", "1800"))
SETS = os.environ.get("SETS", "").split()
GATEWAY_HOLD = float(os.environ.get("GATEWAY_HOLD", "300"))
GPU_CHECK = float(os.environ.get("GPU_CHECK", "1800"))
#: how often one episode is listed again after its gateway ended it, before that counts as an attempt
GATEWAY_RELISTS = 5
#: how long a server may take to start listening (s)
START_LIMIT = 3600.0
#: how long a GPU's check may take (s); one that takes longer finds the GPU unfit
CHECK_LIMIT = 180.0
#: the benchmarks whose environment can check a GPU (MuJoCo with PyOpenGL), in the order tried
CHECKERS = ("libero", "robocasa")
#: outcomes after which the slot's server is started again (a retry gets a fresh one)
SUSPECT = ("simulator failed", "error", "no record", "no result.json", "unreadable result.json")


class Pool:
    def __init__(self, name: str, gpus: list[str], max_running: int, sam3: dict[str, str] | None = None):
        self.name = name
        self.max_running = max_running
        self.sam3 = sam3 or {}
        self.todo = HERE / "logs" / f"{name}.todo"
        self.events = HERE / "logs" / f"{name}.events"
        self.logdir = HERE / "logs" / name
        self.results = HERE / "results" / name
        self.workdir = HERE / "logs" / f"{name}.pool"
        for d in (self.logdir, self.results, self.workdir):
            d.mkdir(parents=True, exist_ok=True)
        self.todo.touch()
        self.events.touch()
        self.sockets = Path(tempfile.mkdtemp(prefix=f"hexa-{name}-"))  # unix socket paths must be short
        self.slots = [{"n": i + 1, "gpu": g, "daemon": None, "bench": None, "ready": False, "served": 0,
                       "gen": 0, "since": 0.0, "last": None, "ep": None, "pending": None, "fails": 0,
                       "not_before": 0.0} for i, g in enumerate(gpus)]
        self.last_start = 0.0
        self.hold_until = 0.0  # no launches before this (after the gateway ended an episode)
        self.gateway_relists: dict[tuple[str, str, str], int] = {}
        self.gpu_checks: dict[str, tuple[bool, float]] = {}  # GPU -> (fit to render, when checked)
        self.unchecked_noted = False
        self.stopping = False

    # -- the list and the events ---------------------------------------------------------------------
    def note(self, text: str) -> None:
        line = f"{time.strftime('%F %T')} {text}"
        print(line, flush=True)
        with open(self.events, "a") as f:
            f.write(line + "\n")

    def _locked(self):
        f = open(f"{self.todo}.lock", "w")
        fcntl.flock(f, fcntl.LOCK_EX)
        return f

    def take(self, prefer: list[tuple[str, ...]]) -> str | None:
        """Remove and return the first line that matches the most preferred (bench[, task]), else the first."""
        with self._locked():
            lines = self.todo.read_text().splitlines()
            words = [ln.split() for ln in lines]
            pick = None
            for want in prefer:
                pick = next((i for i, w in enumerate(words) if w and tuple(w[:len(want)]) == want), None)
                if pick is not None:
                    break
            if pick is None:
                pick = next((i for i, w in enumerate(words) if w), None)
            if pick is None:
                return None
            line = lines.pop(pick)
            new = Path(f"{self.todo}.new")
            new.write_text("".join(ln + "\n" for ln in lines))
            new.replace(self.todo)
            return line.strip()

    def add(self, line: str) -> None:
        with self._locked(), open(self.todo, "a") as f:
            f.write(line + "\n")

    def waiting(self) -> bool:
        return any(ln.strip() for ln in self.todo.read_text().splitlines())

    def cap(self) -> int:
        try:
            digits = "".join(c for c in (HERE / "logs" / f"{self.name}.max").read_text() if c.isdigit())
            return int(digits) if digits else self.max_running
        except OSError:
            return self.max_running

    def outcome(self, task: str, seed: str, since: float) -> str:
        """"ok", or why the episode should be played again: its newest record since the launch."""
        recs = sorted(p for p in self.results.glob(f"{task}_s{seed}-*") if p.is_dir() and p.stat().st_mtime >= since)
        if not recs:
            return "no record"
        result = recs[-1] / "result.json"
        if not result.exists():
            return "no result.json"
        try:
            r = json.loads(result.read_text())
        except (OSError, ValueError):
            return "unreadable result.json"
        if (r.get("verdict") or {}).get("failure"):
            return "simulator failed"
        return r["end"] if r.get("end") in ("error", "gateway_error") else "ok"

    # -- servers ---------------------------------------------------------------------------------------
    def free_mib(self) -> dict[str, int]:
        try:
            out = subprocess.run(["nvidia-smi", "--query-gpu=index,memory.free", "--format=csv,noheader,nounits"],
                                 capture_output=True, text=True, timeout=120).stdout
        except (OSError, subprocess.SubprocessError):
            return {}
        free = {}
        for row in out.splitlines():
            parts = [p.strip() for p in row.split(",")]
            if len(parts) == 2 and parts[1].isdigit():
                free[parts[0]] = int(parts[1])
        return free

    def sock(self, slot) -> str:
        return str(self.sockets / f"slot{slot['n']}.sock")

    def unfit(self, gpu: str) -> bool:
        """Whether a check of the last GPU_CHECK seconds found GPU ``gpu`` unfit to render."""
        seen = self.gpu_checks.get(gpu)
        return seen is not None and not seen[0] and time.time() - seen[1] < GPU_CHECK

    def gpu_fit(self, gpu: str) -> bool:
        """Whether GPU ``gpu`` draws, as a simulator server about to start there needs: the check of the
        last GPU_CHECK seconds, else a new one (hexashell.benches.gpucheck)."""
        if GPU_CHECK <= 0:
            return True
        seen = self.gpu_checks.get(gpu)
        if seen is not None and time.time() - seen[1] < GPU_CHECK:
            return seen[0]
        found = checker()
        if found is None:
            if not self.unchecked_noted:
                self.unchecked_noted = True
                self.note("no GPU checks: " + " and ".join(f"$HEXASHELL_{b.upper()}_PYTHON" for b in CHECKERS)
                          + " are not set")
            return True
        python, env_file = found
        rec = probe(python, server_env(env_file, gpu), gpu)
        ok = rec.get("ok") is True
        self.gpu_checks[gpu] = (ok, time.time())
        if ok:
            self.note(f"gpu={gpu} draws: first frame {rec.get('first_s')}s, then {rec.get('frames_s')}")
        else:
            self.note(f"gpu={gpu} is not fit to render, no servers there for {GPU_CHECK:.0f}s: {rec.get('why')}")
        return ok

    def start_server(self, slot, bench: str) -> None:
        key = bench.upper()
        python = os.environ.get(f"HEXASHELL_{key}_PYTHON")
        if not python:
            raise RuntimeError(f"$HEXASHELL_{key}_PYTHON is not set")
        env = server_env(os.environ.get(f"HEXASHELL_{key}_ENV") or None, slot["gpu"])
        slot["gen"] += 1
        log = self.logdir / f"server-{slot['n']}-{slot['gen']}.log"
        cwd = self.workdir / f"slot{slot['n']}"
        cwd.mkdir(exist_ok=True)
        sock = self.sock(slot)
        if os.path.exists(sock):
            os.unlink(sock)
        with open(log, "ab") as out:
            slot["daemon"] = subprocess.Popen(
                [python, "-m", "hexashell.benches.daemon", f"hexashell.benches.{bench}.server", sock,
                 "--idle-s", str(IDLE)], env=env, cwd=str(cwd), stdin=subprocess.DEVNULL, stdout=out,
                stderr=subprocess.STDOUT, start_new_session=True)
        slot.update(bench=bench, ready=False, served=0, since=time.time())
        self.last_start = time.time()
        self.note(f"server slot{slot['n']} gpu={slot['gpu']} {bench} pid={slot['daemon'].pid} log={log.name}")

    def stop_server(self, slot, why: str) -> None:
        proc = slot["daemon"]
        if proc is not None:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=30)
            self.note(f"server slot{slot['n']} pid={proc.pid} stopped: {why}")
        slot.update(daemon=None, bench=None, ready=False, served=0)

    def check_server(self, slot) -> None:
        proc = slot["daemon"]
        if proc is None:
            return
        code = proc.poll()
        if code is not None:
            self.note(f"server slot{slot['n']} pid={proc.pid} exited with {code}"
                      + ("" if slot["ready"] else " before it was ready"))
            if not slot["ready"]:
                slot["fails"] += 1
                slot["not_before"] = time.time() + min(1800, 120 * 2 ** slot["fails"])
                self.gpu_checks.pop(slot["gpu"], None)
            slot.update(daemon=None, bench=None, ready=False, served=0)
            return
        if not slot["ready"]:
            if os.path.exists(self.sock(slot)):
                slot["ready"] = True
                slot["fails"] = 0
                self.note(f"server slot{slot['n']} ready after {time.time() - slot['since']:.0f}s")
            elif time.time() - slot["since"] > START_LIMIT:
                slot["fails"] += 1
                slot["not_before"] = time.time() + 600
                self.gpu_checks.pop(slot["gpu"], None)
                self.stop_server(slot, f"not ready after {START_LIMIT:.0f}s")

    # -- episodes --------------------------------------------------------------------------------------
    def launch(self, slot, line: str) -> None:
        bench, task, seed, attempt = parse(line)
        tag = f"{bench}_{task}_s{seed}" + (f"_a{attempt}" if attempt > 1 else "")
        cmd = episode_cmd(os.environ["HEXASHELL_PYTHON"], bench, task, seed, str(self.results), slot["gpu"],
                          self.sock(slot), SETS, self.sam3)
        with open(self.logdir / f"{tag}.log", "ab") as out:
            proc = subprocess.Popen(cmd, cwd=str(HERE), stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        slot["ep"] = {"proc": proc, "line": f"{bench} {task} {seed} {attempt}", "tag": tag, "since": time.time()}
        slot["served"] += 1
        slot["last"] = (bench, task)
        self.last_start = time.time()
        self.note(f"launch {tag} pid={proc.pid} gpu={slot['gpu']} slot={slot['n']} running={self.running()}")

    def reap(self, slot) -> None:
        ep = slot["ep"]
        code = ep["proc"].poll()
        if code is None:
            return
        bench, task, seed, attempt = parse(ep["line"])
        why = self.outcome(task, seed, ep["since"])
        self.note(f"exit {ep['tag']} gpu={slot['gpu']} code={code} after {time.time() - ep['since']:.0f}s: {why}")
        key = (bench, task, seed)
        line, by_gateway = relist(why, bench, task, seed, attempt, self.gateway_relists.get(key, 0))
        if why == "gateway_error":
            self.hold_until = time.time() + GATEWAY_HOLD
        if by_gateway:
            self.gateway_relists[key] = self.gateway_relists.get(key, 0) + 1
            self.add(line)
            self.note(f"listed {ep['tag']} again (the gateway ended it, {self.gateway_relists[key]} of "
                      f"{GATEWAY_RELISTS}); no launches for {GATEWAY_HOLD:.0f}s")
        elif line:
            self.add(line)
            self.note(f"listed {ep['tag']} again")
        elif why != "ok":
            self.note(f"given up on {bench} {task} s{seed} after {attempt} attempts")
        slot["ep"] = None
        if why in SUSPECT:
            self.gpu_checks.pop(slot["gpu"], None)  # the next server there waits for a new check
            self.stop_server(slot, f"after {why}")
        elif slot["served"] >= RECYCLE:
            self.stop_server(slot, f"{slot['served']} episodes")

    def running(self) -> int:
        return sum(1 for s in self.slots if s["ep"] is not None or s["pending"] is not None)

    def step(self) -> None:
        for slot in self.slots:
            if slot["ep"] is not None:
                self.reap(slot)
            self.check_server(slot)
            if slot["pending"] is not None:
                if slot["ready"]:
                    line, slot["pending"] = slot["pending"], None
                    self.launch(slot, line)
                elif slot["daemon"] is None:  # the server did not come up
                    self.add(slot["pending"])
                    self.note(f"listed {slot['pending']!r} again (slot{slot['n']} has no server)")
                    slot["pending"] = None
            if slot["daemon"] is not None and slot["ep"] is None and slot["pending"] is None and self.unfit(slot["gpu"]):
                self.stop_server(slot, f"gpu={slot['gpu']} is not fit to render")
        if ((HERE / "logs" / f"{self.name}.pause").exists() or time.time() - self.last_start < SPACING
                or time.time() < self.hold_until):
            return
        free = [s for s in self.slots if s["ep"] is None and s["pending"] is None and time.time() >= s["not_before"]
                and not self.unfit(s["gpu"])]
        if not free or self.running() >= self.cap() or not self.waiting():
            return
        # a slot with a server ready first, so that a launch need not start one
        free.sort(key=lambda s: (not s["ready"], s["n"]))
        slot = free[0]
        prefer = []
        if slot["last"]:
            prefer.append(slot["last"])
        if slot["bench"]:
            prefer.append((slot["bench"],))
        line = self.take(prefer)
        if not line:
            return
        try:
            bench, task, seed, attempt = parse(line)
        except ValueError:
            self.note(f"skipped a bad line: {line}")
            return
        if slot["ready"] and slot["bench"] == bench:
            self.launch(slot, f"{bench} {task} {seed} {attempt}")
            return
        if slot["daemon"] is not None:
            self.stop_server(slot, f"the slot moves to {bench}")
        need = NEED
        have = self.free_mib().get(slot["gpu"])
        if have is not None and have < need:
            self.add(line)
            slot["not_before"] = time.time() + 120
            self.note(f"slot{slot['n']} gpu={slot['gpu']}: {have} MiB free, a server needs {need}; waiting")
            return
        if not self.gpu_fit(slot["gpu"]):
            self.add(line)
            return
        try:
            self.start_server(slot, bench)
        except Exception as exc:  # noqa: BLE001 - noted, the line goes back
            self.add(line)
            self.note(f"slot{slot['n']}: no server ({exc})")
            slot["not_before"] = time.time() + 600
            return
        slot["pending"] = f"{bench} {task} {seed} {attempt}"

    def stop(self) -> None:
        self.note(f"stopping {self.name}: its episodes are stopped and listed again")
        for slot in self.slots:
            ep = slot["ep"]
            if ep is not None and ep["proc"].poll() is None:
                with_group(ep["proc"], signal.SIGTERM)
                try:
                    ep["proc"].wait(timeout=60)
                except subprocess.TimeoutExpired:
                    with_group(ep["proc"], signal.SIGKILL)
                self.add(ep["line"])
                self.note(f"stopped {ep['tag']}; listed again")
            elif ep is not None:
                self.reap(slot)
            if slot["pending"] is not None:
                self.add(slot["pending"])
            self.stop_server(slot, "the runner stops")

    def run(self) -> None:
        sam3 = ",".join(f"{g}={u}" for g, u in sorted(self.sam3.items(), key=lambda kv: int(kv[0])))
        self.note(f"begin {self.name} pool slots={','.join(s['gpu'] for s in self.slots)} max_running={self.max_running}"
                  f" host={os.uname().nodename} recycle={RECYCLE} gpu_check={GPU_CHECK:.0f}s sets=[{' '.join(SETS)}]"
                  + (f" sam3=[{sam3}]" if sam3 else ""))
        try:
            while not self.stopping:
                self.step()
                if not self.waiting() and self.running() == 0:
                    break
                time.sleep(5)
        finally:
            if self.stopping:
                self.stop()
            else:
                for slot in self.slots:
                    self.stop_server(slot, "the list is done")
            try:
                os.rmdir(self.sockets)
            except OSError:
                pass
        self.note(f"end {self.name}")


def episode_cmd(python: str, bench: str, task: str, seed: str, out: str, gpu: str, sock: str,
                sets: list[str], sam3: dict[str, str]) -> list[str]:
    """The command that plays one episode on a slot; the GPU's own SAM3 service, if any, goes last
    so that it wins over a sam3_url in SETS."""
    cmd = [python, "-m", "hexashell.benches.run", "--bench", bench, "--task", task, "--seed", seed,
           "--out", out, "--egl", gpu, "--sim-socket", sock]
    for s in sets:
        cmd += ["--set", s]
    if gpu in sam3:
        cmd += ["--set", f"sam3_url={sam3[gpu]}"]
    return cmd


def checker() -> tuple[str, str | None] | None:
    """The python and the settings file of the first of CHECKERS this machine has (None: none)."""
    for bench in CHECKERS:
        python = os.environ.get(f"HEXASHELL_{bench.upper()}_PYTHON")
        if python:
            return python, os.environ.get(f"HEXASHELL_{bench.upper()}_ENV") or None
    return None


def probe(python: str, env: dict[str, str], gpu: str) -> dict:
    """hexashell.benches.gpucheck's record of GPU ``gpu``; a check without an answer finds it unfit."""
    try:
        out = subprocess.run([python, "-m", "hexashell.benches.gpucheck", gpu], env=env, cwd=str(HERE),
                             stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=CHECK_LIMIT).stdout
        rec = json.loads(out.strip().splitlines()[-1])
    except (OSError, subprocess.SubprocessError, ValueError, IndexError) as exc:
        return {"gpu": gpu, "ok": False, "why": f"the check gave no answer ({type(exc).__name__})"}
    return rec if isinstance(rec, dict) else {"gpu": gpu, "ok": False, "why": f"the check answered {rec!r}"}


def relist(why: str, bench: str, task: str, seed: str, attempt: int, gateway_relists: int) -> tuple[str | None, bool]:
    """After an episode ended with ``why``: the line to list again (None: nothing), and whether that is
    because its gateway ended it (the same attempt again, GATEWAY_RELISTS times); any other failure
    uses up one of the ATTEMPTS."""
    if why == "ok":
        return None, False
    if why == "gateway_error" and gateway_relists < GATEWAY_RELISTS:
        return f"{bench} {task} {seed} {attempt}", True
    if attempt < ATTEMPTS:
        return f"{bench} {task} {seed} {attempt + 1}", False
    return None, False


def gpu_urls(text: str) -> dict[str, str]:
    """SAM3_URLS: '0=http://127.0.0.1:18320,5=http://127.0.0.1:18325' -> {'0': ..., '5': ...}."""
    urls = {}
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        gpu, sep, url = item.partition("=")
        if not sep or not gpu.strip().isdigit() or not url.strip():
            raise ValueError(f"SAM3_URLS: GPU=URL items separated by commas, not {item!r}")
        urls[gpu.strip()] = url.strip()
    return urls


def parse(line: str) -> tuple[str, str, str, int]:
    words = line.split()
    if len(words) not in (3, 4) or not words[2].isdigit() or (len(words) == 4 and not words[3].isdigit()):
        raise ValueError(line)
    return words[0], words[1], words[2], int(words[3]) if len(words) == 4 else 1


def with_group(proc: subprocess.Popen, sig: int) -> None:
    try:
        os.killpg(proc.pid, sig)
    except OSError:
        pass


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__.split("\n\n")[1], file=sys.stderr)
        return 2
    name, gpus = sys.argv[1], [g.strip() for g in sys.argv[2].split(",") if g.strip()]
    if not all(g.isdigit() for g in gpus) or not gpus:
        sys.exit(f"SLOTS: GPU indexes separated by commas, not {sys.argv[2]!r}")
    max_running = int(sys.argv[3]) if len(sys.argv) > 3 else len(gpus)
    try:
        sam3 = gpu_urls(os.environ.get("SAM3_URLS", ""))
    except ValueError as exc:
        sys.exit(str(exc))
    lock = open(HERE / "logs" / f"{name}.runner.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit(f"another runner of {name} is running here")
    pool = Pool(name, gpus, max_running, sam3)

    def on_signal(signum, frame):
        pool.stopping = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    pool.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
