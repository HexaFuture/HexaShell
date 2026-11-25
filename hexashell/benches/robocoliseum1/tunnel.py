"""The online agent of the RoboColiseum challenge: one process that dials the platform's gateway and plays the
cases it is given, as :mod:`.run` plays a local run.

``python -m hexashell.benches.robocoliseum1.tunnel --board spatial --slot 0``

The platform's simulator sends the same requests as a local one (``corobotpolicy.py``), but through the
gateway: we dial it (WebSocket, ``<TUNNEL_ENDPOINT>?job=<JOB_UUID>&agent=<agent id>``, ``Authorization: Bearer
<JOB_TOKEN>``), and it relays the simulator's messages to us inside binary frames that start with the case's
session id (4 bytes big-endian length, the id, the payload); a reply goes back with the same id.  Text frames
are control: ``warmup`` (we answer ``ready``), ``queued``, ``session_open`` / ``session_close`` (a case begins
or ends: the local run's connection), ``drain`` (no more cases: exit 0, never dial again).  Unknown types are
logged and ignored.  A binary frame with an empty payload (the warmup round trip) is answered at once with an
empty result.

Each case gets its own :class:`TunnelServer`: :class:`.link.Server`'s handling of the requests (episodes,
boundaries, the hold row) without its WebSocket server and without a results file (the platform keeps the
verdicts: an episode's verdict here is "online", without a score).  Its episodes are played by
:class:`OnlineSession`, which is :class:`.session.ColiseumSession` with the board given (``--board``, the job's
``config.board``) and the task taken from the request's ``task_name`` (the benchmark's sub-task): the same task
card, notes and prompt as a local run of that task.  A robust case's perturbation is not known online, nor
needed (its notes are the base task's).

Liveness: the gateway pings every 20 s and drops a tunnel after 45 s without bytes from us, so we also send an
unsolicited pong every 15 s (no pings of our own).  The simulator waits at most 120 s per reply, the download
of its request included, so a request still unanswered after ``--hold-s`` (default 60 s) gets the hold row,
or sooner when the request itself was long on the way (``link.hold_after``).
An unclean close is redialled at once with the same agent id (kept in ``~/.hexashell/robocoliseum1/agents/``,
so a restarted process reattaches too): within 30 s the gateway restores the open case; a reply finished while
we were away is sent on the new connection.

Exit codes as the official tunnel.sh: 0 drain; 1 setup failure or no connection for 180 s; 2 the gateway
refused the dial (401/403/404: bad token, or the job is not active); 3 closed right after the upgrade three
times in a row (more agents than the job's parallelism).

Credentials come from the state file the official login and submit steps write (``~/.simubotix-challenge.env``:
``JOB_UUID``, ``JOB_TOKEN`` or else ``CHALLENGE_TOKEN``, ``TUNNEL_ENDPOINT``); none of them is printed or written
to a record (the job is named by a hash of its id).  Proxies are never used (``proxy=None``).
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import os
import shlex
import signal
import struct
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

import msgpack

from . import protocol as P
from .challenge import tls_context
from .link import Server
from .session import ColiseumSession

ROOT = Path(__file__).resolve().parents[3]
STATE = Path.home() / ".simubotix-challenge.env"
DEFAULT_ENDPOINT = "ws://120.92.88.78/api/challenge/tunnel"
AGENTS = Path.home() / ".hexashell" / "robocoliseum1" / "agents"
#: seconds a request waits for the model before the hold row (the simulator's limit is 120 s, download included)
HOLD_S = 60.0
PONG_S = 15.0
#: give up after this long without a working connection
OFFLINE_CAP_S = 180.0
#: the agent's limits online, unless --set gives them: 240 minutes an episode and an hour a cell (requests
#: take seconds to arrive over the platform's link)
ONLINE_LIMITS = {"budget_s": 14400.0, "cell_total_s": 3600.0}
#: a close this soon after the upgrade, before any control frame, is the parallelism cap
INSTANT_CLOSE_S = 3.0
CAPACITY_TRIES = 3
#: how long a finished reply waits for a connection to send it on (the gateway's reconnect window is 30 s)
REPLY_WAIT_S = 30.0
EMPTY_REPLY = msgpack.packb({"result": {}}, use_bin_type=True)


# -- credentials and ids -----------------------------------------------------------------------------
def read_state(path: Path) -> dict[str, str]:
    """KEY=VALUE lines (bash ``printf %q`` quoting) of the official state file."""
    path = Path(path).expanduser()
    if not path.exists():
        raise SystemExit(f"no state file {path}: log in and submit a job first")
    mode = path.stat().st_mode & 0o077
    if mode:
        raise SystemExit(f"{path} is readable by others (mode {oct(path.stat().st_mode & 0o777)}): chmod 600 it")
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.removeprefix("export ").partition("=")
        try:
            parts = shlex.split(value)
        except ValueError:
            parts = [value]
        out[key.strip()] = parts[0] if parts else ""
    return out


def job_tag(job_uuid: str) -> str:
    """What names the job in records: a hash of its id, not the id."""
    return hashlib.sha256(job_uuid.encode()).hexdigest()[:10]


def agent_id(job_uuid: str, slot: int) -> str:
    """This slot's agent id for the job, made once and kept (a restarted process reattaches with it)."""
    AGENTS.mkdir(parents=True, exist_ok=True)
    os.chmod(AGENTS, 0o700)
    path = AGENTS / f"{job_tag(job_uuid)}_{int(slot)}.id"
    if path.exists():
        value = path.read_text().strip()
        if value:
            return value
    value = str(uuid.uuid4())
    tmp = path.with_suffix(".new")
    tmp.write_text(value + "\n")
    os.chmod(tmp, 0o600)
    tmp.replace(path)
    return value


def dial_url(endpoint: str, job_uuid: str, agent: str) -> str:
    parts = urlsplit(endpoint)
    query = parse_qsl(parts.query) + [("job", job_uuid), ("agent", agent)]
    return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), ""))


# -- frames ------------------------------------------------------------------------------------------
def split_frame(data: bytes) -> tuple[str, bytes]:
    if len(data) < 4:
        raise ValueError(f"a data frame of {len(data)} bytes")
    (n,) = struct.unpack(">I", data[:4])
    if 4 + n > len(data):
        raise ValueError(f"session id length {n} overruns a frame of {len(data)} bytes")
    return data[4:4 + n].decode("utf-8"), data[4 + n:]


def join_frame(session_id: str, payload: bytes) -> bytes:
    sid = session_id.encode("utf-8")
    return struct.pack(">I", len(sid)) + sid + payload


def _field(obj: dict, name: str):
    return obj[name] if name in obj else obj.get(name.encode())


def _nd(obj: dict):
    """A msgpack_numpy array or scalar (``{b"nd": True/False, b"type", b"kind", b"shape", b"data"}``) as
    nested lists / a number.  Structured and object arrays are refused (msgpack_numpy unpickles the latter)."""
    import numpy as np

    kind = _field(obj, "kind") or b""
    if kind in (b"V", "V", b"O", "O"):
        raise ValueError(f"msgpack_numpy array of kind {kind!r} refused")
    descr = _field(obj, "type")
    dtype = np.dtype(descr.decode() if isinstance(descr, (bytes, bytearray)) else descr)
    data = _field(obj, "data")
    if _field(obj, "nd") is True:
        return np.frombuffer(data, dtype=dtype).reshape(_field(obj, "shape")).tolist()
    return np.frombuffer(data, dtype=dtype)[0].item()


def _is_nd(obj) -> bool:
    return isinstance(obj, dict) and _field(obj, "nd") in (True, False) and _field(obj, "data") is not None \
        and _field(obj, "type") is not None


#: request fields nothing of ours may read: the online simulator (before v3.5.0) still sends the live
#: progress of the task's checkers (``task_progress``, scoring data that v3.5.0 keeps out of the requests);
#: it is dropped here, before the request reaches anything else
DROPPED = ("task_progress",)


def plain_request(payload: bytes) -> tuple[bytes, bool]:
    """The request as plain msgpack (what :class:`.link.Server` reads): a request packed with msgpack_numpy
    (arrays as ``{b"nd": ...}`` maps) is unpacked, its arrays made lists and packed again; the DROPPED fields
    of its params are left out.  Returns the bytes and whether they changed."""
    seen = [False]

    def hook(obj):
        if _is_nd(obj):
            seen[0] = True
            return _nd(obj)
        return obj

    try:
        data = msgpack.unpackb(payload, raw=False, object_hook=hook, strict_map_key=False)
    except Exception:  # noqa: BLE001 - Server answers what it cannot read, and logs it
        return payload, False
    params = data.get("params") if isinstance(data, dict) else None
    if isinstance(params, dict):
        for key in DROPPED:
            if key in params:
                del params[key]
                seen[0] = True
    if not seen[0]:
        return payload, False
    return msgpack.packb(data, use_bin_type=True), True


def _small(value, depth: int = 0):
    """A copy for a log: bytes and long lists become their size."""
    if isinstance(value, (bytes, bytearray)):
        return f"<{len(value)} bytes>"
    if isinstance(value, dict):
        return {str(k): _small(v, depth + 1) for k, v in value.items()} if depth < 4 else "<dict>"
    if isinstance(value, (list, tuple)):
        return [_small(v, depth + 1) for v in value] if len(value) <= 32 else f"<list of {len(value)}>"
    return value


#: requests per case whose pictures' fields are logged (``calibration_probe``)
PROBES = 1


def calibration_probe(plain: bytes) -> dict | None:
    """What a request carries per picture (encoding, size, the calibration's fields and small values) and how
    long each of its readings is (no values), for the log: the online simulator sends no calibration (the nominal
    cameras of :mod:`.cameras` stand in); a simulator that starts sending one shows here.  Read after DROPPED is gone."""
    try:
        data = msgpack.unpackb(plain, raw=False, strict_map_key=False)
    except Exception:  # noqa: BLE001
        return None
    params = data.get("params") if isinstance(data, dict) else None
    images = params.get("images") if isinstance(params, dict) else None
    if not isinstance(images, dict):
        return None
    out = {}
    for name, entry in images.items():
        if not isinstance(entry, dict):
            out[str(name)] = type(entry).__name__
            continue
        out[str(name)] = {k: _small(v) for k, v in entry.items() if k != "image_data"}
    states = params.get("states") if isinstance(params.get("states"), dict) else {}
    lengths = {str(k): (len(v) if isinstance(v, (list, tuple, dict)) else type(v).__name__) for k, v in states.items()}
    return {"images": out, "params_keys": sorted(str(k) for k in params), "states_lengths": lengths}


# -- the task of a case -----------------------------------------------------------------------------
def online_facts(board: str, task_name: str) -> dict:
    """:func:`.protocol.parse_config`'s facts for a case of ``board`` whose requests name ``task_name`` (the
    benchmark's sub-task): the config itself, except on the robust board, where the perturbation is unknown
    (``kind`` None, config ``g2op_robust_<task>``).  A sub-task we do not know keeps its name as the task
    (``known`` False: no task note)."""
    table = P.INSTRUCTION if board == "robust" else P.TASKS[board]
    found = [t for t, (_, sub, _) in table.items() if sub == task_name] or \
        ([task_name] if task_name in table else [])
    if not found:
        return {"config": f"online_{board}_{task_name or 'unknown'}", "board": board, "task": task_name or "unknown",
                "kind": None, "scene": None, "sub_task": task_name, "layouts": None, "known": False}
    task = found[0]
    if board == "robust":
        scene, sub, layouts = P.INSTRUCTION[task]
        return {"config": f"g2op_robust_{task}", "board": board, "task": task, "kind": None, "scene": scene,
                "sub_task": sub, "layouts": layouts, "known": True}
    return {**P.parse_config(P.config_name(board, task)), "known": True}


class OnlineSession(ColiseumSession):
    """:class:`.session.ColiseumSession` for a case of the online platform (``info``: board, case, episode):
    the same card, notes and robot calls; no layout, no seed; the record is named after the case."""

    def __init__(self, link, info: dict[str, Any]):
        facts = online_facts(info["board"], link.task_name)
        if facts["board"] == "robust" and facts["known"]:
            stand_in = P.config_name("robust", facts["task"], P.ROBUST_KINDS[0])
        elif facts["known"]:
            stand_in = facts["config"]
        else:
            stand_in = P.configs((info["board"],))[0]
        super().__init__(link, {**info, "config": stand_in})
        self.facts = facts
        self.config = facts["config"]
        self.task = facts["task"]
        self.info.update({"config": facts["config"], "task_name": facts["task"], "board": facts["board"]})
        self.case = info.get("case")
        self.args_slot = info.get("slot")

    def record_name(self) -> str | None:
        return f"{self.config}_online_s{self.args_slot}_c{self.case}_e{self.k}"

    def labels(self) -> dict[str, Any]:
        return {**super().labels(), "case": self.case}

    def open(self, settings, rec, attach) -> None:
        super().open(settings, rec, attach)
        if self.host is not None:
            self.host.task_name = self.task
            # depth only when the perception needs it: a request with depth takes seconds more to arrive
            self.host.driver.motion_depth = False
        self.link.hold_depth = False
        if not self.facts["known"]:
            self.errors.append(f"unknown sub-task {self.facts['sub_task']!r} on the {self.facts['board']} board: "
                               "no task note")
        if not self.link.first.depth:
            self.errors.append("the first request of the episode has no depth pictures")
            rec.event({"kind": "online_no_depth", "cams": sorted(self.link.first.images)})

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        return {**super().result_fields(verdict), "online": True, "case": self.case,
                "sub_task": self.facts["sub_task"]}


# -- one case ---------------------------------------------------------------------------------------
class TunnelServer(Server):
    """:class:`.link.Server` for one case of the tunnel: the tunnel hands it the case's requests; the case's
    end is the local run's connection closing; no results file, so every verdict is "online"."""

    def __init__(self, *, on_episode, hold_s: float, log_path: Path):
        super().__init__("127.0.0.1", 0, on_episode=on_episode, results_dir=None, hold_s=hold_s, log_path=log_path)

    def start(self) -> None:
        self.connections = 1
        self.connected_once.set()
        self.disconnected.clear()
        self.log({"kind": "case_open"})

    def close_case(self, why: str) -> None:
        with self._lock:
            ep = self.current
            if ep is not None and not ep.over:
                self._end(ep, None, why)
        self.connections = 0
        self.disconnected.set()
        self.log({"kind": "case_close", "why": why, "episodes": len(self.episodes)})

    def stop(self) -> None:
        self._stopping = True
        self._pool.shutdown(wait=False)
        if self._log is not None:
            with self._log_lock:
                self._log.close()
                self._log = None

    def _poll_verdict(self, ep) -> None:
        v = {"success": None, "progress": None, "online": True}
        ep.set_verdict(v)
        self.log({"kind": "verdict", "episode": ep.index, "result_index": ep.result_index, **v})


class Agent:
    def __init__(self, args, state: dict[str, str]):
        self.args = args
        self.board = args.board
        self.job_uuid = state["JOB_UUID"]
        self.token = state.get("JOB_TOKEN") or state.get("CHALLENGE_TOKEN") or ""
        self.token_kind = "JOB_TOKEN" if state.get("JOB_TOKEN") else "CHALLENGE_TOKEN"
        self.endpoint = args.endpoint or state.get("TUNNEL_ENDPOINT") or DEFAULT_ENDPOINT
        self.agent = agent_id(self.job_uuid, args.slot)
        self.tag = job_tag(self.job_uuid)
        self.out = Path(args.out).resolve()
        self.dir = self.out / "agents" / f"{self.tag}_slot{args.slot}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self._log = open(self.dir / "tunnel.jsonl", "a", encoding="utf-8")
        self._log_lock = threading.Lock()
        from .run import settings_for

        self.settings = settings_for(self.board, args.settings, args.set)
        given = {item.partition("=")[0] for item in args.set}
        for name, value in ONLINE_LIMITS.items():
            if name not in given:
                setattr(self.settings, name, value)
        self.cases: dict[str, TunnelServer] = {}
        self.case_n = 0
        self.played: list[dict] = []
        self.threads: list[threading.Thread] = []
        self.ws = None
        self.connected: asyncio.Event | None = None
        self.drained = False
        self.exit_code = 0
        self.stats = {"dials": 0, "connections": 0, "requests": 0, "replies": 0, "empty": 0, "repacked": 0,
                      "dropped_replies": 0, "controls": {}}

    # -- records -----------------------------------------------------------------------------------
    def log(self, record: dict) -> None:
        line = json.dumps({"t": round(time.time(), 3), **record}, ensure_ascii=False, default=str)
        with self._log_lock:
            if self._log is not None:
                self._log.write(line + "\n")
                self._log.flush()

    def say(self, text: str) -> None:
        print(time.strftime("%H:%M:%S"), f"[slot {self.args.slot}]", text, flush=True)

    # -- cases -------------------------------------------------------------------------------------
    def case(self, session_id: str, implicit: bool = False) -> TunnelServer:
        srv = self.cases.get(session_id)
        if srv is not None:
            return srv
        n = self.case_n
        self.case_n += 1
        stamp = time.strftime("%Y%m%d-%H%M%S")
        cdir = self.dir / "cases" / f"c{n:03d}_{stamp}"
        cdir.mkdir(parents=True, exist_ok=True)

        def play(link, n=n, cdir=cdir) -> None:
            self.play(link, n, cdir)

        srv = TunnelServer(on_episode=play, hold_s=self.args.hold_s, log_path=cdir / "link.jsonl")
        srv.case_index = n
        srv.case_dir = cdir
        srv.start()
        self.cases[session_id] = srv
        self.log({"kind": "case_open", "case": n, "implicit": implicit, "dir": str(cdir)})
        self.say(f"case {n} open{' (data before session_open)' if implicit else ''}")
        return srv

    def play(self, link, n: int, cdir: Path) -> None:
        from ...runtime.episode import Episode

        # eval_seed: the official one (it seeds the IK's random restarts, as in a local run of the test)
        info = {"board": self.board, "case": n, "episode": link.index, "eval_seed": P.OFFICIAL_SEED,
                "run_dir": str(cdir), "online": True, "job": self.tag, "slot": self.args.slot}
        self.log({"kind": "episode_begin", "case": n, "episode": link.index, "task_name": link.task_name,
                  "prompt": link.prompt, "depth": sorted(link.first.depth)})
        if not link.first.depth:
            self.say(f"WARNING case {n}: the first request has no depth pictures (perception needs them)")
        episode = Episode(OnlineSession(link, info), self.settings, self.out)
        entry = {"case": n, "episode": link.index, "task_name": link.task_name}
        try:
            episode.run()
        finally:
            entry["record"] = str(episode.rec.dir)
            result = episode.rec.dir / "result.json"
            if result.exists():
                data = json.loads(result.read_text())
                entry.update({k: data.get(k) for k in ("task", "instruction", "end", "turns", "errors")})
            self.played.append(entry)
            self.log({"kind": "episode_end", **entry})
            self.say(f"case {n} episode {link.index} ({entry.get('task')}) ended: {entry.get('end')}, "
                     f"{entry.get('turns')} turns")

    def close_case(self, session_id: str, why: str) -> None:
        srv = self.cases.pop(session_id, None)
        if srv is None:
            return
        srv.close_case(why)
        self.log({"kind": "case_close", "case": srv.case_index, "why": why, "episodes": len(srv.episodes)})
        self.say(f"case {srv.case_index} closed ({why}), {len(srv.episodes)} episode(s)")
        t = threading.Thread(target=self._finish_case, args=(srv,), name=f"finish{srv.case_index}", daemon=True)
        self.threads.append(t)
        t.start()

    def _finish_case(self, srv: TunnelServer) -> None:
        for t in list(srv.threads):
            t.join(timeout=900)
        srv.stop()

    # -- the wire ----------------------------------------------------------------------------------
    async def send(self, data: bytes | str) -> bool:
        """Send on the current connection; wait up to REPLY_WAIT_S for one if there is none."""
        deadline = time.monotonic() + REPLY_WAIT_S
        while True:
            ws = self.ws
            if ws is not None:
                try:
                    await ws.send(data)
                    return True
                except Exception as exc:  # noqa: BLE001 - the connection went; wait for the next
                    self.log({"kind": "send_failed", "error": f"{type(exc).__name__}: {exc}"[:200]})
            left = deadline - time.monotonic()
            if left <= 0 or self.drained:
                return False
            try:
                await asyncio.wait_for(self.connected.wait(), timeout=min(left, 1.0))
            except asyncio.TimeoutError:
                pass

    async def on_data(self, message: bytes) -> None:
        sid, payload = split_frame(message)
        if not payload:
            self.stats["empty"] += 1
            self.log({"kind": "empty_frame", "sid_len": len(sid)})
            await self.send(join_frame(sid, EMPTY_REPLY))
            return
        srv = self.cases.get(sid) or self.case(sid, implicit=True)
        self.stats["requests"] += 1
        t_in = time.time()
        plain, changed = plain_request(payload)
        if changed:
            self.stats["repacked"] += 1
        loop = asyncio.get_running_loop()
        if getattr(srv, "probes", 0) < PROBES:
            srv.probes = getattr(srv, "probes", 0) + 1
            probe = await loop.run_in_executor(None, calibration_probe, plain)
            self.log({"kind": "calibration_probe", "case": srv.case_index, "n": srv.probes, "probe": probe})
        reply = await loop.run_in_executor(srv._pool, srv._request, plain)
        ok = await self.send(join_frame(sid, reply))
        self.stats["replies" if ok else "dropped_replies"] += 1
        if not ok:
            self.log({"kind": "reply_dropped", "case": srv.case_index, "bytes": len(payload)})
        if self.stats["requests"] <= 3 or changed and self.stats["repacked"] <= 3:
            self.log({"kind": "request_timing", "case": srv.case_index, "bytes": len(payload), "repacked": changed,
                      "answer_s": round(time.time() - t_in, 3)})

    async def on_control(self, text: str, ws) -> None:
        try:
            ctl = json.loads(text)
        except ValueError:
            self.log({"kind": "bad_control", "text": text[:200]})
            return
        kind = ctl.get("type") if isinstance(ctl, dict) else None
        self.stats["controls"][str(kind)] = self.stats["controls"].get(str(kind), 0) + 1
        sid = ctl.get("session_id") if isinstance(ctl, dict) else None
        self.log({"kind": "control", "type": kind, "has_session": sid is not None, "reason": ctl.get("reason")})
        if kind == "warmup":
            problem = await asyncio.get_running_loop().run_in_executor(None, self.warm)
            if problem:
                self.say(f"warmup failed: {problem}; not ready")
                self.log({"kind": "warmup_failed", "problem": problem})
                self.exit_code = 1
                await ws.close()
                return
            await ws.send(json.dumps({"type": "ready"}))
            self.say("warmup -> ready")
        elif kind == "queued":
            self.say(f"queued ({ctl.get('reason')})")
        elif kind == "session_open" and sid:
            self.case(sid)
        elif kind == "session_close" and sid:
            self.close_case(sid, "session_close")
        elif kind == "drain":
            self.drained = True
            self.say("drain: no more cases")
            await ws.close()
        else:
            self.log({"kind": "control_ignored", "type": kind})

    def warm(self) -> str | None:
        """What would stop us playing (the SAM3 service, the model key), or None."""
        import urllib.request

        url = self.settings.sam3_url
        if url:
            try:
                opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                opener.open(url.rstrip("/") + "/health", timeout=10).read()
            except Exception as exc:  # noqa: BLE001
                return f"SAM3 at {url} does not answer ({type(exc).__name__})"
        try:
            self.settings.api_key()
        except Exception:  # noqa: BLE001 - never shown: it may hold the key's path only
            return "no model key"
        return None

    async def pong(self, ws) -> None:
        while True:
            await asyncio.sleep(PONG_S)
            try:
                await ws.pong()
            except Exception:  # noqa: BLE001 - the connection is going
                return

    async def run(self) -> int:
        import websockets
        from websockets.asyncio.client import connect
        from websockets.exceptions import InvalidStatus

        self.connected = asyncio.Event()
        url = dial_url(self.endpoint, self.job_uuid, self.agent)
        headers = {"Authorization": f"Bearer {self.token}"}
        last_ok = time.monotonic()
        instant = 0
        backoff = 0.5
        self.log({"kind": "start", "board": self.board, "slot": self.args.slot, "job": self.tag,
                  "credential": self.token_kind, "endpoint_host": urlsplit(self.endpoint).hostname,
                  "hold_s": self.args.hold_s, "model": self.settings.model,
                  "reasoning_effort": self.settings.reasoning_effort, "skills": self.settings.skills,
                  "depth": self.settings.depth, "sam3": self.settings.sam3_url,
                  "budget_s": self.settings.budget_s, "cell_total_s": self.settings.cell_total_s,
                  "max_turns": self.settings.max_turns})
        self.say(f"board {self.board}, job {self.tag}, dialing {urlsplit(self.endpoint).hostname} "
                 f"({self.token_kind}); notes {'on' if self.settings.skills else 'off'}")
        while not self.drained and self.exit_code == 0:
            self.stats["dials"] += 1
            opened = None
            got_control = False
            try:
                async with connect(url, additional_headers=headers, ping_interval=None, ping_timeout=None,
                                   max_size=None, open_timeout=20, close_timeout=5, proxy=None,
                                   **({"ssl": tls_context()} if url.startswith("wss:") else {})) as ws:
                    opened = time.monotonic()
                    self.stats["connections"] += 1
                    self.ws = ws
                    self.connected.set()
                    self.log({"kind": "connected", "dial": self.stats["dials"]})
                    if self.stats["connections"] > 1:
                        self.say(f"reconnected (connection {self.stats['connections']})")
                    keep = asyncio.create_task(self.pong(ws))
                    tasks: set[asyncio.Task] = set()
                    try:
                        async for message in ws:
                            last_ok = time.monotonic()
                            if isinstance(message, str):
                                got_control = True
                                await self.on_control(message, ws)
                            else:
                                task = asyncio.create_task(self.on_data(message))
                                tasks.add(task)
                                task.add_done_callback(self._done(tasks))
                    finally:
                        keep.cancel()
                        self.ws = None
                        self.connected.clear()
                    code = ws.close_code
                    self.log({"kind": "closed", "code": code, "reason": ws.close_reason, "drained": self.drained})
            except InvalidStatus as exc:
                status = exc.response.status_code
                self.log({"kind": "dial_refused", "status": status})
                if status in (401, 403, 404):
                    self.say(f"the gateway refused the dial (HTTP {status}): token or job not valid / not active")
                    return 2
            except (OSError, asyncio.TimeoutError, websockets.ConnectionClosed, websockets.InvalidHandshake) as exc:
                self.log({"kind": "dial_failed" if opened is None else "connection_lost",
                          "error": f"{type(exc).__name__}: {exc}"[:200]})
            if self.drained or self.exit_code:
                break
            now = time.monotonic()
            if opened is not None:
                if not got_control and now - opened < INSTANT_CLOSE_S:
                    instant += 1
                    self.say(f"closed right after the upgrade ({instant}/{CAPACITY_TRIES}): over the parallelism cap?")
                    if instant >= CAPACITY_TRIES:
                        return 3
                    await asyncio.sleep(5.0)
                    continue
                instant = 0
                backoff = 0.5
                last_ok = now
                self.say("connection lost; redialling")
            if now - last_ok > OFFLINE_CAP_S:
                self.say(f"no connection for {OFFLINE_CAP_S:.0f} s; giving up")
                return 1
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 4.0)
        return self.exit_code

    def _done(self, tasks):
        def cb(task: asyncio.Task) -> None:
            tasks.discard(task)
            if not task.cancelled() and task.exception() is not None:
                exc = task.exception()
                self.log({"kind": "data_failed", "error": f"{type(exc).__name__}: {exc}"[:300]})
        return cb

    def finish(self, code: int) -> None:
        for sid in list(self.cases):
            self.close_case(sid, "the agent stopped")
        for t in list(self.threads):
            t.join(timeout=900)
        summary = {"board": self.board, "slot": self.args.slot, "job": self.tag, "exit": code, "drained": self.drained,
                   "cases": self.case_n, "episodes": len(self.played), "stats": self.stats,
                   "played": sorted(self.played, key=lambda e: (e["case"], e["episode"]))}
        (self.dir / "agent.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n")
        self.log({"kind": "end", "exit": code, "stats": self.stats})
        with self._log_lock:
            self._log.close()
            self._log = None
        self.say(f"exit {code}: {self.case_n} case(s), {len(self.played)} episode(s)")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.tunnel", description=__doc__.splitlines()[0])
    ap.add_argument("--board", required=True, choices=P.BOARDS, help="the job's config.board")
    ap.add_argument("--slot", type=int, default=0, help="this process's slot (its own agent id), 0..parallelism-1")
    ap.add_argument("--state", default=str(STATE), help="the state file with JOB_UUID, JOB_TOKEN, TUNNEL_ENDPOINT")
    ap.add_argument("--endpoint", default=None, help="default: TUNNEL_ENDPOINT of the state file, else the fixed one")
    ap.add_argument("--out", default=str(ROOT / "results" / "benches" / "robocoliseum1_online"), help="records")
    ap.add_argument("--settings", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--hold-s", type=float, default=HOLD_S)
    args = ap.parse_args(argv)

    state = read_state(Path(args.state))
    if not state.get("JOB_UUID"):
        raise SystemExit("no JOB_UUID in the state file: submit the job first")
    if not (state.get("JOB_TOKEN") or state.get("CHALLENGE_TOKEN")):
        raise SystemExit("no JOB_TOKEN / CHALLENGE_TOKEN in the state file")
    if state.get("JOB_BOARD") and state["JOB_BOARD"] != args.board:
        raise SystemExit(f"--board {args.board}, but the state file's job is on the {state['JOB_BOARD']} board")
    agent = Agent(args, state)
    problem = agent.warm()
    if problem:
        raise SystemExit(f"not ready: {problem}")

    def on_term(signum, _frame):
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_term)
    code = 1
    try:
        code = asyncio.run(agent.run())
    except KeyboardInterrupt:
        code = 130
    finally:
        agent.finish(code)
    return code


if __name__ == "__main__":
    sys.exit(main())
