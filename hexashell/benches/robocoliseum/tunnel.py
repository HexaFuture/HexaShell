"""The online agent of RoboColiseum Season II: one process that dials the platform's gateway and plays the cases
it is given, as :mod:`.run` plays a local run.

``python -m hexashell.benches.robocoliseum.tunnel --slot 0`` (state file: ``~/.hexashell/robocoliseum/jobs/v2.env``,
written by ``challenge submit --yes``; :file:`online.sh` starts several slots).

The wire (the official challenge-run-agent skill of Season II, the same as Season I's): we dial the gateway
(WebSocket, ``<TUNNEL_ENDPOINT>?job=<JOB_UUID>&agent=<agent id>``, ``Authorization: Bearer <JOB_TOKEN>``), which
relays the platform simulator's requests to us inside binary frames that start with the case's session id (4
bytes big-endian length, the id, the payload); a reply goes back with the same id.  Text frames are control:
``warmup`` (we answer ``ready`` once the model key and SAM3 answer), ``queued``, ``session_open`` /
``session_close`` (a case begins or ends: the local run's connection), ``drain`` (no more cases: exit 0, never
dial again).  Unknown types are logged and ignored.  A data frame with an empty payload (the warmup round trip)
is answered at once with an empty result.

A job of Season II is on the board ``v2``: its cases come from all four boards (finemanip, crsemb, wbc,
dexmanip), so every case says what it is in its requests.  Each case gets its own :class:`TunnelServer`:
:class:`.link.Server`'s handling of the requests (episodes, boundaries, hold rows, the conclusion, the native
runtimes' replies) without its WebSocket server and without a results file (the platform keeps the scores: an
episode's verdict here is "online").  Before a request is read, :meth:`TunnelServer.prepare` takes the case's
facts from it (:func:`online_facts`): ``task_name`` (the benchmark's sub-task; the scene for the Rubik's cube)
and the robot give the config, so its board, runtime, task card and notes are those of a local run of that
config; the runtime sets how the requests are read (the native runtimes' camera models), the replies (no waist
or head there, MuJoCo's rows per reply, the conclusion) and the reply limit.  The episodes are played by
:class:`OnlineSession` (:class:`.session.ColiseumSession` named after the case).

Reply limit: the platform's simulator waits at most ``--reply-cap-s`` (120 s, the skill's figure, download
included) for a reply, a native runtime less where its own limit is shorter (MuJoCo 60 s,
:data:`.protocol.REPLY_WAIT_S`).  A request still unanswered then gets the hold row (one dispatch) by
:data:`.link.REPLY_MARGIN_S` before the limit, counting its time on the way (:func:`.link.hold_after`;
``--hold-s`` waits less).  Depth (Isaac only; the native runtimes send none): ``--depth when_needed`` (the
default, as Season I online: a request with depth pictures can take 60-90 s to arrive) asks for depth only
when the perception needs it (one hold row, a dispatch, then); ``every_motion`` asks after every motion, as a
local run does.

Liveness: the gateway pings every 20 s and drops a tunnel after 45 s without bytes from us, so we also send an
unsolicited pong every 15 s.  An unclean close is redialled at once with the same agent id (kept in
``~/.hexashell/robocoliseum/agents/``, so a restarted process reattaches too): within 30 s the gateway restores
the open case; a reply finished while we were away is sent on the new connection.

Model guard: once an episode ends on a gateway error (the runtime's retries, a few minutes, ran out), the
slot starts no model until one short request to it (:func:`model_probe`, a single attempt) answers, every
``MODEL_PROBE_S``; meanwhile the episode's requests get hold rows as while the model thinks (one dispatch each, about
one a reply limit; depth not asked), for at most ``MODEL_WAIT_CAP_S`` an episode.  An episode that had sent nothing
but hold rows when the error came (the robot has not moved) is played again from its start once the model answers,
at most ``MODEL_REPLAYS`` times; one that moved ends as before.  A slot whose model does not answer at its start
waits the same way before its first episode.  So a long outage holds a slot on its case instead of using up cases.
An episode's own end does not conclude an episode that has sent only hold rows (:meth:`OnlineSession.conclude`):
the guard decides first, and the episode thread concludes it when no replay follows.

Episodes that wait for the simulator's end (``WAIT_FOR_SIMULATOR_END_TASKS``; the platform's simulator can hang in
the setup of the episode after one our reply has ended, seen on insert_straw_into_milk_tea_cup): when the model is done, our replies hold the robot without ``episode_done`` until
the simulator ends the episode (:attr:`.link.EpisodeLink.linger`).

Exit codes as the official tunnel.sh: 0 drain; 1 setup failure or no connection for 180 s; 2 the gateway
refused the dial (401/403/404: bad token, or the job is not active); 3 closed right after the upgrade three
times in a row (more agents than the job's parallelism); 130 SIGTERM.

Credentials come from the state file (``JOB_UUID``, ``JOB_TOKEN`` or else ``CHALLENGE_TOKEN``,
``TUNNEL_ENDPOINT``); none of them is printed or written to a record (the job is named by a hash of its id, the
gateway not at all).  Proxies are never used (``proxy=None``).
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import hashlib
import json
import os
import re
import shlex
import signal
import struct
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

import msgpack

from . import obs as O
from . import protocol as P
from .challenge import JOB_BOARD, JOBS, SEASON, tls_context
from .link import NATIVE, SIM_REPLY_S, Server
from .session import ColiseumSession

ROOT = Path(__file__).resolve().parents[3]
#: the state file of a Season II job (``challenge submit --yes`` writes it)
STATE = JOBS / f"{JOB_BOARD}.env"
AGENTS = Path.home() / ".hexashell" / "robocoliseum" / "agents"
PONG_S = 15.0
#: give up after this long without a working connection
OFFLINE_CAP_S = 180.0
#: the agent's limits online, unless --set gives them: 240 minutes an episode and
#: an hour a cell (requests take seconds to arrive over the platform's link)
ONLINE_LIMITS = {"budget_s": 14400.0, "cell_total_s": 3600.0}
#: a close this soon after the upgrade, before any control frame, is the parallelism cap
INSTANT_CLOSE_S = 3.0
CAPACITY_TRIES = 3
#: how long a finished reply waits for a connection to send it on (the gateway's reconnect window is 30 s)
SEND_WAIT_S = 30.0
EMPTY_REPLY = msgpack.packb({"result": {}}, use_bin_type=True)
DEPTH_MODES = ("when_needed", "every_motion")
#: the model guard (see the module's docstring): a probe this often while the model is down, at most this long a wait
#: an episode, at most this many replays of an episode that sent only hold rows, a line on the console this often
MODEL_PROBE_S = 60.0
MODEL_WAIT_CAP_S = 10800.0
MODEL_REPLAYS = 2
MODEL_SAY_S = 600.0
PROBE_TIMEOUT_S = 120.0
#: tasks whose episodes our replies never end: held rows until the simulator ends them (link.EpisodeLink.linger)
WAIT_FOR_SIMULATOR_END_TASKS = frozenset({"insert_straw_into_milk_tea_cup"})


# -- credentials and ids -----------------------------------------------------------------------------
def read_state(path: Path) -> dict[str, str]:
    """KEY=VALUE lines (bash ``printf %q`` / shlex quoting) of a state file, which must be mode 600."""
    path = Path(path).expanduser()
    if not path.exists():
        raise SystemExit(f"no state file {path}: log in and submit a job first (challenge submit --yes)")
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


def season_problem(state: dict[str, str]) -> str | None:
    """Why the state file's job is not a Season II job, or None (it must say so: JOB_SEASON 2 or JOB_BOARD v2)."""
    season, board = state.get("JOB_SEASON") or "", state.get("JOB_BOARD") or ""
    if season and season != SEASON:
        return f"the state file's job is of season {season!r}, not {SEASON}"
    if board and board != JOB_BOARD:
        return f"the state file's job is on the board {board!r}, not {JOB_BOARD!r} (Season I's boards are robocoliseum1's)"
    if not season and not board:
        return "the state file does not say the job is of Season II (no JOB_SEASON / JOB_BOARD)"
    return None


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
    if dtype.kind in ("V", "O"):
        raise ValueError(f"msgpack_numpy array of dtype {dtype} refused")
    data = _field(obj, "data")
    if _field(obj, "nd") is True:
        return np.frombuffer(data, dtype=dtype).reshape(_field(obj, "shape")).tolist()
    return np.frombuffer(data, dtype=dtype)[0].item()


def _is_nd(obj) -> bool:
    return isinstance(obj, dict) and _field(obj, "nd") in (True, False) and _field(obj, "data") is not None \
        and _field(obj, "type") is not None


#: request fields nothing of ours may read: the checkers' live progress (``task_progress``: developer data;
#: the native runtimes send it empty, Season I's online simulator sent it filled).  It is dropped here, before
#: the request reaches anything else; only its presence and length are kept (they tell a runtime apart).
DROPPED = ("task_progress",)


def plain_request(payload: bytes) -> tuple[bytes, bool, dict | None, dict]:
    """The request as plain msgpack (what :class:`.link.Server` reads): a request packed with msgpack_numpy
    (arrays as ``{b"nd": ...}`` maps) is unpacked, its arrays made lists and packed again; the DROPPED fields of
    its params are left out.  Returns the bytes, whether they changed, the params (None: not readable) and the
    dropped fields' kind and length (never their values)."""
    seen = [False]

    def hook(obj):
        if _is_nd(obj):
            seen[0] = True
            return _nd(obj)
        return obj

    try:
        data = msgpack.unpackb(payload, raw=False, object_hook=hook, strict_map_key=False)
    except Exception:  # noqa: BLE001 - Server answers what it cannot read, and logs it
        return payload, False, None, {}
    params = data.get("params") if isinstance(data, dict) else None
    dropped: dict[str, dict] = {}
    if isinstance(params, dict):
        for key in DROPPED:
            if key in params:
                value = params.pop(key)
                dropped[key] = {"type": type(value).__name__,
                                "len": len(value) if isinstance(value, (list, tuple, dict, str, bytes)) else None}
                seen[0] = True
    else:
        params = None
    if not seen[0]:
        return payload, False, params, dropped
    return msgpack.packb(data, use_bin_type=True), True, params, dropped


#: requests per case whose fields are logged (``request_probe``)
PROBES = 1


def request_probe(params: dict | None, dropped: dict) -> dict | None:
    """What a request carries, for the log (names and sizes, never values): its params' keys, per picture the
    encoding, size and the calibration's field names, the readings' lengths, the dropped fields.  It shows what
    the online simulator sends (calibration, depth, numpy, task_progress) without a guess."""
    if not isinstance(params, dict):
        return None
    images = params.get("images") if isinstance(params.get("images"), dict) else {}
    pics = {}
    for name, entry in images.items():
        if not isinstance(entry, dict):
            pics[str(name)] = type(entry).__name__
            continue
        data = entry.get("image_data")
        cal = entry.get("calibration")
        pics[str(name)] = {"encoding": entry.get("encoding"), "width": entry.get("width"),
                           "height": entry.get("height"),
                           "bytes": len(data) if isinstance(data, (bytes, bytearray)) else None,
                           "calibration": sorted(str(k) for k in cal) if isinstance(cal, dict) else None,
                           "fields": sorted(str(k) for k in entry if k not in ("image_data", "calibration"))}
    states = params.get("states") if isinstance(params.get("states"), dict) else {}
    lengths = {str(k): (len(v) if isinstance(v, (list, tuple, dict)) else type(v).__name__) for k, v in states.items()}
    return {"params_keys": sorted(str(k) for k in params), "images": pics, "states_lengths": lengths,
            "dropped": dropped, "robot_type": params.get("robot_type"), "ee_type": params.get("ee_type")}


# -- the task of a case -----------------------------------------------------------------------------
_NAME = re.compile(r"[^A-Za-z0-9_-]+")


def clean_name(name: str) -> str:
    """A task name from the wire as a word (it names files: notes, records)."""
    word = _NAME.sub("_", str(name or "")).strip("_")[:80]
    return word or "unknown"


def runtime_of(params: dict | None, dropped: dict | None = None) -> str:
    """The physics runtime a request comes from, by its fields (for a task we do not know; a known task's config
    says it): MuJoCo sends ``simulation_time``, ``episode_start`` and ``action_contract``; Newton no
    ``end_pose``; libuipc an empty ``task_progress``; Isaac none of these.  Only the Wuji hands' tasks run
    outside Isaac."""
    if not isinstance(params, dict):
        return "isaac"
    robot = O.ROBOT_OF_WIRE.get((str(params.get("robot_type") or ""), str(params.get("ee_type") or "")))
    if robot != "g2_wuji":
        return "isaac"
    if any(k in params for k in ("action_contract", "simulation_time", "episode_start")):
        return "mujoco"
    states = params.get("states") if isinstance(params.get("states"), dict) else {}
    if "end_pose" in states and states.get("end_pose") is None:
        return "newton"
    progress = (dropped or {}).get("task_progress")
    if progress is not None and progress.get("len") == 0:
        return "libuipc"
    return "isaac"


def board_guess(robot: str, wheels: bool) -> str:
    """The board of a task we do not know, by its robot (the Wuji hands: dexmanip; the G2 with ctek90d grippers:
    wbc when it reports its chassis, else finemanip; the others: crsemb)."""
    if robot == "g2_wuji":
        return "dexmanip"
    if robot == "g2_90d":
        return "wbc" if wheels else "finemanip"
    return "crsemb"


def online_facts(task_name: str, robot: str, *, runtime: str = "isaac", wheels: bool = False) -> dict:
    """The config of a case whose requests name ``task_name`` with this robot (``protocol.ROBOT_OF_WIRE`` key).

    ``match``: 'sub_task' (the benchmark's sub-task name, as corobotpolicy, libuipc and Newton send it, or the
    Rubik's cube's scene), 'task' (our task name), 'prefix' (one task of the robot and runtime whose name starts
    ``task_name``, e.g. ``rubik_cube_solve_1turn``: a variant; the longest such name wins), or None: unknown
    (``known`` False), played as a stand-in config of the robot's guessed board and the request's runtime
    (``stand_in``) under the task name from the wire (``task``: no task note)."""
    name = str(task_name or "")
    mine = [t for t in P.TASKS.values() if t.robot == robot]
    match, found = None, None
    for how, pick in (("sub_task", lambda t: t.sub_task == name), ("task", lambda t: t.task == name)):
        hits = [t for t in mine if pick(t)]
        if len(hits) == 1:
            match, found = how, hits[0]
            break
    if found is None and name:
        hits = sorted((t for t in mine if t.runtime == runtime and name.startswith(t.task)),
                      key=lambda t: len(t.task), reverse=True)
        if hits and (len(hits) == 1 or len(hits[0].task) > len(hits[1].task)):
            match, found = "prefix", hits[0]
    if found is not None:
        return {"known": True, "match": match, "config": found.config, "stand_in": found.config,
                "board": found.board, "robot": robot, "task": found.task, "sub_task": name or found.sub_task,
                "runtime": found.runtime}
    board = board_guess(robot, wheels)
    pool = [t for t in mine if t.board == board] or mine
    same = [t for t in pool if t.runtime == runtime] or pool
    stand_in = same[0] if same else None
    word = clean_name(name)
    return {"known": False, "match": None, "config": f"online_{board}_{word}",
            "stand_in": None if stand_in is None else stand_in.config, "board": board, "robot": robot, "task": word,
            "sub_task": name, "runtime": runtime if stand_in is None else stand_in.runtime}


def facts_of_request(params: dict | None, dropped: dict | None = None) -> dict | None:
    """:func:`online_facts` of one request's params (None: no robot we know)."""
    if not isinstance(params, dict):
        return None
    robot = O.ROBOT_OF_WIRE.get((str(params.get("robot_type") or ""), str(params.get("ee_type") or "")))
    if robot is None:
        return None
    states = params.get("states") if isinstance(params.get("states"), dict) else {}
    wheels = bool(states.get("chassis_states"))
    return online_facts(str(params.get("task_name") or ""), robot, runtime=runtime_of(params, dropped), wheels=wheels)


def model_probe(settings, timeout_s: float = PROBE_TIMEOUT_S) -> str | None:
    """One short request to the model of these settings (a single attempt, no retries): None when it answered,
    else what went wrong (the gateway's status and words; never the key).  A scripted run has no model: None."""
    from ...runtime.llm import AnthropicGateway, Gateway, GatewayError

    if getattr(settings, "script", ""):
        return None
    try:
        key = settings.api_key()
    except Exception:  # noqa: BLE001 - never shown: it may hold the key's path only
        return "no model key"
    kw = {"reasoning_effort": settings.reasoning_effort, "max_tokens": settings.max_tokens,
          "request_timeout_s": timeout_s, "stream": settings.stream, "retries": 0, "ca_file": settings.ca_file or None}
    if settings.api == "anthropic":
        gateway = AnthropicGateway(settings.api_base, key, settings.model, display=settings.thinking_display, **kw)
    elif settings.api == "openai":
        gateway = Gateway(settings.api_base, key, settings.model, **kw)
    else:
        return None
    try:
        gateway.ask([{"role": "user", "content": "Reply with the single word: ready"}], [],
                    deadline=time.monotonic() + timeout_s + 5.0)
        return None
    except GatewayError as exc:
        return str(exc)[:300]
    except Exception as exc:  # noqa: BLE001 - a probe that cannot be made is a failed probe
        return f"{type(exc).__name__}: {exc}"[:300]
    finally:
        gateway.close()


def sent_actions(link) -> int:
    """The replies of an episode so far that carried its own rows (not hold rows): 0 = the robot has not moved."""
    return int(link.stats["dispatches"]) - int(link.stats["hold_replies"])


class OnlineSession(ColiseumSession):
    """:class:`.session.ColiseumSession` for a case of the online platform: the same card, notes and robot calls
    as a local run of the case's config (``facts``, :func:`online_facts`); the instance is the case's number (no
    layout is known online), the part ``online``, no seed; the record is named after the case."""

    def __init__(self, link, info: dict[str, Any], facts: dict):
        if not facts.get("stand_in"):
            raise RuntimeError(f"no config of robot {facts.get('robot')!r} to play {facts.get('task')!r} with")
        case = int(info["case"])
        super().__init__(link, {**info, "config": facts["stand_in"], "instance": case, "part": "online", "seed": None})
        if not facts["known"]:  # the wire's task under the stand-in's robot, board and runtime: no task note
            self.t = dataclasses.replace(self.t, config=facts["config"], task=facts["task"], sub_task=facts["sub_task"])
            self.config, self.task = self.t.config, self.t.task
            self.info.update({"config": self.config, "task_name": self.task})
        self.facts = dict(facts)
        self.case = case
        self.slot = info.get("slot")
        self.depth_mode = info.get("depth_mode") or DEPTH_MODES[0]

    def record_name(self) -> str | None:
        return f"{self.config}_online_s{self.slot}_c{self.case:03d}_e{self.k}"

    def labels(self) -> dict[str, Any]:
        return {**super().labels(), "case": self.case}

    def open(self, settings, rec, attach) -> None:
        super().open(settings, rec, attach)
        native = getattr(self.link, "runtime", "isaac") in NATIVE
        if self.depth_mode == "when_needed" and not native:
            # depth only when the perception needs it (hold_for_depth): a request with depth arrives slowly online
            self.host.driver.motion_depth = False
            self.link.hold_depth = False
        if not self.facts["known"]:
            self.errors.append(f"unknown online task {self.facts['sub_task']!r} for robot {self.facts['robot']}: "
                               f"played as {self.facts['stand_in']} without a task note")
        elif self.facts["match"] == "prefix":
            self.errors.append(f"online task {self.facts['sub_task']!r} taken as {self.config} (its name's start)")
        if not native and not self.link.first.depth:
            self.errors.append("the first request of the episode has no depth pictures")
            rec.event({"kind": "online_no_depth", "cams": sorted(self.link.first.images)})
        rec.event({"kind": "online_case", "case": self.case, "slot": self.slot, "depth_mode": self.depth_mode,
                   **{k: self.facts.get(k) for k in ("known", "match", "config", "board", "runtime", "sub_task")}})

    def conclude(self, timeout_s: float) -> dict | None:
        """End the episode as a local run does, except one that has sent only hold rows (the model failed before
        the robot moved): it is left to the agent's model guard, which may play it again; the episode thread
        concludes it otherwise (Server._episode)."""
        if sent_actions(self.link) == 0 and not self.link.over:
            if self.rec is not None:
                self.rec.event({"kind": "conclusion_deferred", "why": "only hold rows were sent"})
            return {"success": None, "progress": None, "online": True, "conclusion": "deferred"}
        return super().conclude(timeout_s)

    def result_fields(self, verdict: dict) -> dict[str, Any]:
        return {**super().result_fields(verdict), "online": True, "case": self.case,
                "sub_task": self.facts["sub_task"], "match": self.facts["match"], "known": self.facts["known"]}


# -- one case ---------------------------------------------------------------------------------------
class TunnelServer(Server):
    """:class:`.link.Server` for one case of the tunnel: the tunnel hands it the case's requests
    (:meth:`answer`); the case's end is the local run's connection closing; no results file, so every verdict
    is "online".  ``hold_s``: the longest a request waits before its hold row (None: as late as the reply limit
    allows); ``reply_cap_s``: the platform's limit on one reply."""

    def __init__(self, *, on_episode, hold_s: float | None, reply_cap_s: float = SIM_REPLY_S,
                 log_path: Path | None = None):
        self.hold_arg = None if hold_s is None else float(hold_s)
        self.reply_cap_s = float(reply_cap_s)
        super().__init__("127.0.0.1", 0, on_episode=on_episode, results_dir=None, hold_s=self._hold(self.reply_cap_s),
                         reply_limit_s=self.reply_cap_s, runtime="isaac", log_path=log_path)
        self.facts: dict | None = None
        self._key: tuple | None = None
        self.probes = 0
        self.case_index: int | None = None
        self.case_dir: Path | None = None

    def _hold(self, limit: float) -> float:
        return limit if self.hold_arg is None else min(self.hold_arg, limit)

    def prepare(self, params: dict | None, dropped: dict | None = None) -> dict | None:
        """Take the case's facts from a request before it is read: the runtime (how it is read and answered),
        the reply limit and hold, the WBC board's smoothed depth.  A request that ends an episode changes none."""
        if not isinstance(params, dict) or params.get("episode_done") is True:
            return self.facts
        key = (str(params.get("task_name") or ""), str(params.get("robot_type") or ""), str(params.get("ee_type") or ""))
        if key == self._key:
            return self.facts
        facts = facts_of_request(params, dropped)
        if facts is None:
            return self.facts
        runtime = facts["runtime"]
        limit = min(float(P.REPLY_WAIT_S.get(runtime, SIM_REPLY_S)), self.reply_cap_s)
        with self._lock:
            self._key = key
            self.facts = facts
            self.runtime = runtime
            self.reply_limit_s = limit
            self.hold_s = self._hold(limit)
            self.smooth_depth = facts["board"] == "wbc"
        self.log({"kind": "case_task", "task_name": key[0], "reply_limit_s": limit, "hold_s": self.hold_s,
                  **{k: facts[k] for k in ("known", "match", "config", "board", "robot", "runtime")}})
        return facts

    def answer(self, plain: bytes, params: dict | None = None, dropped: dict | None = None) -> bytes:
        self.prepare(params, dropped)
        return self._request(plain)

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


#: player(link, facts, info) -> fields for the episode's entry: plays one episode (tests script one; default
#: an Episode of OnlineSession)
Player = Callable[[Any, dict, dict], dict | None]


class Agent:
    def __init__(self, args, state: dict[str, str], *, player: Player | None = None,
                 ready_check: Callable[[], str | None] | None = None,
                 model_check: Callable[[], str | None] | None = None):
        self.args = args
        self.job_uuid = state["JOB_UUID"]
        self.token = state.get("JOB_TOKEN") or state.get("CHALLENGE_TOKEN") or ""
        self.token_kind = "JOB_TOKEN" if state.get("JOB_TOKEN") else "CHALLENGE_TOKEN"
        self.endpoint = getattr(args, "endpoint", None) or state.get("TUNNEL_ENDPOINT") or ""
        if not self.endpoint:
            raise SystemExit("no TUNNEL_ENDPOINT in the state file and no --endpoint")
        self.agent = agent_id(self.job_uuid, args.slot)
        self.tag = job_tag(self.job_uuid)
        self.out = Path(args.out).resolve()
        self.dir = self.out / "agents" / f"{self.tag}_slot{args.slot}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self._log = open(self.dir / "tunnel.jsonl", "a", encoding="utf-8")
        self._log_lock = threading.Lock()
        self.player = player
        self.ready_check = ready_check
        #: the model probe (tests give their own): None = the model answered
        self.model_check = model_check
        self.model_probe_s = MODEL_PROBE_S
        self.model_wait_cap_s = MODEL_WAIT_CAP_S
        self.model_replays = MODEL_REPLAYS
        self._model_lock = threading.Lock()
        #: set while the model is taken to be down ({"since", "why"}): no episode starts its model then
        self.model_down: dict | None = None
        self.model_waits: list[dict] = []
        self._settings: dict[str, Any] = {}
        self.cases: dict[str, TunnelServer] = {}
        #: the session ids of the cases the gateway closed (a late frame of one is dropped, not a new case)
        self.closed: set[str] = set()
        self.case_n = 0
        self.played: list[dict] = []
        self.threads: list[threading.Thread] = []
        self.ws = None
        self.connected: asyncio.Event | None = None
        self.drained = False
        self.exit_code = 0
        self.stats = {"dials": 0, "connections": 0, "requests": 0, "replies": 0, "empty": 0, "repacked": 0,
                      "dropped_fields": 0, "dropped_replies": 0, "controls": {}}

    # -- settings ----------------------------------------------------------------------------------
    def settings_of(self, board: str):
        """The agent settings of a board (its limits, config/benches.yml, --set), with ONLINE_LIMITS unless --set
        gives them."""
        if board not in self._settings:
            from .run import settings_for

            settings = settings_for(board, self.args.settings, self.args.set)
            given = {item.partition("=")[0] for item in self.args.set}
            for name, value in ONLINE_LIMITS.items():
                if name not in given:
                    setattr(settings, name, value)
            self._settings[board] = settings
        return self._settings[board]

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

        srv = TunnelServer(on_episode=play, hold_s=self.args.hold_s, reply_cap_s=self.args.reply_cap_s,
                           log_path=cdir / "link.jsonl")
        srv.case_index = n
        srv.case_dir = cdir
        srv.start()
        self.cases[session_id] = srv
        self.log({"kind": "case_open", "case": n, "implicit": implicit, "dir": str(cdir)})
        self.say(f"case {n} open{' (data before session_open)' if implicit else ''}")
        return srv

    def play(self, link, n: int, cdir: Path) -> None:
        facts = online_facts(link.task_name, link.robot, runtime=link.runtime,
                             wheels=link.first.chassis is not None and len(link.first.chassis) > 0)
        info = {"case": n, "episode": link.index, "slot": self.args.slot, "run_dir": str(cdir), "online": True,
                "job": self.tag, "depth_mode": self.args.depth}
        entry = {"case": n, "episode": link.index, "task_name": link.task_name,
                 **{k: facts.get(k) for k in ("config", "board", "runtime", "match", "known")}}
        if {facts.get("task"), facts.get("sub_task")} & WAIT_FOR_SIMULATOR_END_TASKS and link.runtime not in NATIVE:
            link.linger = True
            entry["waits_for_simulator_end"] = True
        self.log({"kind": "episode_begin", **entry, "prompt": link.prompt, "depth": sorted(link.first.depth)})
        if link.runtime not in NATIVE and not link.first.depth:
            self.say(f"WARNING case {n} episode {link.index}: its first request has no depth pictures "
                     f"(perception needs them)")
        if not facts["known"]:
            self.say(f"WARNING case {n}: unknown task {facts['sub_task']!r} for {facts['robot']}: "
                     f"playing it as {facts['stand_in']} without a task note")
        runs: list[dict] = []
        try:
            while True:
                wait = self.wait_for_model(link, n)
                if wait is not None:
                    entry.setdefault("model_waits", []).append(wait)
                    if not wait["ok"]:
                        if not runs:
                            entry["end"] = "model_down"
                        break
                run: dict = {}
                runs.append(run)
                try:
                    self._play_once(link, facts, {**info, **({"replay": len(runs) - 1} if len(runs) > 1 else {})},
                                    run)
                finally:
                    entry.update(run)
                if run.get("end") != "gateway_error":
                    break
                errors = run.get("errors") or []
                self.mark_model_down(str(errors[-1]) if errors else "gateway_error")
                if link.over or sent_actions(link) > 0 or len(runs) > self.model_replays:
                    break
                self.log({"kind": "episode_replay", "case": n, "episode": link.index, "replay": len(runs)})
                self.say(f"case {n} episode {link.index}: only hold rows were sent; it is played again from its "
                         f"start once the model answers (replay {len(runs)})")
        finally:
            if len(runs) > 1:
                entry["runs"] = [{k: r.get(k) for k in ("end", "turns", "record")} for r in runs]
            self.played.append(entry)
            self.log({"kind": "episode_end", **entry})
            self.say(f"case {n} episode {link.index} ({entry.get('config')}) ended: {entry.get('end')}, "
                     f"{entry.get('turns')} turns")

    def _play_once(self, link, facts: dict, info: dict, out: dict) -> None:
        """One play of the episode (the player's, or an Episode of OnlineSession); what it gives goes in ``out``."""
        if self.player is not None:
            out.update(self.player(link, facts, info) or {})
            return
        from ...runtime.episode import Episode

        episode = Episode(OnlineSession(link, info, facts), self.settings_of(facts["board"]), self.out)
        try:
            episode.run()
        finally:
            out["record"] = str(episode.rec.dir)
            result = episode.rec.dir / "result.json"
            if result.exists():
                data = json.loads(result.read_text())
                out.update({k: data.get(k) for k in ("task", "instruction", "end", "turns", "errors")})

    # -- the model guard ---------------------------------------------------------------------------
    def probe_model(self) -> str | None:
        if self.model_check is not None:
            return self.model_check()
        return model_probe(self.settings_of(P.BOARDS[0]))

    def mark_model_down(self, why: str) -> None:
        with self._model_lock:
            first = self.model_down is None
            if first:
                self.model_down = {"since": time.time(), "why": str(why)[:300]}
        if first:
            self.log({"kind": "model_down", "why": str(why)[:300]})
            self.say(f"the model failed ({str(why)[:160]}): no episode starts it until a probe answers")

    def wait_for_model(self, link, n: int) -> dict | None:
        """While the model is taken to be down, probe it every ``model_probe_s`` and hold the episode (its requests
        get hold rows, depth not asked) until it answers, the episode ends, or ``model_wait_cap_s`` passes.  None
        when the model was not down; else the wait's record (``ok``: the model answered)."""
        if self.model_down is None:
            return None
        t0 = time.monotonic()
        probes, last, said = 0, None, t0
        holds0 = int(link.stats["hold_replies"])
        depth = link.hold_depth
        link.hold_depth = False
        self.log({"kind": "model_wait_begin", "case": n, "episode": link.index})
        self.say(f"case {n} episode {link.index}: waiting for the model (hold rows meanwhile)")
        try:
            while True:
                if self.model_down is None:
                    ok, why = True, "the model answered"
                    break
                if link.over:
                    ok, why = False, "the episode ended while waiting"
                    break
                if time.monotonic() - t0 > self.model_wait_cap_s:
                    ok, why = False, f"no answer within {self.model_wait_cap_s:.0f} s"
                    break
                if last is None or time.monotonic() - last >= self.model_probe_s:
                    last = time.monotonic()
                    probes += 1
                    problem = self.probe_model()
                    if problem is None:
                        with self._model_lock:
                            self.model_down = None
                        continue
                    self.log({"kind": "model_probe_failed", "case": n, "episode": link.index, "probe": probes,
                              "problem": str(problem)[:300]})
                    if time.monotonic() - said >= MODEL_SAY_S:
                        said = time.monotonic()
                        self.say(f"case {n} episode {link.index}: the model still fails after "
                                 f"{time.monotonic() - t0:.0f} s ({probes} probes): {str(problem)[:120]}")
                time.sleep(min(1.0, self.model_probe_s))
        finally:
            link.hold_depth = depth
        record = {"case": n, "episode": link.index, "ok": ok, "why": why, "waited_s": round(time.monotonic() - t0, 1),
                  "probes": probes, "hold_replies": int(link.stats["hold_replies"]) - holds0}
        with self._model_lock:
            self.model_waits.append(record)
        self.log({"kind": "model_wait", **record})
        self.say(f"case {n} episode {link.index}: {why} after {record['waited_s']:.0f} s, {probes} probe(s), "
                 f"{record['hold_replies']} hold row(s)")
        return record

    def close_case(self, session_id: str, why: str) -> None:
        srv = self.cases.pop(session_id, None)
        self.closed.add(session_id)
        if srv is None:
            return
        srv.close_case(why)
        self.log({"kind": "case_close", "case": srv.case_index, "why": why, "episodes": len(srv.episodes)})
        self.say(f"case {srv.case_index} closed ({why}), {len(srv.episodes)} episode(s)")
        t = threading.Thread(target=self._finish_case, args=(srv,), name=f"finish{srv.case_index}", daemon=True)
        self.threads.append(t)
        t.start()

    @staticmethod
    def _finish_case(srv: TunnelServer) -> None:
        for t in list(srv.threads):
            t.join(timeout=900)
        srv.stop()

    # -- the wire ----------------------------------------------------------------------------------
    async def send(self, data: bytes | str) -> bool:
        """Send on the current connection; wait up to SEND_WAIT_S for one if there is none."""
        deadline = time.monotonic() + SEND_WAIT_S
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
        try:
            sid, payload = split_frame(message)
        except ValueError as exc:
            self.log({"kind": "bad_frame", "bytes": len(message), "error": str(exc)[:200]})
            return
        if not payload:
            self.stats["empty"] += 1
            self.log({"kind": "empty_frame", "sid_len": len(sid)})
            await self.send(join_frame(sid, EMPTY_REPLY))
            return
        if sid in self.closed and sid not in self.cases:
            self.log({"kind": "frame_after_close", "bytes": len(payload)})
            return
        srv = self.cases.get(sid) or self.case(sid, implicit=True)
        self.stats["requests"] += 1
        t_in = time.time()
        loop = asyncio.get_running_loop()
        plain, changed, params, dropped = await loop.run_in_executor(None, plain_request, payload)
        if changed:
            self.stats["repacked"] += 1
        if dropped:
            self.stats["dropped_fields"] += 1
        if srv.probes < PROBES:
            srv.probes += 1
            self.log({"kind": "request_probe", "case": srv.case_index, "n": srv.probes, "bytes": len(payload),
                      "repacked": changed, "probe": request_probe(params, dropped)})
        reply = await loop.run_in_executor(srv._pool, srv.answer, plain, params, dropped)
        ok = await self.send(join_frame(sid, reply))
        self.stats["replies" if ok else "dropped_replies"] += 1
        if not ok:
            self.log({"kind": "reply_dropped", "case": srv.case_index, "bytes": len(payload)})
        if self.stats["requests"] <= 3:
            self.log({"kind": "request_timing", "case": srv.case_index, "bytes": len(payload), "repacked": changed,
                      "answer_s": round(time.time() - t_in, 3)})

    async def on_control(self, text: str, ws) -> None:
        try:
            ctl = json.loads(text)
        except ValueError:
            self.log({"kind": "bad_control", "text_len": len(text)})
            return
        kind = ctl.get("type") if isinstance(ctl, dict) else None
        self.stats["controls"][str(kind)] = self.stats["controls"].get(str(kind), 0) + 1
        sid = ctl.get("session_id") if isinstance(ctl, dict) else None
        self.log({"kind": "control", "type": kind, "has_session": sid is not None,
                  "reason": ctl.get("reason") if isinstance(ctl, dict) else None})
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
            self.closed.discard(str(sid))
            self.case(str(sid))
        elif kind == "session_close" and sid:
            self.close_case(str(sid), "session_close")
        elif kind == "drain":
            self.drained = True
            self.say("drain: no more cases")
            await ws.close()
        else:
            self.log({"kind": "control_ignored", "type": kind})

    def warm(self) -> str | None:
        """What would stop us playing (the SAM3 service, the model key), or None."""
        if self.ready_check is not None:
            return self.ready_check()
        import urllib.request

        settings = self.settings_of(P.BOARDS[0])
        url = settings.sam3_url
        if url:
            try:
                opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
                opener.open(url.rstrip("/") + "/health", timeout=10).read()
            except Exception as exc:  # noqa: BLE001
                return f"SAM3 does not answer ({type(exc).__name__})"
        try:
            settings.api_key()
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
        start = {"kind": "start", "slot": self.args.slot, "job": self.tag, "credential": self.token_kind,
                 "scheme": urlsplit(self.endpoint).scheme, "hold_s": self.args.hold_s,
                 "reply_cap_s": self.args.reply_cap_s, "depth_mode": self.args.depth}
        if self.player is None:
            s = self.settings_of(P.BOARDS[0])
            start.update({"model": s.model, "reasoning_effort": s.reasoning_effort, "skills": s.skills,
                          "depth": s.depth, "budget_s": s.budget_s, "cell_total_s": s.cell_total_s})
        self.log(start)
        self.say(f"job {self.tag}, dialing the gateway ({self.token_kind}); depth {self.args.depth}")
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
                    self.log({"kind": "closed", "code": ws.close_code, "drained": self.drained})
            except InvalidStatus as exc:
                status = exc.response.status_code
                self.log({"kind": "dial_refused", "status": status})
                if status in (401, 403, 404):
                    self.say(f"the gateway refused the dial (HTTP {status}): token or job not valid / not active")
                    return 2
            except (OSError, asyncio.TimeoutError, websockets.ConnectionClosed, websockets.InvalidHandshake) as exc:
                self.log({"kind": "dial_failed" if opened is None else "connection_lost", "error": type(exc).__name__})
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
        summary = {"slot": self.args.slot, "job": self.tag, "board": JOB_BOARD, "exit": code, "drained": self.drained,
                   "cases": self.case_n, "episodes": len(self.played), "stats": self.stats, "model_waits": self.model_waits,
                   "played": sorted(self.played, key=lambda e: (e["case"], e["episode"]))}
        (self.dir / "agent.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=str) + "\n")
        self.log({"kind": "end", "exit": code, "stats": self.stats})
        with self._log_lock:
            self._log.close()
            self._log = None
        self.say(f"exit {code}: {self.case_n} case(s), {len(self.played)} episode(s)")


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum.tunnel",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--slot", type=int, default=0, help="this process's slot (its own agent id), 0..parallelism-1")
    ap.add_argument("--state", default=str(STATE), help="the job's state file (JOB_UUID, JOB_TOKEN, TUNNEL_ENDPOINT)")
    ap.add_argument("--endpoint", default=None, help="default: TUNNEL_ENDPOINT of the state file")
    ap.add_argument("--out", default=str(ROOT / "results" / "benches" / "robocoliseum_online"), help="records")
    ap.add_argument("--settings", default=str(ROOT / "config" / "benches.yml"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--hold-s", type=float, default=None,
                    help="the longest a request waits for the model before a hold row (default: as late as the "
                         "reply limit allows)")
    ap.add_argument("--reply-cap-s", type=float, default=SIM_REPLY_S,
                    help="the platform's limit on one reply, download included (a native runtime's own limit "
                         "when shorter)")
    ap.add_argument("--depth", choices=DEPTH_MODES, default=DEPTH_MODES[0],
                    help="Isaac: ask for depth only when the perception needs it, or after every motion as locally")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    state = read_state(Path(args.state))
    if not state.get("JOB_UUID"):
        raise SystemExit("no JOB_UUID in the state file: submit the job first")
    if not (state.get("JOB_TOKEN") or state.get("CHALLENGE_TOKEN")):
        raise SystemExit("no JOB_TOKEN / CHALLENGE_TOKEN in the state file")
    problem = season_problem(state)
    if problem:
        raise SystemExit(problem)
    agent = Agent(args, state)
    problem = agent.warm()
    if problem:
        raise SystemExit(f"not ready: {problem}")
    problem = agent.probe_model()
    if problem:
        agent.mark_model_down(f"at the start: {problem}")

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
