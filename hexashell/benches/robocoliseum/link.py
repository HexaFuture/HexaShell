"""The wire to the Season II benchmark: the WebSocket server its policy client dials, and the hand-over
between its requests and the thread of the episode they belong to.

The benchmark (``benchmark/policy/corobotpolicy.py`` of genie_sim 3.6.0) connects once per run and sends one
msgpack request when it needs actions (``{"method": "infer", "params": {...}}``: the pictures, the readings,
the instruction); the reply (``{"result": {...}}``) holds a chunk of rows of joint targets for both arms and
both effectors (and the waist, once we command it), which it plays one row per step; it asks again when they
are used up.  Each reply with rows to a request of a running episode is one *dispatch*: the score counts them
per stage (:mod:`.budget`).  ``need_depth`` in a reply adds the depth pictures and the calibration to the next
request.

Episodes: our runs play one episode each.  The benchmark ends it when every stage is met or a stage runs out
of steps (it then drops the rest of our chunk and sends a request with ``episode_done``), or when a reply
carries ``episode_done`` with its rows: it plays them, then ends the episode the normal way (verified:
the result is a regular one, ``code`` 0).  So when our side is done, :meth:`EpisodeLink.conclude` sends one
held chunk of ``CONCLUDE_ROWS`` rows with ``episode_done`` (one dispatch).  An episode is over at whichever
comes first: a request with ``episode_done``; a request whose instruction or ``episode_idx`` differs from the
episode's; the results file holding one more entry; the connection closing.  The agent2act no-action
termination is never used: the runner then records the episode as invalid.

Waiting for the simulator's end (``EpisodeLink.linger``, set for the tasks the online agent lists): when our
side is done, the replies carry no ``episode_done``; each is one held chunk of ``LINGER_ROWS`` rows (one
dispatch), so the simulator plays held rows until it ends the episode itself (every stage met, or a stage out of
steps; it drops the rest of the chunk).  After ``LINGER_REPLIES`` such replies the normal conclusion goes.
Isaac only (the native runtimes cap their rows).

While the model thinks there is no action: the local simulator waits for the reply without a limit, and
physics goes on with the last targets held.  The online platform may limit a reply's time (Season I: 120 s);
``hold_s`` (None: never) then answers a request still unanswered after that long with one row that holds
everything (one more dispatch), by ``REPLY_MARGIN_S`` before the limit counting the request's time on the way.

Verdicts: after an episode ends, the benchmark adds it to ``details`` of ``evaluate_ret_*.json``; the k-th
episode the run ended is ``details[k]`` (:func:`verdict_of`).

Native runtimes (dexmanip: Newton wok_flip, libuipc cloth tasks, MuJoCo Rubik's cube; developer data in
``protocol.REPLY_WAIT_S`` and ``EPISODE_ROWS``): the same request and reply, with these differences.
- Their robot's body is fixed: a reply never carries the waist, the head or the chassis (MuJoCo makes any
  such value an invalid episode, Newton refuses values other than the start's).
- A late reply ends the run without a score, so a hold row always goes before the limit (Newton 660 s,
  libuipc 120 s, MuJoCo 60 s), counted from the request's own capture time (``timestamps``).
- MuJoCo takes at most ``REPLY_ROWS`` rows a reply: a longer chunk goes as several replies (one dispatch each).
- Ending: Newton and MuJoCo end the episode on a reply of ``episode_done`` alone (no rows, no dispatch);
  libuipc ignores it and plays until its episode's last row, so its conclusion is one held chunk of the rows
  that are left (one dispatch).
- Newton keeps copies of its result file below the task's folder: only ``<root>/<scene>/<sub_task>/`` counts.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable

import msgpack
import numpy as np

from ...motion.driver import EpisodeOver
from . import kinematics
from . import obs as O
from . import protocol as P

#: the online platform's limit on one reply (Season I), counted from when the simulator sends its request
SIM_REPLY_S = 120.0
#: a hold row goes this long before that limit (our reply's upload, clock slack)
REPLY_MARGIN_S = 15.0
#: the shortest wait before a hold row, however late the request came
MIN_HOLD_S = 2.0
#: the request's time on the way when there is no earlier reply to count from (an episode's first request)
UNKNOWN_ARRIVAL_S = 90.0
#: rows of the chunk that ends an episode (with episode_done; the simulator plays them first)
CONCLUDE_ROWS = 30
#: waiting for the simulator's end: rows of one held chunk without episode_done (more than a stage's steps), and
#: how many such replies go before the normal conclusion
LINGER_ROWS = 1000
LINGER_REPLIES = 3
#: seconds to wait for an ended episode's entry in evaluate_ret
VERDICT_S = 300.0
#: ... by runtime: a libuipc episode is scored by a stream scorer that may still be catching up when the
#: episode ends (the benchmark gives it 3600 s; run.EXIT_GRACE_RUNTIME_S)
VERDICT_RUNTIME_S = {"libuipc": 3660.0}


def verdict_wait_s(runtime: str) -> float:
    return VERDICT_RUNTIME_S.get(runtime, VERDICT_S)
#: the runtimes whose robot has a fixed body and whose episodes end by their own rules (see the module text)
NATIVE = ("newton", "libuipc", "mujoco")
#: a request's capture time (``timestamps``, wall-clock ns) counts only this close to our clock
CLOCK_SLACK_S = 600.0
#: how often the results file is read for an episode the requests did not show the end of
RESULTS_POLL_S = 2.0


def start_command(frame: O.Frame) -> dict:
    """The command that holds the robot as it is at the start of an episode."""
    return {"left": frame.arm_joints("left"), "right": frame.arm_joints("right"),
            "grip": {arm: frame.effector(arm) for arm in kinematics.ARMS}, "waist": None, "head": None}


def copy_row(row: dict, *, wheels: bool = False) -> dict:
    """A copy of a command row.  Its chassis target ("wheels", WBC) only with ``wheels``: a drive moves the
    base once, in the chunk it was planned for, so a row sent again (a hold, the conclusion, the start of the
    next motion) never carries it."""
    out = {"left": np.array(row["left"], float), "right": np.array(row["right"], float),
           "grip": {arm: np.array(v, float).reshape(-1) for arm, v in row["grip"].items()},
           "waist": None if row.get("waist") is None else np.array(row["waist"], float),
           "head": None if row.get("head") is None else np.array(row["head"], float)}
    if wheels and row.get("wheels") is not None:
        out["wheels"] = np.array(row["wheels"], float).reshape(-1)
    return out


def pack(rows: list[dict], need_depth: bool, *, episode_done: bool = False, runtime: str = "isaac") -> bytes:
    """One reply.  Native runtimes: no waist, head or chassis, and ``episode_done`` may come without rows."""
    if not rows:
        if not episode_done:
            raise ValueError("a reply without rows must end the episode")
        if runtime not in ("newton", "mujoco"):
            raise ValueError(f"{runtime} takes no reply without rows")
        return msgpack.packb({"result": {"episode_done": True}}, use_bin_type=True)
    native = runtime in NATIVE
    out: dict[str, Any] = {
        "left_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["left"]] for r in rows]},
        "right_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["right"]] for r in rows]},
        "left_effector": [[float(v) for v in np.reshape(r["grip"]["left"], -1)] for r in rows],
        "right_effector": [[float(v) for v in np.reshape(r["grip"]["right"], -1)] for r in rows],
        "need_depth": bool(need_depth),
    }
    if not native and all(r.get("waist") is not None for r in rows):
        out["waist"] = {"kind": "JOINT_ABS", "values": [[float(v) for v in r["waist"]] for r in rows]}
    # the head (WBC: robot.head), once it was moved: until then the simulator holds its start
    if not native and all(r.get("head") is not None for r in rows):
        out["head"] = {"kind": "JOINT_ABS", "values": [[float(v) for v in r["head"]] for r in rows]}
    # the chassis (WBC): a pose target [dx, dy, dyaw] from where the base is at the chunk's start, the same in
    # every row of a drive (the simulator takes the last row's); never carried into the next chunk (copy_row)
    wheels = [r.get("wheels") for r in rows]
    if any(w is not None for w in wheels):
        if native:
            raise ValueError(f"the {runtime} robot has no chassis to drive")
        if any(w is None for w in wheels):
            raise ValueError("a chunk drives the chassis in all of its rows or in none")
        out["wheels"] = {"kind": "POSE_DELTA", "values": [[float(v) for v in np.reshape(w, -1)] for w in wheels]}
    if episode_done:
        out["episode_done"] = True
    return msgpack.packb({"result": out}, use_bin_type=True)


def boundary(ep: "EpisodeLink", frame: O.Frame) -> str | None:
    """Why ``frame`` (not episode_done) is the first request of a new episode, or None."""
    if frame.prompt != ep.prompt:
        return "the instruction changed"
    a, b = ep.first.episode_idx, frame.episode_idx
    if a is not None and b is not None and a != b:
        return f"episode_idx changed ({a} -> {b})"
    return None


def verdict_of(entry: dict) -> dict:
    """Our verdict from one ``details`` entry of evaluate_ret.

    score: the official episode score (``POLICY_BUDGET_WEIGHTED``, 0..1: the stage points kept under the
    dispatch budget plus the completion bonus, over 125); progress: the stage points earned without the
    budget, over 100; success: E2E 1; valid: the runner finished the episode normally (code 0)."""
    res = entry.get("result") or {}
    scores = res.get("scores") or {}
    steps = [{"name": s.get("name"), "score": float(s.get("score") or 0.0)} for s in scores.get("STEPS") or []]
    stages = []
    for s in scores.get("POLICY_BUDGET_STEPS") or []:
        stages.append({k: s.get(k) for k in ("name", "score", "points", "earned_points", "step_out", "baseline",
                                              "count", "over_budget", "factor")})
    points = sum(float(s.get("points") or 0.0) for s in stages)
    progress = (sum(float(s.get("points") or 0.0) * float(s.get("score") or 0.0) for s in stages) / points
                if points > 0 else (sum(s["score"] for s in steps) / len(steps) if steps else None))
    e2e = scores.get("E2E")
    weighted = scores.get("POLICY_BUDGET_WEIGHTED")
    # dexmanip: one budget for the whole episode (its stages carry none)
    budget = (res.get("budget_evidence") or {}).get("episode_budget")
    end = end_of(entry)
    return {"success": e2e is not None and float(e2e) >= 1.0,
            "score": None if weighted is None else round(float(weighted), 4),
            "progress": None if progress is None else round(progress, 4), "e2e": e2e,
            "valid": res.get("code") == 0, "sub_steps": steps, "stages": stages, "stage": res.get("step"),
            "code": res.get("code"), "msg": res.get("msg"), "instruction": entry.get("task_instruction"),
            "dispatches": entry.get("policy_dispatch_count"), "duration_s": entry.get("duration"),
            **({"budget": {k: budget.get(k) for k in ("step_out", "baseline", "count", "over_budget",
                                                       "penalty_fraction", "factor")}}
               if isinstance(budget, dict) else {}),
            **({"end": end} if end is not None else {})}


#: the native runtimes' words for an episode that played its whole span (libuipc's termination_reason, Newton's
#: stop_reason, MuJoCo's reason)
SPAN_ENDS = ("max_steps", "step_limit", "timeout")
#: the native runtimes' ends of an episode for what happened in its task, not failures of the simulator (third-party
#: sources): MuJoCo's Rubik's cube (backends/mujoco/rubik/native.py, TaskFailure: native_result status
#: 'task_failure' with this reason), Newton's wok (genie_sim_engine_newton .../wok_evaluation.py: the pan's rim below
#: the table's top sets failed and failure_reason)
TASK_ENDS = ("joint_limit", "arm_speed_limit", "self_collision", "cross_arm_collision", "drop", "wok_below_table")


def end_of(entry: dict) -> dict | None:
    """Why a native runtime ended the episode, from its own fields of the ``details`` entry (None: an Isaac entry,
    or none given): ``cause`` 'span' (its span of rows or simulated seconds played out), one of :data:`TASK_ENDS`
    (MuJoCo: a joint past its range, the arms touching ...; Newton: the pan below the table), 'failure' (the
    simulator failed: libuipc's "IPC source is failed", its scoring worker
    exiting) or the runtime's own reason; ``reason``: the runtime's words (developer data, never told as such).
    The fields: libuipc ``execution`` (execution_status, termination_reason,
    error), MuJoCo ``native_result`` (status, reason, error, valid), Newton ``native_result`` (stop_reason,
    failed, failure_reason)."""
    res = entry.get("result") or {}
    ex, nr = entry.get("execution"), entry.get("native_result")
    if isinstance(ex, dict):  # libuipc
        error = ex.get("error") or (res.get("msg") if res.get("code") not in (0, None) else None)
        if ex.get("execution_status") == "failed" or error:
            return {"cause": "failure", "reason": str(error or ex.get("execution_status"))[:200]}
        reason = ex.get("termination_reason")
    elif isinstance(nr, dict):  # MuJoCo, Newton
        told = nr.get("failure_reason") or nr.get("reason")
        if told in TASK_ENDS:  # the task's own end: Newton also marks it failed
            return {"cause": str(told), "reason": str(told)}
        if nr.get("failed") or nr.get("failure_reason") or nr.get("valid") is False:
            return {"cause": "failure", "reason": str(nr.get("failure_reason") or nr.get("error") or "invalid")[:200]}
        reason = nr.get("stop_reason") or nr.get("reason") or (nr.get("result") or {}).get("msg")
    else:
        return None
    if not reason:
        return None
    reason = str(reason)
    cause = "span" if reason in SPAN_ENDS else reason
    return {"cause": cause, "reason": reason[:200]}


def hold_after(hold_s: float, spent_s: float, limit_s: float = SIM_REPLY_S) -> float:
    """Seconds a request waits for an action before the hold row: ``hold_s``, cut so that the reply
    reaches the simulator ``REPLY_MARGIN_S`` before its limit (``limit_s``) when the request took ``spent_s``
    to come."""
    return max(MIN_HOLD_S, min(float(hold_s), float(limit_s) - REPLY_MARGIN_S - max(0.0, float(spent_s))))


def captured_s_ago(frame: O.Frame, now: float | None = None) -> float | None:
    """Seconds since the request's readings were captured (``timestamps.states`` or ``.head``, wall-clock ns:
    the native runtimes), or None when it carries no such time."""
    stamps = frame.timestamp
    if not isinstance(stamps, dict):
        return None
    now = time.time() if now is None else now
    for key in ("states", "head"):
        try:
            t = float(stamps[key]) / 1e9
        except (KeyError, TypeError, ValueError):
            continue
        if abs(now - t) <= CLOCK_SLACK_S:
            return max(0.0, now - t)
    return None


def arrival_s(gap_s: float | None) -> float:
    """An upper bound on a request's time on the way: the time since our last reply, or
    ``UNKNOWN_ARRIVAL_S`` without one."""
    return UNKNOWN_ARRIVAL_S if gap_s is None else max(0.0, gap_s)


class EpisodeLink:
    """One episode's side of the wire: the latest request's frame, the chunk the episode thread wants sent,
    and how the episode ended."""

    def __init__(self, index: int, first: O.Frame, *, hold_s: float | None = None,
                 calibrations: O.Calibrations | None = None, runtime: str = "isaac",
                 reply_limit_s: float | None = None):
        self.index = index
        self.runtime = runtime
        #: the benchmark's limit on one reply (hold rows go before it)
        self.reply_limit_s = SIM_REPLY_S if reply_limit_s is None else float(reply_limit_s)
        #: rows the benchmark takes in one reply (None: any) and in the whole episode (None: its stages decide)
        self.max_rows = P.REPLY_ROWS.get(runtime)
        self.episode_rows = P.EPISODE_ROWS.get(runtime)
        self.prompt = first.prompt
        self.task_name = first.task_name
        self.robot = first.robot
        self.hold_s = None if hold_s is None else float(hold_s)
        #: the calibrations this episode's requests brought (requests without one carry them on)
        self.calibrations = calibrations or O.Calibrations()
        #: whether a hold row asks for the depth pictures
        self.hold_depth = True
        self.first = first
        self._cv = threading.Condition()
        self._frame = first
        self._seq = first.seq
        self._pending: tuple[list[dict], bool] | None = None
        self._taken: int | None = None
        #: what was last sent (or holds the start): every row the episode sends starts from it
        self.command = start_command(first)
        self.ended = False
        self.abandoned: str | None = None
        self.concluding = False
        self.concluded = False
        #: when our side is done, hold until the simulator ends the episode instead of sending episode_done
        self.linger = False
        self._verdict: dict | None = None
        #: the k-th episode this run's benchmark ended (its entry in evaluate_ret), once it ended
        self.result_index: int | None = None
        self.stats = {"requests": 1, "dispatches": 0, "rows": 0, "hold_replies": 0, "conclude_replies": 0,
                      "linger_replies": 0, "depth_requests": int(bool(first.depth)), "longest_wait_s": 0.0}
        self.t0 = time.time()

    # -- the server's side ---------------------------------------------------------------------
    def serve(self, frame: O.Frame | None, spent_s: float = 0.0) -> tuple[list[dict], bool, str]:
        """Answer a request (``frame``; None: the episode's first, already given): the rows to send,
        need_depth, and what they are ('action', 'hold', 'linger', 'conclude')."""
        t0 = time.monotonic()
        with self._cv:
            if frame is not None:
                self._frame = frame
                self._seq = frame.seq
                self.stats["requests"] += 1
                self.stats["depth_requests"] += int(bool(frame.depth))
                self._cv.notify_all()
            deadline = None if self.hold_s is None else t0 + hold_after(self.hold_s, spent_s, self.reply_limit_s)
            while True:
                if self._pending is not None and self._taken is None and not (self.ended or self.abandoned):
                    rows, need = self._pending
                    self._pending = None
                    self._taken = self._seq
                    self._sent(rows)
                    kind = "action"
                    break
                if (self.concluding and self.linger and self.runtime not in NATIVE and not (self.ended or self.abandoned)
                        and self.stats["linger_replies"] < LINGER_REPLIES):
                    rows, need, kind = [copy_row(self.command)] * LINGER_ROWS, False, "linger"
                    self.stats["linger_replies"] += 1
                    self._sent(rows)
                    break
                if self.concluding or self.ended or self.abandoned:
                    rows, need, kind = self._conclusion(), False, "conclude"
                    self.stats["conclude_replies"] += 1
                    self.concluded = True
                    if rows:
                        self._sent(rows)
                    break
                left = None if deadline is None else deadline - time.monotonic()
                if left is not None and left <= 0:
                    rows, need, kind = [copy_row(self.command)], self.hold_depth, "hold"
                    self.stats["hold_replies"] += 1
                    self._sent(rows)
                    break
                self._cv.wait(1.0 if left is None else min(left, 1.0))
            waited = time.monotonic() - t0
            self.stats["longest_wait_s"] = round(max(self.stats["longest_wait_s"], waited), 2)
            return rows, need, kind

    def _conclusion(self) -> list[dict]:
        """The rows of the reply that ends the episode (with episode_done): Isaac a held chunk; Newton and
        MuJoCo none; libuipc, which plays to its episode's last row, the rows that are left, held."""
        if self.runtime in ("newton", "mujoco"):
            return []
        n = CONCLUDE_ROWS
        if self.runtime == "libuipc" and self.episode_rows:
            n = max(1, self.episode_rows - self.stats["rows"])
        return [copy_row(self.command)] * n

    def _sent(self, rows: list[dict]) -> None:
        self.command = copy_row(rows[-1])
        self.stats["dispatches"] += 1
        self.stats["rows"] += len(rows)

    def end(self, frame: O.Frame | None, result_index: int) -> None:
        """The benchmark ended the episode (``frame``: its request with episode_done; None: the episode
        was seen to be over otherwise)."""
        with self._cv:
            if frame is not None:
                self._frame = frame
                self._seq = frame.seq
            self.ended = True
            self.result_index = result_index
            self._cv.notify_all()

    def abandon(self, reason: str) -> None:
        with self._cv:
            if not self.ended and not self.abandoned:
                self.abandoned = reason
            self._cv.notify_all()

    def set_verdict(self, verdict: dict) -> None:
        with self._cv:
            self._verdict = dict(verdict)
            self._cv.notify_all()

    # -- the episode's side --------------------------------------------------------------------
    @property
    def over(self) -> bool:
        with self._cv:
            return self.ended or bool(self.abandoned)

    @property
    def verdict(self) -> dict | None:
        with self._cv:
            if self._verdict is not None:
                return dict(self._verdict)
            if self.abandoned:
                return {"success": False, "progress": None, "abandoned": self.abandoned}
            if self.ended:
                return {"success": None, "progress": None, "ended": True}
            return None

    @property
    def dispatches(self) -> int:
        with self._cv:
            return int(self.stats["dispatches"])

    @property
    def rows(self) -> int:
        with self._cv:
            return int(self.stats["rows"])

    def latest(self) -> O.Frame:
        with self._cv:
            return self._frame

    def act(self, rows: list[dict], need_depth: bool) -> O.Frame:
        """Send a chunk of rows (each a full command: both arms, both effectors, the waist or None) as one
        reply (as several where the benchmark takes fewer rows a reply: one dispatch each); block until the
        request after them arrives and return its frame.  Raises EpisodeOver once the episode is over."""
        if not rows:
            raise ValueError("no rows to send")
        if self.max_rows and len(rows) > self.max_rows:
            frame = None
            for i in range(0, len(rows), self.max_rows):
                part = rows[i:i + self.max_rows]
                frame = self.act(part, need_depth and i + self.max_rows >= len(rows))
            return frame
        with self._cv:
            self._raise_if_over()
            if self.concluding:
                raise EpisodeOver(self.verdict or {})
            self._pending = ([copy_row(r, wheels=True) for r in rows], bool(need_depth))
            self._taken = None
            self._cv.notify_all()
            while self._taken is None or self._seq <= self._taken:
                self._raise_if_over()
                self._cv.wait(1.0)
            return self._frame

    def conclude(self) -> None:
        """Nothing more to send: the next reply is the held chunk with episode_done (with ``linger``: held chunks
        without it, until the simulator ends the episode)."""
        with self._cv:
            self.concluding = True
            self._pending = None
            self._cv.notify_all()

    def wait_verdict(self, timeout_s: float) -> dict | None:
        end = time.monotonic() + timeout_s
        with self._cv:
            while self._verdict is None and not self.abandoned:
                left = end - time.monotonic()
                if left <= 0:
                    break
                self._cv.wait(min(left, 5.0))
        return self.verdict

    def _raise_if_over(self) -> None:
        if self.ended or self.abandoned:
            raise EpisodeOver(self.verdict or {})


class Server:
    """The WebSocket server of one benchmark run (one simulator process).  ``on_episode(link)`` is called in
    a new thread for every episode that starts; ``results_dir`` is where the run writes evaluate_ret."""

    def __init__(self, host: str, port: int, *, on_episode: Callable[[EpisodeLink], None],
                 results_dir: Path | None = None, hold_s: float | None = None, reply_limit_s: float | None = None,
                 runtime: str = "isaac", log_path: Path | None = None, smooth_depth: bool = False):
        self.host, self.port = host, int(port)
        #: the depth pictures smoothed (the WBC board; obs.smooth_depth)
        self.smooth_depth = bool(smooth_depth)
        self.on_episode = on_episode
        self.results_dir = None if results_dir is None else Path(results_dir)
        self.hold_s = hold_s
        self.reply_limit_s = reply_limit_s
        self.runtime = runtime
        self.log_path = log_path
        self._log = open(log_path, "a", encoding="utf-8") if log_path else None
        self._log_lock = threading.Lock()
        self.episodes: list[EpisodeLink] = []
        self.current: EpisodeLink | None = None
        self.ended = 0
        self.seq = 0
        self.connections = 0
        self.connected_once = threading.Event()
        self.disconnected = threading.Event()
        self.threads: list[threading.Thread] = []
        #: guards ``current`` / ``ended`` between the request thread and the results watcher
        self._lock = threading.RLock()
        self._pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="ws-request")
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Future | None = None
        self._ready = threading.Event()
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._run, name="ws-server", daemon=True)
        self._last_t: float | None = None
        self._stopping = False

    # -- lifetime ------------------------------------------------------------------------------
    def start(self) -> None:
        self._thread.start()
        if not self._ready.wait(30):
            raise RuntimeError("the WebSocket server did not start")
        if self._error is not None:
            raise RuntimeError(f"the WebSocket server failed: {self._error}")
        if self.results_dir is not None:
            threading.Thread(target=self._watch_results, name="results-watch", daemon=True).start()

    def stop(self) -> None:
        self._stopping = True
        if self._loop is not None and self._stop is not None:
            self._loop.call_soon_threadsafe(lambda: self._stop.done() or self._stop.set_result(None))
        self._thread.join(timeout=10)
        self._pool.shutdown(wait=False)
        if self._log is not None:
            with self._log_lock:
                self._log.close()
                self._log = None

    def _run(self) -> None:
        try:
            asyncio.run(self._serve())
        except BaseException as exc:  # noqa: BLE001 - reported by start()
            self._error = exc
            self._ready.set()

    async def _serve(self) -> None:
        import websockets

        self._loop = asyncio.get_running_loop()
        self._stop = self._loop.create_future()
        async with websockets.serve(self._handler, self.host, self.port, compression=None, max_size=None,
                                    ping_interval=20, ping_timeout=60):
            self._ready.set()
            await self._stop

    async def _handler(self, ws) -> None:
        import websockets

        self.connections += 1
        self.connected_once.set()
        self.disconnected.clear()
        self.log({"kind": "connected", "peer": str(getattr(ws, "remote_address", ""))})
        loop = asyncio.get_running_loop()
        try:
            async for message in ws:
                reply = await loop.run_in_executor(self._pool, self._request, message)
                await ws.send(reply)
        except websockets.ConnectionClosed as exc:
            self.log({"kind": "connection_closed", "code": getattr(exc, "code", None)})
        finally:
            self.connections -= 1
            self.log({"kind": "disconnected", "connections": self.connections})
            if self.connections == 0:
                ep = self.current
                if ep is not None and not ep.over:
                    # the run's last episode ends so (its entry follows); after a crash none comes
                    self._end(ep, None, "the connection closed")
                self.disconnected.set()

    # -- requests ------------------------------------------------------------------------------
    def _request(self, message: bytes) -> bytes:
        t_in = time.time()
        try:
            payload = msgpack.unpackb(message, raw=False)
        except Exception as exc:  # noqa: BLE001 - answered, logged
            self.log({"kind": "bad_request", "error": str(exc)[:200], "bytes": len(message)})
            return self._fallback()
        if not isinstance(payload, dict) or payload.get("method", "infer") != "infer":
            self.log({"kind": "other_request", "method": payload.get("method") if isinstance(payload, dict) else None})
            return msgpack.packb({"result": {}}, use_bin_type=True)
        self.seq += 1
        with self._lock:
            ep = self.current
            calibrations = ep.calibrations if ep is not None and not ep.over else None
        try:
            frame = O.parse(payload, self.seq, calibrations, smooth=self.smooth_depth, runtime=self.runtime)
        except O.ObsError as exc:
            self.log({"kind": "unreadable_request", "seq": self.seq, "error": str(exc)})
            return self._fallback()
        frame.bytes = len(message)
        first = False
        episode_done = False
        with self._lock:
            ep = self.current
            if frame.done:
                # the benchmark ends the episode: our reply is not a dispatch (it plays none of it)
                if ep is not None and not ep.over:
                    self._end(ep, frame, "episode_done")
                    rows, need, kind = [copy_row(ep.command)], False, "done"
                else:
                    rows, need, kind = [start_command(frame)], False, "done"
            else:
                if ep is not None and not ep.over:
                    why = boundary(ep, frame)
                    if why:
                        self._end(ep, None, why)
                if ep is None or ep.over:
                    ep = self._start(frame)
                    first = True
        gap = None if self._last_t is None else t_in - self._last_t
        captured = captured_s_ago(frame) if self.runtime in NATIVE else None
        spent = captured if captured is not None else arrival_s(gap) + (time.time() - t_in)
        if not frame.done:
            rows, need, kind = ep.serve(None if first else frame, spent_s=spent)
            episode_done = kind == "conclude"
        reply = pack(rows, need, episode_done=episode_done, runtime=self.runtime)
        t_out = time.time()
        self.log({"kind": "request", "seq": frame.seq, "episode": ep.index if ep is not None else None,
                  "gap_s": None if gap is None else round(gap, 3), "spent_s": round(spent, 3),
                  "spent_from": "capture" if captured is not None else "gap",
                  "wait_s": round(t_out - t_in, 3), "bytes": len(message), "depth": sorted(frame.depth),
                  "cams": sorted(frame.images), "done": frame.done, "reply": kind, "rows": len(rows),
                  "need_depth": need, "episode_done": episode_done,
                  "dispatches": None if ep is None else ep.stats["dispatches"]})
        self._last_t = t_out
        return reply

    def _fallback(self) -> bytes:
        """The reply to a request we cannot read: the held command (one row); an empty result only before
        any episode, when no command is known (the native runtimes then end the run)."""
        ep = self.current
        if ep is not None:
            return pack([copy_row(ep.command)], True, runtime=self.runtime)
        self.log({"kind": "empty_reply", "why": "no episode yet"})
        return msgpack.packb({"result": {}}, use_bin_type=True)

    def _end(self, ep: EpisodeLink, frame: O.Frame | None, how: str) -> None:
        """Episode ``ep`` is over: it is the run's next entry in the results; fetch its verdict."""
        with self._lock:
            if ep.over:
                return
            ep.end(frame, self.ended)
            self.ended += 1
        self.log({"kind": "episode_over", "episode": ep.index, "result_index": ep.result_index, "how": how,
                  "dispatches": ep.stats["dispatches"], "rows": ep.stats["rows"]})
        threading.Thread(target=self._poll_verdict, args=(ep,), name=f"verdict{ep.index}", daemon=True).start()

    def _watch_results(self) -> None:
        """End the current episode once the results file holds its entry (when no request said so)."""
        while not self._stopping:
            time.sleep(RESULTS_POLL_S)
            ep = self.current
            if ep is None or ep.over:
                continue
            if self._entries() > self.ended:
                self._end(ep, None, "the results file has its entry")

    def _result_files(self) -> list[Path]:
        """The run's result files: ``<root>/<scene>/<sub_task>/evaluate_ret_*.json`` (Newton keeps copies of
        its attempts deeper down)."""
        if self.results_dir is None or not self.results_dir.exists():
            return []
        return list(self.results_dir.glob("*/*/evaluate_ret_*.json"))

    def _entries(self) -> int:
        most = 0
        for path in self._result_files():
            try:
                most = max(most, len(json.loads(path.read_text()).get("details") or []))
            except (OSError, ValueError, AttributeError):
                continue  # being written
        return most

    def _start(self, frame: O.Frame) -> EpisodeLink:
        calibrations = O.Calibrations()
        for cam, calib in frame.calib.items():
            if not calib.carried:
                calibrations.seen(cam, calib, frame)
        ep = EpisodeLink(len(self.episodes), frame, hold_s=self.hold_s, calibrations=calibrations,
                         runtime=self.runtime, reply_limit_s=self.reply_limit_s)
        self.episodes.append(ep)
        self.current = ep
        self.log({"kind": "episode_start", "episode": ep.index, "seq": frame.seq, "task_name": frame.task_name,
                  "robot": frame.robot, "prompt": frame.prompt})
        t = threading.Thread(target=self._episode, args=(ep,), name=f"episode{ep.index}", daemon=True)
        self.threads.append(t)
        t.start()
        return ep

    def _episode(self, ep: EpisodeLink) -> None:
        try:
            self.on_episode(ep)
        except Exception as exc:  # noqa: BLE001 - the episode is ended by our concluding chunk
            self.log({"kind": "episode_failed", "episode": ep.index, "error": f"{type(exc).__name__}: {exc}"})
        finally:
            ep.conclude()

    def _poll_verdict(self, ep: EpisodeLink) -> None:
        wait_s = verdict_wait_s(self.runtime)
        deadline = time.monotonic() + wait_s
        k = ep.result_index
        while time.monotonic() < deadline:
            entry = self._entry(k)
            if entry is not None:
                v = verdict_of(entry)
                if v.get("instruction") not in (None, ep.prompt):
                    v["warning"] = "the evaluate_ret entry's instruction differs from the episode's"
                ep.set_verdict(v)
                self.log({"kind": "verdict", "episode": ep.index, "result_index": k, **v})
                return
            time.sleep(1.0)
        ep.set_verdict({"success": False, "progress": None, "missing": f"no evaluate_ret entry {k} after {wait_s:.0f} s"})
        self.log({"kind": "verdict_missing", "episode": ep.index, "result_index": k})

    def _entry(self, k: int) -> dict | None:
        files = sorted(self._result_files(), key=lambda p: p.stat().st_mtime)
        for path in reversed(files):
            try:
                data = json.loads(path.read_text())
            except (OSError, ValueError):
                continue  # being written
            details = data.get("details") or []
            if len(details) > k:
                return details[k]
        return None

    # -- records -------------------------------------------------------------------------------
    def log(self, record: dict) -> None:
        if self._log is None:
            return
        line = json.dumps({"t": round(time.time(), 3), **record}, ensure_ascii=False, default=str)
        with self._log_lock:
            if self._log is not None:
                self._log.write(line + "\n")
                self._log.flush()
