"""The wire to the benchmark: the WebSocket server its policy client dials, and the hand-over between its
requests and the thread of the episode they belong to.

The benchmark (``corobotpolicy.py`` of genie_sim v3.5.0) connects once per run and sends one msgpack
request per step it needs actions for (``{"method": "infer", "params": {...}}``: the pictures, the
readings, the instruction); the reply (``{"result": {...}}``) holds rows of joint targets for both arms
and both grippers (and the waist, once we command it), which it plays one row per step; it asks again
when they are used up.  ``need_depth`` in a reply adds the depth pictures and the calibration to the
next request.

Episodes: the benchmark plays a run's episodes one after the other on the same connection.  When it
ends one (all sub-steps met, or a sub-step out of steps) it acts once more and resets; that last act
sends a request with ``episode_done`` true only when the rows of our last reply are used up (else it
plays a row from them, and the rest are dropped).  So an episode is over at whichever comes first: a
request with ``episode_done``; a request whose instruction or ``episode_idx`` differs from the episode's
(the next episode's first; ``episode_idx`` is the index of the episode before, so it changes from the
third episode of a run on); the results file holding one more entry; the connection closing.  We never
send ``episode_done`` ourselves (sent while the checkers run, it leaves every later episode of the run
unscored): an episode our side is done with is held still until the benchmark ends it.

While the model thinks there is no action.  The online platform waits at most 120 s for a reply, so a
request still unanswered after ``hold_s`` gets one row that holds everything where it was last sent
(one step); the next request then comes at once.  The limit counts the request's download too (a frame
with depth is about 7 MB; online it has taken up to 90 s), so the hold row comes earlier when the request
was long on the way: by ``REPLY_MARGIN_S`` before the limit, counting the time since our last reply.

Verdicts: after an episode ends, the benchmark adds it to ``details`` of ``evaluate_ret_*.json`` in its
output folder; the k-th episode the run ended is ``details[k]``.
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
from . import g2
from . import obs as O

#: seconds a request waits for the episode's next action before it is answered with a hold row
HOLD_S = 90.0
#: the online simulator's limit on one reply, counted from when it sends its request: the download of
#: the request (seconds with depth on a slow link) is part of it
SIM_REPLY_S = 120.0
#: the hold row goes this long before the simulator's limit (our reply's upload, clock slack)
REPLY_MARGIN_S = 15.0
#: the shortest wait before a hold row, however late the request came
MIN_HOLD_S = 2.0
#: the request's time on the way when there is no earlier reply to count from (a case's first request)
UNKNOWN_ARRIVAL_S = 90.0
#: rows per reply while an episode we are done with is held until the benchmark ends it
CONCLUDE_ROWS = 30
#: seconds to wait for an ended episode's entry in evaluate_ret
VERDICT_S = 300.0
#: how often the results file is read for an episode the requests did not show the end of
RESULTS_POLL_S = 2.0
#: a gripper reading above this at an episode's start is closed on something (the scoop of
#: scoop_popcorn): it is then held closed, so it keeps its grip
HELD_AT_START = -0.45
#: the gripper command that closes the fingers
CLOSED = 0.0


def start_command(frame: O.Frame) -> dict:
    """The command that holds the robot as it is at the start of an episode."""
    grip = {}
    for arm in g2.ARMS:
        g = frame.gripper(arm)
        grip[arm] = CLOSED if g > HELD_AT_START else g
    return {"left": frame.arm_joints("left"), "right": frame.arm_joints("right"), "grip": grip, "waist": None}


def copy_row(row: dict) -> dict:
    return {"left": np.array(row["left"], float), "right": np.array(row["right"], float),
            "grip": dict(row["grip"]), "waist": None if row.get("waist") is None else np.array(row["waist"], float)}


def pack(rows: list[dict], need_depth: bool) -> bytes:
    out: dict[str, Any] = {
        "left_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["left"]] for r in rows]},
        "right_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["right"]] for r in rows]},
        "left_effector": [[float(r["grip"]["left"])] for r in rows],
        "right_effector": [[float(r["grip"]["right"])] for r in rows],
        "need_depth": bool(need_depth),
    }
    if rows and all(r.get("waist") is not None for r in rows):
        out["waist"] = {"kind": "JOINT_ABS", "values": [[float(v) for v in r["waist"]] for r in rows]}
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
    """Our verdict from one ``details`` entry of evaluate_ret: progress = mean of the sub-step scores,
    success = E2E 1."""
    res = entry.get("result") or {}
    scores = res.get("scores") or {}
    steps = [{"name": s.get("name"), "score": float(s.get("score") or 0.0)} for s in scores.get("STEPS") or []]
    progress = sum(s["score"] for s in steps) / len(steps) if steps else None
    e2e = scores.get("E2E")
    return {"success": e2e is not None and float(e2e) >= 1.0, "progress": None if progress is None else round(progress, 4),
            "e2e": e2e, "sub_steps": steps, "stage": res.get("step"), "code": res.get("code"), "msg": res.get("msg"),
            "instruction": entry.get("task_instruction"), "dispatches": entry.get("policy_dispatch_count"),
            "duration_s": entry.get("duration")}


def hold_after(hold_s: float, spent_s: float) -> float:
    """Seconds a request waits for an action before the hold row: ``hold_s``, cut so that the reply
    reaches the simulator ``REPLY_MARGIN_S`` before its limit when the request took ``spent_s`` to come."""
    return max(MIN_HOLD_S, min(float(hold_s), SIM_REPLY_S - REPLY_MARGIN_S - max(0.0, float(spent_s))))


def arrival_s(gap_s: float | None) -> float:
    """An upper bound on a request's time on the way: the time since our last reply (the simulator's step
    and the download), or ``UNKNOWN_ARRIVAL_S`` without one."""
    return UNKNOWN_ARRIVAL_S if gap_s is None else max(0.0, gap_s)


class EpisodeLink:
    """One episode's side of the wire: the latest request's frame, the rows the episode thread wants sent,
    and how the episode ended."""

    def __init__(self, index: int, first: O.Frame, *, hold_s: float = HOLD_S):
        self.index = index
        self.prompt = first.prompt
        self.task_name = first.task_name
        self.hold_s = float(hold_s)
        #: whether a hold row asks for the depth pictures (online: no, see G2Driver.motion_depth)
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
        self._verdict: dict | None = None
        #: the k-th episode this run's benchmark ended (its entry in evaluate_ret), once it ended
        self.result_index: int | None = None
        self.stats = {"requests": 1, "replies": 0, "rows": 0, "hold_replies": 0, "conclude_rows": 0,
                      "depth_requests": int(bool(first.depth)), "longest_wait_s": 0.0}
        self.t0 = time.time()

    # -- the server's side ---------------------------------------------------------------------
    def serve(self, frame: O.Frame | None, spent_s: float = 0.0) -> tuple[list[dict], bool, str]:
        """Answer a request (``frame``; None: the episode's first, already given): the rows to send,
        need_depth, and what they are ('action', 'hold', 'conclude').  ``spent_s``: how much of the
        simulator's ``SIM_REPLY_S`` the request may already have used on its way here; the hold row then
        goes before that limit even when ``hold_s`` is longer."""
        t0 = time.monotonic()
        with self._cv:
            if frame is not None:
                self._frame = frame
                self._seq = frame.seq
                self.stats["requests"] += 1
                self.stats["depth_requests"] += int(bool(frame.depth))
                self._cv.notify_all()
            deadline = t0 + hold_after(self.hold_s, spent_s)
            while True:
                if self._pending is not None and self._taken is None and not (self.ended or self.abandoned):
                    rows, need = self._pending
                    self._pending = None
                    self._taken = self._seq
                    self._sent(rows)
                    kind = "action"
                    break
                if self.concluding or self.ended or self.abandoned:
                    rows, need, kind = [copy_row(self.command)] * CONCLUDE_ROWS, False, "conclude"
                    self.stats["conclude_rows"] += len(rows)
                    self._sent(rows)
                    break
                left = deadline - time.monotonic()
                if left <= 0:
                    rows, need, kind = [copy_row(self.command)], self.hold_depth, "hold"
                    self.stats["hold_replies"] += 1
                    self._sent(rows)
                    break
                self._cv.wait(min(left, 1.0))
            waited = time.monotonic() - t0
            self.stats["longest_wait_s"] = round(max(self.stats["longest_wait_s"], waited), 2)
            return rows, need, kind

    def _sent(self, rows: list[dict]) -> None:
        self.command = copy_row(rows[-1])
        self.stats["replies"] += 1
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

    def latest(self) -> O.Frame:
        with self._cv:
            return self._frame

    def act(self, rows: list[dict], need_depth: bool) -> O.Frame:
        """Send rows (each a full command: both arms, both grippers, the waist or None); block until the
        request after them arrives and return its frame.  Raises EpisodeOver once the episode is over."""
        if not rows:
            raise ValueError("no rows to send")
        with self._cv:
            self._raise_if_over()
            self._pending = ([copy_row(r) for r in rows], bool(need_depth))
            self._taken = None
            self._cv.notify_all()
            while self._taken is None or self._seq <= self._taken:
                self._raise_if_over()
                self._cv.wait(1.0)
            return self._frame

    def conclude(self) -> None:
        """Nothing more to send: hold still until the benchmark ends the episode."""
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
                 results_dir: Path | None = None, hold_s: float = HOLD_S, log_path: Path | None = None):
        self.host, self.port = host, int(port)
        self.on_episode = on_episode
        self.results_dir = None if results_dir is None else Path(results_dir)
        self.hold_s = hold_s
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
        try:
            frame = O.parse(payload, self.seq)
        except O.ObsError as exc:
            self.log({"kind": "unreadable_request", "seq": self.seq, "error": str(exc)})
            return self._fallback()
        frame.bytes = len(message)
        first = False
        with self._lock:
            ep = self.current
            if frame.done:
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
        spent = arrival_s(gap) + (time.time() - t_in)
        if not frame.done:
            rows, need, kind = ep.serve(None if first else frame, spent_s=spent)
        reply = pack(rows, need)
        t_out = time.time()
        self.log({"kind": "request", "seq": frame.seq, "episode": ep.index if ep is not None else None,
                  "gap_s": None if gap is None else round(gap, 3), "spent_s": round(spent, 3),
                  "wait_s": round(t_out - t_in, 3), "bytes": len(message), "depth": sorted(frame.depth),
                  "cams": sorted(frame.images), "done": frame.done, "reply": kind, "rows": len(rows),
                  "need_depth": need})
        self._last_t = t_out
        return reply

    def _fallback(self) -> bytes:
        ep = self.current
        if ep is not None:
            return pack([copy_row(ep.command)], True)
        return msgpack.packb({"result": {}}, use_bin_type=True)

    def _end(self, ep: EpisodeLink, frame: O.Frame | None, how: str) -> None:
        """Episode ``ep`` is over: it is the run's next entry in the results; fetch its verdict."""
        with self._lock:
            if ep.over:
                return
            ep.end(frame, self.ended)
            self.ended += 1
        self.log({"kind": "episode_over", "episode": ep.index, "result_index": ep.result_index, "how": how})
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

    def _entries(self) -> int:
        if self.results_dir is None or not self.results_dir.exists():
            return 0
        most = 0
        for path in self.results_dir.rglob("evaluate_ret_*.json"):
            try:
                most = max(most, len(json.loads(path.read_text()).get("details") or []))
            except (OSError, ValueError, AttributeError):
                continue  # being written
        return most

    def _start(self, frame: O.Frame) -> EpisodeLink:
        ep = EpisodeLink(len(self.episodes), frame, hold_s=self.hold_s)
        self.episodes.append(ep)
        self.current = ep
        self.log({"kind": "episode_start", "episode": ep.index, "seq": frame.seq, "task_name": frame.task_name,
                  "prompt": frame.prompt})
        t = threading.Thread(target=self._episode, args=(ep,), name=f"episode{ep.index}", daemon=True)
        self.threads.append(t)
        t.start()
        return ep

    def _episode(self, ep: EpisodeLink) -> None:
        try:
            self.on_episode(ep)
        except Exception as exc:  # noqa: BLE001 - the episode is held still until the benchmark ends it
            self.log({"kind": "episode_failed", "episode": ep.index, "error": f"{type(exc).__name__}: {exc}"})
        finally:
            ep.conclude()

    def _poll_verdict(self, ep: EpisodeLink) -> None:
        deadline = time.monotonic() + VERDICT_S
        k = ep.result_index
        while time.monotonic() < deadline:
            entry = self._entry(k, ep.prompt)
            if entry is not None:
                v = verdict_of(entry)
                if v.get("instruction") not in (None, ep.prompt):
                    v["warning"] = "the evaluate_ret entry's instruction differs from the episode's"
                ep.set_verdict(v)
                self.log({"kind": "verdict", "episode": ep.index, "result_index": k, **v})
                return
            time.sleep(1.0)
        ep.set_verdict({"success": False, "progress": None, "missing": f"no evaluate_ret entry {k} after {VERDICT_S:.0f} s"})
        self.log({"kind": "verdict_missing", "episode": ep.index, "result_index": k})

    def _entry(self, k: int, prompt: str) -> dict | None:
        if self.results_dir is None or not self.results_dir.exists():
            return None
        files = sorted(self.results_dir.rglob("evaluate_ret_*.json"), key=lambda p: p.stat().st_mtime)
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
