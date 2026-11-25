"""A stand-in for the platform's gateway of Season II, to test the online agent (:mod:`.tunnel`) without the
platform: in-process with made-up requests (tests/test_robocoliseum_online.py), or on a local simulator:

``python -m hexashell.benches.robocoliseum.fakegate --config g2op_crsemb_hang_cups_on_cup_tree --instance 0
--gpu 3 --state /tmp/x/state.env --run-dir /tmp/x/run`` then, in another shell,
``python -m hexashell.benches.robocoliseum.tunnel --state /tmp/x/state.env --out /tmp/x/out``.

It writes a state file for the agent (a made-up JOB_UUID and JOB_TOKEN, TUNNEL_ENDPOINT its own address,
JOB_BOARD v2, JOB_SEASON 2) and plays the gateway's side of the tunnel protocol (the appendix of the official
challenge-run-agent skill): the dial's checks (401 for a wrong token or job; an agent over ``--parallelism`` is
closed right after the upgrade, a second connection of a connected agent id too); ``warmup`` with an empty data
frame to answer, then ``ready`` awaited, then one ``queued``; a ping every 20 s, a tunnel closed after 45 s
without a pong; a 30 s window to redial after an unclean close, which restores the open case (a reply that
arrives on the new connection is relayed).  In-process, the test opens cases (:meth:`Gate.open_case`), asks
(:meth:`FakeCase.ask`), closes them and drains.  On a simulator, once the agent is ready it starts the run of
the case as :mod:`.run` does (:func:`.sim.launch`, the split guard of ``--part``) with the infer host its own
second port: the simulator's connection is a case (``session_open`` ... ``session_close``), every request goes
to the agent in a data frame and the reply comes back to the simulator; when the simulator exits, ``drain``.
The report (``fakegate.json`` in the run dir, and printed) has the cases, the requests and the longest answers,
and the simulator's own verdicts from its results file.

Switches (simulator mode): ``--drop-after N`` cuts the agent's connection (no close frame) right after relaying
the case's N-th request; ``--numpy`` packs the requests with msgpack_numpy (arrays as maps) instead of plain
msgpack; ``--strip-depth`` drops the depth pictures and their calibration from every request;
``--task-progress`` adds a dummy ``task_progress`` (the agent must drop it unread); ``--slow-mb-s R`` holds
every request back by its size at R MB/s before the agent gets it, inside the answer time (online in Season
I a 7 MB frame with depth took 60-90 s), so ``over_cap`` counts the answers that would have failed the case.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import secrets
import signal
import time
import uuid
from pathlib import Path
from typing import Callable
from urllib.parse import parse_qsl, urlsplit

import msgpack

from . import protocol as P
from .challenge import JOB_BOARD, JOB_KEYS, SEASON, write_env
from .tunnel import join_frame, split_frame

PING_S = 20.0
READ_DEADLINE_S = 45.0
RECONNECT_S = 30.0
#: what the platform's simulator waits for a reply
ANSWER_CAP_S = 120.0
PATH = "/api/challenge/tunnel"


class FakeCase:
    """One case of the gate: what a simulator connection is to the platform's gateway."""

    def __init__(self, gate: "Gate", sid: str, record: dict):
        self.gate, self.sid, self.record = gate, sid, record
        self.closed = False

    async def ask(self, payload: bytes, timeout: float = RECONNECT_S + ANSWER_CAP_S) -> bytes:
        """Relay one request to the agent and wait for its reply (the answer time is recorded)."""
        rec = self.record
        rec["requests"] += 1
        rec["bytes_max"] = max(rec["bytes_max"], len(payload))
        t0 = time.monotonic()
        await self.gate.to_agent(join_frame(self.sid, payload))
        reply = await asyncio.wait_for(self.gate.replies[self.sid].get(), timeout=timeout)
        waited = time.monotonic() - t0
        rec["longest_s"] = round(max(rec["longest_s"], waited), 2)
        rec["over_cap"] += int(waited > ANSWER_CAP_S)
        return reply

    async def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        rec = self.record
        rec["minutes"] = round((time.time() - rec["t0"]) / 60, 2)
        self.gate.replies.pop(self.sid, None)
        try:
            await self.gate.to_agent(json.dumps({"type": "session_close", "session_id": self.sid}))
        except Exception as exc:  # noqa: BLE001
            self.gate.note("session_close_not_sent", error=str(exc)[:100])
        self.gate.open_cases -= 1
        self.gate.note("session_close", **{k: v for k, v in rec.items() if k != "t0"})


class Gate:
    """The gateway's side of the tunnel for one made-up job (``parallelism`` agents at most)."""

    def __init__(self, *, parallelism: int = 1, adapt: Callable[[bytes], bytes] | None = None,
                 drop_after: int = 0, slow_mb_s: float = 0.0, quiet: bool = False):
        self.parallelism = int(parallelism)
        self.adapt = adapt or (lambda message: message)
        self.drop_after = int(drop_after)
        self.slow_mb_s = float(slow_mb_s)
        self.quiet = quiet
        self.job = str(uuid.uuid4())
        self.token = secrets.token_urlsafe(24)
        self.job_id = 900000 + secrets.randbelow(99999)
        self.agents: dict[str, dict] = {}  # agent id -> {"ws", "ready", "lost_at", "last_in"}
        self.ready = asyncio.Event()
        self.replies: dict[str, asyncio.Queue] = {}
        self.cases: list[dict] = []
        self.open_cases = 0
        self.events: list[dict] = []
        self.reconnects = 0
        self.lost_case = False
        self.drained = False
        self.done = asyncio.Event()
        self.port: int | None = None
        self._server = None
        self._keep: asyncio.Task | None = None

    # -- records -------------------------------------------------------------------------------
    def note(self, kind: str, **fields) -> None:
        rec = {"t": round(time.time(), 3), "kind": kind, **fields}
        self.events.append(rec)
        if not self.quiet:
            print(time.strftime("%H:%M:%S"), "[gate]", kind, json.dumps(fields, default=str)[:300], flush=True)

    @property
    def endpoint(self) -> str:
        return f"ws://127.0.0.1:{self.port}{PATH}"

    def state_values(self) -> dict[str, str]:
        return {"JOB_ID": str(self.job_id), "JOB_UUID": self.job, "JOB_TOKEN": self.token,
                "PARALLELISM": str(self.parallelism), "TUNNEL_ENDPOINT": self.endpoint, "JOB_BOARD": JOB_BOARD,
                "JOB_SEASON": SEASON}

    def write_state(self, path: Path) -> Path:
        """The agent's state file (mode 600)."""
        path = Path(path)
        write_env(path, self.state_values(), drop=(*JOB_KEYS, "CHALLENGE_TOKEN"))
        return path

    # -- lifetime ------------------------------------------------------------------------------
    async def start(self, port: int = 0) -> int:
        from websockets.asyncio.server import serve

        self._server = await serve(self.on_agent, "127.0.0.1", int(port), process_request=self.check, max_size=None,
                                   compression=None, ping_interval=None)
        self.port = int(next(iter(self._server.sockets)).getsockname()[1])
        self._keep = asyncio.get_running_loop().create_task(self.keepalive())
        self.note("listening", agent_port=self.port)
        return self.port

    async def stop(self) -> None:
        if self._keep is not None:
            self._keep.cancel()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()

    # -- the agent's side ----------------------------------------------------------------------
    def check(self, connection, request):
        from http import HTTPStatus

        parts = urlsplit(request.path)
        q = dict(parse_qsl(parts.query))
        auth = request.headers.get("Authorization", "")
        if parts.path != PATH or auth != f"Bearer {self.token}" or q.get("job") != self.job or not q.get("agent"):
            self.note("dial_refused", path_ok=parts.path == PATH, job_ok=q.get("job") == self.job,
                      token_ok=auth == f"Bearer {self.token}", agent=bool(q.get("agent")))
            return connection.respond(HTTPStatus.UNAUTHORIZED, "Unauthorized\n")
        return None

    def agent_conn(self) -> dict | None:
        live = [a for a in self.agents.values() if a.get("ws") is not None]
        return live[0] if live else None

    async def on_agent(self, ws) -> None:
        q = dict(parse_qsl(urlsplit(ws.request.path).query))
        aid = q["agent"]
        known = self.agents.get(aid)
        if known is not None and known.get("ws") is not None:
            self.note("duplicate_agent_closed")
            await ws.close(1008, "duplicate agent")
            return
        live = [a for k, a in self.agents.items() if k != aid and a.get("ws") is not None]
        if len(live) >= self.parallelism:
            self.note("over_parallelism_closed")
            await ws.close(1008, "parallelism")
            return
        restored = known is not None and known.get("lost_at") is not None
        if restored:
            gap = time.monotonic() - known["lost_at"]
            if gap > RECONNECT_S:
                self.note("redial_after_window", gap_s=round(gap, 1))
                restored = False
            else:
                self.reconnects += 1
                self.note("restored", gap_s=round(gap, 1))
        entry = known if restored else {"ready": False}
        entry.update({"ws": ws, "lost_at": None, "last_in": time.monotonic()})
        self.agents[aid] = entry
        clean = False
        try:
            if not restored and not self.drained:
                self.replies["warmup"] = asyncio.Queue()
                await ws.send(json.dumps({"type": "warmup"}))
                await ws.send(join_frame("warmup", b""))
            async for message in ws:
                entry["last_in"] = time.monotonic()
                if isinstance(message, str):
                    ctl = json.loads(message)
                    self.note("agent_control", type=ctl.get("type"))
                    if ctl.get("type") == "ready" and not entry["ready"]:
                        entry["ready"] = True
                        await ws.send(json.dumps({"type": "queued", "reason": "no_gpu_available"}))
                        self.ready.set()
                    continue
                sid, payload = split_frame(message)
                box = self.replies.get(sid)
                if box is None:
                    self.note("reply_for_unknown_session", bytes=len(payload))
                    continue
                await box.put(payload)
            clean = ws.close_code in (1000, 1001)
        except Exception as exc:  # noqa: BLE001 - an unclean end
            self.note("agent_connection_error", error=type(exc).__name__)
        finally:
            if entry.get("ws") is ws:
                entry["ws"] = None
                entry["lost_at"] = None if (clean or self.drained) else time.monotonic()
            self.note("agent_closed", code=ws.close_code, clean=clean, drained=self.drained)
            if self.drained and self.agent_conn() is None:
                self.done.set()

    async def warm_check(self, timeout: float = 60.0) -> bytes:
        """The agent's answer to the warmup data frame."""
        box = self.replies.get("warmup")
        end = time.monotonic() + timeout
        while box is None and time.monotonic() < end:
            await asyncio.sleep(0.1)
            box = self.replies.get("warmup")
        if box is None:
            raise TimeoutError("no warmup sent")
        reply = await asyncio.wait_for(box.get(), timeout=max(0.1, end - time.monotonic()))
        self.note("warmup_reply", bytes=len(reply), decoded=str(msgpack.unpackb(reply, raw=False))[:80])
        return reply

    async def to_agent(self, data) -> None:
        """Send to the agent, waiting out a reconnect window."""
        end = time.monotonic() + RECONNECT_S + 5
        while time.monotonic() < end:
            a = self.agent_conn()
            if a is not None:
                try:
                    await a["ws"].send(data)
                    return
                except Exception:  # noqa: BLE001 - the connection went: wait for the next
                    pass
            await asyncio.sleep(0.1)
        raise ConnectionError("no agent connection within the reconnect window")

    def cut(self) -> bool:
        """Cut the agent's connection without a close frame (an unclean end: the agent redials).  It counts as
        lost at once, so what is sent meanwhile waits for the redial (the reconnect window) instead of going into
        the dead connection."""
        a = self.agent_conn()
        if a is None:
            return False
        self.note("cutting_agent_connection")
        ws = a["ws"]
        a["ws"], a["lost_at"] = None, time.monotonic()
        ws.transport.abort()
        return True

    async def open_case(self) -> FakeCase:
        sid = uuid.uuid4().hex
        record = {"case": len(self.cases), "requests": 0, "longest_s": 0.0, "over_cap": 0, "t0": time.time(),
                  "bytes_max": 0}
        self.cases.append(record)
        self.replies[sid] = asyncio.Queue()
        self.open_cases += 1
        await self.to_agent(json.dumps({"type": "session_open", "session_id": sid}))
        self.note("session_open", case=record["case"])
        return FakeCase(self, sid, record)

    async def drain(self, timeout: float = 600.0) -> bool:
        """No more cases: ``drain`` to every agent of the job, as the platform does; True when all closed."""
        self.drained = True
        live = [a for a in self.agents.values() if a.get("ws") is not None]
        for a in live:
            try:
                await a["ws"].send(json.dumps({"type": "drain"}))
            except Exception as exc:  # noqa: BLE001 - it is going anyway
                self.note("drain_send_failed", error=type(exc).__name__)
        if not live:
            self.done.set()
            return True
        try:
            await asyncio.wait_for(self.done.wait(), timeout=timeout)
            return True
        except asyncio.TimeoutError:
            self.note("agent_did_not_close_after_drain")
            return False

    async def keepalive(self) -> None:
        """Ping every agent connection every 20 s; close one silent for 45 s (the gateway's read deadline)."""
        while not self.done.is_set():
            await asyncio.sleep(PING_S)
            for a in list(self.agents.values()):
                ws = a.get("ws")
                if ws is None:
                    continue
                try:
                    pong = await ws.ping()
                    asyncio.get_running_loop().create_task(self._pong_seen(a, pong))
                except Exception:  # noqa: BLE001
                    continue
                if time.monotonic() - a["last_in"] > READ_DEADLINE_S:
                    self.note("read_deadline_close")
                    ws.transport.abort()

    @staticmethod
    async def _pong_seen(a: dict, pong) -> None:
        try:
            await asyncio.wait_for(pong, timeout=READ_DEADLINE_S)
            a["last_in"] = time.monotonic()
        except Exception:  # noqa: BLE001
            pass

    # -- the simulator's side (simulator mode) -------------------------------------------------------
    async def on_sim(self, ws) -> None:
        await self.ready.wait()
        case = await self.open_case()
        try:
            async for message in ws:
                payload = self.adapt(message)
                if self.slow_mb_s > 0:  # the download, inside the simulator's limit as online
                    await asyncio.sleep(len(payload) / (self.slow_mb_s * 1e6))
                n = case.record["requests"] + 1
                if self.drop_after and case.record["case"] == 0 and n == self.drop_after:
                    asyncio.get_running_loop().call_later(0.05, self.cut)
                try:
                    reply = await case.ask(payload)
                except asyncio.TimeoutError:
                    self.note("no_reply", case=case.record["case"], request=case.record["requests"])
                    self.lost_case = True
                    await ws.close(1011, "no reply")
                    break
                await ws.send(reply)
        except Exception as exc:  # noqa: BLE001
            self.note("sim_connection_error", error=f"{type(exc).__name__}: {exc}"[:200])
        finally:
            await case.close()


def adapter(*, numpy: bool = False, strip_depth: bool = False, task_progress: bool = False):
    """The simulator mode's change of each request (None: none)."""
    if not (numpy or strip_depth or task_progress):
        return None

    def adapt(message: bytes) -> bytes:
        data = msgpack.unpackb(message, raw=False)
        p = data.get("params") or {}
        if strip_depth:
            images = p.get("images") or {}
            for k in [k for k in images if k.endswith("_depth")]:
                del images[k]
            for entry in images.values():
                if isinstance(entry, dict):
                    entry.pop("calibration", None)
        if task_progress:
            p["task_progress"] = [{"dummy": "fakegate"}]
        if numpy:
            import msgpack_numpy
            import numpy as np

            st = p.get("states") or {}
            for k, v in list(st.items()):
                if isinstance(v, list) and v and all(isinstance(x, (int, float)) for x in v):
                    st[k] = np.asarray(v, np.float64)
            return msgpack_numpy.packb(data, use_bin_type=True)
        return msgpack.packb(data, use_bin_type=True)

    return adapt


async def amain(args) -> dict:
    from websockets.asyncio.server import serve

    from . import sim
    from .link import verdict_of
    from .run import COLD_START_S, START_S, exit_grace_s, free_port, stop_run, take_domain

    t = P.task(args.config)
    case = P.case_of(t.config, args.instance, args.part, args.seed)
    gate = Gate(parallelism=args.parallelism, drop_after=args.drop_after, slow_mb_s=args.slow_mb_s,
                adapt=adapter(numpy=args.numpy, strip_depth=args.strip_depth, task_progress=args.task_progress))
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    await gate.start(args.agent_port)
    state = gate.write_state(Path(args.state))
    sim_port = free_port()
    sim_server = await serve(gate.on_sim, "127.0.0.1", sim_port, max_size=None, compression=None,
                             ping_interval=20, ping_timeout=60)
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, asyncio.current_task().cancel)  # TERM: stop the sim, then go
    gate.note("listening_sim", sim_port=sim_port, state=str(state))
    await gate.ready.wait()
    await gate.warm_check()
    run_id = f"{case.name}_fakegate_{time.strftime('%Y%m%d-%H%M%S')}"
    name = sim.container_for(args.gpu)
    await loop.run_in_executor(None, sim.ensure, name, args.gpu, run_dir / "container.log")
    start_s = COLD_START_S if sim.cold() or t.runtime == "newton" else START_S
    domain, lock_fd = take_domain()
    proc = sim.launch(name, case, sim_port, run_id, domain, run_dir / "bench.log", watchdog_s=sim.WATCHDOG_S)
    gate.note("sim_started", config=t.config, case=case.key, runtime=t.runtime, gpu=args.gpu, run_id=run_id)
    t0 = time.monotonic()
    end = "sim_exit"
    quiet_since = None
    try:
        while proc.poll() is None:
            now = time.monotonic()
            if not gate.cases and now - t0 > start_s:
                end = "no_connection"
                break
            if now - t0 > args.max_h * 3600:
                end = "sim_cap"
                break
            done = bool(gate.cases) and gate.open_cases == 0
            quiet_since = (quiet_since or now) if done else None
            if quiet_since is not None and now - quiet_since > exit_grace_s(t.runtime):
                end = "sim_exit_hung"
                break
            await asyncio.sleep(2.0)
    finally:
        rc = await loop.run_in_executor(None, stop_run, proc, name, run_id,
                                        lambda r: gate.note(str(r.get("kind") or "sim"),
                                                            **{k: v for k, v in r.items() if k != "kind"}))
        os.close(lock_fd)
    gate.note("sim_end", end=end, returncode=rc, minutes=round((time.monotonic() - t0) / 60, 1))
    await gate.drain()
    await gate.stop()
    sim_server.close()
    verdicts = []
    for path in sim.results(run_id, case):
        try:
            verdicts += [verdict_of(e) for e in json.loads(path.read_text()).get("details") or []]
        except (OSError, ValueError):
            continue
    report = {"config": t.config, "case": case.key, "runtime": t.runtime, "run_id": run_id, "end": end,
              "sim_returncode": rc, "cases": [{k: v for k, v in c.items() if k != "t0"} for c in gate.cases],
              "reconnects": gate.reconnects, "lost_case": gate.lost_case,
              "verdicts": [{k: v.get(k) for k in ("success", "score", "progress", "instruction", "dispatches", "valid")}
                           for v in verdicts],
              "events": [e for e in gate.events if e["kind"] not in ("agent_control",)][-60:]}
    (run_dir / "fakegate.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str) + "\n")
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum.fakegate",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True, help="a Season II config (protocol.TASKS)")
    ap.add_argument("--instance", type=int, required=True)
    ap.add_argument("--part", choices=P.PARTS, default="learn", help="the split part the instance must belong to")
    ap.add_argument("--seed", type=int, default=None, help="our own wok_flip layout or Rubik's cube scene order")
    ap.add_argument("--gpu", default="0")
    ap.add_argument("--agent-port", type=int, default=0, help="the gate's port for the agent (0: a free one)")
    ap.add_argument("--state", required=True, help="where to write the agent's state file (made-up credentials)")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--parallelism", type=int, default=1)
    ap.add_argument("--drop-after", type=int, default=0)
    ap.add_argument("--numpy", action="store_true")
    ap.add_argument("--strip-depth", action="store_true")
    ap.add_argument("--task-progress", action="store_true")
    ap.add_argument("--slow-mb-s", type=float, default=0.0,
                    help="delay every request by its size at this many MB/s (a slow download), counted in its answer")
    ap.add_argument("--max-h", type=float, default=3.0)
    args = ap.parse_args(argv)
    try:
        P.case_of(args.config, args.instance, args.part, args.seed)
    except P.TaskError as exc:
        ap.error(str(exc))
    from .challenge import STATE, job_file

    if Path(args.state).expanduser().resolve() in (STATE.resolve(), job_file().resolve()):
        ap.error(f"--state {args.state} is a real job's state file: give a new path for the made-up one")
    try:
        report = asyncio.run(amain(args))
    except asyncio.CancelledError:
        print(json.dumps({"stopped": "SIGTERM"}))
        return 1
    print(json.dumps({k: v for k, v in report.items() if k != "events"}, ensure_ascii=False, default=str))
    return 0 if report["verdicts"] and not report["lost_case"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
