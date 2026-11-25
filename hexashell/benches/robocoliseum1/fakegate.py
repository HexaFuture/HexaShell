"""A stand-in for the challenge's gateway, to test the online agent (:mod:`.tunnel`) on a local simulator.

``python -m hexashell.benches.robocoliseum1.fakegate --config g2op_if_pick_block_color --layout 0 --gpu 3
--state /tmp/x/state.env --run-dir /tmp/x/run`` then, in another shell,
``python -m hexashell.benches.robocoliseum1.tunnel --board instruction --state /tmp/x/state.env --out /tmp/x/out``.

It writes a state file for the agent (a made-up JOB_UUID and JOB_TOKEN, TUNNEL_ENDPOINT its own address) and
plays the gateway's side of the tunnel protocol (the appendix of the official challenge-run-agent skill): the
dial's checks (401 for a wrong token or job; a second agent over ``--parallelism`` is closed right after the
upgrade, a second connection of a connected agent id too); ``warmup`` with an empty data frame to answer, then
``ready`` awaited, then one ``queued``; a ping every 20 s, a tunnel closed after 45 s without a pong; a 30 s
window to redial after an unclean close, which restores the open case (a reply that arrives on the new
connection is relayed).  Once the agent is ready it starts the simulator on the layout (:func:`.sim.launch`, as
:mod:`.run` does, seed and part as there) with ``infer_host`` its own second port: every connection of the
simulator is a case (``session_open`` ... ``session_close``), every request goes to the agent in a data frame
and the reply comes back to the simulator; when the simulator exits, ``drain``.  The report (``fakegate.json``
in the run dir, and printed) has the cases, the requests and the longest answers, and the simulator's own
verdicts from its results file.

Test switches: ``--drop-after N`` cuts the agent's connection (no close frame) right after relaying the case's
N-th request; ``--numpy`` packs the requests with msgpack_numpy (arrays as maps) instead of plain msgpack;
``--strip-depth`` drops the depth pictures and their calibration from every request; ``--like-online`` makes
every request look like the online simulator's (before v3.5.0): no calibration and no ``snapshot_id``,
no ``segments``, ``head_joint_states`` empty, a ``task_progress`` field (a dummy: the agent must drop it
unread), and the hand cameras at 1280x1056 (pictures and depth doubled); ``--slow-mb-s R`` holds every
request back by its size at R MB/s before the agent gets it, inside the answer time (online a 7 MB
frame with depth took 60-90 s), so ``over_cap`` counts the answers that would have failed the case.
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
from urllib.parse import parse_qsl, urlsplit

import msgpack

from . import protocol as P
from . import sim
from .link import verdict_of
from .tunnel import join_frame, split_frame

PING_S = 20.0
READ_DEADLINE_S = 45.0
RECONNECT_S = 30.0
#: what the platform's simulator waits for a reply
ANSWER_CAP_S = 120.0
PATH = "/api/challenge/tunnel"


def like_online(p: dict) -> None:
    """A request's params as the online simulator sends them (in place)."""
    import cv2
    import numpy as np

    images = p.get("images") or {}
    for name, entry in images.items():
        if not isinstance(entry, dict):
            continue
        entry.pop("calibration", None)
        entry.pop("snapshot_id", None)
        if not name.startswith("hand_") or not entry.get("image_data"):
            continue
        w, h = int(entry.get("width") or 0), int(entry.get("height") or 0)
        if (w, h) != (640, 528):
            continue
        if str(entry.get("encoding")).upper() == "RAW_UINT16":
            d = np.frombuffer(entry["image_data"], np.uint16).reshape(h, w)
            entry["image_data"] = np.ascontiguousarray(d.repeat(2, 0).repeat(2, 1)).tobytes()
        else:
            img = cv2.imdecode(np.frombuffer(entry["image_data"], np.uint8), cv2.IMREAD_COLOR)
            img = cv2.resize(img, (2 * w, 2 * h), interpolation=cv2.INTER_LINEAR)
            entry["image_data"] = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
        entry["width"], entry["height"] = 2 * w, 2 * h
    p.pop("segments", None)
    states = p.get("states")
    if isinstance(states, dict) and "head_joint_states" in states:
        states["head_joint_states"] = []
    p["task_progress"] = [{"dummy": "fakegate"}]


class Gate:
    def __init__(self, args):
        self.args = args
        self.job = str(uuid.uuid4())
        self.token = secrets.token_urlsafe(24)
        self.agents: dict[str, dict] = {}  # agent id -> {"ws", "ready", "since"}
        self.ready = asyncio.Event()
        self.replies: dict[str, asyncio.Queue] = {}
        self.cases: list[dict] = []
        self.events: list[dict] = []
        self.reconnects = 0
        self.lost_case = False
        self.drained = False
        self.done = asyncio.Event()

    # -- records -------------------------------------------------------------------------------
    def note(self, kind: str, **fields) -> None:
        rec = {"t": round(time.time(), 3), "kind": kind, **fields}
        self.events.append(rec)
        print(time.strftime("%H:%M:%S"), "[gate]", kind, json.dumps(fields, default=str)[:300], flush=True)

    # -- the agent's side ----------------------------------------------------------------------
    def check(self, connection, request):
        from http import HTTPStatus

        parts = urlsplit(request.path)
        q = dict(parse_qsl(parts.query))
        auth = request.headers.get("Authorization", "")
        if parts.path != PATH or auth != f"Bearer {self.token}" or q.get("job") != self.job or not q.get("agent"):
            self.note("dial_refused", path=parts.path, job_ok=q.get("job") == self.job,
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
        if len(live) >= self.args.parallelism:
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
                await ws.send(json.dumps({"type": "warmup"}))
                self.replies["warmup"] = asyncio.Queue()
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
            self.note("agent_connection_error", error=f"{type(exc).__name__}: {exc}"[:200])
        finally:
            if entry.get("ws") is ws:
                entry["ws"] = None
                entry["lost_at"] = None if (clean or self.drained) else time.monotonic()
            self.note("agent_closed", code=ws.close_code, clean=clean, drained=self.drained)
            if self.drained and self.agent_conn() is None:
                self.done.set()

    async def warm_check(self) -> None:
        box = self.replies.get("warmup")
        while box is None:
            await asyncio.sleep(0.2)
            box = self.replies.get("warmup")
        reply = await asyncio.wait_for(box.get(), timeout=60)
        self.note("warmup_reply", bytes=len(reply), decoded=str(msgpack.unpackb(reply, raw=False))[:80])

    async def to_agent(self, data) -> None:
        """Send to the agent, waiting out a reconnect window."""
        end = time.monotonic() + RECONNECT_S + 5
        while time.monotonic() < end:
            a = self.agent_conn()
            if a is not None:
                await a["ws"].send(data)
                return
            await asyncio.sleep(0.2)
        raise ConnectionError("no agent connection within the reconnect window")

    # -- the simulator's side ------------------------------------------------------------------
    def adapt(self, message: bytes) -> bytes:
        if not (self.args.numpy or self.args.strip_depth or self.args.like_online):
            return message
        data = msgpack.unpackb(message, raw=False)
        p = data.get("params") or {}
        if self.args.like_online:
            like_online(p)
        if self.args.strip_depth:
            images = p.get("images") or {}
            for k in [k for k in images if k.endswith("_depth")]:
                del images[k]
            for entry in images.values():
                if isinstance(entry, dict):
                    entry.pop("calibration", None)
        if self.args.numpy:
            import msgpack_numpy
            import numpy as np

            st = p.get("states") or {}
            for k, v in list(st.items()):
                if isinstance(v, list) and v and all(isinstance(x, (int, float)) for x in v):
                    st[k] = np.asarray(v, np.float64)
            return msgpack_numpy.packb(data, use_bin_type=True)
        return msgpack.packb(data, use_bin_type=True)

    async def on_sim(self, ws) -> None:
        await self.ready.wait()
        sid = uuid.uuid4().hex
        case = {"case": len(self.cases), "requests": 0, "longest_s": 0.0, "over_cap": 0, "t0": time.time(),
                "bytes_max": 0}
        self.cases.append(case)
        self.replies[sid] = asyncio.Queue()
        await self.to_agent(json.dumps({"type": "session_open", "session_id": sid}))
        self.note("session_open", case=case["case"])
        try:
            async for message in ws:
                payload = self.adapt(message)
                case["requests"] += 1
                case["bytes_max"] = max(case["bytes_max"], len(payload))
                t0 = time.monotonic()
                if self.args.slow_mb_s > 0:  # the download, inside the simulator's limit as online
                    await asyncio.sleep(len(payload) / (self.args.slow_mb_s * 1e6))
                await self.to_agent(join_frame(sid, payload))
                if self.args.drop_after and case["case"] == 0 and case["requests"] == self.args.drop_after:
                    a = self.agent_conn()
                    if a is not None:
                        self.note("cutting_agent_connection", request=case["requests"])
                        a["ws"].transport.abort()
                try:
                    reply = await asyncio.wait_for(self.replies[sid].get(), timeout=RECONNECT_S + ANSWER_CAP_S)
                except asyncio.TimeoutError:
                    self.note("no_reply", case=case["case"], request=case["requests"])
                    self.lost_case = True
                    await ws.close(1011, "no reply")
                    break
                waited = time.monotonic() - t0
                case["longest_s"] = round(max(case["longest_s"], waited), 2)
                case["over_cap"] += int(waited > ANSWER_CAP_S)
                await ws.send(reply)
        except Exception as exc:  # noqa: BLE001
            self.note("sim_connection_error", error=f"{type(exc).__name__}: {exc}"[:200])
        finally:
            case["minutes"] = round((time.time() - case["t0"]) / 60, 1)
            self.replies.pop(sid, None)
            try:
                await self.to_agent(json.dumps({"type": "session_close", "session_id": sid}))
            except Exception as exc:  # noqa: BLE001
                self.note("session_close_not_sent", error=str(exc)[:100])
            self.note("session_close", **{k: v for k, v in case.items() if k != "t0"})

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


async def amain(args) -> dict:
    from websockets.asyncio.server import serve

    from .run import free_port, take_domain

    gate = Gate(args)
    state = Path(args.state)
    state.parent.mkdir(parents=True, exist_ok=True)
    tmp = state.with_suffix(".new")
    tmp.write_text(f"JOB_UUID={gate.job}\nJOB_TOKEN={gate.token}\nPARALLELISM={args.parallelism}\n"
                   f"TUNNEL_ENDPOINT=ws://127.0.0.1:{args.agent_port}{PATH}\nJOB_BOARD={P.parse_config(args.config)['board']}\n")
    os.chmod(tmp, 0o600)
    tmp.replace(state)
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    sim_port = free_port()
    facts = P.parse_config(args.config)
    if facts["sub_task"] in P.VLM_JUDGED:
        raise SystemExit("a VLM-judged config needs the local judge: not in this test")
    seed = args.seed if args.seed is not None else P.seed_for(facts["board"], args.part)
    if P.part_of(args.layout) != args.part:
        raise SystemExit(f"layout {args.layout} is not a {args.part} layout")
    agent_server = await serve(gate.on_agent, "127.0.0.1", args.agent_port, process_request=gate.check,
                               max_size=None, compression=None, ping_interval=None)
    sim_server = await serve(gate.on_sim, "127.0.0.1", sim_port, max_size=None, compression=None,
                             ping_interval=20, ping_timeout=60)
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, asyncio.current_task().cancel)  # TERM: stop the sim, then go
    keep = loop.create_task(gate.keepalive())
    gate.note("listening", agent_port=args.agent_port, sim_port=sim_port, state=str(state))
    await gate.ready.wait()
    await gate.warm_check()
    domain, lock_fd = take_domain()
    overrides = [f"--benchmark.infer_host=127.0.0.1:{sim_port}", f"--benchmark.instance_ids={args.layout}",
                 f"--benchmark.seed={seed}"]
    proc = sim.launch(run_dir, args.gpu, args.config, overrides, domain=domain)
    gate.note("sim_started", config=args.config, layout=args.layout, seed=seed, gpu=args.gpu, pid=proc.pid)
    t0 = time.monotonic()
    try:
        while proc.poll() is None:
            if time.monotonic() - t0 > args.max_h * 3600:
                gate.note("sim_cap")
                break
            await asyncio.sleep(2.0)
    finally:
        from .run import stop_sim

        rc = stop_sim(proc, lambda r: gate.note("sim_signal", signal=r.get("signal")))
        os.close(lock_fd)
    gate.note("sim_exit", returncode=rc, minutes=round((time.monotonic() - t0) / 60, 1))
    gate.drained = True
    live = [a for a in gate.agents.values() if a.get("ws") is not None]
    for a in live:  # every agent of the job, as the platform does
        try:
            await a["ws"].send(json.dumps({"type": "drain"}))
        except Exception as exc:  # noqa: BLE001 - it is going anyway
            gate.note("drain_send_failed", error=f"{type(exc).__name__}: {exc}"[:200])
    if live:
        try:
            await asyncio.wait_for(gate.done.wait(), timeout=600)
        except asyncio.TimeoutError:
            gate.note("agent_did_not_close_after_drain")
    keep.cancel()
    agent_server.close()
    sim_server.close()
    verdicts = []
    for path in sorted((run_dir / "root" / "output" / "benchmark").rglob("evaluate_ret_*.json")):
        try:
            verdicts += [verdict_of(e) for e in json.loads(path.read_text()).get("details") or []]
        except (OSError, ValueError):
            continue
    report = {"config": args.config, "layout": args.layout, "seed": seed, "sim_returncode": rc,
              "cases": [{k: v for k, v in c.items() if k != "t0"} for c in gate.cases],
              "reconnects": gate.reconnects, "lost_case": gate.lost_case,
              "verdicts": [{k: v.get(k) for k in ("success", "progress", "instruction", "dispatches")} for v in verdicts],
              "events": [e for e in gate.events if e["kind"] not in ("agent_control",)][-60:]}
    (run_dir / "fakegate.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str) + "\n")
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.fakegate",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True)
    ap.add_argument("--layout", type=int, required=True)
    ap.add_argument("--part", choices=P.PARTS, default="learn")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--gpu", default="0")
    ap.add_argument("--agent-port", type=int, default=18790)
    ap.add_argument("--state", required=True, help="where to write the agent's state file (made-up credentials)")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--parallelism", type=int, default=1)
    ap.add_argument("--drop-after", type=int, default=0)
    ap.add_argument("--numpy", action="store_true")
    ap.add_argument("--strip-depth", action="store_true")
    ap.add_argument("--like-online", action="store_true")
    ap.add_argument("--slow-mb-s", type=float, default=0.0,
                    help="delay every request by its size at this many MB/s (a slow download), counted in its answer")
    ap.add_argument("--max-h", type=float, default=3.0)
    args = ap.parse_args(argv)
    try:
        report = asyncio.run(amain(args))
    except asyncio.CancelledError:
        print(json.dumps({"stopped": "SIGTERM"}))
        return 1
    print(json.dumps({k: v for k, v in report.items() if k != "events"}, ensure_ascii=False, default=str))
    return 0 if report["verdicts"] and not report["lost_case"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
