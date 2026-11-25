"""Capture the local simulator's camera calibrations together with the robot's readings, to build and check
the nominal camera models (:mod:`.cameras`) the online benchmark needs: its simulator (an earlier genie_sim
than v3.5.0) sends depth but no calibration.

``python -m hexashell.benches.robocoliseum1.calcap --config g2op_spatial_pick_object_absolute_position
--layout 0 --gpu 2 --run-dir /tmp/x``

No model: the simulator is started on a learning layout as :mod:`.run` does, and a scripted server answers
it.  The arms and grippers hold their start, then the waist, the head and two wrist joints step through a
few poses (small, inside the joint limits); each pose is reached over ``RAMP`` rows and held ``HOLD`` rows,
and every reply asks for depth, so every request carries the calibration of all three cameras.  Each request
with a calibration is one line of ``<run-dir>/calib.jsonl``: the request's number, per camera the
calibration's fields (no pictures; the depth's and the colour's sizes), and the readings (arm, waist, head
and gripper joints, ``end_pose`` in both frames).  When the poses are done (or ``--max-min``) the simulator
is stopped.  The first request's colour pictures, as sent, are kept as ``first_<camera>.jpg``.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import threading
import time
from pathlib import Path

import msgpack
import numpy as np

from . import obs as O
from . import protocol as P
from . import sim
from .link import start_command
from .run import free_port, stop_sim, take_domain

#: rows per pose (the simulator plays at most 30 rows of a reply; a learning episode may end after about 200
#: steps, so the poses are short)
RAMP = 6
HOLD = 4
#: (what moves, the offsets from the start): waist = body joints 1-5, head = head joints 1-3,
#: wrist = (left joint 7, right joint 6)
POSES = [
    ("start", {}),
    ("head yaw +0.3", {"head": {0: 0.3}}),
    ("head yaw -0.3, pitch", {"head": {0: -0.3, 2: 0.2}}),
    ("head roll", {"head": {1: 0.15}}),
    ("head + waist", {"head": {0: 0.2, 2: -0.15}, "waist": {4: -0.3, 2: 0.05}}),
    ("waist pitch", {"waist": {2: 0.08}}),
    ("waist lift", {"waist": {1: -0.06, 2: 0.06}}),
    ("waist yaw +0.4", {"waist": {4: 0.4}}),
    ("wrists", {"wrist": (0.5, 0.3)}),
    ("wrists + waist yaw", {"wrist": (-0.4, -0.3), "waist": {4: 0.25}}),
    ("waist roll +0.1", {"waist": {3: 0.1}}),
    ("start again", {}),
]


def _small(v):
    if isinstance(v, (bytes, bytearray)):
        return f"<{len(v)} bytes>"
    if isinstance(v, dict):
        return {k: _small(x) for k, x in v.items() if k != "image_data"}
    if isinstance(v, (list, tuple)):
        return [_small(x) for x in v]
    return v


def record(payload: dict, seq: int) -> dict | None:
    p = payload.get("params") or {}
    images = p.get("images") or {}
    cams = {}
    for wire in O.WIRE_CAMS:
        entry = images.get(wire)
        if not isinstance(entry, dict) or not isinstance(entry.get("calibration"), dict):
            continue
        dent = images.get(f"{wire}_depth") or {}
        cams[wire] = {"calibration": _small(entry["calibration"]), "color": [entry.get("width"), entry.get("height")],
                      "depth": [dent.get("width"), dent.get("height")]}
    if not cams:
        return None
    st = p.get("states") or {}
    keep = ("arm_joint_states", "waist_joint_states", "head_joint_states", "gripper_states", "end_pose")
    return {"seq": seq, "task_name": p.get("task_name"), "cams": cams, "states": {k: _small(st.get(k)) for k in keep}}


class Script:
    """The poses, one after the other, from the first request's readings."""

    def __init__(self):
        self.rows: list[dict] = []
        self.heads: list[list[float]] = []
        self.k = -1
        self.done = False
        self.base = None

    def reply(self, frame: O.Frame) -> bytes:
        if self.base is None:
            cmd = start_command(frame)
            cmd["waist"] = None if frame.waist is None else np.asarray(frame.waist, float)
            self.base = {"cmd": cmd, "head": None if frame.head is None else np.asarray(frame.head, float)}
            self.last = {"cmd": cmd, "head": self.base["head"]}
        if not self.rows:
            self.k += 1
            if self.k >= len(POSES):
                self.done = True
                self.k = len(POSES) - 1
            self._plan(POSES[self.k][1])
        n = len(self.rows)
        rows, heads = self.rows[:n], self.heads[:n]
        self.rows, self.heads = [], []
        out = {
            "left_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["left"]] for r in rows]},
            "right_arm": {"kind": "JOINT_ABS", "values": [[float(v) for v in r["right"]] for r in rows]},
            "left_effector": [[float(r["grip"]["left"])] for r in rows],
            "right_effector": [[float(r["grip"]["right"])] for r in rows],
            "need_depth": True,
        }
        if all(r.get("waist") is not None for r in rows):
            out["waist"] = {"kind": "JOINT_ABS", "values": [[float(v) for v in r["waist"]] for r in rows]}
        if all(h is not None for h in heads):
            out["head"] = {"values": [[float(v) for v in h] for h in heads]}
        return msgpack.packb({"result": out}, use_bin_type=True)

    def _plan(self, pose: dict) -> None:
        b = self.base["cmd"]
        to = {"left": np.array(b["left"], float), "right": np.array(b["right"], float), "grip": dict(b["grip"]),
              "waist": None if b["waist"] is None else np.array(b["waist"], float)}
        head = None if self.base["head"] is None else self.base["head"].copy()
        for i, d in pose.get("waist", {}).items():
            if to["waist"] is not None and i < len(to["waist"]):
                to["waist"][i] += d
        for i, d in pose.get("head", {}).items():
            if head is not None and i < len(head):
                head[i] += d
        if "wrist" in pose:
            to["left"][6] += pose["wrist"][0]
            to["right"][5] += pose["wrist"][1]
        fr, hf = self.last["cmd"], self.last["head"]
        for j in range(1, RAMP + HOLD + 1):
            t = min(1.0, j / RAMP)
            row = {"left": fr["left"] + t * (to["left"] - fr["left"]), "right": fr["right"] + t * (to["right"] - fr["right"]),
                   "grip": dict(to["grip"]),
                   "waist": None if to["waist"] is None else fr["waist"] + t * (to["waist"] - fr["waist"])}
            self.rows.append(row)
            self.heads.append(None if head is None else hf + t * (head - hf))
        self.last = {"cmd": to, "head": head}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.calcap",
                                 description=__doc__.splitlines()[0])
    ap.add_argument("--config", required=True)
    ap.add_argument("--layout", type=int, default=0)
    ap.add_argument("--gpu", default="0")
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--max-min", type=float, default=40.0)
    args = ap.parse_args(argv)
    facts = P.parse_config(args.config)
    if P.part_of(args.layout) != "learn":
        ap.error("calcap runs learning layouts only")
    if facts["sub_task"] in P.VLM_JUDGED:
        ap.error("calcap does not start a judge: pick a task the checkers score")
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    out = open(run_dir / "calib.jsonl", "a", encoding="utf-8")
    log = open(run_dir / "calcap.jsonl", "a", encoding="utf-8")

    def note(rec: dict) -> None:
        log.write(json.dumps({"t": round(time.time(), 3), **rec}, default=str) + "\n")
        log.flush()

    script = Script()
    state = {"seq": 0, "samples": 0, "finished": threading.Event()}

    def handle(message: bytes) -> bytes:
        payload = msgpack.unpackb(message, raw=False)
        if not isinstance(payload, dict) or payload.get("method", "infer") != "infer":
            return msgpack.packb({"result": {}}, use_bin_type=True)
        state["seq"] += 1
        rec = record(payload, state["seq"])
        if rec is not None:
            rec["pose"] = POSES[max(script.k, 0)][0] if script.k >= 0 else "first"
            if state["samples"] == 0:  # the start's pictures as sent, to compare with another simulator's
                for wire in O.WIRE_CAMS:
                    data = ((payload["params"].get("images") or {}).get(wire) or {}).get("image_data")
                    if data:
                        (run_dir / f"first_{wire}.jpg").write_bytes(data)
            out.write(json.dumps(rec, default=str) + "\n")
            out.flush()
            state["samples"] += 1
        frame = O.parse(payload, state["seq"])
        if script.done or frame.done:
            state["finished"].set()
        return script.reply(frame)

    async def serve(port: int, stop: asyncio.Event):
        import websockets

        async def handler(ws):
            loop = asyncio.get_running_loop()
            note({"kind": "connected"})
            try:
                async for message in ws:
                    await ws.send(await loop.run_in_executor(None, handle, message))
            except websockets.ConnectionClosed:
                pass
            note({"kind": "disconnected"})

        async with websockets.serve(handler, "127.0.0.1", port, compression=None, max_size=None,
                                    ping_interval=20, ping_timeout=60):
            await stop.wait()

    port = free_port()
    domain, lock_fd = take_domain()
    loop = asyncio.new_event_loop()
    stop = asyncio.Event()
    th = threading.Thread(target=lambda: loop.run_until_complete(serve(port, stop)), daemon=True)
    th.start()
    seed = P.seed_for(facts["board"], "learn")
    overrides = [f"--benchmark.infer_host=127.0.0.1:{port}", f"--benchmark.instance_ids={args.layout}",
                 f"--benchmark.seed={seed}"]
    note({"kind": "start", "config": args.config, "layout": args.layout, "gpu": args.gpu, "port": port})
    proc = sim.launch(run_dir, args.gpu, args.config, overrides, domain=domain, judge_url=None)
    t0 = time.monotonic()
    end = "sim_exit"
    try:
        while proc.poll() is None:
            if state["finished"].wait(2.0):
                end = "poses_done"
                break
            if time.monotonic() - t0 > args.max_min * 60:
                end = "max_min"
                break
    except KeyboardInterrupt:
        end = "interrupted"
    finally:
        rc = stop_sim(proc, note)
        loop.call_soon_threadsafe(stop.set)
        th.join(timeout=10)
        os.close(lock_fd)
    summary = {"config": args.config, "layout": args.layout, "end": end, "returncode": rc,
               "requests": state["seq"], "samples": state["samples"], "minutes": round((time.monotonic() - t0) / 60, 1)}
    note({"kind": "end", **summary})
    print(json.dumps(summary))
    return 0 if state["samples"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
