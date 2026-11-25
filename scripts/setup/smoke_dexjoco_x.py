#!/usr/bin/env python3
"""Headless smoke test for the DexJoCo-X simulator env ("bench-dexjoco-x"), through the server's own
hexashell.benches.dexjoco_x.sim.DexSim (the official evaluation's Runtime with the Wuji hand).

For each task, on the release's smoke seed (20000000 + the task's index; not a test seed):
  * build and reset; the arms and cameras the protocol expects;
  * pictures of every camera at 512 x 512: colour, depth (metres along the optical axis) and the robot's own
    pixels; every picture shows something, the scene camera shows the robot;
  * the camera model: at a few pixels, the depth's back-projected point against a MuJoCo ray cast along the
    same pixel's ray (within 3 mm);
  * 20 steps holding still (no wrist change, the hand's own targets): no success, the time per step.

Usage (env vars from <bench-data>/dexjoco-x/env.sh; MUJOCO_EGL_DEVICE_ID picks the GPU):
    python scripts/setup/smoke_dexjoco_x.py [--task TASK ...] [--out DIR]
Prints one JSON line per task and exits 1 when a check failed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from hexashell.benches.dexjoco_x import protocol as P  # noqa: E402
from hexashell.benches.dexjoco_x.sim import DexSim  # noqa: E402

SMOKE_SEED = 20_000_000
HOLD_STEPS = 20
#: a back-projected point and the ray cast's may differ this much (m)
RAY_TOLERANCE_M = 0.003
#: the geom groups the renderer draws (MuJoCo's default: 0-2; collision geoms sit in the others)
VISIBLE_GROUPS = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
#: pixels checked per camera: fractions of the picture's width and height
PROBES = ((0.5, 0.5), (0.3, 0.6), (0.7, 0.6), (0.5, 0.75))


def ray_check(sim: DexSim, cam: str, depth: np.ndarray) -> list[dict]:
    """Back-projected depth points against MuJoCo ray casts at a few pixels of one camera."""
    import mujoco

    c = sim.camera(cam)
    pose, f, size = np.asarray(c["pose"], float), float(c["focal"]), int(c["width"])
    centre = (size - 1) / 2.0
    out = []
    for fx, fy in PROBES:
        x, y = int(fx * size), int(fy * size)
        d = float(depth[y, x])
        if d <= 0.0:
            continue
        local = np.array([(x - centre) / f, -(y - centre) / f, -1.0])
        point = pose[:3, :3] @ (local * d) + pose[:3, 3]
        direction = pose[:3, :3] @ local
        direction /= np.linalg.norm(direction)
        geom = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(sim.m, sim.shadow, pose[:3, 3], direction, VISIBLE_GROUPS, 1, -1, geom)
        if dist < 0:
            out.append({"px": [x, y], "depth": round(d, 4), "ray": None})
            continue
        hit = pose[:3, 3] + dist * direction
        out.append({"px": [x, y], "depth": round(d, 4), "gap_mm": round(1000.0 * float(np.linalg.norm(hit - point)), 2)})
    return out


def smoke(task: str, out: Path | None) -> dict:
    spec = P.check_task(task)
    seed = SMOKE_SEED + spec.index
    problems = []
    t0 = time.time()
    sim = DexSim(task)
    try:
        facts = sim.reset(seed)
        build_s = time.time() - t0
        if tuple(sorted(sim.sides)) != tuple(sorted(spec.arms)):
            problems.append(f"arms {sim.sides}, protocol {spec.arms}")
        t1 = time.time()
        seen = sim.observe()
        picture_s = time.time() - t1
        cams = {}
        for cam in P.cameras(task):
            rgb, depth = seen["images"][cam], seen["depth"][cam]
            robot = seen["robot"].get(cam)
            row = {"rgb": list(rgb.shape), "depth_range": [round(float(depth[depth > 0].min()), 3),
                                                            round(float(depth.max()), 3)] if (depth > 0).any() else None,
                   "depth_share": round(float((depth > 0).mean()), 3),
                   "robot_share": round(float(robot.mean()), 3) if robot is not None else None,
                   "colour_std": round(float(rgb.std()), 1)}
            if tuple(rgb.shape) != (P.IMAGE_SIZE, P.IMAGE_SIZE, 3) or depth.shape != (P.IMAGE_SIZE, P.IMAGE_SIZE):
                problems.append(f"{cam}: picture {rgb.shape}, depth {depth.shape}")
            if row["colour_std"] < 5.0 or row["depth_share"] < 0.5:
                problems.append(f"{cam}: an empty picture (colour spread {row['colour_std']}, "
                                f"depth on {row['depth_share']:.0%} of it)")
            if cam == "scene" and not (row["robot_share"] or 0) > 0.0:
                problems.append("scene: no robot pixels")
            rays = ray_check(sim, cam, depth)
            row["rays"] = rays
            bad = [r for r in rays if r.get("gap_mm") is None or r["gap_mm"] > 1000 * RAY_TOLERANCE_M]
            if bad:
                problems.append(f"{cam}: depth and ray cast differ at {bad}")
            cams[cam] = row
            if out is not None:
                import imageio.v2 as imageio

                out.mkdir(parents=True, exist_ok=True)
                imageio.imwrite(out / f"{task}_{cam}.png", rgb)
        hold = {s: np.concatenate([np.zeros(6), sim.hand_targets(s)]) for s in sim.sides}
        t2 = time.time()
        for _ in range(HOLD_STEPS):
            sim.step(hold)
        step_ms = 1000.0 * (time.time() - t2) / HOLD_STEPS
        if sim.success:
            problems.append("holding still counted as success")
        if sim.terminated or sim.truncated:
            problems.append(f"holding still ended the episode ({sim.failure})")
        return {"task": task, "seed": seed, "ok": not problems, "problems": problems, "build_s": round(build_s, 2),
                "pictures_s": round(picture_s, 2), "step_ms": round(step_ms, 1), "arms": list(sim.sides),
                "steps_limit": facts["steps"], "instruction": facts["instruction"], "cameras": cams,
                "clipped": sim.clipped}
    finally:
        sim.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--task", action="append", choices=list(P.TASKS))
    ap.add_argument("--out", type=Path, help="write each camera's start picture here")
    args = ap.parse_args()
    ok = True
    for task in args.task or list(P.TASKS):
        try:
            row = smoke(task, args.out)
        except Exception as exc:  # noqa: BLE001
            row = {"task": task, "ok": False, "problems": [f"{type(exc).__name__}: {exc}"]}
        ok &= bool(row["ok"])
        print(json.dumps(row), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
