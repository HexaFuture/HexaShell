"""Re-fit each arm's base pose from recorded episodes.

RoboDojo reports both the joint angles and the link6 pose of each arm at every step; the URDF
chain maps joints to link6 relative to the arm's base.  The base pose (position, heading, and a
possible small tilt) is what makes the two agree.  This script reads ``states.jsonl`` files from
episode records, fits the base by Gauss-Newton on position and orientation residuals, and prints
the fit next to the constants in ``hexashell/world.py``.

    python bench/fit_bases.py results/*/states.jsonl [--tilt]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hexashell.geometry import homogeneous, log_so3, pose_from_xyzwxyz, rot_x, rot_y, rot_z  # noqa: E402
from hexashell.motion.chain import ArmChain, _revolute_chain  # noqa: E402
from hexashell.world import ARM_BASES, ARMS  # noqa: E402

#: Metres of position residual that weigh as much as one radian of orientation residual.
ANGLE_WEIGHT_M = 0.1


def samples(paths: list[Path], arm: str) -> list[tuple[np.ndarray, np.ndarray]]:
    """(joint angles, reported link6 pose) pairs, one per distinct joint reading."""
    out, seen = [], set()
    for path in paths:
        for line in path.read_text().splitlines():
            row = json.loads(line)
            q = row.get(f"{arm}_arm_joint_state")
            pose = row.get(f"{arm}_ee_pose")
            if q is None or pose is None:
                continue
            key = tuple(round(v, 4) for v in q)
            if key in seen:
                continue
            seen.add(key)
            out.append((np.asarray(q, float), pose_from_xyzwxyz(pose)))
    return out


def base_pose(p: np.ndarray) -> np.ndarray:
    x, y, z, heading, tilt_x, tilt_y = p
    return homogeneous(rot_z(heading) @ rot_y(tilt_y) @ rot_x(tilt_x), (x, y, z))


def residuals(p: np.ndarray, data, joints) -> np.ndarray:
    chain = ArmChain("fit", base_pose(p), joints)
    res = []
    for q, seen in data:
        model = chain.flange(q)
        res.append(model[:3, 3] - seen[:3, 3])
        res.append(ANGLE_WEIGHT_M * log_so3(seen[:3, :3].T @ model[:3, :3]))
    return np.concatenate(res)


def fit(data, start: np.ndarray, free: np.ndarray, joints, iters: int = 30) -> np.ndarray:
    p = start.astype(float).copy()
    for _ in range(iters):
        r = residuals(p, data, joints)
        jac = np.zeros((len(r), len(p)))
        for i in np.flatnonzero(free):
            dp = np.zeros_like(p)
            dp[i] = 1e-6
            jac[:, i] = (residuals(p + dp, data, joints) - r) / 1e-6
        cols = np.flatnonzero(free)
        step, *_ = np.linalg.lstsq(jac[:, cols], -r, rcond=None)
        p[cols] += step
        if np.max(np.abs(step)) < 1e-9:
            break
    return p


def report(name: str, p: np.ndarray, data, joints) -> str:
    r = residuals(p, data, joints).reshape(-1, 2, 3)
    pos_mm = 1000 * np.linalg.norm(r[:, 0], axis=1)
    ang_deg = np.degrees(np.linalg.norm(r[:, 1], axis=1) / ANGLE_WEIGHT_M)
    x, y, z, h, tx, ty = p
    return (f"{name:9s} base ({x:+.5f}, {y:+.5f}, {z:.5f}) heading {h:.10f} tilt ({math.degrees(tx):+.3f}°, "
            f"{math.degrees(ty):+.3f}°)  |  position error mean {pos_mm.mean():.2f} mm, max {pos_mm.max():.2f} mm; "
            f"angle mean {ang_deg.mean():.3f}°, max {ang_deg.max():.3f}°")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("states", nargs="+", type=Path)
    ap.add_argument("--tilt", action="store_true", help="also fit a small tilt of the base")
    args = ap.parse_args()
    joints = _revolute_chain()
    for arm in ARMS:
        data = samples(args.states, arm)
        moving = [d for d in data if np.any(np.abs(d[0]) > 1e-3)]
        print(f"{arm}: {len(data)} distinct readings ({len(moving)} away from zero)")
        if len(moving) < 10:
            print("  too few readings away from the start pose to fit; skipped")
            continue
        xyz, heading = ARM_BASES[arm]
        now = np.array([*xyz, heading, 0.0, 0.0])
        free = np.array([1, 1, 1, 1, int(args.tilt), int(args.tilt)], bool)
        fitted = fit(data, now, free, joints)
        print("  " + report("world.py", now, data, joints))
        print("  " + report("fitted", fitted, data, joints))


if __name__ == "__main__":
    main()
