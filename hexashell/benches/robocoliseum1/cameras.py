"""Nominal camera models of the G2 (``G2_omnipicker``), for requests that carry no calibration.

The online benchmark's simulator (a genie_sim before v3.5.0, which added the calibration) sends the pictures and
the depth but no intrinsics and no camera poses.  The cameras are the same prims of the same robot, so their
models are taken from the local simulator (:mod:`.calcap`) and the poses follow the robot's readings:

- a hand camera sits fixed on its gripper: its optical frame is ``end_link @ mount`` (``arm_*_end_link`` in
  ``base_link`` comes with every request);
- the head camera sits fixed on ``head_link3``; the head's three joints turn it on ``head_base_link``, which
  is fixed on ``body_link5`` as ``arm_base_link`` is (``G2_omnipicker.urdf``).  So its optical frame is
  ``arm_base @ inv(B5_AB) @ B5_HB @ head(h1, h2, h3) @ mount``, with ``arm_base`` from the request
  (:func:`.g2.arm_base_from_obs`) and the head joints from ``head_joint_states``.  The simulator's head is not
  quite the URDF's: its second joint turns about +x (the URDF says -x), and its joints' origins are a little
  off (up to 1.1 mm at the camera); the fit takes the origins from the captures (``chain`` in the model).
  The online simulator sends ``head_joint_states`` empty: the head then stands where the task starts it
  (:func:`head_rest`), since nothing of ours turns it;
- the intrinsics and the lens distortion are the local calibration's, at the size it was given for (the hand
  cameras come at 640x528 or 1280x1056: the same lens, scaled).

``MODEL_FILE`` holds the numbers (``python -m hexashell.benches.robocoliseum1.cameras fit OUT calib.jsonl ...``
makes it from captures and prints how well every capture is reproduced; ``check`` only prints that).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from ...geometry import invert
from . import g2

MODEL_FILE = Path(__file__).with_name("g2_cameras.json")
#: the request's camera names
WIRE = ("head", "hand_left", "hand_right")
HAND_ARM = {"hand_left": "left", "hand_right": "right"}


def _t(x, y, z) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _r(axis, q) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s, v = math.cos(q), math.sin(q), 1 - math.cos(q)
    m = np.eye(4)
    m[:3, :3] = [[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                 [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                 [z * x * v - y * s, z * y * v + x * s, c + z * z * v]]
    return m


# G2_omnipicker.urdf: fixed joints on body_link5, the head's joints (origin, axis), the body's joints
B5_AB = _t(0.0, 0.0, 0.3085) @ _r((1, 0, 0), -math.pi / 2)
B5_HB = _t(-0.013211, 0.0, 0.35976)
#: the head's joints as the simulator turns them: the URDF's origins, the second axis +x (the URDF's -x
#: turns the camera the wrong way: 10.6 degrees off at a roll of 0.1 rad in recorded requests)
HEAD = [((0.0, 0.0, 0.0), (0, 0, 1)), ((0.029814, 0.0, 0.21419), (1, 0, 0)),
        ((-0.029814, 0.0, -0.00019325), (0, 1, 0))]
BODY = [((0.102, 0.0, 0.144), (0, 1, 0)), ((0.0, 0.0, 0.36), (0, 1, 0)), ((0.0, 0.0, 0.38), (0, 1, 0)),
        ((0.0, 0.0, 0.0), (1, 0, 0)), ((0.0, 0.0, 0.149), (0, 0, 1))]


#: the head's joints at the start of a task (``head_state`` in genie_sim's ``robot_init_states.py``: the same
#: for every RoboColiseum task in the online version and in v3.5.0), for a simulator that does not report them.
#: The online one sends ``head_joint_states`` empty, and the head stays there: it turns only when a reply
#: says so, and ours never do (checked on online requests: points the wrist cameras measured land on the
#: objects in the head's picture, 1-2 px off, with this head)
HEAD_REST = (0.0, 0.0, 0.11464)
HEAD_REST_BY_TASK = {"scoop_popcorn": (0.0, 0.0, 0.0), "take_wrong_item_shelf": (0.0, 0.0, 0.1745)}


def head_rest(task_name: str) -> tuple:
    """The head's joints at the start of task ``task_name`` (a sub-task name, or a name containing one)."""
    for key, q in HEAD_REST_BY_TASK.items():
        if key in (task_name or ""):
            return q
    return HEAD_REST


def chain(joints, q) -> np.ndarray:
    m = np.eye(4)
    for (origin, axis), v in zip(joints, q):
        m = m @ _t(*origin) @ _r(axis, float(v))
    return m


def head_base(arm_base: np.ndarray) -> np.ndarray:
    """head_base_link in base_link from arm_base_link in base_link (both fixed on body_link5)."""
    return arm_base @ invert(B5_AB) @ B5_HB


def head_link3(arm_base: np.ndarray, head, joints=None) -> np.ndarray:
    """head_link3 in base_link from arm_base_link in base_link and the head's joints (``joints``: the model's
    fitted chain, else ``HEAD``)."""
    return head_base(arm_base) @ chain(_joints(joints), head)


def _joints(joints):
    if not joints:
        return HEAD
    return [(tuple(map(float, o)), tuple(map(float, a))) for o, a in joints]


def arm_base_from_waist(waist) -> np.ndarray:
    """arm_base_link in base_link from the body's five joints (a check of the readings)."""
    return chain(BODY, waist) @ B5_AB


_model: dict | None = None


def model() -> dict | None:
    """The numbers of MODEL_FILE (None when there is none)."""
    global _model
    if _model is None and MODEL_FILE.is_file():
        _model = json.loads(MODEL_FILE.read_text())
    return _model


def nominal(wire: str, *, arm_base: np.ndarray | None, head, end_links: dict, task_name: str = "") -> dict | None:
    """A calibration in the benchmark's own form (``fx`` ... ``optical_to_frame``) for camera ``wire`` from the
    readings, or None when the model or a reading it needs is missing.  Without the head's joints (fewer than
    three) the head is taken where task ``task_name`` starts it (``head_rest``: True in the result)."""
    m = model()
    if m is None or wire not in m:
        return None
    cam = m[wire]
    rest = False
    if wire == "head":
        if arm_base is None:
            return None
        if head is None or len(head) < len(HEAD):
            head, rest = head_rest(task_name), True
        pose = head_link3(arm_base, head, cam.get("chain")) @ np.asarray(cam["mount"], float)
    else:
        end = end_links.get(HAND_ARM[wire])
        if end is None:
            return None
        pose = end @ np.asarray(cam["mount"], float)
    w, h = cam["size"]
    return {"available": True, "nominal": True, "model": cam.get("model", "pinhole"), "width": w, "height": h,
            "fx": cam["fx"], "fy": cam["fy"], "cx": cam["cx"], "cy": cam["cy"],
            "distortion": list(cam.get("distortion") or []), "optical_to_frame": pose.tolist(),
            "frame": "base_link", "depth_scale_m": 0.001, "snapshot_id": "nominal", "head_rest": rest}


# -- fitting -----------------------------------------------------------------------------------------
def _readings(rec: dict):
    st = rec["states"]
    end_pose = st["end_pose"]
    arm_base = g2.arm_base_from_obs(end_pose)
    ends = {arm: g2.pose_of(end_pose["base_link"][f"{arm}_arm"]["position"],
                            end_pose["base_link"][f"{arm}_arm"]["orientation"]) for arm in g2.ARMS}
    return arm_base, st.get("head_joint_states"), st.get("waist_joint_states"), ends


def _carrier(wire: str, arm_base, head, ends, joints=None) -> np.ndarray:
    return head_link3(arm_base, head, joints) if wire == "head" else ends[HAND_ARM[wire]]


def _fit_head_chain(samples) -> tuple[list, np.ndarray]:
    """The head joints' origins and the camera's place on head_link3 from (head joints, optical frame in
    head_base_link) pairs, the axes fixed (``HEAD``'s).  Where a joint's pose is known the camera's position
    is linear in the origins: p = o1 + R1 o2 + R1 R2 o3 + R1 R2 R3 t.  The smallest change from the URDF's
    origins (and from the mean mount's t) that fits is taken, so what the captures cannot tell apart (a shift
    along an axis, shared between two links) stays as the URDF has it."""
    rot = _mean_pose([invert(chain(HEAD, h)) @ m for h, m in samples])
    x0 = np.concatenate([np.asarray(o, float) for o, _ in HEAD] + [rot[:3, 3]])
    a, b = [], []
    for h, m in samples:
        rs = [_r(ax, float(v))[:3, :3] for (_, ax), v in zip(HEAD, h)]
        a.append(np.hstack([np.eye(3), rs[0], rs[0] @ rs[1], rs[0] @ rs[1] @ rs[2]]))
        b.append(m[:3, 3])
    a, b = np.vstack(a), np.concatenate(b)
    x = x0 + np.linalg.pinv(a) @ (b - a @ x0)
    joints = [[x[3 * i:3 * i + 3].tolist(), list(ax)] for i, (_, ax) in enumerate(HEAD)]
    mount = rot.copy()
    mount[:3, 3] = x[9:12]
    return joints, mount


def _mean_pose(ms: list[np.ndarray]) -> np.ndarray:
    t = np.mean([m[:3, 3] for m in ms], axis=0)
    u, _, vt = np.linalg.svd(np.sum([m[:3, :3] for m in ms], axis=0))
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = r, t
    return out


def _err(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """(mm, deg) between two poses."""
    d = invert(a) @ b
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(d[:3, :3]) - 1) / 2))))
    return float(np.linalg.norm(d[:3, 3]) * 1000), ang


def load(paths) -> list[dict]:
    recs = []
    for p in paths:
        for line in Path(p).read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                r["file"] = str(p)
                recs.append(r)
    return recs


def fit(recs: list[dict]) -> dict:
    out: dict = {"source": sorted({Path(r["file"]).parent.name for r in recs}), "samples": len(recs)}
    for wire in WIRE:
        mounts, ks, heads = [], [], []
        for r in recs:
            c = r["cams"].get(wire, {}).get("calibration") or {}
            if not c.get("available") or c.get("optical_to_frame") is None:
                continue
            arm_base, head, _, ends = _readings(r)
            optical = np.asarray(c["optical_to_frame"], float)
            mounts.append(invert(_carrier(wire, arm_base, head, ends)) @ optical)
            heads.append((np.asarray(head, float), invert(head_base(arm_base)) @ optical) if wire == "head" else None)
            ks.append(c)
        if not ks:
            continue
        w, h = ks[0]["width"], ks[0]["height"]
        same = [c for c in ks if (c["width"], c["height"]) == (w, h)]
        out[wire] = {"size": [w, h], "model": same[0].get("model"),
                     **{k: float(np.median([c[k] for c in same])) for k in ("fx", "fy", "cx", "cy")},
                     "distortion": [float(v) for v in np.median([c.get("distortion") or [0.0] * 8 for c in same], axis=0)],
                     "mount": _mean_pose(mounts).tolist()}
        if wire == "head":
            joints, mount = _fit_head_chain(heads)
            out[wire]["chain"], out[wire]["mount"] = joints, mount.tolist()
    return out


def check(recs: list[dict], m: dict) -> dict:
    """How far the model's poses and intrinsics are from every captured calibration."""
    rep = {}
    for wire in WIRE:
        if wire not in m:
            continue
        errs, kerr, body = [], [], []
        for r in recs:
            c = r["cams"].get(wire, {}).get("calibration") or {}
            if not c.get("available") or c.get("optical_to_frame") is None:
                continue
            arm_base, head, waist, ends = _readings(r)
            pose = _carrier(wire, arm_base, head, ends, m[wire].get("chain")) @ np.asarray(m[wire]["mount"], float)
            errs.append(_err(pose, np.asarray(c["optical_to_frame"], float)) + (r.get("pose"),))
            s = c["width"] / m[wire]["size"][0]
            kerr.append(max(abs(c[k] - m[wire][k] * s) for k in ("fx", "fy", "cx", "cy")))
            if waist is not None and wire == "head":
                body.append(_err(arm_base_from_waist(waist), arm_base))
        rep[wire] = {"n": len(errs), "max_mm": round(max(e[0] for e in errs), 3),
                     "max_deg": round(max(e[1] for e in errs), 4), "max_k_px": round(max(kerr), 4),
                     "worst": max(errs, key=lambda e: e[0])[2]}
        if body:
            rep[wire]["arm_base_vs_waist_fk"] = {"max_mm": round(max(b[0] for b in body), 3),
                                                 "max_deg": round(max(b[1] for b in body), 4)}
    return rep


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m hexashell.benches.robocoliseum1.cameras", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("out")
    f.add_argument("captures", nargs="+")
    c = sub.add_parser("check")
    c.add_argument("captures", nargs="+")
    c.add_argument("--model", default=str(MODEL_FILE))
    args = ap.parse_args(argv)
    recs = load(args.captures)
    if args.cmd == "fit":
        m = fit(recs)
        Path(args.out).write_text(json.dumps(m, indent=1) + "\n")
    else:
        m = json.loads(Path(args.model).read_text())
    print(json.dumps(check(recs, m), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
