"""Whether a GPU still draws: a tiny MuJoCo scene rendered off screen on it by two processes at once, as the
simulators render (many to a GPU).

    <python with mujoco> -m hexashell.benches.gpucheck GPU

prints one JSON line: the GPU, its EGL index, ``ok``, the seconds of the slowest first frame (the context
included), the slowest later frame of each process and how many each drew, the depth values outside 0..1
(NaN included), and ``why`` when not ok.

A GPU whose graphics engine stopped switching contexts in time (the kernel log: NVIDIA Xid 109 "CTX
SWITCH TIMEOUT", then Xid 31 MMU faults of the graphics engine) still runs CUDA work, but draws slowly or
not at all, and pictures read back while several contexts draw can be damaged: LIBERO-PRO's depth came back
with NaN on two GPUs of one of our machines, where RoboTwin's renderers had hung the day before
(RoboTwin's servers started there afterwards timed out too).  Only a reset of the GPU repairs it.  Such a
GPU took about 4.5 s for every frame (both then), or drew at full speed alone but took seconds a
frame once two contexts drew at once (one of them after its Xid: alone 0.002 s a frame, two
at once up to 11 s with new Xid 109, and four LIBERO-PRO scenes started there together hung in their first
picture), hence two processes that draw together for DRAW_S seconds.  A healthy GPU draws every frame after
the first in milliseconds, also while it is busy; the first, which makes the context, can take seconds
while many processes start on the machine (seven pools' simulator servers starting
after a reboot: up to 5.1 s on every GPU, the next frames 0.002-0.03 s, no Xid in the kernel log), so the
later frames decide.  scripts/bench_pool.py starts no simulator server on a GPU this check finds unfit.
"""

from __future__ import annotations

import json
import os
import select
import subprocess
import sys
import time

#: a first frame (context included) slower than this (s) means the GPU is not fit to render: it hangs (one that
#: draws can take seconds for the first frame while the machine is busy starting processes)
FIRST_LIMIT_S = 60.0
#: a later frame slower than this (s) as well (a GPU that times out: about 4.5 s every frame, or seconds once
#: two contexts draw)
FRAME_LIMIT_S = 1.0
#: a simulator's picture slower than this (s), but the first of an episode (FIRST_LIMIT_S), means its GPU draws
#: wrongly: on a GPU after its Xid one of LIBERO-PRO's took 128.9 s, and the
#: next pictures were an old one of the other camera and blank ones, with no NaN in the depth
PICTURE_LIMIT_S = 10.0
#: processes that draw at once, each with its own context: a GPU that no longer switches contexts in time can
#: draw at full speed alone
CONTEXTS = 2
#: how long they draw together after their first frames (s): on that GPU the first slow frame of two
#: came after 0 and 111 frames of milliseconds
DRAW_S = 2.0
#: and at least this many frames after the first each
FRAMES = 4
#: what a drawing process may take before its first frame besides FIRST_LIMIT_S (s): Python, nvidia-smi and
#: MuJoCo starting on a busy machine
START_S = 60.0
#: what the processes may take to answer once they draw together, besides DRAW_S (s): with the above, a check
#: ends within scripts/bench_pool.py's 180 s
HANG_S = 30.0
SIZE = 128
MODEL = f"""<mujoco>
  <visual><global offwidth="{SIZE}" offheight="{SIZE}"/></visual>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="1 1 0.1"/>
    <geom type="box" pos="0 0 0.1" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
    <camera name="look" pos="0 -1 1" xyaxes="1 0 0 0 0.7 0.7"/>
  </worldbody>
</mujoco>"""
WHY_SUFFIX = " (a GPU whose graphics engine times out: see the kernel log for NVIDIA Xid 109)"


def faults(frames: list[float], bad: int) -> list[str]:
    """What makes a GPU unfit to render, from the seconds of its frames (the first with the context)
    and its depth values outside 0..1; nothing for a GPU that draws."""
    why = []
    if frames[0] > FIRST_LIMIT_S:
        why.append(f"the first frame took {frames[0]:.1f} s")
    if frames[1:] and max(frames[1:]) > FRAME_LIMIT_S:
        why.append(f"a frame took {max(frames[1:]):.1f} s")
    if bad:
        why.append(f"{bad} depth values outside 0..1")
    return why


def draw(gpu: str) -> dict:
    """One of check()'s processes: a context on nvidia-smi's GPU ``gpu`` (before anything else uses EGL here)
    and its first frame; then, after a line on stdout that says so and a line on stdin, frames for DRAW_S
    seconds, at least FRAMES of them; a frame over its limit ends it."""
    from . import egl

    rec = {"gpu": str(gpu), "egl": egl.pin(gpu)}
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_string(MODEL)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    t0 = time.time()
    context = mujoco.GLContext(SIZE, SIZE)
    context.make_current()
    try:
        scene = mujoco.MjvScene(model, maxgeom=100)
        renderer = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, renderer)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        camera.fixedcamid = 0
        viewport = mujoco.MjrRect(0, 0, SIZE, SIZE)
        rgb = np.zeros((SIZE, SIZE, 3), np.uint8)
        depth = np.zeros((SIZE, SIZE), np.float32)
        frames, bad = [], 0
        while True:
            mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, renderer)
            mujoco.mjr_readPixels(rgb, depth, viewport, renderer)
            frames.append(round(time.time() - t0, 3))
            bad += int(np.count_nonzero(~((depth >= 0.0) & (depth <= 1.0))))
            if frames[-1] > (FIRST_LIMIT_S if len(frames) == 1 else FRAME_LIMIT_S):
                break  # a GPU that times out takes seconds a frame: one after the first is enough to tell
            if len(frames) == 1:
                print(json.dumps({"ready": True}), flush=True)
                sys.stdin.readline()  # check() lets its processes draw on together
                end = time.time() + DRAW_S
            elif len(frames) > FRAMES and time.time() > end:
                break
            t0 = time.time()
        renderer.free()
    finally:
        context.free()
    rec.update(first_s=frames[0], frames=len(frames) - 1, slowest_s=max(frames[1:], default=None), bad_depth=bad)
    return rec


def answer(proc: subprocess.Popen, deadline: float) -> dict | None:
    """The next record a drawing process prints (other lines are skipped); None when none came by ``deadline``."""
    while (left := deadline - time.time()) > 0 and select.select([proc.stdout], [], [], left)[0]:
        line = proc.stdout.readline()
        if not line:
            try:
                code = proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                code = None
            return {"why": f"a drawing process ended (exit code {code}) without a record"}
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict):
            return rec
    return None


def combine(gpu: str, recs: list[dict]) -> dict:
    """The check's record from those of its drawing processes: the first failure of one, else the slowest
    first frame, each one's slowest later frame and the depth values outside 0..1 of all."""
    rec = {"gpu": str(gpu), "egl": next((r["egl"] for r in recs if "egl" in r), None)}
    failed = [r["why"] for r in recs if "why" in r]
    if failed:
        return {**rec, "ok": False, "why": failed[0]}
    frames = [max(r["first_s"] for r in recs)] + [r["slowest_s"] for r in recs if r["frames"]]
    bad = sum(r["bad_depth"] for r in recs)
    rec.update(first_s=frames[0], frames_s=frames[1:], frames=[r["frames"] for r in recs], bad_depth=bad)
    why = faults(frames, bad)
    rec["ok"] = not why
    if why:
        rec["why"] = "; ".join(why) + WHY_SUFFIX
    return rec


def check(gpu: str) -> dict:
    """CONTEXTS processes draw on nvidia-smi's GPU ``gpu``, together once each has made its first frame."""
    cmd = [sys.executable, "-m", "hexashell.benches.gpucheck", "--draw", str(gpu)]
    procs = [subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              bufsize=0) for _ in range(CONTEXTS)]  # unbuffered: what select sees is all there is
    try:
        deadline = time.time() + START_S + FIRST_LIMIT_S
        recs = [answer(p, deadline) or {"why": f"a drawing process made no first frame in"
                                               f" {START_S + FIRST_LIMIT_S:.0f} s{WHY_SUFFIX}"} for p in procs]
        for p, rec in zip(procs, recs):
            if rec.get("ready"):
                p.stdin.write(b"go\n")
        deadline = time.time() + DRAW_S + HANG_S
        recs = [(answer(p, deadline) or {"why": f"a drawing process gave no answer in {DRAW_S + HANG_S:.0f} s"
                                                f" of drawing with the others{WHY_SUFFIX}"})
                if rec.get("ready") else rec for p, rec in zip(procs, recs)]
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass  # stuck in the driver: the check answers all the same
    return combine(gpu, recs)


def check_depth(depth, camera: str):
    """``depth`` as the renderer read it back, when all of it are numbers; depth values that are not
    (NaN), which only a GPU that draws wrongly gives, raise an error that says so (robosuite would
    stop at a bare assertion)."""
    import numpy as np

    bad = int(np.count_nonzero(~np.isfinite(depth)))
    if bad:
        gpu = os.environ.get("HEXA_EGL_DEVICE", "?")
        raise RuntimeError(f"{bad} depth values of camera {camera} are not numbers: GPU {gpu} draws wrongly"
                           f" (python -m hexashell.benches.gpucheck {gpu} checks it){WHY_SUFFIX}")
    return depth


def check_time(seconds: float, camera: str, first: bool = False) -> None:
    """Nothing for a picture of ``camera`` rendered in ``seconds`` within its limit (FIRST_LIMIT_S for the first
    of an episode, else PICTURE_LIMIT_S); a slower one raises an error that says its GPU draws wrongly."""
    limit = FIRST_LIMIT_S if first else PICTURE_LIMIT_S
    if seconds > limit:
        gpu = os.environ.get("HEXA_EGL_DEVICE", "?")
        raise RuntimeError(f"a picture of camera {camera} took {seconds:.1f} s (over {limit:.0f} s): GPU {gpu} draws"
                           f" wrongly (python -m hexashell.benches.gpucheck {gpu} checks it){WHY_SUFFIX}")


def main(argv: list[str]) -> int:
    if len(argv) == 2 and argv[0] == "--draw" and argv[1].isdigit():  # one of check()'s drawing processes
        try:
            rec = draw(argv[1])
        except Exception as exc:  # noqa: BLE001 - the GPU is not fit to render; check() reads why
            rec = {"gpu": argv[1], "why": f"{type(exc).__name__}: {exc}"[:300]}
        print(json.dumps(rec), flush=True)
        return 0
    if len(argv) != 1 or not argv[0].isdigit():
        print(__doc__.split("\n\n")[1], file=sys.stderr)
        return 2
    try:
        rec = check(argv[0])
    except Exception as exc:  # noqa: BLE001 - the GPU is not fit to render; the caller reads why
        rec = {"gpu": argv[0], "ok": False, "why": f"{type(exc).__name__}: {exc}"[:300]}
    print(json.dumps(rec), flush=True)
    return 0 if rec["ok"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
