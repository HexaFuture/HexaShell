"""Signature of the scene a (task, seed) builds, to compare two machines.

    source bench-env.sh; python scene_sig.py CODE_ROOT GPU TASK:SEED ...

Prints per episode: the kitchen (layout, style), each object's model file, and hashes of the start
state (qpos exact, and rounded to 0.1 mm / 1e-4 rad).
"""
import hashlib
import os
import sys
import time

sys.path.insert(0, sys.argv[1])
os.environ["HEXA_EGL_DEVICE"] = sys.argv[2]
import numpy as np  # noqa: E402

from hexashell.benches.robocasa import server as SRV  # noqa: E402

SRV.prepare()
for item in sys.argv[3:]:
    task, seed = item.split(":")
    t = time.time()
    srv = SRV.Server()
    try:
        srv.handle("open", {"task": task, "seed": int(seed)})
        env = srv.sim.env
        qpos = np.array(env.sim.data.qpos, dtype=np.float64)
        exact = hashlib.sha1(qpos.tobytes()).hexdigest()[:12]
        rounded = hashlib.sha1(np.round(qpos, 4).tobytes()).hexdigest()[:12]
        layout, style = getattr(env, "layout_id", "?"), getattr(env, "style_id", "?")
        print(f"{task} s{seed}: layout {layout} style {style} nq {qpos.size} qpos {exact} rounded {rounded}"
              f" ({time.time() - t:.0f} s)")
        for c in getattr(env, "object_cfgs", []):
            print(f"    {c.get('name')}: {c.get('info', {}).get('mjcf_path', '').split('/assets/')[-1]}")
        srv.handle("close", {})
    except Exception as e:  # noqa: BLE001
        print(f"{task} s{seed}: FAIL {type(e).__name__}: {e}")
    sys.stdout.flush()
