"""Start state of (task, seed) scenes, per joint, to compare two machines.

    source bench-env.sh; python scene_dump.py CODE_ROOT GPU OUT_DIR TASK:SEED ...

Writes OUT_DIR/<task>_s<seed>.json: {joint name: qpos values}.
"""
import json
import os
import sys

sys.path.insert(0, sys.argv[1])
os.environ["HEXA_EGL_DEVICE"] = sys.argv[2]
out = sys.argv[3]
os.makedirs(out, exist_ok=True)
import numpy as np  # noqa: E402

from hexashell.benches.robocasa import server as SRV  # noqa: E402

SRV.prepare()
for item in sys.argv[4:]:
    task, seed = item.split(":")
    srv = SRV.Server()
    srv.handle("open", {"task": task, "seed": int(seed)})
    env = srv.sim.env
    m, d = env.sim.model, env.sim.data
    sizes = {0: 7, 1: 4, 2: 1, 3: 1}  # free, ball, slide, hinge
    joints = {}
    for j in range(m.njnt):
        name = m.joint_id2name(j) if hasattr(m, "joint_id2name") else str(j)
        adr = int(m.jnt_qposadr[j])
        joints[name or f"j{j}"] = [float(x) for x in np.array(d.qpos[adr:adr + sizes[int(m.jnt_type[j])]])]
    json.dump(joints, open(os.path.join(out, f"{task}_s{seed}.json"), "w"))
    print(task, seed, len(joints), "joints")
    srv.handle("close", {})
    sys.stdout.flush()
