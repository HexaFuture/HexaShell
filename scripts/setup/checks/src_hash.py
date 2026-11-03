"""Hash of the .py/.xml/.json sources of robosuite, robocasa and mujoco as imported here."""
import hashlib
import os
import sys

import mujoco
import numpy
import robocasa
import robosuite

for mod in (robosuite, robocasa):
    root = os.path.dirname(mod.__file__)
    h = hashlib.sha1()
    n = 0
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if x not in ("__pycache__", "assets", "models"))
        for f in sorted(files):
            if f.endswith((".py", ".json", ".yaml")):
                p = os.path.join(d, f)
                h.update(os.path.relpath(p, root).encode())
                h.update(open(p, "rb").read())
                n += 1
    print(mod.__name__, root, n, h.hexdigest()[:12])
print("mujoco", mujoco.__version__, "numpy", numpy.__version__, sys.version.split()[0])
numpy.show_config() if "--np" in sys.argv else None
