"""egl.pin for each GPU from a code root: python egl_pin.py CODE_ROOT"""
import sys

sys.path.insert(0, sys.argv[1])
from hexashell.benches import egl  # noqa: E402

rows = egl.devices()
smi = egl.gpus()
for g in sorted(smi):
    i = egl.egl_index(g, rows, smi)
    print(g, "->", i, rows[i] if i < len(rows) else None)
