"""Compare scene_dump outputs of two machines: python -I scene_cmp.py DIR_A DIR_B"""
import json
import os
import sys

import math

a_dir, b_dir = sys.argv[1], sys.argv[2]
for f in sorted(os.listdir(a_dir)):
    if not os.path.exists(os.path.join(b_dir, f)):
        continue
    a, b = json.load(open(os.path.join(a_dir, f))), json.load(open(os.path.join(b_dir, f)))
    rows = []
    for name in a:
        if name not in b:
            rows.append((float("inf"), name, "missing"))
            continue
        va, vb = a[name], b[name]
        if len(va) == 7:  # free joint: position (m) and orientation (deg)
            dp = math.dist(va[:3], vb[:3])
            dot = abs(sum(x * y for x, y in zip(va[3:], vb[3:])))
            dr = math.degrees(2 * math.acos(min(1.0, dot)))
            rows.append((dp, name, f"pos {dp * 1000:.2f} mm, rot {dr:.2f} deg"))
        else:
            dq = max(abs(x - y) for x, y in zip(va, vb))
            rows.append((dq, name, f"{dq:.2e}"))
    rows.sort(reverse=True)
    big = [r for r in rows if r[0] > 1e-4]
    print(f"{f}: {len(rows)} joints, {len(big)} differ by > 1e-4; largest:")
    for r in rows[:4]:
        print(f"    {r[1]}: {r[2]}")
