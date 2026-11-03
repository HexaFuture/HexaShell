"""State of a pool batch: python3 -I scripts/pool_status.py DIR NAME [MINUTES]

DIR: the HexaShell folder the batch runs in (the one holding logs/NAME.todo).

Runner events (launches, exits by outcome, server trouble), episodes running, and the model requests
of the last MINUTES (default 10) over all records: status codes, retries and latency.
"""
import collections
import json
import os
import sys
import time

folder, name = sys.argv[1], sys.argv[2]
minutes = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
events = os.path.join(folder, "logs", f"{name}.events")
results = os.path.join(folder, "results", name)
todo = os.path.join(folder, "logs", f"{name}.todo")

launch = exits = 0
outcomes = collections.Counter()
trouble = []
running = {}
for line in open(events):
    words = line.split()
    if len(words) < 3:
        continue
    kind = words[2]
    if kind == "launch":
        launch += 1
        running[words[3]] = line[:19]
    elif kind == "exit":
        exits += 1
        running.pop(words[3], None)
        outcomes[line.split(": ", 1)[-1].strip()] += 1
    elif kind in ("given", "listed", "skipped") or "exited" in line or "no server" in line or "waiting" in line:
        trouble.append(line.strip())
    elif kind == "stopped":
        running.pop(words[1] if len(words) > 1 else "", None)
left = sum(1 for ln in open(todo) if ln.strip()) if os.path.exists(todo) else 0
cap = open(os.path.join(folder, "logs", f"{name}.max")).read().strip() if os.path.exists(
    os.path.join(folder, "logs", f"{name}.max")) else "-"
print(f"{name}: launched {launch}, exited {exits} {dict(outcomes)}, running {len(running)}, todo {left}, cap {cap}")
for t in trouble[-8:]:
    print("  !", t)

# model requests of the last minutes, from each record's events.jsonl (t is seconds since its start)
now = time.time()
status = collections.Counter()
lat, first = [], []
retries = 0
ends = collections.Counter()
for rec in sorted(os.listdir(results)) if os.path.isdir(results) else []:
    path = os.path.join(results, rec, "events.jsonl")
    if not os.path.exists(path):
        continue
    start = os.path.getmtime(os.path.join(results, rec, "prompt.md")) if os.path.exists(
        os.path.join(results, rec, "prompt.md")) else None
    res = os.path.join(results, rec, "result.json")
    if os.path.exists(res):
        try:
            r = json.load(open(res))
            ends[(r.get("end"), bool(r.get("success")))] += 1
        except ValueError:
            ends[("unreadable", False)] += 1
    if os.path.getmtime(path) < now - minutes * 60:
        continue
    for line in open(path):
        if '"kind": "request"' not in line:
            continue
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if start is not None and start + e.get("t", 0) < now - minutes * 60:
            continue
        status[e.get("status")] += 1
        if e.get("attempt", 1) > 1:
            retries += 1
        if e.get("status") == 200:
            lat.append(e.get("seconds", 0.0))
            if e.get("first_s") is not None:
                first.append(e["first_s"])


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else float("nan")


print(f"requests (last {minutes:.0f} min): {dict(status)}, retries {retries}, "
      f"seconds p50 {pct(lat, .5):.1f} p90 {pct(lat, .9):.1f} max {max(lat) if lat else float('nan'):.1f}, "
      f"first token p50 {pct(first, .5):.1f}")
done = sum(ends.values())
wins = sum(n for (end, ok), n in ends.items() if ok)
fails = collections.Counter()
for (end, ok), n in ends.items():
    if not ok:
        fails[end] += n
print(f"results: {done} records with result.json, {wins} success; failures by end {dict(fails)}")
