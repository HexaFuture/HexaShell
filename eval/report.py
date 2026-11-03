"""Summarise episode records: success, progress, turns, time, tokens and pictures, per task and overall.

    python eval/report.py RESULTS_DIR [RESULTS_DIR ...] [--vs OTHER_DIR ...] [--layouts 0-9] [--seeds 0]

Each RESULTS_DIR holds one folder per episode (``<task>_s<seed>_l<layout>-<time>/result.json``).  An
episode played more than once counts once: its latest record, unless the gateway cut that one short and
an earlier one was not.  With ``--vs`` the two sets are compared on the episodes both have.
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def load(dirs: list[Path]) -> dict[tuple, dict]:
    plays: dict[tuple, list[dict]] = defaultdict(list)
    for folder in dirs:
        for path in folder.glob("*/result.json"):
            try:
                rec = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            rec["_dir"] = path.parent.name  # <episode>-<start time>: sorts by start time
            plays[(rec.get("task"), rec.get("seed"), rec.get("layout"))].append(rec)
    return {key: max(recs, key=lambda r: (r.get("end") != "gateway_error", r["_dir"]))
            for key, recs in plays.items()}


def score(rec: dict) -> float:
    """RoboDojo's progress (0..100); a success is 100, a failure without a score 0."""
    if rec.get("success"):
        return 100.0
    return float(rec.get("progress") or 0.0)


def _span(text: str | None) -> set[int] | None:
    if not text:
        return None
    out: set[int] = set()
    for part in text.split(","):
        lo, _, hi = part.partition("-")
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


def row(recs: list[dict]) -> dict:
    n = len(recs)
    mean = lambda xs: statistics.fmean(xs) if xs else 0.0  # noqa: E731
    return {
        "n": n,
        "sr": 100.0 * sum(bool(r.get("success")) for r in recs) / n if n else 0.0,
        "progress": mean([score(r) for r in recs]),
        "turns": mean([r.get("turns") or 0 for r in recs]),
        "minutes": mean([(r.get("seconds") or 0) / 60 for r in recs]),
        "steps": mean([r.get("step") or 0 for r in recs]),
        "prompt_k": mean([(r.get("tokens") or {}).get("prompt", 0) / 1000 for r in recs]),
        "out_k": mean([(r.get("tokens") or {}).get("completion", 0) / 1000 for r in recs]),
        "cached": 100.0 * sum((r.get("tokens") or {}).get("cached", 0) for r in recs)
        / max(1, sum((r.get("tokens") or {}).get("prompt", 0) for r in recs)),
        "pictures": mean([(r.get("pictures") or {}).get("shown", 0) for r in recs]),
        "cut": sum(r.get("end") == "gateway_error" for r in recs),
    }


HEAD = f"{'':34s} {'n':>3s} {'SR%':>5s} {'prog':>5s} {'turns':>5s} {'min':>5s} {'steps':>5s} " \
       f"{'in k':>6s} {'out k':>5s} {'cach%':>5s} {'pics':>4s} {'cut':>3s}"


def line(name: str, r: dict) -> str:
    return (f"{name[:34]:34s} {r['n']:3d} {r['sr']:5.1f} {r['progress']:5.1f} {r['turns']:5.1f} {r['minutes']:5.1f} "
            f"{r['steps']:5.0f} {r['prompt_k']:6.0f} {r['out_k']:5.1f} {r['cached']:5.1f} {r['pictures']:4.0f} "
            f"{r['cut']:3d}")


def report(kept: dict[tuple, dict]) -> None:
    by_task: dict[str, list[dict]] = defaultdict(list)
    for (task, _, _), rec in sorted(kept.items(), key=lambda kv: tuple(str(k) for k in kv[0])):
        by_task[task].append(rec)
    print(HEAD)
    for task in sorted(by_task):
        print(line(task, row(by_task[task])))
    everything = list(kept.values())
    print(line("ALL", row(everything)))
    print("ends:", dict(Counter(r.get("end") for r in everything)))
    statuses = Counter((r.get("finish") or {}).get("status") for r in everything if r.get("finish"))
    if statuses:
        print("finish() statuses:", dict(statuses))


def compare(a: dict[tuple, dict], b: dict[tuple, dict], names: tuple[str, str]) -> None:
    both = sorted(set(a) & set(b), key=lambda k: tuple(str(x) for x in k))
    print(f"\n{len(both)} episodes in both ({names[0]} vs {names[1]})")
    by_task: dict[str, list[tuple]] = defaultdict(list)
    for key in both:
        by_task[key[0]].append(key)
    print(f"{'':34s} {'n':>3s} {'SR ' + names[0]:>9s} {'SR ' + names[1]:>9s} {'prog':>6s} {'prog':>6s}  only-a only-b")
    for task in sorted(by_task) + ["ALL"]:
        keys = both if task == "ALL" else by_task[task]
        ra, rb = row([a[k] for k in keys]), row([b[k] for k in keys])
        wins = sum(bool(a[k].get("success")) and not b[k].get("success") for k in keys)
        losses = sum(bool(b[k].get("success")) and not a[k].get("success") for k in keys)
        print(f"{task[:34]:34s} {len(keys):3d} {ra['sr']:9.1f} {rb['sr']:9.1f} {ra['progress']:6.1f} {rb['progress']:6.1f}"
              f"  {wins:6d} {losses:6d}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--vs", nargs="+", type=Path, default=None)
    ap.add_argument("--names", default="a,b")
    ap.add_argument("--layouts", default=None, help="e.g. 0-9 or 10-19")
    ap.add_argument("--seeds", default=None, help="e.g. 0 or 0-2")
    args = ap.parse_args()
    layouts, seeds = _span(args.layouts), _span(args.seeds)

    def pick(kept):
        return {k: v for k, v in kept.items()
                if (layouts is None or k[2] in layouts) and (seeds is None or k[1] in seeds)}

    a = pick(load(args.dirs))
    report(a)
    if args.vs:
        compare(a, pick(load(args.vs)), tuple(args.names.split(",")[:2]))


if __name__ == "__main__":
    main()
