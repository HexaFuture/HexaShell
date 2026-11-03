"""Score a batch: the latest finished record per (task, seed).

  python scripts/bench_score.py [--per-task] RESULTS_DIR [RESULTS_DIR2 ...]   (several dirs: compare per task)

RoboCasa records are also scored the official way: the success rate of each task set (atomic seen,
composite seen, composite unseen) and their weighted mean 18/16/16 over 50, each set's rate counting
its tasks equally.  --per-task lists every task of each run.  Dirs joined with "+" are one run (a batch whose
list was split over several runners).
"""
import glob
import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from hexashell.benches.robocasa import protocol as casa  # noqa: E402  (pure Python)


def load(d):
    best = {}
    files = [f for part in d.split("+") for f in glob.glob(os.path.join(part, "*", "result.json"))]
    for f in sorted(files, key=lambda f: os.path.basename(os.path.dirname(f))):
        try:
            r = json.load(open(f))
        except (OSError, ValueError):
            continue
        key = (r.get("task"), r.get("seed"))
        if r.get("end") in ("error", "gateway_error") and key in best:
            continue  # no verdict of its own: an earlier record's stands
        if r.get("end") == "error":
            r = {**r, "success": False}  # its verdict may be the server's previous episode
        best[key] = r  # sorted by stamp: the last one wins
    return best


def per_task(best):
    tasks = defaultdict(list)
    for (task, _), r in best.items():
        tasks[task].append(bool(r.get("success")))
    return tasks


def casa_score(best):
    """Lines of the official RoboCasa score, or none when the run has no RoboCasa records."""
    rows = {k: r for k, r in best.items() if r.get("bench") == "robocasa" or k[0] in casa.TASK_SET}
    if not rows:
        return []
    tasks = per_task(rows)
    splits = sorted({str(r.get("split") or casa.SPLITS[0]) for r in rows.values()})
    seeds = sorted({int(s) for _, s in rows if s is not None})
    lines = [f"  RoboCasa, split {', '.join(splits)}, seeds {seeds[0]}-{seeds[-1]}" if seeds else "  RoboCasa"]
    weighted, complete = 0.0, True
    for name, members in casa.TASKS.items():
        names = [t for t, _ in members]
        done = [t for t in names if t in tasks]
        ok = sum(sum(tasks[t]) for t in done)
        n = sum(len(tasks[t]) for t in done)
        # each task counts equally in its set (the official runs have 50 episodes per task)
        rate = sum(sum(tasks[t]) / len(tasks[t]) for t in done) / len(done) if done else 0.0
        complete &= len(done) == len(names)
        weighted += len(names) * rate
        lines.append(f"    {name:17} {ok:4}/{n:<4} {100 * rate:5.1f} %   ({len(done)}/{len(names)} tasks)")
    total = sum(len(m) for m in casa.TASKS.values())
    lines.append(f"    weighted (18/16/16): {100 * weighted / total:.1f}" + ("" if complete else "   (incomplete: each set's rate over the tasks run)"))
    return lines


def libero_table(best):
    """LIBERO-PRO: successes per perturbation and suite, and how many came within LIBERO-PRO's own step limit."""
    from hexashell.benches.libero import protocol as LP

    cells = defaultdict(lambda: [0, 0, 0])  # (perturbation, suite) -> successes, within the limit, episodes
    for (task, _), r in best.items():
        suite, _ = LP.parse_task(task)
        base, _, perturbation = suite.rpartition("_")
        if perturbation not in LP.PERTURBATIONS:
            continue
        for key in ((perturbation, base), (perturbation, "all"), ("all", base), ("all", "all")):
            c = cells[key]
            c[2] += 1
            if r.get("success"):
                c[0] += 1
                c[1] += int(r.get("step", float("inf")) <= LP.official_steps(task))
    if not cells:
        return
    bases = list(LP.BASES) + ["all"]
    print("  successes/episodes (within LIBERO-PRO's step limit):")
    print("  " + " " * 8 + "".join(f"{b.replace('libero_', ''):>16}" for b in bases))
    for p in list(LP.PERTURBATIONS) + ["all"]:
        row = [cells[(p, b)] if (p, b) in cells else None for b in bases]
        print(f"  {p:8}" + "".join(f"{f'{c[0]}/{c[2]} ({c[1]})' if c else '-':>16}" for c in row))


def main():
    args = sys.argv[1:]
    show_tasks = "--per-task" in args
    dirs = [a for a in args if a != "--per-task"]
    if not dirs:
        sys.exit(__doc__)
    runs = [("+".join(os.path.basename(os.path.normpath(p)) for p in d.split("+")), load(d)) for d in dirs]
    for name, best in runs:
        ok = sum(bool(r.get("success")) for r in best.values())
        ends = defaultdict(int)
        for r in best.values():
            if not r.get("success"):
                ends[r.get("end")] += 1
        print(f"{name}: {ok}/{len(best)} ({100.0 * ok / max(1, len(best)):.1f} %)  failures by end: {dict(ends)}")
        for line in casa_score(best):
            print(line)
        if any(r.get("bench") == "libero" for r in best.values()):
            libero_table(best)
        if show_tasks:
            for task, results in sorted(per_task(best).items()):
                print(f"    {task:32} {sum(results):3}/{len(results)}")
    if len(runs) > 1:
        tasks = sorted({k[0] for _, b in runs for k in b})
        print("per task (successes / episodes):")
        for t in tasks:
            cells = []
            for _, b in runs:
                rs = [r for k, r in b.items() if k[0] == t]
                cells.append(f"{sum(bool(r.get('success')) for r in rs)}/{len(rs)}")
            if len(set(cells)) > 1:
                print(f"  {t:32} " + "  ".join(f"{c:>5}" for c in cells))


if __name__ == "__main__":
    main()
