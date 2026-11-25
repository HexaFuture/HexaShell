#!/usr/bin/env python3
"""The episode list of a LIBERO-Pro 2.0 split, for scripts/bench_pool.py and scripts/bench_batch.sh:

    python3 scripts/liberopro2_lists.py SPLIT [--suite SUITE ...] [--base BASE ...] [--case CASE ...]
        [--seeds A-B] > logs/NAME.todo

SPLIT: learn, validation, test, revalidation or revalidation2 (hexashell/benches/liberopro2/protocol.py: all five
on the official test instances, each with its own seeds).  One line per episode, "liberopro2 TASK SEED", suite by
suite, base task, case, then seed, so that the episodes of a task follow each other (a warm server keeps the
task's environment).  --suite / --base (a base task, e.g. libero_goal_t3) / --case keep only those; --seeds
replaces the split's seeds (A-B inclusive) with some of them (any of the learning seeds for the learning split).
The count goes to stderr.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hexashell.benches.liberopro2 import protocol as P  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("split", choices=tuple(P.SPLITS))
    ap.add_argument("--suite", action="append", choices=P.SUITES, default=[])
    ap.add_argument("--base", action="append", default=[], help="a base task, e.g. libero_goal_t3")
    ap.add_argument("--case", action="append", choices=P.CASES, default=[])
    ap.add_argument("--seeds", default=None, help="A-B: these of the split's seeds (any learning seeds to learn)")
    args = ap.parse_args(argv)
    seeds = P.SPLITS[args.split]
    bases = {f"{s}_t{t}" for s in P.SUITES for t in P.TEST_TASKS[s]}
    for base in args.base:
        if base not in bases:
            ap.error(f"--base {base}: the base tasks are {', '.join(sorted(bases))}")
    if args.seeds:
        a, _, b = args.seeds.partition("-")
        seeds = tuple(range(int(a), int(b or a) + 1))
        allowed = P.LEARN_SEEDS + P.MORE_LEARN_SEEDS if args.split == "learn" else P.SPLITS[args.split]
        if not set(seeds) <= set(allowed):
            ap.error(f"--seeds: learning seeds are {P.LEARN_SEEDS[0]}-{P.LEARN_SEEDS[-1]} (initial state "
                     f"{P.INIT_STATE}) and {P.MORE_LEARN_SEEDS[0]}-{P.MORE_LEARN_SEEDS[-1]} (initial state = seed)"
                     if args.split == "learn" else f"--seeds: the {args.split} seeds are {allowed[0]}-{allowed[-1]}")
    n = 0
    for task in P.instances(args.split):
        suite, _, case = P.parse_task(task)
        if ((args.suite and suite not in args.suite) or (args.case and case not in args.case)
                or (args.base and task.rsplit("_", 1)[0] not in args.base)):
            continue
        for seed in seeds:
            print(f"liberopro2 {task} {seed}")
            n += 1
    print(f"{n} episodes", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
