"""The LIBERO-Pro 2.0 protocol: which episodes, how they start and end, and how they are scored.

LIBERO-Pro 2.0 (github.com/LIBERO-Pro-Team/LIBERO-Pro2.0; evaluation bundle 4046dc4 on Hugging Face) perturbs
the 40 tasks of LIBERO's four suites one way at a time: 22 static perturbations (S01-S22, in place from the start)
and 20 dynamic ones (D01-D18, D20, D21: events during the episode; the paper and the leaderboard call the
runtime's D20 and D21 D19 and D20).  Its task manifest (libero-pro2.0-1706-v1) lists 1688 instances (suite, base
task, case); BASE is the task unperturbed.  The official split (benchmark_split_v1) holds out two base tasks per
suite, with all their cases, for the test (:data:`TEST_TASKS`: 339 instances); the other 32 base tasks (1349
instances) are for training.  A task here is ``"<suite>_t<base task>_<case>"``, e.g. ``libero_goal_t9_S15``.

An episode starts as the official evaluation's (libero_remote/environment/libero.py): ``env.seed(seed)`` and the
perturbation seed, reset, an initial state of the base task, 10 control steps with a still arm and the gripper
opening, the perturbation seed again, and the rollout begins under relative OSC_POSE control (world-frame deltas).
It ends at the first control step after which the task's goal holds, or after the suite's ``max_steps``
(:data:`MAX_STEPS`, 20 Hz).

Our splits share the 339 test instances and the official start, initial state 0 (:func:`init_state_of`): of the
official example plan's 30 seeds per instance (142-171, :data:`OFFICIAL_SEEDS`) learning takes 142-146 (1695
episodes), validation 147-151 (1695) and the test 152-171 (6780 episodes; the reported test runs take its first ten
seeds, 152-161: 3390 episodes).  Learning may also start from initial state s with seed s, s = 1-49.  So no test
seed is seen before the test, but its instances and initial state are.  Two more splits validate changes on seeds
no run has used, from initial state 0 too: ``revalidation``, 172-181 (:data:`REVALIDATION_SEEDS`), and
``revalidation2``, 182-191 (:data:`REVALIDATION2_SEEDS`).  The other 32 base tasks (the official training ones,
1349 instances) are in no split: debugging only (``dev``, initial state s with seed s).

Scoring as the leaderboard's (:func:`score`): the success rate of each (suite, case) cell, the tasks of a cell
pooled; a group's score is the mean of its cells, a cell without episodes left out (never counted as 0); Base is
the mean of the four suites' BASE cells, and a group's Δ its score minus Base.  Only episodes that ran count.

Pure Python 3.8: both the episode side and the simulator server import it.
"""

from __future__ import annotations

import hashlib
import re

#: control period of the environment (20 Hz)
STEP_S = 0.05
#: control steps with a still arm and an opening gripper before the rollout begins (reset_stabilization_steps)
SETTLE_STEPS = 10
SETTLE_ACTION = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
#: our camera name -> the environment's camera
CAMERAS = {"agentview": "agentview", "wrist": "robot0_eye_in_hand"}
IMAGE_SIZE = 512
#: the robot's parts as the predicted robot pixels number them (1, 2, 3; 0 off the robot): the arm's moving links
#: with the hand, the fingers, and the rest (the arm's base and its stand, which do not move)
ROBOT_PARTS = ("arm", "fingers", "body")
#: the initial state the official plan starts every episode from (--init-state-ids 0)
INIT_STATE = 0
#: the initial states of each base task (LIBERO's init files)
INIT_STATES = 50

SUITES = ("libero_spatial", "libero_object", "libero_goal", "libero_10")
#: control steps of an episode after the settling steps: the manifest's max_steps of the suite
MAX_STEPS = {"libero_spatial": 220, "libero_object": 280, "libero_goal": 300, "libero_10": 520}
TASKS_PER_SUITE = 10
#: the official split (benchmark_split_v1.json): base tasks per suite
TRAIN_TASKS = {"libero_spatial": (1, 2, 3, 4, 5, 6, 7, 9), "libero_object": (0, 2, 3, 4, 5, 6, 7, 9),
               "libero_goal": (0, 1, 2, 4, 5, 7, 8, 9), "libero_10": (0, 1, 2, 3, 4, 6, 7, 9)}
TEST_TASKS = {"libero_spatial": (0, 8), "libero_object": (1, 8), "libero_goal": (3, 6), "libero_10": (5, 8)}

STATIC = tuple(f"S{i:02d}" for i in range(1, 23))
DYNAMIC = tuple(f"D{i:02d}" for i in (*range(1, 19), 20, 21))
CASES = ("BASE",) + STATIC + DYNAMIC
#: the runtime's case -> the paper's and leaderboard's name, where they differ
PAPER_ID = {"D20": "D19", "D21": "D20"}
#: (suite, base task) -> the cases the manifest has no instance of
MISSING = {
    **{("libero_spatial", t): ("S04",) for t in (0, 1, 2, 4, 6, 7, 8, 9)},
    ("libero_goal", 0): ("S02", "S05", "D02", "D03", "D05", "D17", "D18"),
    ("libero_goal", 3): ("S05",),
    ("libero_goal", 5): ("S02", "S03", "S09", "D02", "D13", "D17", "D18"),
    ("libero_goal", 7): ("S02", "S05", "D02", "D03", "D05", "D18"),
    ("libero_goal", 9): ("D03",),
    ("libero_10", 8): ("S04", "D06"),
}
#: the leaderboard's perturbation categories (catalogue.json), each with static and dynamic cases
CATEGORIES = {
    "condition": ("S01", "S02", "S03", "S04", "S05", "S07", "S08", "S09", "D01", "D02", "D03", "D04", "D05", "D06"),
    "environment": ("S06", "S10", "S11", "S12", "S13", "S14", "S22", "D07", "D08", "D09"),
    "observation": ("S15", "S16", "S19", "D10", "D11", "D12", "D13"),
    "execution": ("S20", "D14", "D15", "D16"),
    "robot": ("S17", "S18", "D17", "D18", "D20"),
    "language": ("S21", "D21"),
}

#: the official example plan's seeds (--seed 142 --num-seeds 30), all from INIT_STATE, shared out per instance:
#: the first 5 to learn on, the next 5 to validate on, the other 20 to test on
OFFICIAL_SEEDS = tuple(range(142, 172))
LEARN_SEEDS = OFFICIAL_SEEDS[:5]
VALIDATION_SEEDS = OFFICIAL_SEEDS[5:10]
TEST_SEEDS = OFFICIAL_SEEDS[10:]
#: further learning starts: seed s from initial state s (the start the official plan does not take)
MORE_LEARN_SEEDS = tuple(range(1, INIT_STATES))
#: after the test: new seeds, from INIT_STATE as the official ones, to validate the changes made after it
REVALIDATION_SEEDS = tuple(range(172, 182))
#: the same again for the fix after that revalidation
REVALIDATION2_SEEDS = tuple(range(182, 192))
SPLITS = {"learn": LEARN_SEEDS, "validation": VALIDATION_SEEDS, "test": TEST_SEEDS,
          "revalidation": REVALIDATION_SEEDS, "revalidation2": REVALIDATION2_SEEDS}

#: replies of the model per episode, and the wall-clock budget of an episode (s)
REPLIES = 40
EPISODE_S = 7200.0

_TASK = re.compile(r"^(libero_(?:spatial|object|goal|10))_t(\d)_(BASE|[SD]\d\d)$")


def task_name(suite: str, task: int, case: str) -> str:
    return f"{suite}_t{int(task)}_{case}"


def parse_task(name: str) -> tuple:
    """(suite, base task, case) of a task name ``"<suite>_t<base task>_<case>"``; raises for an instance the
    manifest does not have."""
    m = _TASK.match(str(name))
    if not m:
        raise ValueError(f"{name!r}: a LIBERO-Pro 2.0 task is named <suite>_t<base task>_<case>, "
                         "e.g. libero_goal_t9_S15")
    suite, task, case = m.group(1), int(m.group(2)), m.group(3)
    if case not in CASES:
        raise ValueError(f"{name!r}: LIBERO-Pro 2.0 has no case {case}")
    if case in MISSING.get((suite, task), ()):
        raise ValueError(f"{name!r}: the task manifest has no instance of {case} for {suite} task {task}")
    return suite, task, case


def record_name(task: str, seed: int) -> str:
    """The name of an episode's record folder before its time stamp: suite and base task, the case hidden behind a
    hash (the cells' working directory is in that folder)."""
    suite, index, _ = parse_task(task)
    return f"{suite}_t{index}_{hashlib.sha256(task.encode()).hexdigest()[:8]}_s{int(seed)}"


def kind(case: str) -> str:
    return "base" if case == "BASE" else "static" if case in STATIC else "dynamic"


def category(case: str):
    for name, cases in CATEGORIES.items():
        if case in cases:
            return name
    return None


def is_test_instance(task: str) -> bool:
    suite, index, _ = parse_task(task)
    return index in TEST_TASKS[suite]


def split_of(task: str, seed: int) -> str:
    """The split an episode is in ('learn', 'validation', 'test', 'revalidation', 'revalidation2'; 'dev' on a
    training base task); raises for a start no split has (a test instance with seed 0, or a seed that is neither
    official, revalidation nor an initial state's)."""
    seed = int(seed)
    if not is_test_instance(task):
        init_state_of(task, seed)
        return "dev"
    if seed in TEST_SEEDS:
        return "test"
    if seed in VALIDATION_SEEDS:
        return "validation"
    if seed in REVALIDATION_SEEDS:
        return "revalidation"
    if seed in REVALIDATION2_SEEDS:
        return "revalidation2"
    if seed in LEARN_SEEDS or seed in MORE_LEARN_SEEDS:
        return "learn"
    raise ValueError(f"{task} seed {seed}: no split starts a test instance so (learning seeds "
                     f"{LEARN_SEEDS[0]}-{LEARN_SEEDS[-1]} and {MORE_LEARN_SEEDS[0]}-{MORE_LEARN_SEEDS[-1]}, "
                     f"validation {VALIDATION_SEEDS[0]}-{VALIDATION_SEEDS[-1]}, test {TEST_SEEDS[0]}-{TEST_SEEDS[-1]}, "
                     f"revalidation {REVALIDATION_SEEDS[0]}-{REVALIDATION2_SEEDS[-1]})")


def init_state_of(task: str, seed: int) -> int:
    """The initial state an episode starts from: INIT_STATE for an official or revalidation seed, else the
    seed's own."""
    seed = int(seed)
    if seed in OFFICIAL_SEEDS or seed in REVALIDATION_SEEDS or seed in REVALIDATION2_SEEDS:
        return INIT_STATE
    if not 0 <= seed < INIT_STATES:
        raise ValueError(f"{task} seed {seed}: seeds other than the official ones start from initial state = seed, "
                         f"0-{INIT_STATES - 1}")
    return seed


def instances(split: str) -> list:
    """The task names of a split (all five, 'learn' to 'revalidation2': all the official test instances), suite
    by suite, base task, then case."""
    if split not in SPLITS:
        raise ValueError(f"no split {split!r}")
    return [task_name(s, t, c) for s in SUITES for t in TEST_TASKS[s] for c in CASES
            if c not in MISSING.get((s, t), ())]


def episodes(split: str) -> list:
    """(task, seed) of every episode of a split."""
    return [(task, seed) for task in instances(split) for seed in SPLITS[split]]


def max_steps(task: str) -> int:
    return MAX_STEPS[parse_task(task)[0]]


def _mean(values):
    values = list(values)
    return sum(values) / len(values) if values else None


def score(outcomes) -> dict:
    """The leaderboard's scores from ``(task, success)`` of the episodes that ran: per-cell rates (successes,
    episodes, rate), per group (static, dynamic, and each category within them) the mean rate of its cells with
    Δ against Base, Base, and Overall (all static and dynamic cells).  Rates are fractions (0..1)."""
    cells: dict = {}
    for task, success in outcomes:
        suite, _, case = parse_task(task)
        ok, n = cells.get((suite, case), (0, 0))
        cells[(suite, case)] = (ok + bool(success), n + 1)
    rate = {key: ok / n for key, (ok, n) in cells.items()}
    base = _mean(rate[(s, "BASE")] for s in SUITES if (s, "BASE") in rate)

    def group(cases):
        mean = _mean(r for (_, case), r in rate.items() if case in cases)
        return {"score": mean, "cells": sum(1 for (_, case) in rate if case in cases),
                "delta": None if mean is None or base is None else mean - base}

    out = {"base": base, "cells": {f"{s}/{c}": {"successes": ok, "episodes": n, "rate": ok / n}
                                   for (s, c), (ok, n) in sorted(cells.items())}}
    for mode, cases in (("static", STATIC), ("dynamic", DYNAMIC)):
        out[mode] = group(cases)
        out[mode]["categories"] = {name: group(tuple(c for c in members if c in cases))
                                   for name, members in CATEGORIES.items()}
    out["overall"] = group(STATIC + DYNAMIC)
    return out
