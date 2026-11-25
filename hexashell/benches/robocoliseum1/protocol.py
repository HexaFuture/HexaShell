"""RoboColiseum (AgiBot Genie Sim v3.5.0, G2 omnipicker): its boards, tasks and layouts, our split of them,
and what an episode may use.

The benchmark's own configs (``geniesim_benchmark/config/g2op_*.yaml``) name a scene (``task_name``) and a
sub-task; the sub-task's layouts are the folders ``benchmark/config/llm_task/<sub_task>/<n>/`` (a fixed
scene: objects, poses, instructions).  A run of the benchmark plays the layouts given by
``--benchmark.instance_ids``; on every board a layout is played once per instruction it has (on the robust
board with the perturbation drawn from the seed, the layout number and the instruction's place).  The
robust board's official run draws its layouts from the seed (10 per config, seed 1) instead.

Split: a sub-task's layouts in order, number mod 5 = 0 for learning, = 1 for validation, the
rest for the test; all instructions of a layout fall in the same part.  The robust board's learning and
validation episodes use our own seed (2) on one layout per (task, perturbation) - the k-th perturbation of
a task takes the k-th learning or validation layout of its base task, cycling (:func:`robust_layout`);
its test is the official draw (seed 1) restricted to test layouts.

Scoring (the benchmark's): a task is a sequence of sub-steps, each with its own step limit; one action row
sent is one step; the checkers look at the scene every 30 steps; an episode ends when the last sub-step
is met, when a sub-step runs out of steps, or when the robot or an object leaves the allowed region (later
sub-steps score 0).  Episode progress = mean of the sub-step scores; success = all of them (E2E = 1).

Pure Python (numpy only in :func:`official_draw`).
"""

from __future__ import annotations

import os
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
#: the folder that holds Genie Sim 3.5 (local runs only): $HEXASHELL_GENIESIM35, else inside this checkout
GENIESIM = Path(os.environ.get("HEXASHELL_GENIESIM35") or ROOT / ".bench-data" / "geniesim-3.5")
#: genie_sim v3.5.0's checkout and benchmark package, and the copy of its benchmark/config whose scene files name
#: the assets by their local path (scripts/setup/geniesim35_native_config.sh makes it)
GENIESIM_ROOT = Path(os.environ.get("GENIESIM_ROOT") or GENIESIM / "genie_sim")
PKG = GENIESIM_ROOT / "source" / "geniesim_benchmark" / "src" / "geniesim_benchmark"
NATIVE_CONFIG = Path(os.environ.get("GENIESIM_NATIVE_CONFIG") or GENIESIM / "benchmark_config")

BOARDS = ("instruction", "spatial", "manip", "robust")
#: config prefix of each board's tasks (the robust ones are g2op_robust_<kind>_<instruction task>)
PREFIX = {"instruction": "g2op_if_", "spatial": "g2op_spatial_", "manip": "g2op_manip_"}
#: task -> (scene, sub-task, layouts)
INSTRUCTION = {
    "pick_billiards_color": ("table_task_1_g2_op", "pick_billiards_color", 25),
    "pick_block_color": ("table_task_1_g2_op", "pick_block_color", 20),
    "pick_block_number": ("table_task_0_g2_op", "pick_block_number", 19),
    "pick_block_shape": ("table_task_0_g2_op", "pick_block_shape", 20),
    "pick_block_size": ("table_task_1_g2_op", "pick_block_size", 50),
    "pick_common_sense": ("table_task_0_g2_op", "pick_common_sense", 25),
    "pick_follow_logic_or": ("table_task_0_g2_op", "pick_follow_logic_or", 50),
    "pick_object_type": ("table_task_0_g2_op", "pick_object_type", 25),
    "pick_specific_object": ("table_task_0_g2_op", "pick_specific_object", 25),
    "straighten_object": ("table_task_0_g2_op", "straighten_object", 50),
}
SPATIAL = {name: ("table_task_2_g2_op", name, 30) for name in (
    "pick_object_absolute_position", "pick_object_relative_position", "place_beverage_to_anothers_position",
    "place_object_relative_position", "sort_cubes_by_size", "sort_number", "stack_bowls",
    "stack_three_building_blocks")}
MANIP = {
    "bimanual_hold_pot": ("kitchen_02_g2_op", "hold_pot", 20),
    "clean_the_desktop": ("study_room_05_g2_op", "clean_the_desktop", 10),
    "open_door": ("home_g2_op", "open_door", 20),
    "place_block_into_box": ("table_task_2_g2_op", "place_block_into_box", 20),
    "pour_workpiece": ("laboratory_06_g2_op", "pour_workpiece", 20),
    "scoop_popcorn": ("popcorn_g2_op", "scoop_popcorn", 20),
    "sorting_packages_continuous": ("warehouse_g2_op", "sorting_packages_continuous", 10),
    "sorting_packages": ("warehouse_g2_op", "sorting_packages", 10),
    "stock_and_straighten_shelf": ("market_00_g2_op", "stock_and_straighten_shelf", 10),
    "take_wrong_item_shelf": ("market_01_g2_op", "take_wrong_item_shelf", 20),
}
ROBUST_KINDS = ("backgen", "camposgen", "camqualgen", "instructgen", "posegen")
TASKS = {"instruction": INSTRUCTION, "spatial": SPATIAL, "manip": MANIP}

PARTS = ("learn", "validation", "test")
#: sub-tasks whose checkers ask a vision-language judge (:mod:`.judge`)
VLM_JUDGED = frozenset({"scoop_popcorn"})
#: the robust board's seeds for learning and validation (the official draw is seed 1)
ROBUST_OWN_SEED = 2
OFFICIAL_SEED = 1
#: layouts the official run of a robust config draws (``num_instances`` of every g2op_robust_*.yaml)
OFFICIAL_DRAW = 10

#: replies of the model and wall-clock seconds per episode, by board
REPLIES = {"instruction": 40, "robust": 40, "spatial": 60, "manip": 80}
EPISODE_S = {"instruction": 3600.0, "robust": 3600.0, "spatial": 5400.0, "manip": 5400.0}

CAMERAS = ("head", "left_wrist", "right_wrist")
CAMERA_WORDS = {"head": "on the head, looking down at the work area in front of the robot",
                "left_wrist": "on the left gripper, looking along its fingers",
                "right_wrist": "on the right gripper, looking along its fingers"}


class TaskError(ValueError):
    pass


def board_of(config: str) -> str:
    for board, prefix in PREFIX.items():
        if config.startswith(prefix):
            return board
    if config.startswith("g2op_robust_"):
        return "robust"
    raise TaskError(f"not a RoboColiseum config: {config!r}")


def parse_config(config: str) -> dict:
    """``g2op_if_pick_block_color`` -> its board, task, scene, sub-task, layouts (and, for the robust board,
    the perturbation and the instruction task it perturbs)."""
    config = str(config).removesuffix(".yaml")
    board = board_of(config)
    if board == "robust":
        m = re.fullmatch(r"g2op_robust_(\w+?)_(\w+)", config)
        if not m or m.group(1) not in ROBUST_KINDS or m.group(2) not in INSTRUCTION:
            raise TaskError(f"not a RoboColiseum robust config: {config!r}")
        kind, base = m.group(1), m.group(2)
        scene, sub, layouts = INSTRUCTION[base]
        return {"config": config, "board": board, "task": base, "kind": kind, "scene": f"{scene}_{kind}",
                "sub_task": sub, "layouts": layouts}
    task = config[len(PREFIX[board]):]
    if task not in TASKS[board]:
        raise TaskError(f"unknown {board} task {task!r}")
    scene, sub, layouts = TASKS[board][task]
    return {"config": config, "board": board, "task": task, "kind": None, "scene": scene, "sub_task": sub,
            "layouts": layouts}


def config_name(board: str, task: str, kind: str | None = None) -> str:
    if board == "robust":
        if kind not in ROBUST_KINDS or task not in INSTRUCTION:
            raise TaskError(f"robust task {task!r} / perturbation {kind!r}")
        return f"g2op_robust_{kind}_{task}"
    if task not in TASKS.get(board, {}):
        raise TaskError(f"unknown {board} task {task!r}")
    return PREFIX[board] + task


def configs(boards=BOARDS) -> list[str]:
    out = []
    for board in boards:
        if board == "robust":
            out += [config_name(board, t, k) for t in INSTRUCTION for k in ROBUST_KINDS]
        else:
            out += [config_name(board, t) for t in TASKS[board]]
    return out


def part_of(layout: int) -> str:
    """Split A: which part a layout belongs to."""
    r = int(layout) % 5
    return "learn" if r == 0 else "validation" if r == 1 else "test"


def layouts(config: str, part: str) -> list[int]:
    if part not in PARTS:
        raise TaskError(f"part must be one of {PARTS}, not {part!r}")
    n = parse_config(config)["layouts"]
    return [k for k in range(n) if part_of(k) == part]


def check_layouts() -> list[str]:
    """Differences between the table above and the layout folders of the native config (none expected)."""
    problems = []
    for board in ("instruction", "spatial", "manip"):
        for task, (_, sub, n) in TASKS[board].items():
            folder = NATIVE_CONFIG / "llm_task" / sub
            found = sorted(int(p.name) for p in folder.iterdir() if p.is_dir() and p.name.isdigit()) \
                if folder.is_dir() else []
            if found != list(range(n)):
                problems.append(f"{task}: {len(found)} layout folders, expected 0..{n - 1}")
    return problems


def robust_layout(config: str, part: str) -> int:
    """The one layout a robust config plays in learning or validation: the k-th perturbation of its task
    (in ``ROBUST_KINDS`` order) takes the k-th layout of that part, cycling."""
    c = parse_config(config)
    if c["board"] != "robust" or part == "test":
        raise TaskError(f"robust_layout is for robust configs in learning or validation, not {config!r} / {part!r}")
    ls = layouts(config, part)
    return ls[ROBUST_KINDS.index(c["kind"]) % len(ls)]


def official_draw(config: str) -> list[int]:
    """The layouts the official run of a robust config plays, drawn as the benchmark draws them
    (``task_benchmark.py``: ``np.random.RandomState(seed).choice(ids, num_instances, replace=False)``, sorted;
    all of them when there are not more).  Needs numpy, unlike the rest of this module."""
    import numpy as np

    c = parse_config(config)
    if c["board"] != "robust":
        raise TaskError(f"official_draw is for robust configs, not {config!r}")
    ids = list(range(c["layouts"]))
    if OFFICIAL_DRAW >= len(ids):
        return ids
    return sorted(np.random.RandomState(OFFICIAL_SEED).choice(ids, size=OFFICIAL_DRAW, replace=False).tolist())


def check_draw() -> list[str]:
    """Differences between OFFICIAL_SEED / OFFICIAL_DRAW and the robust configs of the benchmark (none expected)."""
    problems = []
    for config in configs(("robust",)):
        path = PKG / "config" / f"{config}.yaml"
        text = path.read_text() if path.exists() else ""
        seed = re.findall(r"^\s*seed:\s*(\S+)\s*$", text, re.M)
        draw = re.findall(r"^\s*num_instances:\s*(\S+)\s*$", text, re.M)
        if seed != [str(OFFICIAL_SEED)] or draw != [str(OFFICIAL_DRAW)]:
            problems.append(f"{config}: seed {seed}, num_instances {draw}")
    return problems


def seed_for(board: str, part: str) -> int:
    """The benchmark seed of an episode: the robust board's own draw for learning and validation, else the
    official one."""
    return ROBUST_OWN_SEED if board == "robust" and part != "test" else OFFICIAL_SEED


def limits(board: str) -> dict:
    """The agent's limits for an episode of this board (benches.yml and --set may change them)."""
    return {"max_turns": REPLIES[board], "budget_s": EPISODE_S[board], "ladder": False}
