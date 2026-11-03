"""The RoboTwin 2.0 test protocol: which tasks and seeds, and what an episode may use.

The test follows RoboTwin's own evaluation of the clean2random setting (upstream
``scripts/eval_policy.sh``): the ``demo_randomized`` settings, each task's own step limit
(:data:`STEP_LIMITS`, RoboTwin's ``_eval_step_limit.yml``: 400 to 1700 steps), an instruction made from
the task's 'unseen' description templates (``eval_instruction: unseen`` of RoboTwin's demo_randomized
config), and per task the first 15 seeds from 100000 that the expert solves, found the way RoboTwin's
evaluation finds them (``scripts/setup/scan_robotwin_seeds.py``): 750 episodes.  Each task has 10 seeds for
learning and 10 for validation (``seeds.json``): the first development seeds the expert solves, from
seed 0 up; their instructions are made from the 'seen' templates (RoboTwin's demonstrations have those).
Seeds below 100000 are for development.

The tasks are those of RPent's RoboTwin evaluation list (``eval_seeds.json``, a copy of its
``demo_randomized.json``: 50 tasks, five seeds each, with their instructions; see NOTICE), which the seed
scan compares its first kept seeds with.

Scenes are built with three fixes to RoboTwin's runtime (:mod:`.sim`): the clutter area set back before
every build (a task with a shifted table, :data:`TABLE_SHIFT_X`, gets no clutter, as RoboTwin's published
evaluation built it), model files in numerical order, and for the tasks whose ``get_info`` runs the
expert's motions, the scene built again after it.

One step is one ``take_action`` of the environment: the joint targets of both arms and the two
gripper openings (14 numbers); the controller moves to them along a time-optimal path, so a step
lasts as long as its motion needs (0.2 s when nothing moves).  The episode ends the moment the
task's success check passes, or at the task's step limit.

Pure Python: both the episode side and the simulator server import it.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

EVAL_LIST = Path(__file__).resolve().parent / "eval_seeds.json"
SEED_LIST = Path(__file__).resolve().parent / "seeds.json"
#: the parts of a task's seeds in ``seeds.json``, and how many seeds each part has
PARTS = ("learn", "validation", "test")
PART_SEEDS = {"learn": 10, "validation": 10, "test": 15}
#: the step limit of a task's episodes, RoboTwin's own (``_eval_step_limit.yml`` of its evaluation)
STEP_LIMITS = {"adjust_bottle": 400, "beat_block_hammer": 400, "blocks_ranking_rgb": 1200,
               "blocks_ranking_size": 1200, "click_alarmclock": 400, "click_bell": 400, "dump_bin_bigbin": 600,
               "grab_roller": 400, "handover_block": 800, "handover_mic": 600, "hanging_mug": 900, "lift_pot": 400,
               "move_can_pot": 400, "move_pillbottle_pad": 400, "move_playingcard_away": 400,
               "move_stapler_pad": 400, "open_laptop": 700, "open_microwave": 1500, "pick_diverse_bottles": 400,
               "pick_dual_bottles": 400, "place_a2b_left": 400, "place_a2b_right": 400, "place_bread_basket": 700,
               "place_bread_skillet": 500, "place_burger_fries": 500, "place_can_basket": 700,
               "place_cans_plasticbox": 800, "place_container_plate": 400, "place_dual_shoes": 600,
               "place_empty_cup": 500, "place_fan": 400, "place_mouse_pad": 400, "place_object_basket": 700,
               "place_object_scale": 400, "place_object_stand": 400, "place_phone_stand": 400, "place_shoe": 500,
               "press_stapler": 400, "put_bottles_dustbin": 1700, "put_object_cabinet": 700, "rotate_qrcode": 400,
               "scan_object": 500, "shake_bottle": 700, "shake_bottle_horizontally": 700, "stack_blocks_three": 1200,
               "stack_blocks_two": 800, "stack_bowls_three": 1200, "stack_bowls_two": 900, "stamp_seal": 400,
               "turn_switch": 400}
#: replies of the model per episode, and the wall-clock budget of an episode (s)
REPLIES = 40
EPISODE_S = 7200.0
#: seeds from here on are evaluation seeds; below, development seeds
FIRST_EVAL_SEED = 100000
#: simulated seconds of a step in which nothing moves (50 physics steps of 1/250 s)
STEP_S = 0.2

#: our camera name -> RoboTwin's (the head camera is a static camera; the wrist cameras ride on link6)
CAMERAS = {"head": "head_camera", "left_wrist": "left_camera", "right_wrist": "right_camera"}
#: the robot's parts in a picture's part numbers (``observe(parts=True)``: 0 where the robot is not, else
#: the part's place here plus one), by aloha-agilex's links: each arm's (its base, link1-6, its camera),
#: its fingers (link7 and link8) and the rest of the robot (the base it stands on, the arms it does not use)
ROBOT_PARTS = ("left arm", "right arm", "left fingers", "right fingers", "body")

#: the table: 1.2 m along x, 0.7 m along y, centred on (shift, 0); its top is at TABLE_TOP_Z less
#: TABLE_LOWERED (m) in some tasks (RoboTwin's table_height_bias) and less a random lowering of up to
#: 3 cm per episode (measured from the start pictures)
TABLE_X_HALF = 0.6
TABLE_Y_HALF = 0.35
TABLE_TOP_Z = 0.74
TABLE_SHIFT_X = {"put_bottles_dustbin": 0.3, "dump_bin_bigbin": 0.3}
TABLE_LOWERED = {"place_dual_shoes": 0.1}


@lru_cache(maxsize=1)
def eval_list() -> dict:
    return json.loads(EVAL_LIST.read_text(encoding="utf-8"))


def tasks() -> list[str]:
    return sorted(eval_list()["tasks"])


def check_task(task: str) -> str:
    if task not in eval_list()["tasks"]:
        raise ValueError(f"{task!r} is not a RoboTwin 2.0 task (one of: {', '.join(tasks())})")
    return task


@lru_cache(maxsize=1)
def seed_list() -> dict:
    return json.loads(SEED_LIST.read_text(encoding="utf-8"))


def seeds(task: str, part: str) -> list[int]:
    """The task's learning, validation or test seeds (``part``: one of :data:`PARTS`)."""
    if part not in PARTS:
        raise ValueError(f"{part!r} is not one of {', '.join(PARTS)}")
    return [int(s) for s in seed_list()["tasks"][check_task(task)][part]]


def test_episodes() -> list[tuple[str, int]]:
    """(task, seed) of the 750 test episodes, task by task."""
    return [(task, seed) for task in tasks() for seed in seeds(task, "test")]


def published_episodes() -> list[tuple[str, int]]:
    """(task, seed) of the 250 episodes of RPent's evaluation list, task by task."""
    return [(task, int(e["seed"])) for task in tasks() for e in eval_list()["tasks"][task]]


def step_limit(task: str) -> int:
    """The steps of an episode of this task."""
    return STEP_LIMITS[check_task(task)]


def instruction_kind(seed: int) -> str:
    """The description templates an episode's instruction is made from: 'unseen' for a test episode,
    as RoboTwin's evaluation of the demo_randomized setting makes it, 'seen' for a development episode."""
    return "unseen" if is_eval_seed(seed) else "seen"


def is_eval_seed(seed: int) -> bool:
    return int(seed) >= FIRST_EVAL_SEED


def check_seed(task: str, seed: int) -> None:
    """An evaluation seed must be one of the task's test seeds (development seeds are free)."""
    if is_eval_seed(seed) and int(seed) not in seeds(task, "test"):
        raise ValueError(f"seed {seed} is not one of {task}'s test seeds (development seeds are below "
                         f"{FIRST_EVAL_SEED})")


def part_of(link: str) -> int:
    """The part number (see ROBOT_PARTS) of one of the robot's links."""
    for arm, side in (("left", "fl"), ("right", "fr")):
        if link in (f"{side}_link7", f"{side}_link8"):
            return ROBOT_PARTS.index(f"{arm} fingers") + 1
        if link.startswith(f"{side}_link") or link in (f"{side}_base_link", f"{arm}_camera"):
            return ROBOT_PARTS.index(f"{arm} arm") + 1
    return ROBOT_PARTS.index("body") + 1


def table_x(task: str) -> tuple[float, float]:
    shift = TABLE_SHIFT_X.get(task, 0.0)
    return (shift - TABLE_X_HALF, shift + TABLE_X_HALF)


def table_y() -> tuple[float, float]:
    return (-TABLE_Y_HALF, TABLE_Y_HALF)


def table_top(task: str) -> float:
    """The task's table top before an episode's random lowering."""
    return TABLE_TOP_Z - TABLE_LOWERED.get(task, 0.0)
