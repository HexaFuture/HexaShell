"""SIMPLE: what the episode side and the server side share (standard library only).

Tasks: the five of HITCH's six SIMPLE tasks whose code is public (XMoveBendPick's is not, and it is
left out).  The two motion-planning (MP) tasks run SIMPLE's AMO whole-body controller, the three
teleoperation tasks its decoupled whole-body controller.

Time: one environment step is 0.02 s (50 Hz commands; MP tasks: MuJoCo steps 0.002 s, 10 per command;
teleoperation tasks: 0.005 s, 4 per command).  Before an episode's first step the robot stands as in the
official evaluation, and those steps do not count: MP tasks STAND_STEPS steps (SIMPLE's
StandStabilizationWrapper), teleoperation tasks until the robot reports that it stands still, at most 300
steps (the stabilisation loop of SIMPLE's teleoperation evaluation).  An episode runs until the task's first success
or RELAXED_STEPS (45 s), whichever comes first; it is also counted against the official limit: a
success within the official limit is one whose first success step is at most the task's
``official_steps``.

Episodes: learning and validation episodes are fresh layouts that SIMPLE's domain randomisation draws
after Python's and NumPy's global generators are seeded with the episode's seed (SIMPLE's
``reset(seed=...)`` does not reach them).  They are made the way the official episodes were made, as their
data shows (meta/episodes.jsonl.pre-fk holds each layout as drawn; meta/episodes.jsonl the same layouts with
only the robot's pose changed, by about what one stand moves the robot; in four of the five tasks by one
change for all 30 episodes, as if one stand's had been applied to every layout: the teleoperation tasks
6.07 cm back, 0.25 cm right, 0.17 deg right; BendPick 3.34 cm back, 0.35 cm left, 3.27 deg right; ours
differ per layout, each stood on its own; TabletopGrasp's differ per episode too): the layout is drawn, the robot
stands (the official stand above), the layout is saved with the robot's pose replaced by where that stand
left it (the pelvis's x and y and its heading; the height as drawn), the saved layout is restored as the
official evaluation restores an episode (SIMPLE's reset with the state dict, no DR level), and the robot
stands again.  SIMPLE's restore takes the saved position but not the saved heading (no task has a robot
orientation range), so the robot starts the second stand facing along x, in ours and in the official
evaluation alike.  Only the steps are copied, not their official results; the lights and everything else
are as the task's randomisation draws them, with two exceptions where all official episodes depart from it:
TabletopGrasp's robot x is drawn from the half of its configured range nearer the table, where the official
episodes' drawn robots all are, and XMovePick's robot x and table height are the ones all its official
episodes were drawn with, -0.80 m and 0.58 m (Task.fresh_layout_robot_x_range_m,
Task.fresh_layout_table_height_range_m).  Test episodes are the official evaluation episodes (dr-level-0, -1,
-2, ten each) and are never used while developing.

Gaze camera (HITCH's active camera, added to SIMPLE's head): RGB-D, GAZE_WIDTH x GAZE_HEIGHT pixels,
GAZE_HFOV_DEG across (64 deg up and down), on a pan-tilt that turns about the camera's own centre at
SIMPLE's head-camera position on the torso.  Pitch is down from the torso's horizontal, yaw is to the
left of the torso's forward direction; it turns at GAZE_SPEED_DEG_S and the turn takes simulated time.

What the model gets: the gaze camera's RGB-D pictures and the camera model,
the robot's own state, the instruction and the budget.  One frame for everything, the start frame: origin on
the ground under the pelvis at the episode's first step, x along the pelvis's heading then, y to its left, z
up; it stays fixed while the robot walks.  Where the pelvis is in it (and so where the camera is) comes from
the simulator: the robot's odometry without drift.  This departs from HITCH, whose observation frame O is
taken anew at each observation and whose walks are only approximately known; a fixed frame is what
HexaShell's other mobile robot (RoboCasa's base) has, and it keeps measured objects valid across walks.
Objects' poses and the success check's terms stay in the server.

Episode seeds (SPLITS): learning and validation layouts are drawn fresh (LEARNING_SEED, VALIDATION_SEED); the
test is SIMPLE's official evaluation episodes (the 30 of dr-level-0, -1 and -2, TEST_SEED: official_episode),
restored from their saved configuration as SIMPLE's evaluation restores them.  Test seeds are refused unless
the run allows them (OFFICIAL_TEST_VARIABLE set to 1 for the episode side; ``official_test`` in the server's
open): development runs, scripts included, never open an official episode by accident.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Task:
    #: gymnasium id that SIMPLE registers
    env_id: str
    #: lower-body controller: "amo" (MP tasks) or "decoupled_wbc" (teleoperation tasks)
    controller: str
    #: the official limit: max_episode_steps of the task's metadata (the official evaluation's TimeLimit)
    official_steps: int
    #: the official instruction (the task's language randomiser has this one template)
    instruction: str
    #: target object of fresh layouts (gym.make's target_object for MP tasks, target for teleoperation tasks), when
    #: SIMPLE's default differs from the one all official episodes hold; None: SIMPLE's default.  The official
    #: evaluation restores each episode's saved target, so this only matters for learning and validation layouts.
    target_object: str | None = None
    #: the range (m) fresh layouts draw the robot's x from, when it differs from the task's DR config; None: the
    #: config's.  Each seed draws the same fraction of the range as with the config's (one uniform draw, same
    #: ends in the same order), so the layouts are otherwise those of the config's range; equal ends fix the
    #: value (the draw still happens).  The official evaluation restores each episode's saved robot position,
    #: so this only matters for learning and validation layouts.
    fresh_layout_robot_x_range_m: tuple[float, float] | None = None
    #: likewise the range (m) fresh layouts draw the table top's height from (the scene randomiser's
    #: table_height); None: the config's
    fresh_layout_table_height_range_m: tuple[float, float] | None = None


TASKS: dict[str, Task] = {
    "bend_pick": Task("simple/G1WholebodyBendPickMP-v0", "amo", 400,
                      "bend to pick up the cracker box"),
    # SIMPLE's code defaults to the apple (graspnet1b:12); the task's own DR config and all 30 official episodes
    # have the chips can (graspnet1b:10)
    # the robot's x: the task's DR config (and the one saved in every official episode) says -1.1 to -1.0 m, but
    # the 30 official episodes' drawn robots (meta/episodes.jsonl.pre-fk) all have x in -1.045 to -1.002, the
    # half nearer the table (ten uniform draws all in one half: about 1 in 1000).  Fresh layouts draw from that
    # half; with the config's range their targets stood 5.4 cm farther on average.
    "tabletop_grasp": Task("simple/G1WholebodyTabletopGraspMP-v0", "amo", 800,
                           "grasp the object on the tabletop.", target_object="graspnet1b:10",
                           fresh_layout_robot_x_range_m=(-1.05, -1.0)),
    # the task's DR config (and the one saved in every official episode) has the apple (graspnet1b:12), but all 30
    # official episodes hold the cracker box (graspnet1b:0), which the distractor config excludes; with the apple
    # a distractor can be a second apple, and MuJoCo then refuses the scene (a repeated body name).
    # The robot and the table: the config (and the one saved in every official episode) draws the robot's x from
    # -1.0 to -0.9 m and the table top's height from 0.45 to 0.58 m, but all 30 official episodes were drawn with
    # the robot at x -0.80 m (y 0, facing along x; meta/episodes.jsonl.pre-fk) and the top at 0.58 m, as if from
    # another version of the config.  Fresh layouts use those two values; with the config's
    # the robot stood 10-20 cm farther and the table up to 13 cm lower.
    "xmove_pick": Task("simple/G1WholebodyXMovePickTeleop-v0", "decoupled_wbc", 800,
                       "move forward to pick up the {}", target_object="graspnet1b:0",
                       fresh_layout_robot_x_range_m=(-0.80, -0.80),
                       fresh_layout_table_height_range_m=(0.58, 0.58)),
    "locomotion_pick_between_tables": Task("simple/G1WholebodyLocomotionPickBetweenTablesTeleop-v0",
                                           "decoupled_wbc", 1200,
                                           " pick up the {} from table1,locomotion to table2,and place  on table2."),
    "handover": Task("simple/G1WholebodyHandoverTeleop-v0", "decoupled_wbc", 800,
                     "Hand over {} from right hand to left hand and place it on the container."),
}

STEP_S = 0.02
STAND_STEPS = 60
RELAXED_SECONDS = 45.0
RELAXED_STEPS = int(round(RELAXED_SECONDS / STEP_S))
#: render rate SIMPLE's G1 tasks require (their metadata), = 1 / STEP_S
RENDER_HZ = 50
#: MP tasks' success criterion (the MP evaluation passes it); the teleoperation evaluation passes none, so those
#: tasks keep their own (XMovePick 0.9, LocomotionPickBetweenTables 0.4 of a reward that grows each step the
#: object rests in the container untouched, Handover 0.9)
SUCCESS_CRITERIA = 0.9

#: official evaluation episodes: folder per task under the evaluation data, DR levels, episodes per level
OFFICIAL_LEVELS = ("dr-level-0", "dr-level-1", "dr-level-2")
OFFICIAL_EPISODES_PER_LEVEL = 10
#: where the official evaluation data lie, in the server's environment (bench-simple's env file):
#: <dir>/<the task's environment>/<level>/meta/episodes.jsonl, as SIMPLE's evaluation data unpack
OFFICIAL_DATA_VARIABLE = "SIMPLE_EVAL_DIR"
#: in the episode side's environment, "1" allows the official evaluation episodes (test seeds) in this run
OFFICIAL_TEST_VARIABLE = "HEXASHELL_SIMPLE_OFFICIAL_TEST"

#: seeds of fresh layouts: learning episodes LEARNING_SEED + i, validation episodes VALIDATION_SEED + i
LEARNING_SEED = 1000
VALIDATION_SEED = 2000
SPLIT_SIZE = 1000
#: seeds of the official evaluation episodes (the test): TEST_SEED + OFFICIAL_EPISODES_PER_LEVEL x level + the
#: episode's index in its level, i.e. 3000-3009 dr-level-0, 3010-3019 dr-level-1, 3020-3029 dr-level-2
TEST_SEED = 3000
TEST_EPISODES = len(OFFICIAL_LEVELS) * OFFICIAL_EPISODES_PER_LEVEL

GAZE_CAMERA = "gaze"
GAZE_WIDTH = 640
GAZE_HEIGHT = 400
GAZE_HFOV_DEG = 90.0
GAZE_PITCH_RANGE_DEG = (0.0, 60.0)
GAZE_YAW_RANGE_DEG = (-45.0, 45.0)
#: where the gaze points at the start of an episode (SIMPLE's own head camera looks 47.6 deg down)
GAZE_START_DEG = (0.0, 45.0)  # (yaw, pitch)
#: the pan-tilt's speed (an assumption; HITCH does not state one)
GAZE_SPEED_DEG_S = 120.0
#: depth range of the gaze camera (m); depth beyond the far end reads 0 (no depth), as a depth camera gives
GAZE_NEAR_M = 0.05
GAZE_FAR_M = 10.0

#: who draws the pictures: "isaac" (Isaac Sim, as in the official evaluation) or "mujoco" (MuJoCo's own
#: renderer, no room around the scene: for quick tests); physics is MuJoCo's either way
RENDERERS = ("isaac", "mujoco")
#: Isaac Sim frames rendered for one picture (Isaac only draws when a picture is taken).  Its real-time
#: renderer builds indirect light over frames: the first frame after the scene changed is darker where light
#: is indirect.  Against SIMPLE's every-step rendering of the same state (learning seed 1001,
#: TabletopGrasp), pixels off by more than 30 of 255: 1 frame 3.1 %, 4 frames 0.7 %, 8 frames 0.47 %,
#: 16 to 128 frames 0.4-0.8 %; two every-step frames in a row 0.37 %.
ISAAC_FRAMES_PER_PICTURE = 8

#: HITCH's body command ranges (pelvis height above the ground, torso pitch forward, torso yaw left)
PELVIS_HEIGHT_RANGE_M = (0.30, 0.79)
TORSO_PITCH_RANGE_DEG = (0.0, 60.0)
TORSO_YAW_RANGE_DEG = (-45.0, 45.0)
#: HITCH's hand command ranges: waypoints per hand per command, seconds per waypoint
HAND_WAYPOINTS_MAX = 12
HAND_SEGMENT_S = (0.4, 8.0)

SIDES = ("left", "right")

#: an episode's budget for the model: replies, and wall-clock seconds
REPLIES = 60
EPISODE_S = 7200.0
#: the cameras the model has (one), and other names it may use for them
CAMERAS = (GAZE_CAMERA,)
CAMERA_ALIASES = {"head": GAZE_CAMERA}
#: steps one wait may hold
WAIT_MAX_STEPS = 400
#: named palm orientations, in the robot's own axes (x its heading, y its left, z up): the palm's normal (the way
#: the palm faces) and the fingers' direction (from the wrist to the finger roots); "inward" faces the palm toward
#: the robot's middle (left hand: to its right), "outward" away from it
PALM_FACING = ("down", "up", "forward", "inward", "outward")
#: episodes by seed: fresh layouts for learning [1000, 2000) and validation [2000, 3000), the official evaluation
#: episodes for the test [3000, 3030)
SPLITS = {"learning": (LEARNING_SEED, LEARNING_SEED + SPLIT_SIZE),
          "validation": (VALIDATION_SEED, VALIDATION_SEED + SPLIT_SIZE),
          "test": (TEST_SEED, TEST_SEED + TEST_EPISODES)}


def palm_axes(facing: str, side: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A named palm orientation's (normal, finger direction) in the robot's own axes."""
    inward_y = -1.0 if side == "left" else 1.0
    table = {"down": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)), "up": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
             "forward": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "inward": ((0.0, inward_y, 0.0), (1.0, 0.0, 0.0)),
             "outward": ((0.0, -inward_y, 0.0), (1.0, 0.0, 0.0))}
    if facing not in table:
        raise ValueError(f"palm facing {facing!r}: one of {', '.join(PALM_FACING)}")
    return table[facing]


def split_of(seed: int) -> str | None:
    """The split a seed's episode belongs to (None: no split's)."""
    for name, (lo, hi) in SPLITS.items():
        if lo <= int(seed) < hi:
            return name
    return None


def official_test_allowed(environ=None) -> bool:
    """Whether this run may open the official evaluation episodes (OFFICIAL_TEST_VARIABLE is 1)."""
    value = (os.environ if environ is None else environ).get(OFFICIAL_TEST_VARIABLE, "")
    return value.strip().lower() in ("1", "true", "yes", "on")


def check_seed(seed: int, official_test: bool = False) -> int:
    """A seed of one of the splits; a test seed (an official evaluation episode) only with ``official_test``."""
    split = split_of(seed)
    if split is None:
        raise ValueError(f"seed {seed}: learning seeds are {LEARNING_SEED}-{LEARNING_SEED + SPLIT_SIZE - 1}, "
                         f"validation seeds {VALIDATION_SEED}-{VALIDATION_SEED + SPLIT_SIZE - 1}, test seeds (the "
                         f"official evaluation episodes) {TEST_SEED}-{TEST_SEED + TEST_EPISODES - 1}")
    if split == "test" and not official_test:
        raise ValueError(f"seed {seed} is an official evaluation episode (the test), which development runs do not "
                         f"open; a test run sets {OFFICIAL_TEST_VARIABLE}=1")
    return int(seed)


def check_task(task: str) -> Task:
    if task not in TASKS:
        raise ValueError(f"unknown SIMPLE task {task!r}; tasks: {', '.join(TASKS)}")
    return TASKS[task]


def official_success(first_success_step: int | None, task: str) -> bool:
    """A success counts within the official limit when it came at or before the task's official step limit."""
    return first_success_step is not None and first_success_step <= TASKS[task].official_steps


def learning_seed(index: int) -> int:
    if not 0 <= index < SPLIT_SIZE:
        raise ValueError(f"learning episode index {index} outside 0..{SPLIT_SIZE - 1}")
    return LEARNING_SEED + index


def validation_seed(index: int) -> int:
    if not 0 <= index < SPLIT_SIZE:
        raise ValueError(f"validation episode index {index} outside 0..{SPLIT_SIZE - 1}")
    return VALIDATION_SEED + index


def test_seed(level: str, index: int) -> int:
    """The seed of the official evaluation episode ``index`` (its episode_index) of DR level ``level``."""
    if level not in OFFICIAL_LEVELS:
        raise ValueError(f"official level {level!r}: one of {', '.join(OFFICIAL_LEVELS)}")
    if not 0 <= int(index) < OFFICIAL_EPISODES_PER_LEVEL:
        raise ValueError(f"official episode index {index} outside 0..{OFFICIAL_EPISODES_PER_LEVEL - 1}")
    return TEST_SEED + OFFICIAL_EPISODES_PER_LEVEL * OFFICIAL_LEVELS.index(level) + int(index)


def official_episode(seed: int) -> tuple[str, int]:
    """(DR level, episode index) of a test seed's official evaluation episode."""
    if split_of(seed) != "test":
        raise ValueError(f"seed {seed} is not a test seed ({TEST_SEED}-{TEST_SEED + TEST_EPISODES - 1})")
    level, index = divmod(int(seed) - TEST_SEED, OFFICIAL_EPISODES_PER_LEVEL)
    return OFFICIAL_LEVELS[level], index


def official_episodes_file(task: str, level: str, data_dir: str | os.PathLike) -> Path:
    """The official evaluation data's episode list of one task and level (meta/episodes.jsonl, which SIMPLE's
    evaluation reads; the .pre-fk copy beside it has the robot as drawn, before the generation's first stand)."""
    return Path(data_dir) / check_task(task).env_id.split("/", 1)[1] / level / "meta" / "episodes.jsonl"


def official_environment_config(task: str, seed: int, data_dir: str | os.PathLike) -> tuple[dict, dict]:
    """A test seed's official evaluation episode: (its environment_config as SIMPLE's evaluation loads it, the
    episode's line in meta/episodes.jsonl without it).  As simple.datasets.lerobot.get_episode_lerobot, which
    both of SIMPLE's evaluations call: the JSON string loaded, the scene uid "102344280" renamed "scene3"."""
    level, index = official_episode(seed)
    path = official_episodes_file(task, level, data_dir)
    with open(path, encoding="utf-8") as f:
        lines = [json.loads(line) for line in f if line.strip()]
    found = [e for e in lines if int(e["episode_index"]) == index]
    if len(found) != 1:
        raise ValueError(f"{path}: {len(found)} episodes with episode_index {index}")
    entry = dict(found[0])
    conf = entry.pop("environment_config")
    conf = json.loads(conf) if isinstance(conf, str) else conf
    scene = conf["dr_state_dict"]["scene"]
    scene["uid"] = scene["uid"].replace("102344280", "scene3")
    entry.update({"level": level, "file": str(path)})
    return conf, entry
