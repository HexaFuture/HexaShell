"""The DexJoCo-X test protocol: which tasks and seeds, and what an episode may use.

DexJoCo-X (MIT code, CC BY-NC-SA 4.0 data) has six tasks for Franka Panda arms with dexterous hands, in MuJoCo:
three for two arms, two for the right arm alone, and a Tower of Hanoi for one right arm in the middle.  We use
the Wuji hand only: five fingers with four joints each, every joint driven by a position
actuator.  An episode is the official one: ``make_env(task, "wuji", seed)`` reset with the scene seed, the
official camera profile, the official per-step bounds and the official step limit (:data:`TASKS`).  Each control
step (20 Hz) sends every arm a world-frame change of its wrist pose (translation and rotation vector, at most the
task's bounds) and its hand's 20 joint targets.  The episode ends at the first step at which the environment
reports success, when the environment ends it (the tablet task's gate failing, the Hanoi tower in an illegal
state), or at the step limit.

Seeds: the scene seed.  Learning and validation use the scenes of the Wuji demonstrations of the official
training data (Training v1): per task the first ten seeds in order for learning, the next ten for validation.
The five tasks share one list of demonstration scenes; the Tower of Hanoi has its own.  The test is the official
evaluation's scene seeds, 10,000,000 + 1000 * (the task's place in the official order) + i; the official result
has three repeats of 50 (i = 0..149), and we run the first repeat (i = 0..49) first.  Test seeds open only in a
run that sets :data:`OFFICIAL_TEST_VARIABLE` to 1; other seeds outside the test block are free for development.

Pure Python: both the episode side and the simulator server import it.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

#: control period of the environment (20 Hz)
STEP_S = 0.05
#: the side of the picture of every camera (square, so that the official vertical field of view holds)
IMAGE_SIZE = 512
#: the hand of every episode
HAND = "wuji"
SIDES = ("left", "right")
FINGERS = ("thumb", "index", "middle", "ring", "little")

#: replies of the model per episode, and the wall-clock budget of an episode (s)
REPLIES = 50
EPISODE_S = 7200.0
#: most steps one wait may take
WAIT_MAX_STEPS = 400


@dataclass(frozen=True)
class Task:
    #: the task's place in the official order (it sets the test seeds)
    index: int
    #: the arms the task uses
    arms: tuple[str, ...]
    #: the official step limit (control steps)
    steps: int
    #: the official bound of one step's wrist change: metres per axis, radians per axis of the rotation vector
    translation_m: float
    rotation_rad: float
    #: where the fixed scene camera stands: "across" (across the table, facing the robot) or "behind" (behind and
    #: above the robot, looking over it)
    scene_camera: str


TASKS = {
    "bimanual_microwave_cook": Task(0, ("left", "right"), 1100, 0.08, 0.30, "behind"),
    "bimanual_photograph": Task(1, ("left", "right"), 1000, 0.08, 0.30, "behind"),
    "bimanual_unlock_ipad": Task(2, ("left", "right"), 1200, 0.08, 0.30, "behind"),
    "hammer_nail": Task(3, ("right",), 1000, 0.02, 0.03, "across"),
    "pick_bucket": Task(4, ("right",), 1200, 0.02, 0.03, "across"),
    "single_hand_hanoi": Task(5, ("right",), 2000, 0.08, 0.30, "across"),
}

#: the scenes of the Wuji demonstrations (Training v1 manifests), in order of seed: the five tasks share one list
DEMO_SCENES = (101165, 104904, 104961, 105090, 108864, 110337, 112778, 113684, 117616, 124101,
               124778, 126614, 127637, 130751, 136216, 137603, 138560, 139473, 140675, 140824)
HANOI_DEMO_SCENES = (82001, 82002, 83001, 83002, 83003, 83004, 83005, 84002, 84003, 84004,
                     84005, 84006, 84007, 84008, 84010, 85001, 85002, 85003, 85004, 85005)
SPLIT_SIZE = 10

#: the official evaluation's scene seeds: TEST_SEED + TEST_STRIDE * task index + i, i < OFFICIAL_TEST_EPISODES
TEST_SEED = 10_000_000
TEST_STRIDE = 1000
OFFICIAL_TEST_EPISODES = 150
#: the test we run: the first repeat of the official three
TEST_EPISODES = 50
#: a run sets this to 1 to open test seeds
OFFICIAL_TEST_VARIABLE = "HEXASHELL_DEXJOCO_X_OFFICIAL_TEST"

#: our camera name -> the camera of the official profile (contracts.json, "cameras"); the scene camera is the
#: profile's "front" camera (MuJoCo camera "back" in the two-arm tasks, which stands behind the robot)
CAMERA_ROLES = {"scene": "front", "right_wrist": "wrist_right", "left_wrist": "wrist_left"}
CAMERA_ALIASES = {"front": "scene", "back": "scene", "main": "scene", "wrist": "right_wrist", "right": "right_wrist",
                  "left": "left_wrist", "wrist_right": "right_wrist", "wrist_left": "left_wrist"}

#: the hand's named shapes (their joint targets are the server's, control.SHAPE_TARGETS)
SHAPES = ("open", "flat", "power", "wrap", "hook", "pinch", "tripod", "point")
#: the points of a hand a motion can place: the palm's centre (on its face), the grasp point (where what a closed
#: hand holds sits), a fingertip (its pad), the wrist camera
HAND_POINTS = ("palm", "grasp") + FINGERS + ("camera",)
#: named palm orientations (palm_axes)
PALM_FACING = ("down", "up", "forward", "inward", "outward")
#: a hand motion's default and highest speed (m/s) and an arc's (deg/s), by the task's bound of one step's wrist
#: change (the one-arm tasks' stiff controller follows faster than the softer one of the others)
HAND_SPEED = {0.02: (0.15, 0.30), 0.08: (0.12, 0.18)}
TURN_SPEED = {0.02: (25.0, 27.0), 0.08: (30.0, 40.0)}


def tasks() -> list[str]:
    """The tasks in the official order."""
    return sorted(TASKS, key=lambda t: TASKS[t].index)


def check_task(task: str) -> Task:
    if task not in TASKS:
        raise ValueError(f"{task!r} is not a DexJoCo-X task (one of: {', '.join(tasks())})")
    return TASKS[task]


def arms(task: str) -> tuple[str, ...]:
    return check_task(task).arms


def speeds(task: str) -> dict[str, tuple[float, float]]:
    """The task's (default, highest) speeds: 'hand' (m/s), 'turn' (deg/s)."""
    bound = check_task(task).translation_m
    return {"hand": HAND_SPEED[bound], "turn": TURN_SPEED[bound]}


def cameras(task: str) -> tuple[str, ...]:
    """The task's cameras: the scene camera and a wrist camera on each hand it uses."""
    return ("scene",) + tuple(f"{side}_wrist" for side in SIDES if side in arms(task))


def demo_scenes(task: str) -> tuple[int, ...]:
    check_task(task)
    return HANOI_DEMO_SCENES if task == "single_hand_hanoi" else DEMO_SCENES


def seeds(task: str, split: str) -> list[int]:
    """The task's learning, validation or test seeds."""
    scenes = demo_scenes(task)
    if split == "learning":
        return list(scenes[:SPLIT_SIZE])
    if split == "validation":
        return list(scenes[SPLIT_SIZE:2 * SPLIT_SIZE])
    if split == "test":
        return [test_seed(task, i) for i in range(TEST_EPISODES)]
    raise ValueError(f"split {split!r}: learning, validation or test")


def test_seed(task: str, i: int) -> int:
    """The official evaluation's scene seed ``i`` (0..149) of the task."""
    if not 0 <= int(i) < OFFICIAL_TEST_EPISODES:
        raise ValueError(f"official episode index {i} outside 0..{OFFICIAL_TEST_EPISODES - 1}")
    return TEST_SEED + TEST_STRIDE * check_task(task).index + int(i)


def in_test_block(seed: int) -> bool:
    """Whether a seed lies in the block the official evaluation draws its scenes from."""
    return TEST_SEED <= int(seed) < TEST_SEED + TEST_STRIDE * len(TASKS)


def split_of(task: str, seed: int) -> str:
    """'learning', 'validation', 'test' (an official evaluation scene of this task) or 'development'."""
    seed = int(seed)
    scenes = demo_scenes(task)
    if seed in scenes[:SPLIT_SIZE]:
        return "learning"
    if seed in scenes[SPLIT_SIZE:2 * SPLIT_SIZE]:
        return "validation"
    if in_test_block(seed):
        i = seed - TEST_SEED - TEST_STRIDE * check_task(task).index
        if 0 <= i < OFFICIAL_TEST_EPISODES:
            return "test"
        raise ValueError(f"seed {seed} is in the official evaluation's block but not one of {task}'s scenes "
                         f"({test_seed(task, 0)}-{test_seed(task, OFFICIAL_TEST_EPISODES - 1)})")
    return "development"


def official_test_allowed(environ=None) -> bool:
    """Whether this run may open test seeds (OFFICIAL_TEST_VARIABLE is 1)."""
    value = (os.environ if environ is None else environ).get(OFFICIAL_TEST_VARIABLE, "")
    return value.strip().lower() in ("1", "true", "yes", "on")


def check_seed(task: str, seed: int, official_test: bool = False) -> int:
    """A seed the run may open: a test seed only with ``official_test``."""
    if split_of(task, seed) == "test" and not official_test:
        raise ValueError(f"seed {seed} is an official evaluation scene (the test), which development runs do not "
                         f"open; a test run sets {OFFICIAL_TEST_VARIABLE}=1")
    return int(seed)


def test_episodes() -> list[tuple[str, int]]:
    """(task, seed) of the test we run, task by task."""
    return [(task, seed) for task in tasks() for seed in seeds(task, "test")]


def palm_axes(facing: str, side: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A named palm orientation's (normal, finger direction) in the scene frame (the robot faces +x).  'inward'
    turns the palm toward the robot's middle: +y for the right hand, -y for the left.  'forward' has the fingers
    down: with them up the wrist is at the end of its travel."""
    inward_y = -1.0 if side == "left" else 1.0
    table = {"down": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)), "up": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
             "forward": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0)), "inward": ((0.0, inward_y, 0.0), (1.0, 0.0, 0.0)),
             "outward": ((0.0, -inward_y, 0.0), (1.0, 0.0, 0.0))}
    if facing not in table:
        raise ValueError(f"palm facing {facing!r}: one of {', '.join(PALM_FACING)}")
    return table[facing]
