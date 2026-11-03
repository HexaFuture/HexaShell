"""The LIBERO-PRO test protocol: which tasks and seeds, and what an episode may use.

LIBERO-PRO perturbs the four LIBERO suites (spatial, object, goal, 10) one way at a time.  We test four
of its five perturbations: the objects start in exchanged places (``*_swap``), the objects
look different (``*_object``), the instruction is reworded (``*_lan``), a different task in the same
scene (``*_task``) -- 16 suites of ten tasks.  The fifth, another environment, has no suites in the
release (they would have to be generated).  Four tasks have no initial states in the release and are
left out: 156 tasks.  Task ``"<suite>_t<i>"`` is task ``i`` of the suite (the suite's own task order).
Seed ``s`` starts from the suite's initial state ``s % 50`` of that task, with the fixtures placed by
``np.random`` seeded with ``s``.  Seeds 0-9 are for learning, 10-19 for validation, 20-29 the test:
1560 episodes.

Every episode first lets the scene settle for 10 control steps with a still arm and the gripper
opening, as LIBERO-PRO's own evaluation waits; then the budget begins: LIBERO-PRO's own step limit of
the suite (:data:`OFFICIAL_STEPS`: 220 to 520 control steps of 20 Hz).  The episode ends at the first
step at which the task's goal predicates hold (``env.check_success()``), or when the budget is spent.

Pure Python: both the episode side and the simulator server import it.
"""

from __future__ import annotations

import re

#: control period of the environment (20 Hz)
STEP_S = 0.05
#: control steps with a still arm and an opening gripper before the episode starts
SETTLE_STEPS = 10
#: our camera name -> MuJoCo camera
CAMERAS = {"agentview": "agentview", "wrist": "robot0_eye_in_hand"}
IMAGE_SIZE = 512

#: LIBERO's suites, and the perturbations of them we test
BASES = ("libero_spatial", "libero_object", "libero_goal", "libero_10")
PERTURBATIONS = ("swap", "object", "lan", "task")
#: the test suites, in order
SUITES = tuple(f"{base}_{p}" for p in PERTURBATIONS for base in BASES)
TASKS_PER_SUITE = 10
#: tasks whose initial states the release leaves empty
NO_INIT_STATES = frozenset({"libero_10_object_t4", "libero_spatial_task_t3", "libero_spatial_task_t7",
                            "libero_10_task_t2"})
LEARN_SEEDS = range(0, 10)
VALIDATION_SEEDS = range(10, 20)
TEST_SEEDS = range(20, 30)
#: control steps of an episode, after the settling steps: LIBERO-PRO's own step limit of the suite
OFFICIAL_STEPS = {"libero_spatial": 220, "libero_object": 280, "libero_goal": 300, "libero_10": 520}
#: initial states per task in LIBERO's init files
INIT_STATES = 50

#: replies of the model per episode, and the wall-clock budget of an episode (s)
REPLIES = 40
EPISODE_S = 7200.0

_TASK = re.compile(r"^(libero_[a-z0-9_]+?)_t(\d+)$")


def task_name(suite: str, index: int) -> str:
    return f"{suite}_t{int(index)}"


def parse_task(task: str) -> tuple[str, int]:
    """(suite, index) of a task name ``"<suite>_t<i>"``."""
    m = _TASK.match(str(task))
    if not m:
        raise ValueError(f"{task!r}: a LIBERO-PRO task is named <suite>_t<index>, e.g. libero_goal_swap_t3")
    return m.group(1), int(m.group(2))


def test_tasks() -> list[str]:
    return [t for t in (task_name(suite, i) for suite in SUITES for i in range(TASKS_PER_SUITE))
            if t not in NO_INIT_STATES]


def official_steps(task: str) -> int:
    """The control steps of an episode of this task: LIBERO-PRO's step limit for the task's suite."""
    suite, _ = parse_task(task)
    for base, steps in OFFICIAL_STEPS.items():
        if suite == base or suite.startswith(base + "_"):
            return steps
    raise ValueError(f"{task}: LIBERO-PRO gives no step limit for {suite}")


def init_index(seed: int, count: int = INIT_STATES) -> int:
    """Which of the task's initial states seed ``seed`` starts from."""
    return int(seed) % int(count)
