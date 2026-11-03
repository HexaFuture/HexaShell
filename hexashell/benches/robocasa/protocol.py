"""The RoboCasa test protocol: which tasks, kitchens and seeds, and what an episode may use.

Two splits (sets of kitchens), both of RoboCasa's pretraining scenes with the pretraining objects:

- ``pretrain``: RoboCasa's own evaluation split, as ``create_env(split="pretrain")`` builds it:
  layouts and styles 11-60, the seed drawing the kitchen (a kitchen in which the task cannot be
  built is drawn again by RoboCasa).
- ``pretrain50`` (the default): 50 fixed kitchens (layout, style).  Seed ``s`` builds kitchen
  ``PRETRAIN50[s % 50]``; seeds 0-9 are for learning (debugging, skills), 10-19 for validation, 20-49
  for the test.  A kitchen in which RoboCasa cannot build a task is replaced by the next kitchen of the
  same set that the task can use.

The tasks are RoboCasa's 50 target tasks: 18 atomic (seen), 16 composite seen, 16 composite unseen,
each with its official horizon (20 Hz control steps).  The episode ends at the first step at which
``env._check_success()`` holds, or at the horizon.

Pure Python: both the episode side and the simulator server import it.
"""

from __future__ import annotations

#: control period of the environment (20 Hz)
STEP_S = 0.05
ROBOT = "PandaOmron"
#: our camera name -> MuJoCo camera
CAMERAS = {"left": "robot0_agentview_left", "right": "robot0_agentview_right", "wrist": "robot0_eye_in_hand"}
IMAGE_SIZE = 512

#: task set -> (task, official horizon in steps)
TASKS: dict[str, tuple[tuple[str, int], ...]] = {
    "atomic_seen": (
        ("CloseBlenderLid", 900), ("CloseFridge", 900), ("CloseToasterOvenDoor", 450), ("CoffeeSetupMug", 600),
        ("NavigateKitchen", 450), ("OpenCabinet", 1050), ("OpenDrawer", 750), ("OpenStandMixerHead", 450),
        ("PickPlaceCounterToCabinet", 750), ("PickPlaceCounterToStove", 600), ("PickPlaceDrawerToCounter", 750),
        ("PickPlaceSinkToCounter", 900), ("PickPlaceToasterToCounter", 600), ("SlideDishwasherRack", 450),
        ("TurnOffStove", 750), ("TurnOnElectricKettle", 450), ("TurnOnMicrowave", 450), ("TurnOnSinkFaucet", 600),
    ),
    "composite_seen": (
        ("DeliverStraw", 2550), ("GetToastedBread", 3000), ("KettleBoiling", 1500), ("LoadDishwasher", 1800),
        ("PackIdenticalLunches", 3900), ("PreSoakPan", 2400), ("PrepareCoffee", 1800), ("RinseSinkBasin", 1350),
        ("ScrubCuttingBoard", 1200), ("SearingMeat", 4350), ("SetUpCuttingStation", 2400),
        ("StackBowlsCabinet", 2100), ("SteamInMicrowave", 2100), ("StirVegetables", 2400),
        ("StoreLeftoversInBowl", 2550), ("WashLettuce", 1650),
    ),
    "composite_unseen": (
        ("ArrangeBreadBasket", 4350), ("ArrangeTea", 2250), ("BreadSelection", 1950),
        ("CategorizeCondiments", 1650), ("CuttingToolSelection", 1200), ("GarnishPancake", 2700),
        ("GatherTableware", 2250), ("HeatKebabSandwich", 2700), ("MakeIceLemonade", 3000), ("PanTransfer", 1800),
        ("PortionHotDogs", 2250), ("RecycleBottlesByType", 2850), ("SeparateFreezerRack", 2400),
        ("WaffleReheat", 4050), ("WashFruitColander", 3150), ("WeighIngredients", 3000),
    ),
}
HORIZON = {task: steps for tasks in TASKS.values() for task, steps in tasks}
TASK_SET = {task: name for name, tasks in TASKS.items() for task, _ in tasks}

#: the splits; the first is the default
SPLITS = ("pretrain50", "pretrain")
#: robocasa's scene registry id of the pretraining kitchens (layouts and styles 11-60), as create_env uses it
PRETRAIN_SCENES = -2
#: the 50 kitchens (layout id, style id) of split pretrain50
PRETRAIN50 = ((54, 28), (29, 54), (34, 60), (56, 34), (33, 19), (58, 17), (26, 14), (53, 57), (19, 20), (60, 58),
              (38, 30), (55, 18), (16, 46), (17, 50), (48, 43), (51, 25), (45, 59), (49, 33), (13, 48), (35, 11),
              (25, 47), (36, 41), (15, 39), (57, 31), (43, 36), (40, 12), (14, 27), (31, 52), (50, 53), (20, 40),
              (21, 21), (41, 32), (47, 23), (46, 15), (24, 24), (22, 22), (30, 37), (23, 44), (37, 42), (27, 51),
              (32, 55), (18, 56), (52, 16), (44, 49), (59, 45), (42, 13), (39, 26), (28, 38), (12, 35), (11, 29))
#: seed set -> seeds (the kitchen of seed s is s % 50)
SEED_SETS = {"learn": range(0, 10), "val": range(10, 20), "test": range(20, 50)}
#: kitchens in which RoboCasa cannot build the task; (layout, style, seed) skips it for that seed only
UNBUILDABLE: dict[str, tuple[tuple[int, ...], ...]] = {
    "PickPlaceToasterToCounter": ((14, 27), (41, 32)),
    "HeatKebabSandwich": ((48, 43), (45, 59), (21, 21), (41, 32), (47, 23), (32, 55), (18, 56), (44, 49)),
    "GatherTableware": ((59, 45),),
    "SeparateFreezerRack": ((30, 37), (37, 42), (27, 51, 38)),
    "WaffleReheat": ((21, 21),),
    "GetToastedBread": ((29, 54), (58, 17), (19, 20), (60, 58), (48, 43), (43, 36), (14, 27), (21, 21), (41, 32),
                        (24, 24), (44, 49)),
    "PackIdenticalLunches": ((53, 57), (60, 58), (16, 46), (17, 50), (13, 48), (35, 11), (43, 36), (21, 21), (47, 23),
                             (46, 15), (37, 42), (32, 55), (18, 56), (54, 28, 9), (41, 32, 30)),
    "StoreLeftoversInBowl": ((34, 60), (56, 34), (33, 19), (58, 17), (22, 22)),
}

#: replies of the model per episode, by task set (the step budget is the official horizon)
REPLIES = {"atomic_seen": 40, "composite_seen": 100, "composite_unseen": 100}
#: wall-clock budget of an episode (s)
EPISODE_S = 7200.0


def task_set(task: str) -> str:
    try:
        return TASK_SET[task]
    except KeyError:
        raise ValueError(f"{task!r} is not one of RoboCasa's 50 target tasks") from None


def split_of(split: str | None) -> str:
    """``split``, or the default one when it is empty."""
    split = split or SPLITS[0]
    if split not in SPLITS:
        raise ValueError(f"split {split!r}: one of {', '.join(SPLITS)}")
    return split


def seed_set(seed: int) -> str:
    i = int(seed) % len(PRETRAIN50)
    return next(name for name, seeds in SEED_SETS.items() if i in seeds)


def skipped(task: str, seed: int) -> list[tuple[int, int]]:
    """Kitchens the task cannot be built in, for this seed."""
    return [tuple(k[:2]) for k in UNBUILDABLE.get(task, ()) if len(k) == 2 or k[2] == int(seed)]


def kitchen(seed: int, exclude_layouts=(), exclude_styles=(), unbuildable=()) -> tuple[int, int]:
    """(layout, style) of ``seed``: its own kitchen, unless the task cannot be built there (then the next
    kitchen of the same seed set); a layout or style the task excludes is taken from the next kitchen of
    the set that the task allows."""
    i = int(seed) % len(PRETRAIN50)
    seeds = SEED_SETS[seed_set(seed)]
    a, b = seeds.start, seeds.stop
    order = [a + (i - a + k) % (b - a) for k in range(b - a)]
    banned = {tuple(k) for k in unbuildable}
    while order and tuple(PRETRAIN50[order[0]]) in banned:
        order = order[1:]
    if not order:
        raise ValueError(f"no kitchen of seed {seed}'s set can be built")
    layout = next(PRETRAIN50[j][0] for j in order if PRETRAIN50[j][0] not in set(exclude_layouts))
    style = next(PRETRAIN50[j][1] for j in order if PRETRAIN50[j][1] not in set(exclude_styles))
    return layout, style
