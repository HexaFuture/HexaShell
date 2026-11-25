"""RoboColiseum Season II (AgiBot Genie Sim 3.6.0, github main 847bb9e): its four sub-boards and 50 scored tasks,
the robots they use, our split of their instances, and where things are.

A scored task is a benchmark config ``geniesim_benchmark/config/<config>.yaml``; its name is
``<robot>_<board>_<task>``.  45 tasks run in Isaac Sim: their instances are the fixed scene folders
``benchmark/config/llm_task/<sub_task>/<n>/`` (0-9, one instruction each), and a run
plays the ones given by ``--benchmark.instance_ids``.  The other five generate their instances when they start and only take a count
(``--benchmark.num_instances``): three cloth tasks in libuipc (instance i = the i-th base offset and the
(i mod M)-th cloth material of the config: M = 6 for fold_cloth and cloth_hanger, 5 for drawstring_bag; every
base offset is (0, 0), so two instances with the same material are the same scene),
wok_flip in Newton (instance i = the i-th base offset drawn by ``random.Random(seed)``) and the Rubik's cube in
MuJoCo (the scenes of a bundled archive in the order of a permutation by the seed, which also picks each
episode's background picture).  One of their instances alone (:mod:`.sim`): a cloth instance from our copy of
the config with only its offset and its material first; a wok_flip instance from the layout bundle of the
config's seed (``run-layouts --layout-ids``); a Rubik's cube instance from a seed that puts its scene first
with the same background (:data:`RUBIK_SEEDS`).

Split: Isaac instance number mod 5 = 0 learning, = 1 validation, the rest test (2 / 2 / 6 a task: learning
0, 5; validation 1, 6; test 2-4, 7-9); cloth
tasks the same by instance number (2 / 2 / 6), and since their offsets are all zero the parts share scenes:
fold_cloth and cloth_hanger validation 6 = learning 0 and test 7 = validation 1 (also test 8 = 2, 9 = 3);
drawstring_bag learning 0 = 5, validation 1 = 6, test 2-4 = 7-9 (no scene in two parts).  wok_flip and the
Rubik's cube learn and validate on instances that are not scored and test on all 10 scored ones: wok_flip on
base offsets of other seeds (each about 2 cm or more from every scored offset: 19.9-28.0 mm), the Rubik's cube
on scenes of the archive that is unrelated to the scored one (``rubik_2turn_5_public.zip`` is not used: its five
scenes are the setups that scored instances 1, 3, 5, 6 and 8 were made from).  100 learning, 100 validation and
308 test episodes.

Scoring (the benchmark's): each instance's ``problems.json`` splits the task into stages, each with points and
a step limit (StepOut); one action row the simulator plays is one step.  The official result turns the stage
points into a 100-point score and scales each stage by the dispatch budget (our replies with actions) and a
completion bonus (see :mod:`.budget`).  The checkers look at the scene every 30 steps (the cloth tasks' at the
first physical frame, every tenth and the last).

Pure Python.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
#: the folder that holds Genie Sim 3.6 (local runs only): $HEXASHELL_GENIESIM36, else inside this checkout
GENIESIM = Path(os.environ.get("HEXASHELL_GENIESIM36") or ROOT / ".bench-data" / "geniesim-3.6")
#: genie_sim 3.6.0 (main 847bb9e, with scripts/patches/geniesim36-initialization-timeout.patch applied), mounted at
#: /workspace in the container
SOURCE = Path(os.environ.get("GENIESIM_SOURCE") or GENIESIM / "genie_sim")
PKG = SOURCE / "source" / "geniesim_benchmark" / "src" / "geniesim_benchmark"
#: the assets (ModelScope agibot_world/GenieSimAssets at dfa5864), mounted at /geniesim_assets
ASSETS = Path(os.environ.get("GENIESIM_ASSETS") or GENIESIM / "GenieSimAssets")
#: the script that starts a container, and the image it starts (built from the Dockerfile of genie_sim 3.6)
RUN_CONTAINER = ROOT / "scripts" / "setup" / "geniesim36_container.sh"
IMAGE = os.environ.get("GENIESIM_IMAGE", "geniesim4-local:3.6.0")
#: the source and the assets inside the container
CONTAINER_SOURCE = "/workspace"
CONTAINER_ASSETS = "/geniesim_assets"

#: the order of a batch; the official total is the mean of the four
BOARDS = ("finemanip", "crsemb", "wbc", "dexmanip")
PARTS = ("learn", "validation", "test")
RUNTIMES = ("isaac", "libuipc", "newton", "mujoco")


@dataclass(frozen=True)
class Robot:
    """One simulator robot configuration as the wire names it (protocol.md, "Embodiment routing")."""
    key: str
    #: ``benchmark.platform`` of its configs, and the robot part of their names
    platform: str
    config_robot: str
    robot_type: str
    ee_type: str
    #: joints per arm, effector values per side, head and waist joints in the requests
    arm_joints: int
    effector: int
    head: int
    waist: int
    #: the arms that move (every robot moves both: the source calls the ARX and Agilex left arm a decorative
    #: dual, but the environment applies both arms' rows and the scoring of some of their instances names the
    #: left gripper, e.g. Agilex pour instance 0: PickUpOnGripper jug|left_gripper)
    arms: tuple[str, ...]
    #: whether the arms take EEF_ABS rows (the ARX / Agilex registrations replay joints only)
    eef: bool


ROBOTS = {r.key: r for r in (
    Robot("g2_90d", "g2_crsb_90d", "g2crsb90d", "g2a_sim", "zhiyuan_gripper_ctek90d", 7, 1, 3, 5,
          ("left", "right"), True),
    Robot("g2_omnipicker", "G2_omnipicker", "g2op", "g2a_sim", "zhiyuan_gripper_omnipicker", 7, 1, 3, 5,
          ("left", "right"), True),
    Robot("g1", "G1", "g1", "a2d_sim", "zhiyuan_gripper_omnipicker", 7, 1, 2, 2, ("left", "right"), True),
    Robot("arx", "arxone", "arxone", "arx_sim", "arx_gripper", 6, 1, 0, 0, ("left", "right"), False),
    Robot("agilex", "agilex", "agilex", "agilex_sim", "agilex_gripper", 6, 1, 0, 0, ("left", "right"), False),
    Robot("g2_wuji", "g2_crsb_wuji", "g2crsbwuji", "g2a_sim", "wuji", 7, 20, 3, 5, ("left", "right"), True),
)}
ROBOT_OF_CONFIG = {r.config_robot: r.key for r in ROBOTS.values()}


@dataclass(frozen=True)
class Task:
    config: str
    board: str
    robot: str
    #: the task part of the config name
    task: str
    runtime: str
    #: ``benchmark.task_name`` (the scene) and ``benchmark.sub_task_name`` (the instance folders)
    scene: str
    sub_task: str
    #: scored instances
    instances: int
    seed: int
    #: Isaac physics and render rates (Hz) and renderer; None for the other runtimes
    physics_hz: int | None = None
    render_hz: int | None = None
    render_mode: str | None = None

    @property
    def mobile(self) -> bool:
        """The WBC board's robot drives its chassis (``wheels`` rows)."""
        return self.board == "wbc"


#: scored instances of an Isaac task
ISAAC_INSTANCES = 10


def _isaac(robot: str, board: str, task: str, scene: str, sub_task: str, seed: int = 1,
           rates: tuple[int, int] = (120, 60), render_mode: str | None = None) -> Task:
    config = f"{robot}_{board}_{task}"
    return Task(config, board, ROBOT_OF_CONFIG[robot], task, "isaac", scene, sub_task, ISAAC_INSTANCES, seed,
                rates[0], rates[1], render_mode)


def _crsemb(robot: str, suffix: str, scene: str, scene_pour: str, scene_write: str,
            write_rates: tuple[int, int]) -> list[Task]:
    """The six cross-embodiment tasks of one robot (the trash can sub-task is shared by all four robots)."""
    return [
        _isaac(robot, "crsemb", "hang_cups_on_cup_tree", scene, f"hang_cups_on_cup_tree_{suffix}"),
        _isaac(robot, "crsemb", "keyboard_keycap_swap", scene, f"keyboard_keycap_swap_{suffix}"),
        _isaac(robot, "crsemb", "open_short_trash_can_and_throw_garbage", scene,
               "open_short_trash_can_and_throw_garbage_multi_robot"),
        _isaac(robot, "crsemb", "pour_50ml_into_graduated_cylinder", scene_pour,
               f"pour_50ml_into_graduated_cylinder_{suffix}", rates=(240, 30), render_mode="PathTracing"),
        _isaac(robot, "crsemb", "scrape_chicken_bone_into_bin", scene, f"scrape_chicken_bone_into_bin_{suffix}",
               rates=(240, 60)),
        _isaac(robot, "crsemb", "write_uppercase_letter_with_pen", scene_write,
               f"write_uppercase_letter_with_pen_{suffix}", rates=write_rates),
    ]


def _room2(task: str, scene_suffix: str, seed: int = 1) -> Task:
    return _isaac("g2crsb90d", "finemanip", task, f"table_task_g2_crsb_90d_room2{scene_suffix}", task, seed=seed)


_TASKS = [
    # finemanip: G2 with ctek90d grippers, fixed base
    _room2("insert_building_block", "_building_block_main"),
    _room2("insert_coin_into_money_box", "_coin_main", seed=30),
    _room2("insert_ethernet_cable", "_ethernet_main"),
    _room2("insert_straw_into_milk_tea_cup", "_straw_main", seed=30),
    _room2("install_bearing_shaft_seat5mm", "_bearing_main"),
    _isaac("g2crsb90d", "finemanip", "phone_packaging", "phone_packaging_g2_crsb90d", "phone_packaging"),
    _room2("plug_charging_cable_into_phone", ""),
    _room2("put_the_cap_on_the_pen_new", "_pen_cap_main", seed=30),
    # crsemb: the same six tasks on four robots
    *_crsemb("g2op", "g2_omnipicker", "table_task_g2_omnipicker", "pour_50ml_g2_omnipicker",
             "table_task_g2_omnipicker_hdr", (60, 30)),
    *_crsemb("g1", "g1", "table_task_g1_op", "pour_50ml_g1", "table_task_g1_op_hdr", (60, 30)),
    *_crsemb("arxone", "arxone", "table_task_arxone", "pour_50ml_arxone", "table_task_arxone_hdr", (240, 30)),
    *_crsemb("agilex", "agilex", "table_task_agilex", "pour_50ml_agilex", "table_task_agilex_hdr", (240, 30)),
    # wbc: G2 with ctek90d grippers, driving its chassis (deliver_tissues plays the sending_food scene)
    *[_isaac("g2crsb90d", "wbc", task, f"{scene}_g2_90d", sub_task, seed=0) for task, scene, sub_task in (
        ("clean_the_blackboard", "clean_the_blackboard", "clean_the_blackboard"),
        ("deliver_tissues", "sending_food", "sending_food"),
        ("match_and_pick_from_conveyor", "match_and_pick_from_conveyor", "match_and_pick_from_conveyor"),
        ("open_the_door", "open_the_door", "open_the_door"),
        ("open_the_roller_shutter", "open_the_roller_shutter", "open_the_roller_shutter"),
        ("pick_up_trash", "pick_up_trash_wbc", "pick_up_trash_wbc"),
        ("put_the_plates_on_the_shelf", "put_the_plates_on_the_shelf", "put_the_plates_on_the_shelf"),
        ("transport_cartons_to_table", "transport_cartons_to_table_wbc", "transport_cartons_to_table_wbc"),
    )],
    # dexmanip: G2 with Wuji hands; five Isaac tasks, three cloth tasks (libuipc), wok_flip (Newton),
    # the Rubik's cube (MuJoCo)
    _isaac("g2crsbwuji", "dexmanip", "cover_cup_lid", "table_task_g2_crsB_wuji_cover_cup_lid", "cover_cup_lid"),
    _isaac("g2crsbwuji", "dexmanip", "pick_up_a_package", "table_task_g2_crsB_wuji_warehouse_backgen",
           "pick_up_a_package"),
    _isaac("g2crsbwuji", "dexmanip", "stack_bowls", "table_task_2_g2_crsB_wuji", "stack_bowls_wuji"),
    _isaac("g2crsbwuji", "dexmanip", "take_the_book", "table_task_g2_crsB_wuji_take_the_book", "take_the_book"),
    _isaac("g2crsbwuji", "dexmanip", "take_the_scissors", "table_task_g2_crsB_wuji_take_the_scissors",
           "take_the_scissors"),
    *[Task(f"g2crsbwuji_dexmanip_{t}", "dexmanip", "g2_wuji", t, "libuipc", t, t, 10, 1)
      for t in ("cloth_hanger", "drawstring_bag", "fold_cloth")],
    Task("g2crsbwuji_dexmanip_wok_flip", "dexmanip", "g2_wuji", "wok_flip", "newton", "wok_flip", "wok_flip", 10,
         1398484294),
    Task("g2crsbwuji_dexmanip_rubik_cube_solve", "dexmanip", "g2_wuji", "rubik_cube_solve", "mujoco",
         "rubik_cube_solve_2turn", "rubik_cube_solve_2turn", 10, 0),
]
TASKS = {t.config: t for t in _TASKS}

#: the scored Rubik's cube scenes; the seed that plays scored instance i alone (``--benchmark.num_instances=1``):
#: its scene first and the same background picture as in the official run of all ten (seed 0)
RUBIK_SCORED_ARCHIVE = "rubik_2turn_10_ranked.zip"
RUBIK_SEEDS = (99, 13, 108, 93, 549, 22, 222, 95, 27, 115)
#: our Rubik's cube scenes: an archive of the benchmark that shares no scene and no setup with the scored one,
#: and per part (scene index in its manifest, the seed that puts that scene first)
RUBIK_OWN_ARCHIVE = "rubik_2turn_10.zip"
RUBIK_OWN_SCENES = {"learn": ((0, 1009), (1, 1001)), "validation": ((2, 1007), (3, 1014))}
#: wok_flip's own layouts for learning and validation: instance 0 of each of these seeds (one run each; 201 and
#: 202 were dropped, their offsets lie 2 mm and 10 mm from scored ones)
WOK_OWN_SEEDS = {"learn": (101, 102), "validation": (203, 204)}
#: wok_flip's scored layouts: a bundle of this many from the config's seed
WOK_LAYOUTS = 10

#: the benchmark's limits per runtime (not Isaac's): seconds it waits for one reply and rows one reply may carry
#: (None: any) are developer data, for our link and our runs, never for the model; rows an episode plays at most
#: (MuJoCo: its 60 s of simulated time at the nominal step) are the episode's one span of motion, a fact of the
#: runtime that the native task cards give
REPLY_WAIT_S = {"libuipc": 120.0, "newton": 660.0, "mujoco": 60.0}
EPISODE_ROWS = {"libuipc": 2400, "newton": 600, "mujoco": 900}
REPLY_ROWS = {"libuipc": None, "newton": None, "mujoco": 256}
#: the native runtimes' facts the task card gives: the simulated time of one step (one row; MuJoCo's is the nominal
#: 2/30 s of control_hz 30 x action_time_scale 2, stretched when a joint would exceed its motor's speed limit), and
#: whether the scene stands still while the model thinks (Newton plays in lockstep, MuJoCo in one process; libuipc
#: holds its physics between actions: ``libuipc.pause_when_idle: true`` in the three cloth tasks' evaluation scenes,
#: the benchmark README's "physical settling during a model wait no longer occurs")
STEP_S = {"libuipc": 1.0 / 30.0, "newton": 1.0 / 60.0, "mujoco": 2.0 / 30.0}
PAUSES = {"libuipc": True, "newton": True, "mujoco": True}


class TaskError(ValueError):
    pass


def task(config: str) -> Task:
    config = str(config).removesuffix(".yaml")
    if config not in TASKS:
        raise TaskError(f"not a RoboColiseum Season II task: {config!r}")
    return TASKS[config]


def configs(boards=BOARDS, robots=None) -> list[str]:
    """The scored configs of these boards (in BOARDS order), optionally of some robots only."""
    unknown = set(boards) - set(BOARDS)
    if unknown:
        raise TaskError(f"unknown boards {sorted(unknown)}")
    return [t.config for b in boards for t in _TASKS if t.board == b and (robots is None or t.robot in robots)]


def part_of(instance: int) -> str:
    """Our split of an Isaac or cloth task's instance numbers."""
    r = int(instance) % 5
    return "learn" if r == 0 else "validation" if r == 1 else "test"


@dataclass(frozen=True)
class Case:
    """One episode we play: an instance of a task, from the official instances or from our own source."""
    config: str
    #: the official instance; for our own: wok_flip's layout of the seed (0), the Rubik's cube's scene index in
    #: our archive
    instance: int
    #: None for the official instances; else the seed of our own layout (wok_flip) or scene order (the Rubik's
    #: cube, with the archive)
    seed: int | None = None
    archive: str | None = None

    @property
    def key(self) -> str:
        own = "".join(f"{p}_" for p in (self.archive and self.archive.removesuffix(".zip"),
                                         self.seed is not None and f"seed{self.seed}") if p)
        return f"{self.config}/{own}i{self.instance:02d}"

    @property
    def name(self) -> str:
        """The key as one word (unit ids, run ids)."""
        return self.key.replace("/", "_")

    @property
    def official(self) -> bool:
        return self.seed is None and self.archive is None


def cases(config: str, part: str) -> list[Case]:
    """The episodes of a task in a part of the split."""
    if part not in PARTS:
        raise TaskError(f"part must be one of {PARTS}, not {part!r}")
    t = task(config)
    if t.runtime in ("isaac", "libuipc"):
        return [Case(t.config, i) for i in range(t.instances) if part_of(i) == part]
    if part == "test":
        return [Case(t.config, i) for i in range(t.instances)]
    if t.runtime == "newton":
        return [Case(t.config, 0, seed=s) for s in WOK_OWN_SEEDS[part]]
    return [Case(t.config, k, seed=s, archive=RUBIK_OWN_ARCHIVE) for k, s in RUBIK_OWN_SCENES[part]]


def case_of(config: str, instance: int, part: str, seed: int | None = None) -> Case:
    """The case of ``part`` with this instance (and seed, for our own wok_flip and Rubik's cube episodes): the
    split guard of a run."""
    found = [c for c in cases(config, part) if c.instance == int(instance) and c.seed == seed]
    if not found:
        have = ", ".join(f"{c.instance}" + (f" --seed {c.seed}" if c.seed is not None else "")
                         for c in cases(config, part))
        raise TaskError(f"{config} has no {part} case instance {instance}"
                        f"{f' seed {seed}' if seed is not None else ''} (the {part} cases: {have})")
    return found[0]


def all_cases(part: str, boards=BOARDS, robots=None) -> list[Case]:
    return [c for config in configs(boards, robots) for c in cases(config, part)]


def check_catalog(pkg: Path = PKG) -> list[str]:
    """Differences between the table above and the benchmark's configs and instance folders (none expected).
    Reads the configs as text (they are third-party files)."""
    problems = []
    found = sorted(p.stem for p in (pkg / "config").glob("*.yaml")
                   if any(f"_{b}_" in p.stem for b in BOARDS) and not p.stem.startswith("_"))
    if found != sorted(TASKS):
        problems.append(f"configs differ: extra {sorted(set(found) - set(TASKS))}, "
                        f"missing {sorted(set(TASKS) - set(found))}")

    def value(text, key):
        m = re.findall(rf"^\s+{key}:\s*(\S+)\s*(?:#.*)?$", text, re.M)
        return m[0].strip("'\"") if m else None

    for t in _TASKS:
        path = pkg / "config" / f"{t.config}.yaml"
        if not path.exists():
            continue
        text = path.read_text()
        if t.runtime != "isaac":
            continue
        got = {"task_name": value(text, "task_name"), "platform": value(text, "platform"),
               "sub_task_name": value(text, "sub_task_name"), "seed": value(text, "seed"),
               "num_instances": value(text, "num_instances"), "physics_step": value(text, "physics_step") or "120",
               "rendering_step": value(text, "rendering_step") or "60", "render_mode": value(text, "render_mode")}
        want = {"task_name": t.scene, "platform": ROBOTS[t.robot].platform, "sub_task_name": t.sub_task,
                "seed": str(t.seed), "num_instances": str(t.instances), "physics_step": str(t.physics_hz),
                "rendering_step": str(t.render_hz), "render_mode": t.render_mode}
        for k in want:
            if got[k] != want[k]:
                problems.append(f"{t.config}: {k} {got[k]!r}, expected {want[k]!r}")
        folder = pkg / "benchmark" / "config" / "llm_task" / t.sub_task
        ids = sorted(int(p.name) for p in folder.iterdir() if p.is_dir() and p.name.isdigit()) \
            if folder.is_dir() else []
        if ids != list(range(t.instances)):
            problems.append(f"{t.config}: instance folders {ids[:3]}..{ids[-1:]} ({len(ids)}), "
                            f"expected 0..{t.instances - 1}")
    return problems
