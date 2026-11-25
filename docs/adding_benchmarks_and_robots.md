# Adding a benchmark or a robot

**English | [简体中文](adding_benchmarks_and_robots_zh.md)**

This guide describes how to connect HexaShell to a new simulated benchmark or a new real robot. It is written for a
person and equally for a coding agent, to which the work can be handed with one sentence ([Section 1](#1-one-sentence)).
The existing adapters are the most direct reference: find the closest one in [Section 4](#4-choose-a-template) and follow
it file by file; the decisions made for each are in the docstring of its `protocol.py`. All paths are relative to the
root of the repository.

1. [One sentence](#1-one-sentence)
2. [Structure](#2-structure)
3. [Settle the protocol first](#3-settle-the-protocol-first)
4. [Choose a template](#4-choose-a-template)
5. [A benchmark that HexaShell drives](#5-a-benchmark-that-hexashell-drives)
6. [Registration, settings and batches](#6-registration-settings-and-batches)
7. [A benchmark with its own evaluator](#7-a-benchmark-with-its-own-evaluator)
8. [Prompts and skills](#8-prompts-and-skills)
9. [Environments, GPUs and data](#9-environments-gpus-and-data)
10. [Tests and acceptance](#10-tests-and-acceptance)
11. [Real robots](#11-real-robots)
12. [Conventions](#12-conventions)

## 1. One sentence

Open the repository in a coding agent (Claude Code, Codex, ...) and say, for a simulated benchmark:

```text
Following docs/adding_benchmarks_and_robots.md, adapt HexaShell to <benchmark> (<repository URL or local path>).
Write an adaptation plan and a learning plan first, and wait for my confirmation before writing code.
```

and for a real robot:

```text
Following docs/adding_benchmarks_and_robots.md, adapt HexaShell to <robot> (arms: ..., hand: ..., cameras: ...,
control interface: ...). Write an adaptation plan and a learning plan first. Start with the service's simulator and its
read-only mode, and do not move the robot before I say so.
```

The two plans should answer the following.

**Adaptation plan**

| Question | Where it is answered |
|---|---|
| What are the tasks, the robot, the control interface, the cameras (RGB or RGB-D), the step limits, the success check, and the official seeds and splits? | the benchmark's code and paper |
| What input do the benchmark's rules allow the agent (task descriptions, skills, demonstrations)? | the benchmark's rules |
| Who drives the loop, and which existing adapter is closest? | Sections [2](#2-structure) and [4](#4-choose-a-template) |
| What goes into `protocol.py`: task names, the meaning of a seed, splits, limits, cameras, budgets, coordinate frame? | [Section 3](#3-settle-the-protocol-first) |
| Which calls of the robot API carry over, and which are new (a moving base, a torso, fingers, a movable camera)? | Sections [4](#4-choose-a-template) and [5.3](#53-the-episode-side) |
| Which files, tests and acceptance checks, in which order? | Sections [5](#5-a-benchmark-that-hexashell-drives), [10](#10-tests-and-acceptance) and [11.5](#115-the-order-of-work-on-a-real-robot) |
| Where does it depart from the official evaluation, and why? | to be listed in the docstring of `protocol.py` |

**Learning plan**

| Question | Where it is answered |
|---|---|
| Which episodes are for learning, for validation and for the test? The test's seeds are not run during development. | [Section 3](#3-settle-the-protocol-first) |
| How many rounds, and what does each decide? | [Section 8.3](#83-learning-in-rounds) |
| How many episodes, hours and tokens does a round cost with the chosen model? | `result.json` of the first episodes |
| What ends the development, and which run is reported? | [Section 8.3](#83-learning-in-rounds) |

## 2. Structure

`Episode` (`hexashell/runtime/episode.py`) runs one episode: the conversation with the model, the two tools `python` and
`finish`, the cells in the sandbox, the budgets and the record. Everything that concerns the benchmark or the robot
comes from a subclass of `Session` (`hexashell/runtime/session.py` has the interface and its defaults). To add a
benchmark is to write that session with the host, proxy and server below it.

```
model endpoint ⇄ Episode (runtime/episode.py): conversation, budgets, record
              │ python tool
              ▼
          sandboxed worker (kernel/worker.py: python -I, no API key, Landlock + seccomp)
              robot in a cell = importlib.import_module(session.proxy).Robot(call)
              │ call(name, args, kwargs)
              ▼
          Session.dispatch → host: checks arguments, plans or forwards, perceives, annotates pictures, records
              │
              ▼
          simulator server (in the benchmark's own Python environment) or robot service (HTTP)
```

There are three ways to connect:

| Way | Who drives the loop | Existing adapters |
|---|---|---|
| HexaShell drives the simulator | Episode starts a simulator server; every robot call advances it by some steps | RoboCasa365, LIBERO-PRO, LIBERO-Pro 2.0, RoboTwin 2.0, SIMPLE, DexJoCo-X (`hexashell/benches/<bench>/`) |
| The evaluator drives | the benchmark's own evaluator asks for actions at its own pace, and HexaShell answers | RoboDojo (XPolicyLab, `hexashell/xpl/`), RoboColiseum (`hexashell/benches/robocoliseum1/`, `hexashell/benches/robocoliseum/`) |
| Real robot | a service on the robot's side plans and executes; HexaShell talks to it over HTTP only | URAI (`hexashell/real/urai/`), Franka FR3 (`hexashell/real/franka/`) |

A simulator that can be reset and stepped from Python takes the first way ([Section 5](#5-a-benchmark-that-hexashell-drives)).
A benchmark whose official evaluation runs only through its own client takes the second ([Section 7](#7-a-benchmark-with-its-own-evaluator)).
Real robots are in [Section 11](#11-real-robots).

### The sandbox's boundary

The following holds for every adapter.

- `session.proxy` must be a module of the package `hexashell.` (checked by `kernel/host.py`). Its `Robot(call)` only
  forwards; every method is `return self._call(name, args, kwargs)`. A method's docstring is what the model reads with
  `help(robot.goto)`, and matters as much as the prompt.
- Objects returned to a cell must be classes of `hexashell.robot.results` or `hexashell.benches.results`
  (`_RESULT_MODULES` in `kernel/wire.py`); others cannot be restored in the sandbox. A new field goes on the existing
  class, with a default that leaves existing output unchanged.
- The proxy imports the result modules at its top, because nothing can be imported once the sandbox is closed. LIBERO's
  lines:

  ```python
  from ...robot import results as _perception  # noqa: F401  (loaded before the sandbox closes)
  from ...robot.proxy import NotExecuted, RobotError  # noqa: F401  (the names cells catch)
  from .. import results as _results  # noqa: F401
  ```

  The exception classes are always those of `hexashell.robot.proxy`; the worker takes them from there.
- A host that declines a call raises `CallFailed(kind, text)` (`kernel/host.py`), where `kind` is the exception's name
  as the cell sees it: `NotFound`, `NotExecuted`, `Blocked`, `Dropped`, `ValueError`, `TypeError`, `RobotError`. When the
  episode is over it raises `Stop(text)`, which ends the running cell.
- The sandbox can read the whole `hexashell/` folder (`read_only` in `kernel/worker.py`). Ground truth, task solutions
  and analyses therefore stay out of it; skills are in `skills/` and reach the model only through the task card.

## 3. Settle the protocol first

Before any code, settle the following in `hexashell/benches/<bench>/protocol.py`. The file uses the standard library
only and is imported by both the episode side and the server.

| To settle | LIBERO-PRO's version (`hexashell/benches/libero/protocol.py`) |
|---|---|
| task names and how they are parsed | `<suite>_t<i>`; `parse_task()` parses it and refuses a wrong name |
| the meaning of a seed | seed s takes the task's initial state s % 50, with placements from `np.random.seed(s)` |
| the splits | `LEARN_SEEDS` 0-9, `VALIDATION_SEEDS` 10-19, `TEST_SEEDS` 20-29 |
| step limits, control period, settling steps | `OFFICIAL_STEPS` (220-520), `STEP_S` 0.05, `SETTLE_STEPS` 10 |
| cameras and resolution | `CAMERAS` (our name → the simulator's camera), `IMAGE_SIZE` 512 |
| budgets per episode | `REPLIES` 40, `EPISODE_S` 7200 s |

Rules:

- Follow the official evaluation: its step limits, instructions, success check (checked every step, the episode ends at
  success), controller and gains, in an unmodified environment. Write every departure and its reason into the
  docstring of `protocol.py`.
- Take the official train/test split as given. Do not run test seeds during development; skills rest on learning and
  validation episodes only.
- Keep the ground truth out: the model gets the pictures of the official cameras (RGB or RGB-D) and their parameters,
  proprioception, the instruction and the remaining budget. Object poses and the terms of the success check stay in the
  server and are used only for the verdict of each step.

## 4. Choose a template

| Robot and control | Template | Notes |
|---|---|---|
| one Panda arm, robosuite's OSC_POSE delta control | LIBERO-PRO, `hexashell/benches/libero/` (8 files, about 700 lines) | shares `ArmSession`, `ArmHost`, `ArmServer` and `arm.Arm`. RoboCasa365 adds a moving base (`drive`) and two contact motions (`push_arc`, `rim_pick`); LIBERO-Pro 2.0 adds calibration without depth, changing instructions and timed perturbations |
| two arms of the ARX X5 family (as on RoboDojo), joint targets | RoboTwin 2.0, `hexashell/benches/robotwin/` | motions are planned on the episode side (`RobotHost` with the `Driver` and `Rig` of `hexashell/motion/`); the server only executes joint targets |
| other two-arm robots and wheeled humanoids, parallel grippers | RoboColiseum Season I (AgiBot G2 in Genie Sim 3.5), `hexashell/benches/robocoliseum1/` | its own host and driver (`G2Host`, `G2Driver`); the result types are those of `robot.results` |
| several robots in one benchmark; a moving base, torso and head; five-fingered hands | RoboColiseum Season II (AgiBot G2 and G1, ARX One, AgileX; Genie Sim 3.6), `hexashell/benches/robocoliseum/` | one host for all robots, with kinematics per robot (`kinematics.py`), `body.py` for base, torso and head, `dexterous.py` and `wuji.py` for the hands |
| a legged humanoid with a waist, two dexterous hands and a movable head camera, moved by a whole-body controller | SIMPLE (Unitree G1), `hexashell/benches/simple/` | shares `ArmSession`, `ArmHost` and `armserver.serve`; the server hands posture, walking, palm waypoints, fingers and gaze to SIMPLE's AMO or GR00T whole-body controller; result types in `benches.results` (`BodyMove`, `Walk`, `HandMove`, `Fingers`, `Gaze`) |
| arms with five-fingered hands; each step takes a wrist pose increment and 20 finger joint targets | DexJoCo-X (Franka Panda + Wuji hand), `hexashell/benches/dexjoco_x/` | shares `ArmSession`, `ArmHost` and `armserver.serve`; the server solves the inverse kinematics and turns straight moves of a point on the hand, named hand shapes and single fingers into the official action of each step; contact is judged from the finger joints' proprioception only; result types in `benches.results` (`DexterousMove`, `DexterousFingers`, ...) |
| the evaluator drives | RoboDojo, RoboColiseum | [Section 7](#7-a-benchmark-with-its-own-evaluator) |
| a real robot | `hexashell/real/urai/`, `hexashell/real/franka/` | [Section 11](#11-real-robots) |

`arm.Arm` assumes MuJoCo's `model` and `data` and OSC_POSE delta actions. Another single arm on MuJoCo and OSC needs only
its own `control.py`; with a different kind of control, write a host of your own after RoboTwin or RoboColiseum.

## 5. A benchmark that HexaShell drives

LIBERO-PRO serves as the template here.

### 5.1 Files and processes

```
hexashell/benches/<bench>/
  __init__.py   a docstring only; imports nothing
  protocol.py   the protocol (Section 3), shared by both sides
  sim.py        server: builds the environment, reset(seed), step, verdict, cameras, RGB-D, the robot's own pixels
  control.py    server: a subclass of arm.Arm that puts the 6-D arm action and the gripper into the benchmark's action vector
  server.py     server: a subclass of ArmServer, python -m hexashell.benches.<bench>.server <fd>
  host.py       episode side: a subclass of ArmHost (public calls, camera names and aliases, gripper stroke, hints)
  session.py    episode side: a subclass of ArmSession (arguments of open, task card, record name, fields of result.json)
  proxy.py      in the sandbox: the robot a cell sees
prompts/<bench>/01_rules.md … 06_checking.md
skills/<bench>/<task>.md            once there are skills
tests/fakes_<bench>.py, tests/test_<bench>.py
scripts/setup/setup_<bench>.sh, smoke_<bench>.py
```

An episode has two processes:

- the episode side runs in HexaShell's environment (Python 3.11);
- the server runs in the benchmark's own (`$HEXASHELL_<BENCH>_PYTHON`), because the Python and numpy versions of a
  simulator often conflict with ours (RoboCasa uses Python 3.12 and numpy 2.2).

They talk over a socketpair (the server gets the file descriptor in argv) or over the unix socket of a server that is
kept running. The format of `benches/wire.py` is an 8-byte length, a JSON header and the raw bytes of the arrays; no
pickle. The server's output goes to `server.log` in the episode's record, and the record survives a crash of the
simulator.

Three constraints follow:

- `__init__.py` imports nothing. When the server imports `hexashell.benches.<bench>.server`, the `__init__.py` files of
  `hexashell`, `hexashell.benches` and the package run first; an import of an episode-side module in any of them
  makes the server fail to start.
- Server code (protocol, sim, control, server) imports only the standard library, numpy, the benchmark's own packages and
  `wire`, `armserver`, `arm`, `egl` and `gpucheck` of `hexashell.benches`.
- Server code must run on the Python of every benchmark's environment (down to 3.8): start files with
  `from __future__ import annotations` and avoid what came with 3.9 or later, such as `str.removesuffix`.

### 5.2 The server side

**sim.py** wraps the benchmark's environment. `arm.Arm` uses from it:

- MuJoCo's `model` and `data`, the gripper site's `site_id` and the arm base's `link0_id`;
- `link0()`, `site_pose()`, `tcp()` (4×4, in the episode's frame), `finger_width()`, `finger_speed()`;
- `steps` and `over` (attributes) and `step(action)` (one control step, then one check of success).

`ArmServer` and the episode side use:

| Method | Returns |
|---|---|
| `reset(seed)` | the episode's facts: at least `instruction` (the official one) and `horizon` (the step limit); other fields go into the record and the task card |
| `status()` | `{"step", "horizon", "success", "over"}`, attached to every reply |
| `proprio()`, `arm_joints()` | proprioception; the joint angles at the start are the goal of `home` |
| `camera(cam)` | `{"pose", "focal", "width", "height"}`; the pose is in the episode's frame, the camera looks along −z with +y up in the picture |
| `observe(cams, robot_mask)` | `{"step", "images", "depth", "robot", "cameras", "proprio"}`: RGB, depth (along the optical axis, in metres), the robot's own pixels, camera models |
| `close()` | releases the environment |

What needs care:

- Frames: the model gets exactly one frame, described in the proxy's docstring and in `02_world.md` (LIBERO: the arm's
  base; RoboCasa: the base's projection on the floor at the start; RoboTwin: the world). Convert the camera poses into it.
- Pictures: MuJoCo renders upside down, so turn them back; convert depth to metres with robosuite's
  `get_real_depth_map`. Convert cameras in OpenCV's convention to the one that looks along −z (`from_cv` in
  `vision/cameras.py`).
- Time every picture with `gpucheck.check_time`: it fails when the first picture of an episode takes over 60 s or a
  later one over 10 s, and names the GPU. Check depth for NaN with `gpucheck.check_depth`. A GPU that has reported Xid 109
  can return corrupt pictures without a crash, which only these two checks find.
- The same seed must give the same start. When a running server reuses an environment, its start pictures must equal
  those of a cold start pixel by pixel.

**control.py** is a subclass of `arm.Arm`. Its constructor takes the arm's OSC controller and gripper;
`action(arm6, grip, **extra)` puts the 6-D arm increment and the gripper into the benchmark's action vector. It also
checks the control period and the size of the action and raises when they differ (LIBERO checks `DT == STEP_S` and
`action_dim == 7`). `FLOOR_Z` is the lowest height the grasp point may reach.

**server.py**, after LIBERO's:

```python
from __future__ import annotations

import sys
import time

import numpy as np

from ..armserver import ArmServer, main as serve_main, prepare  # noqa: F401 - prepare: for the warm server
from . import control, sim as S


class Server(ArmServer):
    floor_z = control.FLOOR_Z
    reach_where = "(the arm's base does not move)"

    def __init__(self):
        super().__init__()
        self.start_tcp = None

    def open(self, task: str, seed: int, size: int = 512) -> dict:
        t0 = time.time()
        if self.sim is None or self.sim.task != task or self.sim.size != int(size):  # a warm server keeps the task's environment
            if self.sim is not None:
                self.sim.close()
            self.sim = None
            self.sim = S.NewBenchSim(task, size=int(size))
        facts = self.sim.reset(int(seed))
        self.robot = control.Robot(self.sim)
        self.start_tcp = self.sim.tcp()
        self.home_joints = self.sim.arm_joints()
        return {"facts": facts, "start": self.sim.proprio(), "seconds": round(time.time() - t0, 2),
                "level": self.robot.level()}

    def home_goal(self) -> np.ndarray:
        return self.start_tcp.copy()


if __name__ == "__main__":
    serve_main(sys.argv[1:], Server())
```

- `ArmServer` already implements `observe`, `state`, `reach`, `goto`, `arc`, `push_arc`, `home`, `gripper`, `wait` and
  `close`. A new request is a method `op_<name>(self, **args) -> dict`, such as RoboCasa's `op_drive`. The class attributes
  `floor_z`, `reach_where` and `elbow_room` can be changed (`elbow_room` is how far from straight a pose must leave the
  arm: 35° by default, 30° on LIBERO).
- A request that is not carried out raises `armserver.Refused(reason)`: nothing has moved, and the model gets a ValueError
  with the reason. Any other exception counts as a crash of the simulator: the episode is recorded as "simulator
  failed", and `bench_pool.py` runs it again on a new server.
- `prepare()` runs once before the simulator is imported (also in a server that is kept running). MuJoCo benchmarks use
  `armserver.prepare`, which picks the rendering GPU from `$HEXA_EGL_DEVICE` (`egl.pin`). Others bring their own: RoboTwin
  sets `CUDA_DEVICE_ORDER=PCI_BUS_ID` and `CUDA_VISIBLE_DEVICES` before it imports SAPIEN.
- A server that is kept running (`hexashell.benches.daemon`) serves many episodes with one `Server()` and sets
  `server.warm` to True; `op_close` then keeps the environment, and an episode of the same task need not build it again.

### 5.3 The episode side

**session.py**:

```python
class NewBenchSession(ArmSession):
    bench = "newbench"                                   # the bench of result.json, and the folder under prompts/ and skills/
    proxy = "hexashell.benches.newbench.proxy"
    server_module = "hexashell.benches.newbench.server"  # must start with hexashell.benches.
    host_class = NewBenchHost

    def __init__(self, info, *, python, env_file=None, egl_device=None, size=P.IMAGE_SIZE):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=size)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])

    def open_args(self):        # arguments of the server's open() (ArmSession adds size)
        return {"task": self.task, "seed": self.seed}

    def record_name(self):      # the record folder's name before its time stamp
        return f"{self.task}_s{self.seed}"

    def labels(self):           # {task} and {seed} in the path of --set script=...
        return {"task": self.task, "seed": self.seed}

    def card(self, settings):   # the task card's text, and what goes into result.json as "used"
        lines = ["# Task card", "",
                 f"Instruction (from the benchmark; it is authoritative): {self.facts.get('instruction')!r}"]
        ...
        used = {"task": self.task, "skill": False}
        if settings.skills:
            text = skill(self.task)  # skills/newbench/<task>.md
            if text:
                lines += ["", "## Notes for this task", "", text]
                used["skill"] = True
        return "\n".join(lines) + "\n", used

    def result_fields(self, verdict):  # the first fields of result.json
        return {"task": self.task, "seed": self.seed, "instruction": self.instruction}
```

`ArmSession` already:

- starts the server (or connects to a running one) and sends `open`;
- sets up perception, a `DepthScene` with SAM3 and depth; when SAM3 does not answer, it notes "perception unavailable"
  in `errors`, and the episode goes on;
- sets up the host, takes the start pictures and forwards the robot calls;
- reads `over`, `step`, `step_limit` and `verdict` from the status attached to each reply of the server;
- writes one line of `states.jsonl` per control step.

Optional hooks:

| Hook | Purpose | Example |
|---|---|---|
| `prompt_variant()` | replaces some prompt parts under some setting | RoboTwin and LIBERO-Pro 2.0 return `"rgb"` without depth |
| `cell_note()` | tells the model after a cell what changed on the benchmark's side | LIBERO-Pro 2.0: the instruction changes during an episode |
| `server_timeout_s` | the first request imports, builds the scene and renders the first picture, which takes long | 900 s by default, 1800 s on RoboTwin and SIMPLE |
| `row_extra(rest)` | turns columns the server adds to each step's record into fields of `states.jsonl` | RoboCasa's base |

The task card holds the official instruction (and says that it is authoritative), the task's name unless the
benchmark's rules exclude it, the step budget (steps and the simulated seconds they stand for), the hand's start pose,
the frame, and which camera each start picture comes from. It holds no ground truth. Episode adds the sentence "You have
up to N replies and M minutes for this episode;" followed by `timing_note()`, whose default for a simulator is "the
simulation stands still while you think."

**host.py** sets the class attributes of `ArmHost`:

- `public`: the names of the calls it offers, by default `ARM_CALLS`;
- `cams`, `aliases`: camera names and aliases;
- `finger_stroke_m`: the distance between the fingers when the gripper is fully open;
- `step_s`: the control period;
- the advice given when a pose is out of reach, a motion is blocked or the arm is stretched: `unreachable_note`,
  `stalled_advice`, `stretched_advice`, `jammed_advice`.

For `robot.<name>`, `ArmHost.dispatch` checks that the name is in `public` and calls `self._<name>(...)`. Requests to
the server go through `self._call(op, **args)`.

A new robot call takes four changes (RoboCasa's `drive` was added this way):

1. proxy: the method with its docstring;
2. host: the name in `public`, and `_<name>`;
3. server: `op_<name>` where needed;
4. prompt: its use in `03_api.md`; and tests.

**proxy.py**: a single-arm benchmark can copy LIBERO's (which has no arm argument) or inherit an existing class and
change the docstrings only (LIBERO-Pro 2.0 inherits LIBERO's, RoboTwin inherits RoboDojo's `hexashell.robot.proxy.Robot`).
The module's docstring states the units, the frame and the pixel convention; each method states its arguments, its
result, and which exception it raises when. The docstrings and the prompt are all the model knows of these calls.

### 5.4 Two arms, after RoboTwin

RoboTwin's arms are of the same X5 family as RoboDojo's, so RoboTwin uses RoboDojo's motion planner, and its server only
executes joint targets:

- `rig.py` describes the robot with `motion.rig.Rig`: the two bases, the URDF, the end, the joint ranges, how far the
  grasp point and the finger tips reach, the table, the gripper's model (`GripperModel`) and the control period.
- `TwinHost` (`host.py`) inherits `robot.host.RobotHost` and passes it the `rig`. It overrides `act(action)`, which puts
  both arms' joints and grippers into 14 numbers, calls `session.call("act", action=q)` and returns the new `Snapshot`.
  It also overrides `over()`, `dispatch()` (which turns `SimRefused` and `SimError` into the errors the model sees) and the
  picture calls `views()`, `pictures_now()` and `cameras_now()`.
- `RoboTwinSession` (`session.py`) inherits `BenchSession` (not `ArmSession`) and implements `open` itself: it starts the
  server, sends `open`, and sets up perception and `TwinHost`.
- `server.py` handles only `ping`, `open`, `close`, `act`, `state` and `observe`, and has `status()`. It does not inherit
  `ArmServer` but uses `armserver.serve` for the wire.
- Without depth, perception is RoboDojo's `Scene` (`TwinScene` in `robotwin/scene.py`), `prompt_variant()` returns
  `"rgb"`, and the `*.rgb.md` files of `prompts/robotwin/` replace the parts of the same name.

## 6. Registration, settings and batches

| Where | What to add | When |
|---|---|---|
| `hexashell/benches/run.py` | the name in `BENCHES`; in `bench_defaults()`, a check of the task's name by the protocol and `{"max_turns": P.REPLIES, "budget_s": P.EPISODE_S, "ladder": False}`; the session in `make_session()` | always |
| `config/env.example.sh` | the lines `HEXASHELL_<BENCH>_PYTHON` and `HEXASHELL_<BENCH>_ENV`; each machine's own `config/env.sh` (not in git) follows it | always |
| `record_name` in `scripts/bench_batch.sh` and `scripts/bench_pool.py` | the same rule as the session's `record_name()` | when the name is not `<task>_s<seed>` (LIBERO-Pro 2.0 uses a hash) |
| `CHECKERS` in `scripts/bench_pool.py` | the benchmark's name | when its environment can run MuJoCo's GPU check |
| `scripts/bench_score.py` | a table in the official measure | when the official score is not a plain success rate |
| `NOTICE` | the benchmark and its license; borrowed code and facts | always |
| `README.md` | a short section: environment, commands, results | always |

Both batch scripts find results by the record's name to decide what still has to run, so the three rules for it must
agree.

Agent settings are overridden in this order: the benchmark's defaults (`bench_defaults`) → `config/benches.yml` →
environment variables `HEXASHELL_<SETTING>` → `--set name=value`. Settings of the machine (Python, environment file, GPU)
are not in benches.yml but in the command line of `run.py`.

To run (after `source config/env.sh`):

```bash
# one episode: starts its own server, writes its record under --out
python -m hexashell.benches.run --bench newbench --task <task> --seed 0 --egl 0 --out results/newbench_dev

# cells from a file (separated by "# %%") in place of the model: checks every tool without a model
python -m hexashell.benches.run --bench newbench --task <task> --seed 0 --set script=cells.py

# a batch: each line of logs/NAME.todo is "BENCH TASK SEED"; servers are kept running, one GPU per slot
python3 scripts/bench_pool.py NAME 0,0,1,1
```

Benchmarks that start slowly should use `bench_pool.py`: a server starts once and plays episode after episode, and a
free slot prefers an episode of the task it played last. `bench_batch.sh` starts cold for every episode and suits
benchmarks that start quickly. Run batches from a separate checkout of a fixed commit, so that edits do not reach a run
under way.

## 7. A benchmark with its own evaluator

Some benchmarks run only through their own evaluator (an official client, its own timing). HexaShell is then the policy
server, and the session does not start a simulator but waits for the observations the evaluator sends:

- RoboDojo: XPolicyLab asks the policy for an action at every step. While the model thinks, `Bridge`
  (`hexashell/xpl/bridge.py`) answers WAIT (`[{"hexashell": "wait"}]`), and the simulator stands still.
  `RoboDojoSession` (`hexashell/xpl/session.py`) wraps it as a session.
- RoboColiseum Season I: `hexashell/benches/robocoliseum1/run.py` starts a WebSocket server and the Genie Sim process
  (`app.py` of genie_sim, see `sim.py`). The simulator connects to the server (`link.py`), and the episodes of a run
  follow one another on that connection. For each episode the server calls `play(link)` in a thread, which runs
  `Episode(ColiseumSession(link, info), settings, out)`. `conclude()` holds the robot still and waits for the
  evaluator's verdict (`VERDICT_CAP_S`). The evaluator gives no step count, so `step` and `step_limit` return None.
- RoboColiseum Season II (`hexashell/benches/robocoliseum/`) works in the same way with one simulator container per GPU.
  In both seasons `tunnel.py` plays on the challenge platform: it dials the platform's gateway in place of the local
  simulator, and the sessions stay the same.

To keep in mind:

- The simulation must not advance while the model thinks. Where it does, set `deadline_budgets` to 1 as on a real robot,
  and say so in `timing_note()`. Where the evaluator limits the time of a reply, answer in time with an action that holds
  everything still.
- Take verdicts from the evaluator only.
- Use only your own API keys. Where a judge model is needed, call it with your own key (in local RoboColiseum runs
  `judge.py` stands in for the official judge).

## 8. Prompts and skills

### 8.1 The prompt

- The system prompt is in `prompts/<bench>/`: `01_rules.md`, `02_world.md`, `03_api.md`, `04_seeing.md`, `05_acting.md`,
  `06_checking.md`, joined in the order of their names (`system_prompt` in `runtime/card.py`; files with a "." in the
  stem are variants). Copy the closest benchmark's and change what differs: the frame, the cameras, the calls, the units,
  the control period. The parts begin with `# 1.` to `# 6.`, which the tests check.
- Variants: when `prompt_variant()` returns a name, `<part>.<variant>.md` replaces `<part>.md` (RoboTwin's
  `02_world.rgb.md`).
- The prompt states general rules for the tools. It does not say how to do a task and holds no scene coordinates.

### 8.2 Skills

A skill is `skills/<bench>/<task>.md`, typically about a page. With `skills: true` the session's `card()` puts it at the
end of the task card under "## Notes for this task". The sections used so far:

| Section | Content |
|---|---|
| Judging | what counts as done, as earlier runs showed it; no internal threshold of the checker |
| Flow | the steps that worked, in order, with the calls to use |
| Verified parameters | numbers measured in earlier runs: grasp heights, offsets, openings |
| Common failures and fixes | what went wrong, and what to do instead |
| Typical step costs | how many steps each phase took, for planning the budget |

Rules:

- Learn on learning seeds and check on validation seeds; do not look at test seeds.
- Write only what the agent could observe: no positions read from the simulator, no internal names, no thresholds of a
  checker.
- Leave out what changes with the layout (absolute coordinates, colours, counts); give positions relative to objects.
- Mark the evidence of each claim (how many episodes, learning or validation), and have a person review every skill.
- Add an entry to `skills/README.md`.

When the benchmark's rules exclude task-specific input, as RoboDojo's agent track does, keep `skills: false`.

### 8.3 Learning in rounds

1. **A round without skills** on the learning seeds, a few episodes per task. Read the records of the failures
   (`prompt.md`, `cells.jsonl`, `pictures/`, `result.json`).
2. **Fix what is general first**: a tool that misreports, an unclear docstring or error message, a missing check, a line
   of the prompt. Such a fix helps every task. Verify it on learning seeds before going on.
3. **Draft the skills**: an agent reads the records of a group of related tasks, including every cell, output and
   picture, and writes what is left that is specific to a task; a person reviews each draft.
4. **Validate**: run the validation seeds with and without the skills in the same batch, so that both see the same
   machines and the same model endpoint, and compare the same episodes pairwise. Revise the skills that did not help, and
   check the revised ones on the validation seeds again.
5. **Freeze and test**: fix the commit, then run the test seeds once, without and with skills, with the official step
   limits. Report the settings with the result.

Keep the runs of a comparison on the same settings (model, reasoning effort, picture window, limits), and record the
number of requests that failed at the endpoint: an overloaded endpoint costs episodes that the method would have solved.

## 9. Environments, GPUs and data

- Each benchmark has its own Python environment. Put an install script `setup_<bench>.sh`, a locked list of packages and
  a smoke test `smoke_<bench>.py` into `scripts/setup/`.
- `$HEXASHELL_<BENCH>_ENV` names a file with the server's environment variables. Only lines of the form
  `export NAME=value` are read (`read_env_file` in `benches/session.py`); `$VAR` in a value is expanded from the current
  environment, and no other shell statement runs.
- `server_env()` builds the server's environment: it drops `PYTHON*`, `VIRTUAL_ENV*`, `CONDA*` and every variable whose
  name contains `KEY`, `TOKEN` or `SECRET`, then adds the variables of the environment file, `PYTHONPATH` (the repository),
  `PYTHONUNBUFFERED` and `HEXA_EGL_DEVICE`. The API key never reaches the server; do not write it into the file.
- GPUs: `--egl` and `$HEXASHELL_EGL_DEVICE` use nvidia-smi's numbers. For MuJoCo, `egl.pin` converts them to EGL's device
  numbers, whose order may differ; SAPIEN takes `CUDA_VISIBLE_DEVICES`.
- `bench_pool.py` runs `python -m hexashell.benches.gpucheck GPU` before it starts the first server on a GPU and after a
  server has failed there; a GPU that fails the check gets no episodes.
- Link to copies of large assets and datasets that a machine already has instead of copying them.

## 10. Tests and acceptance

The tests need no simulator: each benchmark has a fake server that answers the same requests.

- `tests/fakes_<bench>.py` has two parts:
  - a fake scene that answers the requests of the simulator. LIBERO's `FakeTable` reuses the rendering, straight motions,
    gripper and colour-based SAM3 of `fakes_casa.py`.
  - a subclass of the session that replaces `start_server` by `fakes_casa.FakeLink(scene)` (requests still pass through the
    wire's encoding) and `_scene` by a `DepthScene` with `FakeSam3`.
- `tests/test_<bench>.py` (or an addition to `tests/test_benches.py`) covers at least:
  - the protocol: task names, splits, step limits;
  - the host: the results of some motions;
  - the task card and the prompt folder: six parts, no call of another benchmark;
  - one complete scripted episode: `fakes.ScriptedGateway` in place of the endpoint, no environment variable starting with
    `HEXASHELL_`, `Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30)`, then
    checks of `result.json`, `cells.jsonl` and the system prompt. `test_scripted_libero_episode` in
    `tests/test_benches.py` can be copied.
- All tests: `python -m pytest -q tests/`.

Acceptance on the real simulator:

1. Smoke: `smoke_<bench>.py` builds the environment, resets and renders.
2. Scripted episode: `--set script=` walks through every call. `server.log` has no exception, the pictures in `pictures/`
   have the right orientation and colours, and `to_world` and `to_pixel` agree with each other.
3. Determinism: two starts with the same seed give the same pictures, and so do a running server and a cold start.
4. Verdict: the success check agrees with official demonstrations or the official evaluation script on several episodes.
5. A few episodes with the model on learning seeds; read `prompt.md`, `cells.jsonl` and `events.jsonl`.
6. After a change to a shared module, check that the prompts, task cards, docstrings and result texts of the other
   benchmarks are unchanged byte by byte.

## 11. Real robots

### 11.1 How they differ from simulation

A service on the robot's side plans and executes (inverse kinematics, collision checks, time scaling, execution).
HexaShell talks to it over HTTP only and is responsible for the conversation, the cells, the record, the pictures and
the conversion between pixels and the world.

| | Simulation | Real robot |
|---|---|---|
| verdict | the benchmark's, at every step | none: `success` in `result.json` is null and the report of `finish` is recorded as `agent_report`; `finish` has one more status, `done` |
| time | stands still while the model thinks; an episode lasts at most 2 `budget_s` (`deadline_budgets = 2`) | goes on; at most 1 (`deadline_budgets = 1`), and `timing_note()` says so |
| steps | control steps of the simulator | the service's execution time divided by 0.04 s, rounded up |
| perception | SAM3 and depth: `find`, `measure`, `things` | without SAM3: `look` and `image` show pictures, and `to_world` takes points by depth, by a given plane or from two views |
| skills | optional, in the task card | none so far (the session's `card()` would add them) |
| record name | `<task>_s<seed>-<time>` | `<bench>-<name>-<time>` |

### 11.2 A robot that URAI already runs

```bash
python -m hexashell.real.urai --url http://127.0.0.1:7860 \
  --task 'Move the red block to (0.20, -0.02).' --name move_block --results runs
```

[URAI](https://github.com/HexaFuture/URAI)'s side is started as its README describes, in a version that answers the
requests of 11.3 (the public one does not answer `/api/harness/...` yet). `--url` or `URAI_URL` is required; a token goes
into `URAI_TOKEN`. The service reports the robot's arms, cameras and TCP (`/api/harness/model`). If the camera names, the
planning camera or the wording differ from the PiPER-X's, write a subclass as in [11.4](#114-the-hexashell-side).

### 11.3 A new robot: a service that speaks URAI's interface

`UraiClient` (`hexashell/real/urai/client.py`) uses only the requests below. A new robot implements them as the FR3 does
(`service.py` and `server.py` in `hexashell/real/franka/`); little is then left to write on HexaShell's side.

| Request | Body | Reply |
|---|---|---|
| `GET /api/harness/model` | | `name`, `arm_names`, `table_z`, `cameras`; per arm `t_world_base`, `tcp_offset_mm`, `table_z`, `tip_past_tcp_mm`, `max_opening_mm`, `joint_limits_deg` |
| `POST /api/harness/observe` | `{"camera"}` | `id`, `timestamp`, `camera`; `rgb` and `depth` (`{"encoding": "npy-base64", "data"}`, depth in metres and aligned to RGB); `k`, `t_world_camera`, `arms`; `capsules` (per arm, the links' collision capsules `a`, `b`, `radius_m`) |
| `GET /api/state` | | `arms` (per arm `xyz`, `rpy_deg`, `joints_deg`, `gripper_mm`, `feedback_age_s`, `enabled`), `execution` |
| `PUT /api/draft` | `{"observation_id", "completion": "hold", "arms": {arm: action}}` | `{"revision"}` |
| `POST /api/preview?trajectory=full` | `{"expected_revision", "planning_only"}` | `id`, `duration_s`, `pose_tolerance` (`position_mm`, `orientation_deg`), `arms[arm].xyz` |
| `POST /api/execute` | `{"preview_id"}` | returns at once; the client polls `/api/state` until `execution.state` is neither running nor cancelling |

The FR3's service also has `POST /api/cancel` and `POST /api/recover` (for the operator; the client does not call them)
and `GET /api/cameras/<camera>/image.jpg` and `GET /api/live.jpg` for watching.

A draft holds one action per arm (both arms of a two-arm robot may move in one plan):

```text
{"primitive": "goto", "xyz": [x, y, z], "rpy_deg": [a, b, c], "motion": "line" or "swing", "speed_m_s": 0.12, "opening_mm": optional}
{"primitive": "gripper", "opening_mm": opening, "effort": 1000, "duration_s": full opening / 100 + 0.3}
{"primitive": "home", "joints_deg": the joint angles at the start of the episode, "speed_m_s": 0.12}
{"primitive": "wait", "duration_s": steps × 0.04}
```

- `rpy_deg` are extrinsic xyz Euler angles, R = Rz(c)·Ry(b)·Rx(a) (`hexashell/real/urai/frames.py`).
- `motion` is `"line"` (straight) or `"swing"` (a path in joint space). `goto(path="auto")` tries the straight line first
  and then the joint path, which it accepts only if it stays within 3 cm of the line and at most 1 cm below both ends.

Rules the service keeps (all implemented for the FR3):

- Read-only mode: without `--allow-motion` it takes pictures, reports its state and plans (`planning_only`, the model's
  `dry_run`) but does not move. Other previews get 409, execute and recover 403.
- A plan belongs to a picture: the draft's `observation_id` must be the latest of the planning camera, the preview's
  `expected_revision` the latest draft, and execute takes only the latest plan that has not run and is not
  planning_only, with the arm at most 0.03 rad from where it was planned. Otherwise 409, and the model plans again.
- While a motion runs, previews and executes get 409. While the robot reports an error, execute gets 409 until the operator
  recovers it.
- The `detail` of a 409 states the reason; the host passes it on to the model (NotExecuted).
- `execution` is `{"id", "state", "execution_time_s", "error", "last_error"}` with `state` one of idle, running, cancelling,
  completed, cancelled, error. After an empty grasp, `execution.result.outcome` is `"empty_grasp"` and `last_error` says so in
  one sentence; the model gets Dropped.
- It listens on loopback by default. On another address it needs a token (`Authorization: Bearer <token>`), and answers 401
  without it.
- It is the only holder of the robot's control interface (the FR3's FCI takes one controlling client).
- The capsules must fit the real arm: HexaShell projects them into the picture and compares them with the measured depth;
  what lies within 1 cm counts as the robot.

The FR3's service starts in three ways:

```bash
python -m hexashell.real.franka.server --sim --port 7861              # kinematic simulator
python -m hexashell.real.franka.server --rig rig.json --port 7861     # read-only: plans, never moves
python -m hexashell.real.franka.server --rig rig.json --allow-motion  # the robot moves
```

| File | Content |
|---|---|
| `service.py` | everything that does not depend on HTTP, with the rules above |
| `server.py` | routes and token |
| `driver.py` | the connection to franky |
| `arm.py`, `planner.py` | kinematics and planning |
| `cameras.py` | the connection to RealSense cameras |
| `rig.py` | reads `rig.json`: FCI address, camera serial numbers, hand-eye calibration, table height |
| `simulated.py` | the kinematic simulator for tests and `--sim` |

### 11.4 The HexaShell side

As for the FR3, inherit URAI's host and session and change the cameras, the wording and the proxy. The main part of
`hexashell/real/franka/session.py`:

```python
CAMERAS = {"external": "external", "wrist": "wrist"}  # HexaShell's camera names -> the service's


class FrankaHost(UraiHost):
    cameras = CAMERAS
    plan_camera = "external"      # the camera a plan belongs to
    texts = texts                 # the sentences the model gets
    gentle_speed_m_s = 0.03       # the speed of goto(gentle=True)
    public = tuple(name for name in PUBLIC if name != "both")  # one arm

    def check_cam(self, cam) -> str:
        if cam not in self.cameras:
            raise Refused(f"camera must be one of {', '.join(self.cameras)}, not {cam!r}")
        return cam


class FrankaSession(UraiSession):
    bench = "franka"
    proxy = "hexashell.real.franka.proxy"
    host_class = FrankaHost
    texts = texts


def main(argv=None) -> int:
    return urai_session.main(argv, prog="python -m hexashell.real.franka", service="FR3", env="FRANKA",
                             session_class=FrankaSession)
```

It also takes:

- `proxy.py`: the one-arm `Robot` (the FR3's arm is named `"arm"`) with its docstrings, without `find`, `measure` and
  `things`;
- `texts.py`: the sentences the model gets; those that are URAI's are imported, others (`TIMING`, `CONTACT`, `CANCELLED`)
  describe this robot;
- `prompts/<bench>/`: the six parts;
- `__main__.py`: calls `main()`.

`env="FRANKA"` gives the environment variables `FRANKA_URL` and `FRANKA_TOKEN`. One episode:

```bash
python -m hexashell.real.franka --url http://127.0.0.1:7861 \
  --task 'Put the red block in the bowl.' --name red_block --results runs
```

The other arguments are URAI's: `--script cells.py`, `--config settings.json`, `--step-limit`, `--speed`.

### 11.5 The order of work on a real robot

1. Unit tests with a fake client (`tests/fakes_urai.py`, `tests/test_real_urai.py`); no service needed.
2. Complete episodes against the service's kinematic simulator (`--sim`; `tests/test_real_franka.py`,
   `tests/test_real_urai_sim.py`).
3. The real robot, read-only: connect, read the state, take pictures, plan, and see that other clients of the robot are
   not disturbed.
4. Calibration: hand-eye calibration, table height and the measured length of the tool in `rig.json`; check in the
   annotated pictures that the robot's pixels fit the real arm.
5. The first run with `--allow-motion`: slow and short, with a person at the emergency stop.
6. Episodes with the model.

## 12. Conventions

- API keys stay out of the repository, the sandbox and the servers; check the index before every push.
- Ground truth is used in the server only, for the verdict. `hexashell/` holds neither ground truth nor task knowledge,
  because the sandbox can read it.
- No test seeds during development; skills come from learning and validation seeds only.
- Additions to shared modules (runtime, kernel, vision, robot) are optional, with defaults that keep today's behaviour.
- What a new benchmark does differently goes into its own subclasses; the packages of other benchmarks stay as they are.
- Borrowed code and facts are named in NOTICE (for example `capsules.py`, copied from URAI under Apache-2.0).
- Names are complete English words.
