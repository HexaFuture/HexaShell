# 接入新的仿真基准或真机

**[English](adding_benchmarks_and_robots.md) | 简体中文**

本文说明怎样把 HexaShell 接到一个新的仿真基准或一台新的真机上。它既写给人，也写给编程智能体：一句话就可以把这件事交给后者（[第 1 节](#1-一句话)）。
现有的适配是最直接的参考：先在[第 4 节](#4-选模板)找到最接近的一个，再照着它的文件逐个修改；各适配的取舍写在各自 `protocol.py` 的 docstring 里。
文中路径都相对仓库根目录。

1. [一句话](#1-一句话)
2. [结构](#2-结构)
3. [先定协议](#3-先定协议)
4. [选模板](#4-选模板)
5. [HexaShell 驱动的仿真基准](#5-hexashell-驱动的仿真基准)
6. [注册、设置和批量](#6-注册设置和批量)
7. [由评测程序驱动的基准](#7-由评测程序驱动的基准)
8. [提示词和技能](#8-提示词和技能)
9. [环境、GPU 和数据](#9-环境gpu-和数据)
10. [测试和验收](#10-测试和验收)
11. [真机](#11-真机)
12. [约定](#12-约定)

## 1. 一句话

在编程智能体（Claude Code、Codex 等）里打开本仓库。接入仿真基准时说：

```text
参考 docs/adding_benchmarks_and_robots_zh.md，把 HexaShell 适配到 <基准名>（<仓库地址或本地路径>）。
先制定适配计划和学习计划，等我确认后再写代码。
```

接入真机时说：

```text
参考 docs/adding_benchmarks_and_robots_zh.md，把 HexaShell 适配到 <机器人>（手臂：…，手：…，相机：…，控制接口：…）。
先制定适配计划和学习计划。先用服务的仿真模式和只读模式，没有我的同意不要让机器人动。
```

两份计划要回答下面这些问题。

**适配计划**

| 问题 | 依据 |
|---|---|
| 有哪些任务？机器人、控制接口、相机（RGB 还是 RGB-D）、步数上限、成功判定、官方的种子和划分是什么？ | 基准的代码和论文 |
| 基准的规则允许智能体用哪些输入（任务说明、技能、示范）？ | 基准的规则 |
| 谁推进循环？哪个现有适配最接近？ | [第 2 节](#2-结构)、[第 4 节](#4-选模板) |
| `protocol.py` 里写什么：任务名、种子的含义、划分、上限、相机、预算、坐标系？ | [第 3 节](#3-先定协议) |
| robot 的哪些调用可以沿用，哪些要新增（底盘、躯干、手指、可动相机）？ | [第 4 节](#4-选模板)、[5.3](#53-episode-一侧) |
| 要写哪些文件、测试和验收，按什么顺序？ | [第 5 节](#5-hexashell-驱动的仿真基准)、[第 10 节](#10-测试和验收)、[11.5](#115-上真机的顺序) |
| 与官方评测有哪些不同，为什么？ | 列进 `protocol.py` 的 docstring |

**学习计划**

| 问题 | 依据 |
|---|---|
| 哪些回合用于学习、验证和测试？开发期间不跑测试种子。 | [第 3 节](#3-先定协议) |
| 分几轮，每轮要决定什么？ | [8.3](#83-分轮学习) |
| 用选定的模型，一轮要多少个回合、多少时间、多少 token？ | 头几个回合的 `result.json` |
| 开发到什么时候停，报告哪一次运行？ | [8.3](#83-分轮学习) |

## 2. 结构

一个回合由 `Episode`（`hexashell/runtime/episode.py`）运行，它负责与模型的对话、`python` 和 `finish` 两个工具、沙箱里的 cell、预算和记录。
与基准或机器人有关的部分都由一个 `Session` 子类提供（接口和默认实现见 `hexashell/runtime/session.py`）。接入新基准，就是编写这个 Session
以及它下面的 host、proxy 和服务端。

```
模型网关 ⇄ Episode（runtime/episode.py）：对话、预算、记录
              │ python 工具
              ▼
          沙箱 worker（kernel/worker.py：python -I，没有 API key，Landlock + seccomp）
              cell 里的 robot = importlib.import_module(session.proxy).Robot(call)
              │ call(name, args, kwargs)
              ▼
          Session.dispatch → host：检查参数、规划或转发、感知、叠图、记录
              │
              ▼
          仿真服务（基准自己的 Python 环境）或真机服务（HTTP）
```

接入方式有三种：

| 方式 | 谁推进循环 | 现有例子 |
|---|---|---|
| HexaShell 驱动仿真 | Episode 启动一个仿真服务进程，每个 robot 调用推进若干仿真步 | RoboCasa365、LIBERO-PRO、LIBERO-Pro 2.0、RoboTwin 2.0、SIMPLE、DexJoCo-X（`hexashell/benches/<基准>/`） |
| 评测程序驱动 | 基准的评测程序按自己的节奏索要动作，HexaShell 应答 | RoboDojo（XPolicyLab，`hexashell/xpl/`）、RoboColiseum（`hexashell/benches/robocoliseum1/`、`hexashell/benches/robocoliseum/`） |
| 真机 | 机器人一侧的服务负责规划和执行，HexaShell 只通过 HTTP 与它通信 | URAI（`hexashell/real/urai/`）、Franka FR3（`hexashell/real/franka/`） |

能在 Python 里自行 reset 和 step 的仿真基准用第一种（[第 5 节](#5-hexashell-驱动的仿真基准)）；官方评测只能通过它自己的客户端运行时用第二种
（[第 7 节](#7-由评测程序驱动的基准)）；真机见[第 11 节](#11-真机)。

### 沙箱的边界

以下几条对所有适配都成立：

- `session.proxy` 必须是 `hexashell.` 包里的模块（`kernel/host.py` 会检查）。它的 `Robot(call)` 只做转发，每个方法都是
  `return self._call(name, args, kwargs)`。方法的 docstring 就是模型通过 `help(robot.goto)` 看到的说明，和提示词同样重要。
- 返回给 cell 的结果对象只能是 `hexashell.robot.results` 或 `hexashell.benches.results` 里的类（`kernel/wire.py` 的
  `_RESULT_MODULES`），其他类在沙箱里无法还原。需要新字段时加在原类上，并给一个不改变现有输出的默认值。
- proxy 模块开头要 import 结果模块，因为沙箱关闭后就无法再 import。LIBERO 的写法：

  ```python
  from ...robot import results as _perception  # noqa: F401  (loaded before the sandbox closes)
  from ...robot.proxy import NotExecuted, RobotError  # noqa: F401  (the names cells catch)
  from .. import results as _results  # noqa: F401
  ```

  异常类一律使用 `hexashell.robot.proxy` 里的，worker 从那里取。
- host 拒绝一个调用时抛 `CallFailed(kind, text)`（`kernel/host.py`），`kind` 是 cell 里看到的异常名：`NotFound`、`NotExecuted`、
  `Blocked`、`Dropped`、`ValueError`、`TypeError`、`RobotError`。回合已经结束时抛 `Stop(text)`，当前 cell 随即停止。
- 沙箱可以读取整个 `hexashell/` 目录（`kernel/worker.py` 的 `read_only`）。因此这里不放评测真值、任务做法或分析；技能放在 `skills/`，
  只经由任务卡交给模型。

## 3. 先定协议

写代码之前，先把下面这些事定下来，写进 `hexashell/benches/<基准>/protocol.py`。这个文件只依赖标准库，episode 一侧和服务端都会 import 它。

| 要定的事 | LIBERO-PRO 的写法（`hexashell/benches/libero/protocol.py`） |
|---|---|
| 任务名及其解析 | `<suite>_t<i>`；`parse_task()` 解析，名字不对就报错 |
| 种子的含义 | 种子 s 使用该任务的第 s % 50 个初始状态，摆放用 `np.random.seed(s)` |
| 划分 | `LEARN_SEEDS` 0-9、`VALIDATION_SEEDS` 10-19、`TEST_SEEDS` 20-29 |
| 步数上限、控制周期、起始等待 | `OFFICIAL_STEPS`（220-520 步）、`STEP_S` 0.05、`SETTLE_STEPS` 10 |
| 相机和分辨率 | `CAMERAS`（我们的相机名 → 仿真里的相机）、`IMAGE_SIZE` 512 |
| 每个回合的预算 | `REPLIES` 40 次回复、`EPISODE_S` 7200 s |

几条规矩：

- 评测与官方对齐：使用官方的步数上限、指令、成功判定（每步判一次，成功即结束）、控制器和增益，不修改环境。每一处不同及其原因都写进
  `protocol.py` 的 docstring。
- 划分以官方的 train/test 为准。开发期间不跑测试种子，技能只依据学习种子和验证种子上的回合。
- 真值隔离：模型只能拿到官方相机的画面（RGB 或 RGB-D）和相机参数、本体感知、指令和剩余预算。物体的真实位姿和成功判定的内部条件都留在服务端，
  只用于每步的判定。

## 4. 选模板

| 机器人和控制方式 | 参考 | 说明 |
|---|---|---|
| 单臂 Panda，robosuite 的 OSC_POSE 增量控制 | LIBERO-PRO，`hexashell/benches/libero/`（8 个文件，约 700 行） | 共用 `ArmSession`、`ArmHost`、`ArmServer`、`arm.Arm`。RoboCasa365 在此基础上增加了底盘（`drive`）和两个接触动作（`push_arc`、`rim_pick`）；LIBERO-Pro 2.0 增加了无深度的相机标定、会变化的指令和带时间的扰动 |
| ARX X5 系双臂（与 RoboDojo 同系），关节目标控制 | RoboTwin 2.0，`hexashell/benches/robotwin/` | 运动在 episode 一侧规划（`RobotHost` + `hexashell/motion/` 的 `Driver` 和 `Rig`），服务端只执行关节目标 |
| 其他双臂机器人和轮式人形，平行夹爪 | RoboColiseum 第一赛季（Genie Sim 3.5 里的 AgiBot G2），`hexashell/benches/robocoliseum1/` | 自己的 host 和 driver（`G2Host`、`G2Driver`），结果类型仍用 `robot.results` |
| 一个基准里有多种机器人；可动的底盘、躯干和头部；五指灵巧手 | RoboColiseum 第二赛季（AgiBot G2 和 G1、ARX One、AgileX；Genie Sim 3.6），`hexashell/benches/robocoliseum/` | 所有机器人共用一个 host，运动学按机器人分开（`kinematics.py`），底盘、躯干和头部在 `body.py`，手在 `dexterous.py` 和 `wuji.py` |
| 双足人形：有腰、两只灵巧手和可转的头部相机，由全身控制器执行身体命令 | SIMPLE（Unitree G1），`hexashell/benches/simple/` | 共用 `ArmSession`、`ArmHost` 和 `armserver.serve`；服务端把身体、行走、手掌路径点、手指和视线命令交给 SIMPLE 的 AMO 或 GR00T 全身控制器执行，结果类型在 `benches.results`（`BodyMove`、`Walk`、`HandMove`、`Fingers`、`Gaze`） |
| 手臂加五指灵巧手，每步给手腕位姿增量和 20 个手指关节目标 | DexJoCo-X（Franka Panda + Wuji 灵巧手），`hexashell/benches/dexjoco_x/` | 共用 `ArmSession`、`ArmHost` 和 `armserver.serve`；服务端求逆运动学，把手上一点的直线运动、命名手型和逐指弯曲换成官方每步的动作，握住和碰到只从手指关节的本体感知判断，结果类型在 `benches.results`（`DexterousMove`、`DexterousFingers` 等） |
| 评测程序自己推进 | RoboDojo、RoboColiseum | [第 7 节](#7-由评测程序驱动的基准) |
| 真机 | `hexashell/real/urai/`、`hexashell/real/franka/` | [第 11 节](#11-真机) |

`arm.Arm` 假定 MuJoCo 的 `model`、`data` 和 OSC_POSE 增量动作。其他同样基于 MuJoCo 和 OSC 的单臂仿真，只需要写自己的 `control.py`；
控制方式差别较大时，照 RoboTwin 或 RoboColiseum 写自己的 host。

## 5. HexaShell 驱动的仿真基准

这里以 LIBERO-PRO 为模板。

### 5.1 文件和进程

```
hexashell/benches/<基准>/
  __init__.py   只有 docstring，不 import 任何东西
  protocol.py   协议（第 3 节），两侧共用
  sim.py        服务端：建环境、reset(seed)、step、判定、相机、RGB-D、机器人像素
  control.py    服务端：arm.Arm 的子类，把 6 维臂动作和夹爪拼成基准的动作向量
  server.py     服务端：ArmServer 的子类，python -m hexashell.benches.<基准>.server <fd>
  host.py       episode 侧：ArmHost 的子类（公开哪些调用、相机名和别名、夹爪行程、提示语）
  session.py    episode 侧：ArmSession 的子类（open 参数、任务卡、记录名、result.json 字段）
  proxy.py      沙箱里：cell 看到的 robot
prompts/<基准>/01_rules.md … 06_checking.md
skills/<基准>/<任务>.md            学到之后才有
tests/fakes_<基准>.py、tests/test_<基准>.py
scripts/setup/setup_<基准>.sh、smoke_<基准>.py
```

一个回合有两个进程：

- episode 一侧运行在 HexaShell 的环境里（Python 3.11）。
- 服务端运行在基准自己的环境里（`$HEXASHELL_<基准>_PYTHON`），因为仿真所需的 Python 和 numpy 版本常与主环境冲突（RoboCasa 用的是
  Python 3.12 和 numpy 2.2）。

两侧通过 socketpair 通信（服务端从 argv 拿到文件描述符），或者连接常驻服务的 unix socket。`benches/wire.py` 的格式是 8 字节长度、
JSON 头、原始数组字节，不使用 pickle。服务端的输出写进本回合记录里的 `server.log`；仿真崩溃时，这个回合的记录仍然保留。

由此有三条约束：

- `__init__.py` 不 import。服务端 import `hexashell.benches.<基准>.server` 时，会先执行 `hexashell`、`hexashell.benches` 和本包的
  `__init__.py`；其中只要 import 了 episode 一侧的模块，服务端就会启动失败。
- 服务端代码（protocol、sim、control、server）只 import 标准库、numpy、基准自己的包，以及 `hexashell.benches` 下的
  `wire`、`armserver`、`arm`、`egl`、`gpucheck`。
- 服务端代码要能在各基准环境的 Python 上运行（最低 3.8）：文件开头写 `from __future__ import annotations`，不用 3.9 以后才有的写法，
  例如 `str.removesuffix`。

### 5.2 服务端

**sim.py** 封装基准的环境。`arm.Arm` 用到其中的：

- MuJoCo 的 `model`、`data`，夹爪 site 的 `site_id`，臂基座的 `link0_id`；
- `link0()`、`site_pose()`、`tcp()`（4×4，episode 坐标系）、`finger_width()`、`finger_speed()`；
- `steps`、`over`（属性）和 `step(action)`（一个控制步，之后判一次成功）。

`ArmServer` 和 episode 一侧用到：

| 方法 | 返回 |
|---|---|
| `reset(seed)` | 本回合的 facts，至少包含 `instruction`（官方指令）和 `horizon`（步数上限）；其余字段写进记录和任务卡 |
| `status()` | `{"step", "horizon", "success", "over"}`，每次回复都会附上 |
| `proprio()`、`arm_joints()` | 本体感知；起始时的关节角是 `home` 的目标 |
| `camera(cam)` | `{"pose", "focal", "width", "height"}`；pose 在 episode 坐标系中，相机沿 −z 看，+y 指向图像上方 |
| `observe(cams, robot_mask)` | `{"step", "images", "depth", "robot", "cameras", "proprio"}`：RGB、深度（沿光轴，单位米）、机器人自身的像素、相机模型 |
| `close()` | 释放环境 |

需要注意的几处：

- 坐标系：给模型的坐标系只有一个，写在 proxy 的 docstring 和 `02_world.md` 里（LIBERO 用臂基座系，RoboCasa 用起始时底盘在地面的投影，
  RoboTwin 用世界系）。相机位姿也要换算到这个坐标系。
- 图像：MuJoCo 渲染的图像上下颠倒，要翻转回来；深度用 robosuite 的 `get_real_depth_map` 换算成米。采用 OpenCV 约定的相机要换成沿 −z 看的
  约定（`vision/cameras.py` 的 `from_cv`）。
- 每张图都用 `gpucheck.check_time` 计时：每个回合第一张超过 60 s、之后的超过 10 s 就报错，并写明是哪块 GPU。深度用 `gpucheck.check_depth`
  检查 NaN。报过 Xid 109 的 GPU 会给出坏图但不会崩溃，只有这两项检查能发现。
- 同一个 seed 必须得到同一个起始状态。常驻服务复用环境时，起始图片要与冷启动逐像素一致。

**control.py** 是 `arm.Arm` 的子类。构造时传入这台臂的 OSC 控制器和夹爪；`action(arm6, grip, **extra)` 把 6 维臂增量和夹爪拼成基准的动作向量。
同时检查控制周期和动作维度，不一致就报错（LIBERO 检查 `DT == STEP_S` 和 `action_dim == 7`）。`FLOOR_Z` 是抓取点允许到达的最低高度。

**server.py** 照 LIBERO 写：

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

- `ArmServer` 已经实现了 `observe`、`state`、`reach`、`goto`、`arc`、`push_arc`、`home`、`gripper`、`wait`、`close`。新的请求写成
  `op_<名字>(self, **args) -> dict`，例如 RoboCasa 的 `op_drive`。可以调整的类属性有 `floor_z`、`reach_where` 和 `elbow_room`
  （位姿离手臂伸直至少要留的角度，默认 35°，LIBERO 用 30°）。
- 不予执行的请求抛 `armserver.Refused(原因)`：此时什么都没有动，模型收到一个带原因的 ValueError。其他异常都按仿真崩溃处理：这个回合记为
  simulator failed，`bench_pool.py` 会换一个新服务重跑。
- `prepare()` 在 import 仿真之前运行一次（常驻服务也只运行一次）。MuJoCo 的基准直接使用 `armserver.prepare`，它按 `$HEXA_EGL_DEVICE`
  选择渲染用的 GPU（`egl.pin`）。SAPIEN 之类的需要自己写，例如 RoboTwin 在 import SAPIEN 之前设置 `CUDA_DEVICE_ORDER=PCI_BUS_ID` 和
  `CUDA_VISIBLE_DEVICES`。
- 常驻服务（`hexashell.benches.daemon`）用同一个 `Server()` 连续服务多个回合，并把 `server.warm` 设为 True；这时 `op_close` 保留环境，
  下一个回合如果是同一任务，就不必重建。

### 5.3 episode 一侧

**session.py**：

```python
class NewBenchSession(ArmSession):
    bench = "newbench"                                   # result.json 的 bench，也是 prompts/ 和 skills/ 下的目录名
    proxy = "hexashell.benches.newbench.proxy"
    server_module = "hexashell.benches.newbench.server"  # 必须以 hexashell.benches. 开头
    host_class = NewBenchHost

    def __init__(self, info, *, python, env_file=None, egl_device=None, size=P.IMAGE_SIZE):
        super().__init__(info, python=python, env_file=env_file, egl_device=egl_device, size=size)
        self.task = str(info["task_name"])
        self.seed = int(info["eval_seed"])

    def open_args(self):        # 服务端 open() 的参数（size 由 ArmSession 补上）
        return {"task": self.task, "seed": self.seed}

    def record_name(self):      # 记录目录名中时间戳之前的部分
        return f"{self.task}_s{self.seed}"

    def labels(self):           # --set script=... 的路径里可用 {task}、{seed}
        return {"task": self.task, "seed": self.seed}

    def card(self, settings):   # 任务卡的文字，以及记进 result.json 的 used
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

    def result_fields(self, verdict):  # result.json 开头的几项
        return {"task": self.task, "seed": self.seed, "instruction": self.instruction}
```

`ArmSession` 已经完成的工作：

- 启动服务（或连接常驻服务）并发送 `open`。
- 建立感知：SAM3 + 深度的 `DepthScene`。SAM3 不可用时，在 `errors` 里记一条 "perception unavailable"，这个回合照常进行。
- 建立 host、取起始图片、转发 robot 调用。
- `over`、`step`、`step_limit`、`verdict` 都取自服务端每次回复所附的 status。
- `states.jsonl` 每个控制步记一行。

可选的钩子：

| 钩子 | 用途 | 例子 |
|---|---|---|
| `prompt_variant()` | 某种设置下替换部分提示词 | RoboTwin 和 LIBERO-Pro 2.0 不用深度时返回 `"rgb"` |
| `cell_note()` | 在 cell 之后告诉模型基准一侧的变化 | LIBERO-Pro 2.0：指令在回合中改变 |
| `server_timeout_s` | 第一个请求要 import、建场景、渲染第一张图，耗时较长 | 默认 900 s，RoboTwin 和 SIMPLE 为 1800 s |
| `row_extra(rest)` | 把服务端在每步记录后附加的列转成 `states.jsonl` 的字段 | RoboCasa 的底盘 |

任务卡写这些内容：官方指令（注明以它为准）、任务名（基准的规则不允许时不写）、步数预算（步数和对应的仿真秒数）、手的起始位姿、坐标系、
起始图片分别来自哪个相机。不写真值。Episode 会在任务卡最后加一句 "You have up to N replies and M minutes for this episode;"，后接
`timing_note()`；仿真的默认值是 "the simulation stands still while you think."。

**host.py** 设置 `ArmHost` 的类属性：

- `public`：公开的调用名，默认 `ARM_CALLS`；
- `cams`、`aliases`：相机名和别名；
- `finger_stroke_m`：夹爪全开时两指的间距；
- `step_s`：控制周期；
- 够不着、卡住、伸直时给模型的建议：`unreachable_note`、`stalled_advice`、`stretched_advice`、`jammed_advice`。

调用 `robot.<名字>` 时，`ArmHost.dispatch` 先检查名字是否在 `public` 里，再调用 `self._<名字>(...)`。需要发给服务端的请求用
`self._call(op, **args)`。

新增一个 robot 调用要改四处（RoboCasa 的 `drive` 就是这样加的）：

1. proxy：加方法，写 docstring；
2. host：把名字加进 `public`，实现 `_<名字>`；
3. 服务端：需要时加 `op_<名字>`；
4. 提示词：在 `03_api.md` 里写明用法，并补上测试。

**proxy.py**：单臂的基准可以照抄 LIBERO 的（没有 arm 参数），也可以继承已有的类、只改 docstring（LIBERO-Pro 2.0 继承 LIBERO 的；
RoboTwin 继承 RoboDojo 的 `hexashell.robot.proxy.Robot`）。模块 docstring 写单位、坐标系和像素约定；每个方法写清参数、返回值，以及何时抛出
何种异常。模型对这些调用的了解，全部来自 docstring 和提示词。

### 5.4 两臂：照 RoboTwin

RoboTwin 的手臂与 RoboDojo 同属 X5 系，所以直接使用 RoboDojo 的运动规划，服务端只执行关节目标：

- `rig.py` 用 `motion.rig.Rig` 描述机器人：两臂的基座、URDF、末端、关节范围、抓取点和指尖的可达距离、桌面、夹爪模型（`GripperModel`）、
  控制周期。
- `host.py` 的 `TwinHost` 继承 `robot.host.RobotHost`，并传入 `rig`。它重写 `act(action)`：把两臂的关节和夹爪拼成 14 个数，调用
  `session.call("act", action=q)`，返回新的 `Snapshot`。它还重写了 `over()`、`dispatch()`（把 `SimRefused`、`SimError` 换成模型看到的错误），
  以及取图的 `views()`、`pictures_now()`、`cameras_now()`。
- `session.py` 的 `RoboTwinSession` 继承 `BenchSession`（不是 `ArmSession`），自己实现 `open`：启动服务、发送 `open`、建立感知、建立 `TwinHost`。
- `server.py` 只处理 `ping`、`open`、`close`、`act`、`state`、`observe`，另有 `status()`。它不继承 `ArmServer`，但收发用的是 `armserver.serve`。
- 不用深度时，感知改用 RoboDojo 的 `Scene`（`robotwin/scene.py` 的 `TwinScene`），`prompt_variant()` 返回 `"rgb"`，
  `prompts/robotwin/` 下的 `*.rgb.md` 替换对应的段落。

## 6. 注册、设置和批量

| 位置 | 加什么 | 何时需要 |
|---|---|---|
| `hexashell/benches/run.py` | 在 `BENCHES` 加上名字；`bench_defaults()` 先用 protocol 检查任务名，再返回 `{"max_turns": P.REPLIES, "budget_s": P.EPISODE_S, "ladder": False}`；`make_session()` 建立 Session | 都需要 |
| `config/env.example.sh` | `HEXASHELL_<基准>_PYTHON`、`HEXASHELL_<基准>_ENV` 两行；每台机器自己的 `config/env.sh`（不进 git）照此填写 | 都需要 |
| `scripts/bench_batch.sh` 和 `scripts/bench_pool.py` 里的 `record_name` | 与 Session 的 `record_name()` 相同的规则 | 记录名不是 `<任务>_s<seed>` 时（LIBERO-Pro 2.0 用哈希） |
| `scripts/bench_pool.py` 的 `CHECKERS` | 基准名 | 该基准的环境能运行 MuJoCo 的 GPU 检查时 |
| `scripts/bench_score.py` | 按官方口径打分的表 | 官方分数不是简单的成功率时 |
| `NOTICE` | 基准及其许可证；借用的代码和事实 | 都需要 |
| `README.md` | 一小节：环境、运行方法、结果 | 都需要 |

两个批量脚本按记录名查找结果，以此决定是否重跑，所以这三处的记录名规则必须一致。

Agent 设置的覆盖顺序：基准自己的默认值（`bench_defaults`）→ `config/benches.yml` → `HEXASHELL_<设置名>` 环境变量 → `--set 名字=值`。
与机器有关的设置（Python、env 文件、GPU）不在 benches.yml 里，见 `run.py` 的命令行参数。

运行（先 `source config/env.sh`）：

```bash
# 一个回合：自行启动服务，记录写在 --out 下
python -m hexashell.benches.run --bench newbench --task <任务> --seed 0 --egl 0 --out results/newbench_dev

# 用 cell 文件代替模型（cell 之间用 "# %%" 分隔），不调用模型就能把每个工具检查一遍
python -m hexashell.benches.run --bench newbench --task <任务> --seed 0 --set script=cells.py

# 批量：logs/NAME.todo 每行 "BENCH TASK SEED"；使用常驻服务，每个槽位一块 GPU
python3 scripts/bench_pool.py NAME 0,0,1,1
```

启动慢的基准用 `bench_pool.py`：服务启动一次后连续接回合，空闲槽位优先接它上一个回合的同一任务。`bench_batch.sh` 每个回合冷启动，适合启动快的基准。
批量从某个固定提交的单独检出运行，这样改代码不会影响正在跑的批量。

## 7. 由评测程序驱动的基准

有些基准只能通过它自己的评测程序运行（官方客户端、它自己的计时）。这时 HexaShell 作为策略服务，Session 不启动仿真，而是等待评测程序送来的观测：

- RoboDojo：XPolicyLab 每步向策略索要动作。模型思考时，`hexashell/xpl/bridge.py` 的 `Bridge` 回复 WAIT（`[{"hexashell": "wait"}]`），
  仿真因此停住。`hexashell/xpl/session.py` 的 `RoboDojoSession` 把它封装成 Session。
- RoboColiseum 第一赛季：`hexashell/benches/robocoliseum1/run.py` 启动一个 WebSocket 服务和 Genie Sim 的仿真进程（genie_sim 的 `app.py`，
  见 `sim.py`）。仿真进程连接这个服务（`link.py`），一次运行中的各回合在同一个连接上依次进行。每来一个回合，服务就在一个线程里调用 `play(link)`，运行
  `Episode(ColiseumSession(link, info), settings, out)`。`conclude()` 让机器人保持静止，等待评测程序给出判定（`VERDICT_CAP_S`）。
  评测程序不提供步数，所以 `step` 和 `step_limit` 返回 None。
- RoboColiseum 第二赛季（`hexashell/benches/robocoliseum/`）的做法相同，每块 GPU 一个仿真容器。两个赛季的 `tunnel.py` 都用于在挑战平台上
  参赛：它连接平台的网关，代替本地仿真，Session 不变。

需要守住的几点：

- 模型思考时，仿真不能向前推进。做不到时，像真机一样把 `deadline_budgets` 设为 1，并在 `timing_note()` 里如实告诉模型。评测程序限制每次回复的
  时间时，要在时限前先回一个保持不动的动作。
- 判定只采用评测程序给出的结果。
- 只使用自己的 API key。需要评判模型时，用自己的 key 调用（RoboColiseum 本地运行时，`judge.py` 代替官方评判）。

## 8. 提示词和技能

### 8.1 提示词

- 系统提示词放在 `prompts/<基准>/`：`01_rules.md`、`02_world.md`、`03_api.md`、`04_seeing.md`、`05_acting.md`、`06_checking.md`，
  按文件名排序后拼接（`runtime/card.py` 的 `system_prompt`；文件名主干含 "." 的是变体）。先复制最接近的基准的那一份，再修改不同之处：
  坐标系、相机、调用、单位、控制周期。各段以 `# 1.` 至 `# 6.` 开头，测试会检查。
- 变体：`prompt_variant()` 返回某个变体名时，`<段>.<变体>.md` 替换 `<段>.md`，例如 RoboTwin 的 `02_world.rgb.md`。
- 提示词只写通用的工具规则，不写某个任务的做法，也不写场景坐标。

### 8.2 技能

技能是 `skills/<基准>/<任务>.md`，通常约一页。`skills: true` 时，Session 的 `card()` 把它放在任务卡末尾的 "## Notes for this task" 下。
目前使用的小节：

| 小节 | 内容 |
|---|---|
| 判定 | 从以往的回合看出怎样算完成；不写判定程序内部的阈值 |
| 流程 | 有效的步骤，按顺序写，带上要用的调用 |
| 验证过的参数 | 以往的回合里量出来的数：抓取高度、偏移、开口 |
| 常见失败和对策 | 出过什么错，应该怎么做 |
| 典型步数 | 每个阶段用了多少步，供安排预算 |

规则：

- 在学习种子上学习，在验证种子上验证，不看测试种子。
- 只写智能体自己能观察到的东西：不写从仿真里读出的位置、内部名字和判定程序的阈值。
- 不写随布局变化的内容（绝对坐标、颜色、数量）；位置相对物体来写。
- 每条结论都标出依据（多少个回合，学习还是验证），每个技能都由人审阅。
- 在 `skills/README.md` 加一段说明。

基准的规则不允许任务专属输入时（例如 RoboDojo 的 agent track），保持 `skills: false`。

### 8.3 分轮学习

1. **不带技能的一轮**：在学习种子上每个任务跑几个回合，阅读失败回合的记录（`prompt.md`、`cells.jsonl`、`pictures/`、`result.json`）。
2. **先改通用的问题**：工具报告有误、docstring 或报错不清楚、缺少某项检查、提示词少一条规则。这类修改对所有任务都有用。改完先在学习种子上验证。
3. **起草技能**：由智能体阅读一组相近任务的记录，包括每个 cell、输出和图像，把剩下的、只属于某个任务的知识写成技能；每份草稿都由人审阅。
4. **验证**：在验证种子上，同一批里分别跑带技能和不带技能的回合，使两者面对同样的机器和同样的模型网关，再按相同的回合成对比较。没有帮助的技能要修改，
   改完再在验证种子上检查。
5. **固定并测试**：固定提交，测试种子只跑一次，分别不带和带技能，使用官方步数上限。报告结果时写明设置。

同一组比较里各次运行的设置要一致（模型、推理强度、图像窗口、上限），并记下网关请求失败的次数：网关过载会让本来能成功的回合失败。

## 9. 环境、GPU 和数据

- 每个基准有自己的 Python 环境。在 `scripts/setup/` 放安装脚本 `setup_<基准>.sh`、锁定的依赖清单和冒烟脚本 `smoke_<基准>.py`。
- `$HEXASHELL_<基准>_ENV` 是服务端所需的环境变量文件。它只读取 `export 名字=值` 形式的行（`benches/session.py` 的 `read_env_file`），
  值中的 `$变量` 按当前环境展开，不执行其他 shell 语句。
- 服务端的环境由 `server_env()` 生成：先去掉 `PYTHON*`、`VIRTUAL_ENV*`、`CONDA*` 以及名字中含 `KEY`、`TOKEN`、`SECRET` 的变量，
  再加入 env 文件里的变量、`PYTHONPATH`（仓库根目录）、`PYTHONUNBUFFERED` 和 `HEXA_EGL_DEVICE`。API key 到不了服务端，也不要写进 env 文件。
- GPU：`--egl` 和 `$HEXASHELL_EGL_DEVICE` 使用 nvidia-smi 的编号。MuJoCo 由 `egl.pin` 换算成 EGL 设备号（两种编号的顺序可能不同）；
  SAPIEN 使用 `CUDA_VISIBLE_DEVICES`。
- `bench_pool.py` 在一块 GPU 上启动第一个服务之前，以及那里有服务失败之后，会先运行 `python -m hexashell.benches.gpucheck GPU`；
  检查不合格的 GPU 不接新回合。
- 大的资产和数据集链接到机器上已有的副本，不另外复制。

## 10. 测试和验收

测试不需要仿真器：每个基准都有一个假服务端，回答同样的请求。

- `tests/fakes_<基准>.py` 包含两部分：
  - 一个假场景类，回答与 sim 相同的请求。LIBERO 的 `FakeTable` 复用了 `fakes_casa.py` 的渲染、直线运动、夹爪和按颜色分割的假 SAM3。
  - 一个 Session 子类：把 `start_server` 换成 `fakes_casa.FakeLink(场景)`（请求照样经过 wire 的编码和解码），把 `_scene` 换成使用
    `FakeSam3` 的 `DepthScene`。
- `tests/test_<基准>.py`（或加进 `tests/test_benches.py`）至少包括：
  - 协议：任务名、划分、步数上限；
  - host：几个动作的结果；
  - 任务卡和提示词目录：六段齐全，不含其他基准的调用；
  - 一个脚本化的完整回合：用 `fakes.ScriptedGateway` 代替网关，清除 `HEXASHELL_` 开头的环境变量，使用
    `Settings(sam3_url="fake", ladder=False, max_turns=10, budget_s=600, cell_cpu_s=20, cell_idle_s=30)`，跑完后检查 `result.json`、
    `cells.jsonl` 和系统提示词。可以照抄 `tests/test_benches.py` 的 `test_scripted_libero_episode`。
- 运行全部测试：`python -m pytest -q tests/`。

在真实仿真上验收：

1. 冒烟：`smoke_<基准>.py` 能建环境、reset、渲染。
2. 脚本回合：用 `--set script=` 把每个调用都走一遍。`server.log` 里没有异常，`pictures/` 里的图像方向和颜色正确，`to_world` 与 `to_pixel`
   能互相对上。
3. 确定性：同一 seed 启动两次，起始图片相同；常驻服务与冷启动的起始图片相同。
4. 判定：用官方的示范轨迹或官方评测脚本，在若干回合上核对成功判定。
5. 在学习种子上跑几个由模型操作的回合，阅读 `prompt.md`、`cells.jsonl`、`events.jsonl`。
6. 改动过共享模块时，确认其他基准的提示词、任务卡、工具说明和结果文字逐字节不变。

## 11. 真机

### 11.1 与仿真的区别

机器人一侧的服务负责规划和执行（逆运动学、碰撞检查、时间缩放、执行）。HexaShell 只通过 HTTP 与它通信，负责对话、cell、记录、显示图像，
以及像素与世界坐标之间的换算。

| | 仿真 | 真机 |
|---|---|---|
| 判定 | 基准每步判定 | 没有评判：`result.json` 的 `success` 为 null，finish 的报告记为 `agent_report`；finish 多一个状态 `done` |
| 时间 | 模型思考时仿真停住；一个回合最长 2 个 `budget_s`（`deadline_budgets = 2`） | 不停；最长 1 个（`deadline_budgets = 1`），`timing_note()` 如实说明 |
| 步数 | 仿真的控制步 | 服务端实际执行时间除以 0.04 s，向上取整 |
| 感知 | SAM3 + 深度：`find`、`measure`、`things` | 不用 SAM3：用 `look`、`image` 看图，`to_world` 按深度、指定平面或双视角取点 |
| 技能 | 可选，放在任务卡里 | 目前没有（需要时由 Session 的 `card()` 加入） |
| 记录名 | `<任务>_s<seed>-<时间>` | `<bench>-<name>-<时间>` |

### 11.2 已由 URAI 管理的机器人

```bash
python -m hexashell.real.urai --url http://127.0.0.1:7860 \
  --task 'Move the red block to (0.20, -0.02).' --name move_block --results runs
```

[URAI](https://github.com/HexaFuture/URAI) 一侧的启动方法见它的 README，所用版本要实现 11.3 的接口（公开版暂不支持 `/api/harness/...`）。
`--url` 或 `URAI_URL` 必须给出，token 通过 `URAI_TOKEN` 传入。
机型的臂、相机和 TCP 都由服务的 `/api/harness/model` 报告给这一侧。如果相机名、规划用的相机或给模型的说法与 PiPER-X 不同，照
[11.4](#114-hexashell-一侧) 写子类。

### 11.3 新机器人：先写一个 URAI 兼容的服务

HexaShell 一侧的 `UraiClient`（`hexashell/real/urai/client.py`）只用到下面这些接口。新机器人照 FR3 的做法
（`hexashell/real/franka/` 的 `service.py`、`server.py`）实现它们之后，HexaShell 一侧只需写很少的代码。

| 请求 | 请求体 | 回复 |
|---|---|---|
| `GET /api/harness/model` | | `name`、`arm_names`、`table_z`、`cameras`；每臂的 `t_world_base`、`tcp_offset_mm`、`table_z`、`tip_past_tcp_mm`、`max_opening_mm`、`joint_limits_deg` |
| `POST /api/harness/observe` | `{"camera"}` | `id`、`timestamp`、`camera`；`rgb` 和 `depth`（`{"encoding": "npy-base64", "data"}`，深度单位为米，已对齐到 RGB）；`k`、`t_world_camera`、`arms`；`capsules`（每臂连杆的碰撞胶囊 `a`、`b`、`radius_m`） |
| `GET /api/state` | | `arms`（每臂的 `xyz`、`rpy_deg`、`joints_deg`、`gripper_mm`、`feedback_age_s`、`enabled`）、`execution` |
| `PUT /api/draft` | `{"observation_id", "completion": "hold", "arms": {臂: 动作}}` | `{"revision"}` |
| `POST /api/preview?trajectory=full` | `{"expected_revision", "planning_only"}` | `id`、`duration_s`、`pose_tolerance`（`position_mm`、`orientation_deg`）、`arms[臂].xyz` |
| `POST /api/execute` | `{"preview_id"}` | 立即返回；客户端随后轮询 `/api/state`，直到 `execution.state` 不再是 running 或 cancelling |

FR3 的服务另有 `POST /api/cancel`、`POST /api/recover`（供操作员使用，客户端不调用），以及查看画面用的 `GET /api/cameras/<相机>/image.jpg` 和
`GET /api/live.jpg`。

draft 里每臂一个动作（双臂机器人的两臂可以在一个计划里同时动）：

```text
{"primitive": "goto", "xyz": [x, y, z], "rpy_deg": [a, b, c], "motion": "line" 或 "swing", "speed_m_s": 0.12, "opening_mm": 可选}
{"primitive": "gripper", "opening_mm": 开口, "effort": 1000, "duration_s": 最大开口 / 100 + 0.3}
{"primitive": "home", "joints_deg": 本回合开始时的关节角, "speed_m_s": 0.12}
{"primitive": "wait", "duration_s": 步数 × 0.04}
```

- `rpy_deg` 是 xyz 外旋欧拉角，R = Rz(c)·Ry(b)·Rx(a)（`hexashell/real/urai/frames.py`）。
- `motion` 为 `"line"`（直线）或 `"swing"`（关节空间路径）。模型调用 `goto(path="auto")` 时先试直线；直线不行再用关节路径，但只接受偏离直线不超过
  3 cm、且比两端低不超过 1 cm 的路径。

服务要守的规则（FR3 都已实现）：

- 只读模式：不加 `--allow-motion` 启动时，可以取图、读状态、规划（`planning_only`，即模型的 `dry_run`），但不会动。非 planning_only 的
  preview 回 409，execute 和 recover 回 403。
- 计划与画面绑定：draft 的 `observation_id` 必须是规划相机最新一张图的；preview 的 `expected_revision` 必须是最新的 draft；execute 只接受最新的、
  未执行过的、非 planning_only 的计划，而且从规划到执行手臂的移动不超过 0.03 rad。否则回 409，模型需要重新规划。
- 有动作正在执行时，新的 preview 和 execute 回 409。机器人报错时，execute 回 409，等操作员 recover。
- 409 的 `detail` 要写清原因：host 会原样转告模型（NotExecuted）。
- `execution` 的形式是 `{"id", "state", "execution_time_s", "error", "last_error"}`，`state` 取 idle、running、cancelling、completed、
  cancelled、error 之一。抓空时在 `execution.result.outcome` 填 `"empty_grasp"`，并在 `last_error` 写一句说明，模型会收到 Dropped。
- 默认只监听 loopback。监听其他地址时必须设置 token，请求头为 `Authorization: Bearer <token>`，不符时回 401。
- 服务是机器人控制接口的唯一持有者（FR3 的 FCI 只接受一个控制端）。
- 胶囊要与真实手臂贴合：HexaShell 用它投影出机器人在图像中的像素，再与实测深度比较，相差 1 cm 以内的算作机器人。

FR3 服务的三种启动方式：

```bash
python -m hexashell.real.franka.server --sim --port 7861              # 运动学仿真
python -m hexashell.real.franka.server --rig rig.json --port 7861     # 只读：只规划，不动
python -m hexashell.real.franka.server --rig rig.json --allow-motion  # 会动
```

| 文件 | 内容 |
|---|---|
| `service.py` | 与 HTTP 无关的逻辑，上面的规则都在这里 |
| `server.py` | 路由和 token |
| `driver.py` | 连接 franky |
| `arm.py`、`planner.py` | 运动学和规划 |
| `cameras.py` | 连接 RealSense |
| `rig.py` | 读取 `rig.json`：FCI 地址、相机序列号、手眼标定、桌面高度 |
| `simulated.py` | 供测试和 `--sim` 使用的运动学仿真 |

### 11.4 HexaShell 一侧

与 FR3 一样，继承 URAI 的 host 和 session，修改相机、文案和 proxy。下面是 `hexashell/real/franka/session.py` 的主要内容：

```python
CAMERAS = {"external": "external", "wrist": "wrist"}  # HexaShell 的相机名 -> 服务的相机名


class FrankaHost(UraiHost):
    cameras = CAMERAS
    plan_camera = "external"      # 计划与之绑定的相机
    texts = texts                 # 给模型的句子
    gentle_speed_m_s = 0.03       # goto(gentle=True) 的速度
    public = tuple(name for name in PUBLIC if name != "both")  # 单臂

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

另外还需要：

- `proxy.py`：单臂的 `Robot`（FR3 的臂名是 `"arm"`）和它的 docstring，没有 `find`、`measure`、`things`；
- `texts.py`：给模型的句子。与 URAI 相同的直接 import，不同的（`TIMING`、`CONTACT`、`CANCELLED`）改用这台机器人的说法；
- `prompts/<bench>/`：六段提示词；
- `__main__.py`：调用 `main()`。

`env="FRANKA"` 决定了环境变量名 `FRANKA_URL`、`FRANKA_TOKEN`。运行一个回合：

```bash
python -m hexashell.real.franka --url http://127.0.0.1:7861 \
  --task 'Put the red block in the bowl.' --name red_block --results runs
```

其余参数与 URAI 相同：`--script cells.py`、`--config settings.json`、`--step-limit`、`--speed`。

### 11.5 上真机的顺序

1. 单元测试：使用假客户端（`tests/fakes_urai.py`、`tests/test_real_urai.py`），不需要服务。
2. 对着服务的运动学仿真跑完整的回合（`--sim`；`tests/test_real_franka.py`、`tests/test_real_urai_sim.py`）。
3. 真机只读：连接后读状态、取图、规划，确认不影响机器人上的其他客户端。
4. 标定：`rig.json` 里的手眼标定、桌面高度、实测的工具长度；在叠图里检查机器人像素是否与真实手臂贴合。
5. 第一次加 `--allow-motion`：低速、短距离，有人守在急停旁。
6. 之后再跑由模型操作的回合。

## 12. 约定

- API key 不进仓库、不进沙箱、不进服务端；推送前检查一遍索引。
- 真值只在服务端用于判定。`hexashell/` 里不放真值和任务知识，因为沙箱能读取这个目录。
- 开发期间不跑测试种子；技能只来自学习种子和验证种子。
- 共享模块（runtime、kernel、vision、robot）的新增内容一律可选，默认值等于现在的行为。
- 新基准需要的差别写在自己的子类里，不改其他基准的包。
- 借用的代码和事实记进 NOTICE（例如从 URAI 复制的 `capsules.py`，Apache-2.0）。
- 名字用完整的英文单词。
