<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/hexafuture-logo-dark.png">
    <source media="(prefers-color-scheme: light)" srcset="assets/hexafuture-logo-light.png">
    <img src="assets/hexafuture-logo-light.png" alt="HexaFuture" width="420">
  </picture>
</p>

<h1 align="center">HexaShell</h1>

<p align="center">
  <strong>A Thin Harness for Exploring the Limits of VLM Agents as Robot Coding Agents</strong>
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2610.23479">📄 Paper PDF</a> ·
  <a href="https://hexafuture.ai">🌐 Homepage</a> ·
  <a href="https://github.com/HexaFuture/HexaShell">💻 Project</a>
</p>

---

## 📰 News

- **2026-10-19** — HexaShell is released and reaches the top of the RoboDojo leaderboard: success rate 46.9%,
  progress score 53.2 in RoboDojo's official evaluation.

## 🧩 HexaShell

HexaShell asks how far a coding agent can go when it controls a robot instead of a computer. The harness is thin.
It only turns the agent's commands into robot motion. The agent gets one command line, `robo`, three RGB camera
images with their calibration, and a manual of one page. Its prompt is the task instruction, the task recipe and
short notes on the task's tools. It drives the dual-arm ARX X5 of RoboDojo in Isaac Sim. The eight Open tasks
share one setup instead: one manual and one set of tools for all eight, with no tools or notes for any single task.

What the agent learns is kept as **tools**: small Python programs that add sub-commands to `robo` and may use only
what the robot observes. After a failed attempt, a second agent of the same model reads the episode, finds the
cause, and writes or fixes a tool. The same layout is then tried again. Both agents are Codex with GPT-6-Astra.
This repository releases the frozen tools, the shared setup of the Open tasks and the interface they are written
against.

<p align="center">
  <img src="assets/paper-figure-1.png" alt="HexaShell turns robot failures into reusable tools" width="100%">
</p>

## 💎 Highlights

- **State of the art on RoboDojo.** In RoboDojo's official evaluation on a hidden seed, 50 episodes for each of the
  42 tasks, HexaShell reaches a success rate of 46.9% and a progress score of 53.2, the top of the leaderboard.
- **One interface for all 42 tasks.** The same command line, camera images and one-page manual. Nothing in the
  harness is specific to a task.
- **Failures become tools.** Each failed attempt can lead to a new or fixed tool, with its own unit tests. A
  repair round takes a median of 6.3 minutes.
- **Triangulation instead of depth.** A point seen by two calibrated cameras lies where the two pixel rays meet.
  A point on a measured plane, such as the table or the top of a block, lies where its ray meets that plane.
  When the head camera is not enough, the robot moves a wrist camera to get a second view.
- **Robust to clutter.** In the official evaluation, with 13 to 18 extra objects on the table, HexaShell keeps 82% of
  its success (38.0% standard, 31.0% cluttered), more than any learned policy on the leaderboard, because a tool
  measures the object the agent points at and nothing else.
- **Clear limits.** Success follows the contact a task needs: memory tasks reach 81.0% and precision tasks 32.0%.

## 📊 Evaluation

### RoboDojo's official evaluation

RoboDojo ran the frozen system on its own machines, on a hidden seed: 50 episodes per task, scored by RoboDojo's own
checkers (Generalization tasks: 25 standard and 25 cluttered). The 34 tasks outside Open use their own tools; the
eight Open tasks use the shared setup. The overall score uses the leaderboard's formula: the mean of the five task
types, Generalization being the mean of its standard and cluttered halves.

| Task type | Success rate | Progress score |
| --- | ---: | ---: |
| Generalization | 34.5% | 39.2 |
| Precision | 32.0% | 41.3 |
| Long-Horizon | 31.5% | 42.9 |
| Memory | 81.0% | 84.5 |
| Open | 55.3% | 58.0 |
| **Overall** | **46.9%** | **53.2** |

<p align="center">
  <img src="assets/paper-figure-2.png" alt="The RoboDojo leaderboard" width="100%">
</p>

<p align="center">
  <img src="assets/paper-figure-3.png" alt="RoboDojo comparison by task type" width="100%">
</p>

`results/official_robodojo.md` lists every task, and `results/official_robodojo.json` holds the same numbers.

### Our retests

Frozen tools, one episode on each layout, with RoboDojo's own checkers, on our machines.

- **New layouts:** seed-0 layouts 10–19 of every task (Generalization tasks: five standard and five cluttered
  layouts, 5–9). The prompt carries the recipe and the notes. `swap_blocks` runs with RoboDojo's fix of its button
  reset. The Open tasks ran with their earlier task tools.
- **Development layouts:** the ten seed-0 layouts the tools were built on (Generalization tasks: layouts 0–4 of
  each kind), scored before the recipes, the notes and the new rules were added.

| Task type | New layouts: success rate | New layouts: progress score | Development layouts: success rate | Development layouts: progress score |
| --- | ---: | ---: | ---: | ---: |
| Generalization | 28.3% | 35.4 | 34.2% | 37.7 |
| Precision | 31.3% | 41.4 | 21.3% | 28.4 |
| Long-Horizon | 23.8% | 40.6 | 18.8% | 32.2 |
| Memory | 65.0% | 69.2 | 73.3% | 73.3 |
| Open | 50.0% | 53.6 | 43.8% | 46.6 |
| **Overall** | **39.7%** | **48.0** | **38.3%** | **43.6** |

`results/new_layout_tests.md` and `results/dev_layout_retests.md` list every task, and the `.json` files next to them
hold the same numbers.

## 🔧 What is here

```
tasks/<task>/enabled_tools.txt   the tools loaded for the task (and its _random variant), for the 34 tasks outside Open
tasks/<task>/tools/<name>/       tool.py (the program), interface.md (the text the agent reads), helper modules
tasks/<task>/notes.md            notes on the task's tools from earlier runs, added to the agent's prompt
tools/table_plane/, tools/pick/  shared tools: table height from two views, and a grasp from above
tools/scene/                     Open setup: separates objects from the table and the arms, locates them in two views
library/                         Open setup: eight skills taken from the tools of the 34 other tasks
profiles/open/                   Open setup: the eight Open tasks and the tools they load
interface/INTERFACE.md           the base manual the agent sees; tool interface files are appended to it
interface/INTERFACE_open.md      the manual of the Open setup (adds goto, reach and a Python session)
interface/EpisodeAPI.md          the contract a tool is written against
recipes/                         the task recipes added to the agent's prompt (from RoboProbe, Apache-2.0)
hexashell/                       the constants and helpers that tools import from the harness
results/                         the evaluations above, per task
paper/HexaShell.pdf              the paper; assets/ holds its first three figures
```

The 34 tasks outside Open hold 196 tools. The 137 enabled ones add 270 commands: 141 measure without moving the
robot, and 129 move it. The Open setup loads `table_plane`, `scene` and the eight skills of `library/`, 16 commands
in all (9 measure, 7 move), the same for every Open task. What each tool computes is described in its `interface.md`.

The harness itself (server, executors, agent container and evolution loop) is not included.

## 🚀 Using the tools

A tool runs inside the HexaShell server, which calls `run(api, command, args)` with the `api` object documented in
`interface/EpisodeAPI.md`. To run the tools you need a harness that provides that object over RoboDojo (ours, or
your own built from the contract) together with RoboDojo's Isaac Sim environment. Tools depend on numpy, OpenCV,
SciPy, Pillow and transforms3d, and import constants and helpers from `hexashell/`.

The agent's prompt has three parts:

```
<official instruction>

TASK RECIPE:
<recipes/<task>.md>

TASK NOTES (from earlier runs of the same tools in other scenes):
<tasks/<task>/notes.md>
```

For the eight Open tasks the prompt holds the instruction and the recipe, without notes, and the manual is
`interface/INTERFACE_open.md`. `_random` variants use the recipe and the notes of the base task. The recipe is the RoboDojo wiki text of the task.
A model wrote the notes from the development-layout runs of the frozen tools. They cover time, failed commands and
what worked after them, checks before stopping, and known problems.

The tools are frozen at the versions scored above, and `enabled_tools.txt` lists the tools each task was scored
with; `profiles/open/enabled_tools.txt` lists those of the Open tasks. The optimizer's development notes and the tool tests are not included.

## 📚 Citation

```bibtex
@misc{gao2026hexashellharnessexploringlimits,
      title={HexaShell: A Thin Harness for Exploring the Limits of VLM Agents as Robot Coding Agents}, 
      author={Hongcheng Gao and Alex Zhou and Jay Zhu and Zelin Zheng and Shijia Ge and Xiping Li and Di Wu and Yazhe Wang and Jianshu Zeng and Xuan Shangguan and Yongli Wang and Yexing Wan and Lingyu He and Zhiqi Jia and Sihang Wu and Xiao He},
      year={2026},
      eprint={2610.23479},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2610.23479}, 
}
```

## License

Apache License 2.0; see `LICENSE`.
