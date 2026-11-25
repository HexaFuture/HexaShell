# Third-party versions

The versions our tests ran with. Nothing listed here is part of this repository except the two submodules (pinned
commits, not copies) and the changes to RoboCasa described below.

## RoboDojo

| Component | Version |
|---|---|
| RoboDojo | [RoboDojo-Benchmark/RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo) `fbfbbcb3801397fa00dcffaef3ca64d97c3fe76f` (the runs under the agent track's rules; from this commit on an observation carries the official task description, `detailed_instruction`); `09bd5f5e762b949f5858ece31347ce62057b3a5e` (the earlier runs with and without skills) |
| XPolicyLab | [XPolicyLab/XPolicyLab](https://github.com/XPolicyLab/XPolicyLab) `53c5da5a01bc720d1cf0e6ab6ed212e8cef0eda2` (cloned by RoboDojo's install script); `30b2d4a2e7c421c8f0d853755ec1f8393ec85ec4` for the agent-track runs |
| Simulator | Isaac Sim 5.1, RoboDojo's own environment; robot config `arx_x5` |
| Policy environment | CPython 3.11, [requirements.txt](requirements.txt) |

`scripts/patches/robodojo-layout-ids.patch` lets `scripts/run_one.sh` and `scripts/batch.sh` choose layouts; RoboDojo's
own evaluation runs without it. It applies to `09bd5f5`, `fbfbbcb` and the later
`c3af1ac6ad7302c0c09d17f4b050fd3a5c68c921`.

## RoboCasa365

| Component | Version |
|---|---|
| RoboCasa | submodule `third_party/robocasa`: [robocasa/robocasa](https://github.com/robocasa/robocasa) `b4684e6ee37d377cc392e98302a6b916d588b415` (1.0.1), with the changes below |
| robosuite | submodule `third_party/robosuite`: [ARISE-Initiative/robosuite](https://github.com/ARISE-Initiative/robosuite) `5ce6643f3092639d08f7b0f90ed1c6a84f50552c` (1.5.2), unchanged |
| Simulator environment | CPython 3.12, MuJoCo 3.3.1, numpy 2.2.5; all packages in [scripts/setup/robocasa-requirements.txt](scripts/setup/robocasa-requirements.txt) |
| Assets | RoboCasa's kitchen assets, from its own download script |

Changes to RoboCasa, applied by `python3 scripts/patch_third_party.py` (`--check` reports whether they are in place;
`setup_robocasa.sh` runs it):

- `scripts/patches/robocasa.patch`:
  - `robocasa/__init__.py`: the exact-version asserts on MuJoCo and numpy are turned off (`[capx-relax-assert]`);
    the environment has those versions anyway.
  - `robocasa/models/__init__.py`, `environments/kitchen/kitchen.py`, `utils/texture_swap.py`: the asset folder can be
    elsewhere (`$HEXA_ROBOCASA_ASSETS`, default the package's own `models/assets`), and the kitchens' asset paths are
    resolved against it.
  - `models/objects/objects.py`, `utils/model_zoo/mjcf_obj.py`: the object XML derived for each episode is written to
    a temporary file outside the asset folder (which can then be read-only) and removed after loading.
- `[roboshell-determinism]`, in `robocasa/models/fixtures/counter.py`: `list(set(valid_geoms))` becomes
  `list(dict.fromkeys(valid_geoms))`. A set of XML elements iterates in memory-address order, so the sampled counter
  regions, and with them the scene, could differ between two resets with the same seed.

None of the changes touches tasks, success checks, controllers or physics.

## LIBERO-PRO

The simulator stack RPent uses for LIBERO-PRO (RPent `886b3b2`), built by `scripts/setup/setup_libero.sh`:

| Component | Version |
|---|---|
| LIBERO-PRO | [RLinf/LIBERO-PRO](https://github.com/RLinf/LIBERO-PRO) `d1e11fb181b8544487d27742c0caa3a4d46452ad` (branch `rpent`; package rpent-liberopro 0.2.0) |
| LIBERO | [RLinf/LIBERO](https://github.com/RLinf/LIBERO) `a8323074d93a09e32bd898630a70531b1f51bc77` (branch `rpent`; package rpent-libero 0.2.0) |
| Assets | Hugging Face dataset RLinf/LIBERO-PRO-assets `e4b10c9adc5735c9c8776917527178e693ef10b3` (865 files, checksums in `scripts/setup/libero-pro-assets.md5`) |
| Simulator environment | CPython 3.11, robosuite 1.5.2, MuJoCo 3.3.0, numpy 1.26.4, gym 0.26.2, torch 2.7.1+cpu; all packages in `scripts/setup/bench-libero.lock.txt` |

LIBERO-PRO itself: [Zxy-MLlab/LIBERO-PRO](https://github.com/Zxy-MLlab/LIBERO-PRO).

## LIBERO-Pro 2.0

| Component | Version |
|---|---|
| Evaluation bundle | Hugging Face dataset [zhouxueyang/LIBERO-Pro2.0-Eval](https://huggingface.co/datasets/zhouxueyang/LIBERO-Pro2.0-Eval), bundle commit `4046dc477dc392a11325815ae91fab6cef6d9af8` (packages libero 0.1.0, libero-remote 1.0.0) |
| Simulator environment | the bundle's own: CPython 3.8.10, robosuite 1.4.0, MuJoCo 3.2.3, numpy 1.22.4 |

LIBERO-Pro 2.0 itself: [LIBERO-Pro-Team/LIBERO-Pro2.0](https://github.com/LIBERO-Pro-Team/LIBERO-Pro2.0).

## RoboTwin 2.0

The simulator stack RPent uses for RoboTwin, built by `scripts/setup/setup_robotwin.sh`:

| Component | Version |
|---|---|
| Runtime | rlinf-robotwin-runtime 0.1.1 (PyPI): RoboTwin 2.0's tasks and controller, SAPIEN 3.0.0b1, mplib 0.2.1 |
| Planner | cuRobo v0.7.8 (`d64c4b005459db10c5dd867d8b30a87d5bda9bdb`), built locally |
| Assets | Hugging Face TianxingChen/RoboTwin2.0 `c15cc97be71e35244b6605d2d84c187f8565cc4d` (the revision the runtime pins) |
| Simulator environment | CPython 3.11, torch 2.7.1+cu128, numpy 1.26.4, warp-lang 1.11.1; all packages in `scripts/setup/robotwin-lock.txt` |
| Setting | `demo_randomized`, aloha-agilex, the official step limits |

RoboTwin 2.0 itself: [RoboTwin-Platform/RoboTwin](https://github.com/RoboTwin-Platform/RoboTwin).

## RoboColiseum

Online runs need no simulator of their own. Local runs:

| Component | Season I | Season II |
|---|---|---|
| Genie Sim | [AgibotTech/genie_sim](https://github.com/AgibotTech/genie_sim) `v3.5.0` | `847bb9ebc002f7b0d2f21364970b3290c02e5f44` (3.6.0), submodule genie_sim_robot_model `fb6de335f6a293605204a17608cf0b22b1fef6de`, with `scripts/patches/geniesim36-initialization-timeout.patch` |
| Assets | GenieSimAssets of the four boards | ModelScope agibot_world/GenieSimAssets `dfa5864`, with 21 behaviour scripts and `mujoco/rubik_cube/README.md` of `ad5ca655` (see [scripts/setup/README.md](scripts/setup/README.md)) |
| Simulator environment | Isaac Sim 5.1, CPython 3.11, numpy 1.26.4, scipy 1.13.1 (native) | its docker image: Isaac Sim 6.2, Ubuntu 24.04, ROS 2 Jazzy |

## SIMPLE

| Component | Version |
|---|---|
| SIMPLE | [physical-superintelligence-lab/SIMPLE](https://github.com/physical-superintelligence-lab/SIMPLE) `b75c6786f87b4a7f43b52b0ac5b83a1d9ea62e11` |
| Simulator environment | its own: CPython 3.10, MuJoCo 3.3.6, Isaac Sim 4.5 |
| Official episodes | Hugging Face dataset USC-PSI-Lab/psi-data (`simple-eval/<TaskEnv>.zip`) |

## DexJoCo-X

| Component | Version |
|---|---|
| DexJoCo-X | [dexjoco-x/DexJoCo-X-public](https://github.com/dexjoco-x/DexJoCo-X-public) `39f7c25` |
| Simulator environment | the release's reference environment, built by `scripts/setup/setup_dexjoco_x.sh`: CPython 3.11, MuJoCo 3.4.0 (conda-forge), numpy 1.26.4 |

## Real robots

| Component | Version |
|---|---|
| PiPER-X arms | [HexaFuture/URAI](https://github.com/HexaFuture/URAI), over its HTTP interface |
| Franka FR3 | franky and the RealSense SDK, as in [requirements-franka.txt](requirements-franka.txt) |

## SAM3

| Component | Version |
|---|---|
| sam3 | [facebookresearch/sam3](https://github.com/facebookresearch/sam3), package version 0.1.0 |
| Checkpoint | `sam3.pt` from [facebook/sam3](https://huggingface.co/facebook/sam3) |
| Environment | torch 2.7.0, torchvision 0.22.0 (CUDA), timm |
