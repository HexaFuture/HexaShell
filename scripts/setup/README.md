# Simulator environments for the benchmarks

RoboCasa365, LIBERO-PRO, LIBERO-Pro 2.0, RoboTwin 2.0, SIMPLE and DexJoCo-X each run in their own Python environment,
separate from the policy's. The scripts here build four of them; LIBERO-Pro 2.0 and SIMPLE have installers of their
own (below), and so has Genie Sim, which RoboColiseum's local runs need. By default everything goes inside the
checkout: the environments in `.venvs/`, the data (sources, assets, run-time environment files, logs) in
`.bench-data/`; both are git-ignored. Each script says at its top which paths and package indexes it uses and how to
override them (for example a pip index or a Hugging Face mirror), and each is safe to rerun: a step whose result is
already in place is skipped.

| File | What it does |
|---|---|
| `setup_robocasa.sh` | RoboCasa venv (CPython 3.12) with the submodules `third_party/robocasa` and `third_party/robosuite` on its path, the changes to RoboCasa applied (`scripts/patch_third_party.py`), `bench-env.sh` next to the venv |
| `robocasa-requirements.txt` | the RoboCasa venv's packages, at the tested versions |
| `setup_libero.sh`, `constraints-libero.txt`, `bench-libero.lock.txt` | LIBERO-PRO venv (CPython 3.11, the simulator stack RPent uses: RLinf's LIBERO-PRO and LIBERO packaging, CPU-only torch), assets from the Hugging Face dataset RLinf/LIBERO-PRO-assets, `env.sh` |
| `libero-pro-assets.md5` | checksums of the 865 LIBERO-PRO asset files |
| `setup_robotwin.sh`, `robotwin-lock.txt` | RoboTwin 2.0 venv (CPython 3.11, rlinf-robotwin-runtime 0.1.1, torch cu128, cuRobo built locally with a CUDA 12.8 build toolchain it assembles itself), the asset snapshot, `env.sh` |
| `scan_robotwin_seeds.py` | finds RoboTwin's test seeds as its own evaluation does (the result is `hexashell/benches/robotwin/seeds.json`) |
| `setup_dexjoco_x.sh` | DexJoCo-X environment (the release's reference numerical environment: conda-forge MuJoCo 3.4.0, Python 3.11) from its locks, the release at 39f7c25, its training data (`TRAINING=0` leaves them out), `env.sh` |
| `geniesim35_native_config.sh`, `geniesim36_container.sh` | what RoboColiseum's local runs need beside Genie Sim itself: the native copy of Season I's configuration, and one headless container per GPU for Season II |
| `hf_snapshot.py` | resumable download of one Hugging Face snapshot into a plain folder |
| `smoke_libero.py`, `smoke_robotwin.py`, `smoke_dexjoco_x.py` | headless checks of three of the environments: rendering, cameras, stepping, success checks |

Then set `HEXASHELL_<BENCH>_PYTHON` and `HEXASHELL_<BENCH>_ENV` in `config/env.sh` (see `config/env.example.sh`,
which has the default paths). The versions are listed in `THIRD_PARTY_VERSIONS.md`.

The environment files (`bench-env.sh`, `env.sh`) are read by `hexashell.benches.session.read_env_file` without a
shell: one plain `export NAME=value` per line, `$VAR` allowed, no `${VAR:+...}` and no several names on one line.

RoboCasa's kitchen assets are not downloaded by `setup_robocasa.sh`: run RoboCasa's own script, which asks before
it starts (`$VENV/bin/python -m robocasa.scripts.download_kitchen_assets --type all`), or point `ASSETS` at a
complete asset folder.

## LIBERO-Pro 2.0

The evaluation bundle is the Hugging Face dataset
[zhouxueyang/LIBERO-Pro2.0-Eval](https://huggingface.co/datasets/zhouxueyang/LIBERO-Pro2.0-Eval); it brings its own
environment (Python 3.8, MuJoCo 3.2.3, robosuite 1.4.0):

```bash
hf download zhouxueyang/LIBERO-Pro2.0-Eval --repo-type dataset --include evaluation.zip evaluation.zip.sha256 \
    assets.zip assets.zip.sha256 "manifests/**" "scripts/**" "docs/**" --local-dir ./LIBERO-Pro2.0-Eval
python ./LIBERO-Pro2.0-Eval/scripts/prepare_evaluation.py --download-dir ./LIBERO-Pro2.0-Eval \
    --output-dir ./libero-pro-install
cd libero-pro-install/LIBERO-Pro-evaluation
export LIBERO_PRO_CONDA_PREFIX="$PWD/.envs/libero-pro"
bash install.sh "$LIBERO_PRO_CONDA_PREFIX"
```

Set `HEXASHELL_LIBEROPRO2_PYTHON` to that environment's python and write `.bench-data/libero-pro2/env.sh`
(`HEXASHELL_LIBEROPRO2_ENV`), with `BUNDLE` standing for the absolute path of `LIBERO-Pro-evaluation`:

```bash
export LIBERO_PRO2_BUNDLE=BUNDLE
export LIBERO_PRO_ROOT=BUNDLE/simulator-source
export LIBERO_CONFIG_PATH=/path/to/HexaShell/.bench-data/libero-pro2/config
export LIBERO_REMOTE_TASK_MANIFEST=BUNDLE/service-source/assets/task_manifest.json
export LIBERO_REMOTE_BDDL_ROOT=BUNDLE/service-source/assets/bddl/libero-pro2.0-1680-v1
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
```

`LIBERO_CONFIG_PATH` is a folder with LIBERO's `config.yaml`: `benchmark_root` (`BUNDLE/simulator-source/libero/libero`),
`assets` and `init_states` (its `assets` and `init_files`), `bddl_files` (`LIBERO_REMOTE_BDDL_ROOT`) and `datasets`
(any path; nothing is read from it).

## SIMPLE (Unitree G1)

Install [SIMPLE](https://github.com/physical-superintelligence-lab/SIMPLE) at
`b75c6786f87b4a7f43b52b0ac5b83a1d9ea62e11` as its README describes
(`uv sync --all-groups --index-strategy unsafe-best-match`: Python 3.10, MuJoCo 3.3.6, Isaac Sim 4.5), set
`HEXASHELL_SIMPLE_PYTHON` to its `.venv/bin/python`, and write `.bench-data/simple/env.sh` (`HEXASHELL_SIMPLE_ENV`):

```bash
export MUJOCO_GL=egl
export SIMPLE_RENDERER=isaac
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=N
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export SIMPLE_EVAL_DIR=/path/to/HexaShell/.bench-data/simple/eval
```

`SIMPLE_EVAL_DIR` holds the official evaluation episodes, the archives `simple-eval/<TaskEnv>.zip` of the Hugging Face
dataset USC-PSI-Lab/psi-data, unpacked; only the test needs them.

## RoboColiseum (Genie Sim)

The **online** agents need no simulator: the platform runs it, and the agent only dials the platform's gateway. Local
runs need Genie Sim ([AgibotTech/genie_sim](https://github.com/AgibotTech/genie_sim)), whose README and docker guide
describe its installation; what follows is how our runs used it.

**Season I (Genie Sim 3.5)** runs natively, without docker. In `.bench-data/geniesim-3.5` (`HEXASHELL_GENIESIM35`):

1. `genie_sim/`: the repository at `v3.5.0`; `GenieSimAssets/`: the assets of the four boards
   (`geniesim assets download`, see `source/geniesim_cli/docs/assets-download.md` there).
2. A Python 3.11 environment with the pip packages of Isaac Sim 5.1 and the benchmark's requirements
   (`source/geniesim_benchmark/pyproject.toml`; numpy 1.26.4, scipy 1.13.1, the `ik_solver` wheel), by default
   `.venvs/bench-geniesim35` (`GENIESIM_PYTHON`).
3. `bash scripts/setup/geniesim35_native_config.sh`: the copy of the benchmark's configuration whose scene files name
   the assets by their local path.

**Season II (Genie Sim 3.6)** runs in docker, one container per GPU. In `.bench-data/geniesim-3.6`
(`HEXASHELL_GENIESIM36`):

1. `genie_sim/`: the repository at `847bb9e` with its submodule `genie_sim_robot_model`, and our patch, which lets one
   waiting time of its startup be raised:
   `patch -d .bench-data/geniesim-3.6/genie_sim -p1 < scripts/patches/geniesim36-initialization-timeout.patch`.
2. The image: `geniesim docker build` (Isaac Sim 6.2, ROS 2 Jazzy), tagged `geniesim4-local:3.6.0` (or named by
   `GENIESIM_IMAGE`).
3. `GenieSimAssets/`: `geniesim assets download finemanip dexmanip wbc crsemb` (ModelScope
   agibot_world/GenieSimAssets `dfa5864`). That revision lacks two things its own tasks need, which a later one has
   (we took them from `ad5ca655`): the behaviour scripts that import `omni.kit.scripting`, a module Isaac Sim 6 no
   longer has (21 files), and `mujoco/rubik_cube/README.md`.
4. `bash scripts/setup/geniesim36_container.sh hexashell-geniesim-gpu0 0`, then once per machine `geniesim ros build dev`
   inside it (`docker exec -u isaac-sim -w /workspace hexashell-geniesim-gpu0 bash -lc "geniesim ros build dev"`). A run
   starts its GPU's container itself when it is missing. The first task on a machine builds shader and kernel caches
   and takes much longer than later ones.

## checks/

Small probes for checking a new machine against one that works. Run them with the benchmark's venv:

- `egl_probe.py`, `egl_uuid.py`: EGL devices with their CUDA ordinal, DRM node and UUID;
  `egl_pin.py CODE_ROOT`: which EGL device `hexashell.benches.egl` picks for each GPU.
- `src_hash.py`: hashes of the robosuite and robocasa sources and the mujoco and numpy versions, to compare two
  machines.
- `scene_sig.py CODE_ROOT GPU TASK:SEED ...`, `scene_dump.py CODE_ROOT GPU OUT TASK:SEED ...` and
  `scene_cmp.py DIR_A DIR_B`: whether a (task, seed) builds the same kitchen and start state on two machines.
