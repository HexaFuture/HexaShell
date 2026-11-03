# Simulator environments for the benchmarks

RoboCasa365, LIBERO-PRO and RoboTwin 2.0 each run in their own Python environment, separate from the policy's.
These scripts build them. By default everything goes inside the checkout: the venvs in `.venvs/`, the data
(sources, assets, run-time environment files, logs) in `.bench-data/`; both are git-ignored. Each script says at its
top which paths and package indexes it uses and how to override them (for example a pip index or a Hugging Face
mirror), and each is safe to rerun: a step whose result is already in place is skipped.

| File | What it does |
|---|---|
| `setup_robocasa.sh` | RoboCasa venv (CPython 3.12) with the submodules `third_party/robocasa` and `third_party/robosuite` on its path, the changes to RoboCasa applied (`scripts/patch_third_party.py`), `bench-env.sh` next to the venv |
| `robocasa-requirements.txt` | the RoboCasa venv's packages, at the tested versions |
| `setup_libero.sh`, `constraints-libero.txt`, `bench-libero.lock.txt` | LIBERO-PRO venv (CPython 3.11, the simulator stack RPent uses: RLinf's LIBERO-PRO and LIBERO packaging, CPU-only torch), assets from the Hugging Face dataset RLinf/LIBERO-PRO-assets, `env.sh` |
| `libero-pro-assets.md5` | checksums of the 865 LIBERO-PRO asset files |
| `setup_robotwin.sh`, `robotwin-lock.txt` | RoboTwin 2.0 venv (CPython 3.11, rlinf-robotwin-runtime 0.1.1, torch cu128, cuRobo built locally with a CUDA 12.8 build toolchain it assembles itself), the asset snapshot, `env.sh` |
| `hf_snapshot.py` | resumable download of one Hugging Face snapshot into a plain folder |
| `smoke_libero.py`, `smoke_robotwin.py` | headless checks of the two venvs: rendering, cameras, stepping, success checks |

Then set `HEXASHELL_<BENCH>_PYTHON` and `HEXASHELL_<BENCH>_ENV` in `config/env.sh` (see `config/env.example.sh`,
which has the default paths). The versions are listed in `THIRD_PARTY_VERSIONS.md`.

The environment files (`bench-env.sh`, `env.sh`) are read by `hexashell.benches.session.read_env_file` without a
shell: one plain `export NAME=value` per line, `$VAR` allowed, no `${VAR:+...}` and no several names on one line.

RoboCasa's kitchen assets are not downloaded by `setup_robocasa.sh`: run RoboCasa's own script, which asks before
it starts (`$VENV/bin/python -m robocasa.scripts.download_kitchen_assets --type all`), or point `ASSETS` at a
complete asset folder.

## checks/

Small probes for checking a new machine against one that works. Run them with the benchmark's venv:

- `egl_probe.py`, `egl_uuid.py`: EGL devices with their CUDA ordinal, DRM node and UUID;
  `egl_pin.py CODE_ROOT`: which EGL device `hexashell.benches.egl` picks for each GPU.
- `src_hash.py`: hashes of the robosuite and robocasa sources and the mujoco and numpy versions, to compare two
  machines.
- `scene_sig.py CODE_ROOT GPU TASK:SEED ...`, `scene_dump.py CODE_ROOT GPU OUT TASK:SEED ...` and
  `scene_cmp.py DIR_A DIR_B`: whether a (task, seed) builds the same kitchen and start state on two machines.
