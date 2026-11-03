# Machine settings for HexaShell's scripts (scripts/run_one.sh, batch.sh, bench_batch.sh,
# sam3_service.sh).
#
#   cp config/env.example.sh config/env.sh     # config/env.sh is git-ignored; edit the paths below
#
# The scripts source $HEXASHELL_ENV when it is set, else config/env.sh. Without it run_one.sh and
# batch.sh stop; sam3_service.sh falls back to its defaults.
# Agent settings (model, prompt, limits, motion profile...) are not here: they are in
# xpolicylab/HexaShell/deploy.yml, and HEXASHELL_<NAME> variables override them per run.

# RoboDojo checkout: the folder holding scripts/robodojo.sh and XPolicyLab/
export ROBODOJO_ROOT=/path/to/RoboDojo/repo

# The two Python environments robodojo.sh activates (`conda activate <env>`): conda env names, or
# venv paths when a conda shim that activates venv paths is on PATH.
export POLICY_ENV=hexashell          # the policy server: Python 3.11 + requirements.txt
export EVAL_ENV=robodojo             # the simulator: Isaac Sim 5.1, RoboDojo's own setup

# SAM3 segmentation service (scripts/sam3_service.sh start GPU PORT); deploy.yml sam3_url points at it
export SAM3_PYTHON=$HOME/miniconda3/envs/hexashell/bin/python    # needs torch and sam3 as well
export SAM3_CHECKPOINT_PATH=/path/to/HexaShell/checkpoints/sam3.pt

# Benchmarks besides RoboDojo (python -m hexashell.benches.run, scripts/bench_batch.sh, bench_pool.py):
# the policy side's Python, and per benchmark its own Python and a file of export lines its server
# needs. The paths below are where scripts/setup/setup_*.sh put them by default (inside the checkout).
export HEXASHELL_PYTHON=$HOME/miniconda3/envs/hexashell/bin/python
export HEXASHELL_ROBOCASA_PYTHON=/path/to/HexaShell/.venvs/bench-robocasa/bin/python
export HEXASHELL_ROBOCASA_ENV=/path/to/HexaShell/.venvs/bench-robocasa/bench-env.sh
export HEXASHELL_LIBERO_PYTHON=/path/to/HexaShell/.venvs/bench-libero/bin/python
export HEXASHELL_LIBERO_ENV=/path/to/HexaShell/.bench-data/libero-pro/env.sh
export HEXASHELL_ROBOTWIN_PYTHON=/path/to/HexaShell/.venvs/bench-robotwin/bin/python
export HEXASHELL_ROBOTWIN_ENV=/path/to/HexaShell/.bench-data/robotwin/env.sh
# RoboCasa's kitchens: pretrain (RoboCasa's official evaluation split) or pretrain50 (the default when unset)
export HEXASHELL_ROBOCASA_SPLIT=pretrain

# The model's API key is read from deploy.yml api_key_file (~/.config/hexashell/api-key: one line,
# chmod 600), else from $OPENAI_API_KEY. Never put the key in this repository.

# A proxy would also catch the websocket between the simulator and the policy server, and the
# model API; the runs need direct connections.
unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy

# Isaac Sim: accept the EULA non-interactively; keep user site-packages out of both environments
export ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=N PYTHONNOUSERSITE=1
# a temp folder of our own (IsaacLab writes $TMPDIR/isaaclab/logs, which another user may own in /tmp)
export TMPDIR=/tmp/$USER-hexashell; mkdir -p "$TMPDIR"
