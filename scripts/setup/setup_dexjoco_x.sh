#!/usr/bin/env bash
# Reproducible, idempotent setup of the DexJoCo-X simulator env ("bench-dexjoco-x") that HexaShell's
# DexJoCo-X server process runs in, following the release's README "Get started": its reference
# numerical environment (MuJoCo 3.4.0 from conda-forge, Python 3.11, numpy 1.26.4, OpenBLAS Haswell,
# one thread), under which its demonstrations replay bit for bit.
#   release: github.com/dexjoco-x/DexJoCo-X-public @ 39f7c25 (code MIT; data CC BY-NC-SA 4.0)
#   Training v1 (the demonstrations; only for the verdict check and the learning seeds' scenes):
#   the release's tools/dataset_packages.py, one 5.21 GB archive from Hugging Face
#
# Re-running is safe: every step checks whether it is already done.  Paths can be overridden, e.g.
#   DATA_DIR=/path/to/data/dexjoco-x ENV=/path/to/envs/bench-dexjoco-x bash scripts/setup/setup_dexjoco_x.sh
# Afterwards (from the repository):
#   source "$DATA_DIR/env.sh"; MUJOCO_EGL_DEVICE_ID=0 "$ENV/bin/python" -I scripts/setup/smoke_dexjoco_x.py
# and in config/env.sh: HEXASHELL_DEXJOCO_X_PYTHON=$ENV/bin/python, HEXASHELL_DEXJOCO_X_ENV=$DATA_DIR/env.sh.
set -euo pipefail

# ---- paths ---------------------------------------------------------------------------------------
SETUP_DIR=${SETUP_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}
DATA_DIR=${DATA_DIR:-$(cd "$SETUP_DIR/../.." && pwd)/.bench-data/dexjoco-x}
ENV=${ENV:-$(cd "$SETUP_DIR/../.." && pwd)/.venvs/bench-dexjoco-x}
CONDA=${CONDA:-$HOME/miniconda3/bin/conda}
SRC=${SRC:-$DATA_DIR/src/DexJoCo-X-public}
RELEASE_GIT=${RELEASE_GIT:-https://github.com/dexjoco-x/DexJoCo-X-public.git}
RELEASE_COMMIT=${RELEASE_COMMIT:-39f7c25}
TRAINING=${TRAINING:-1}               # 1: also download and check Training v1 (4.9 GB extracted)
HF_ENDPOINT=${HF_ENDPOINT:-https://huggingface.co}
CACHE=${CACHE:-$DATA_DIR/..}          # conda packages, pip cache, temp files (not the home folder)

# An inherited X display would make MuJoCo's EGL renderer look for it.
unset DISPLAY XAUTHORITY
export CONDA_PKGS_DIRS=$CACHE/conda-pkgs PIP_CACHE_DIR=$CACHE/pip-cache TMPDIR=$CACHE/tmp
export PIP_DISABLE_PIP_VERSION_CHECK=1 HF_ENDPOINT HF_HUB_DISABLE_XET=1 HF_HOME=$DATA_DIR/hf-home
mkdir -p "$CONDA_PKGS_DIRS" "$PIP_CACHE_DIR" "$TMPDIR" "$DATA_DIR/src"
# the release's reference numerical environment (README "Get started")
export OPENBLAS_CORETYPE=Haswell OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

echo "[1] the release at $RELEASE_COMMIT"
if [ ! -d "$SRC/.git" ]; then
    git clone "$RELEASE_GIT" "$SRC"
fi
git -C "$SRC" cat-file -e "$RELEASE_COMMIT^{commit}" 2>/dev/null || git -C "$SRC" fetch origin
git -C "$SRC" checkout -q "$RELEASE_COMMIT"
cd "$SRC"

echo "[2] conda env from the release's explicit lock"
[ -x "$ENV/bin/python" ] || "$CONDA" create -y -p "$ENV" --override-channels -c conda-forge \
    --file requirements/native-replay-linux64.conda.txt
"$CONDA" env config vars set -p "$ENV" OPENBLAS_CORETYPE=Haswell OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 >/dev/null

echo "[3] pip lock, the simulator package (editable), runtime assets"
"$ENV/bin/python" -m pip install -q -r requirements/native-replay-pip.txt
"$ENV/bin/python" -m pip install -q --force-reinstall --no-deps numpy==1.26.4 PyOpenGL==3.1.10
"$ENV/bin/python" -m pip install -q -c requirements/constraints-python311.txt -e './simulator[benchmark,dataset,download]'
"$ENV/bin/python" tools/runtime_assets.py check

if [ "$TRAINING" = 1 ]; then
    echo "[4] Training v1"
    if [ -f "$DATA_DIR/training_v1/metadata/final_saved_label_replay.json" ]; then
        echo "    already in $DATA_DIR/training_v1"
    else
        "$ENV/bin/python" tools/dataset_packages.py download --tasks all --data-root "$DATA_DIR/training_v1" \
            --archives-root "$DATA_DIR/training_v1_archives/training_v1"
    fi
fi

echo "[5] the server's env file"
cat > "$DATA_DIR/env.sh" <<EOF
# Environment for the DexJoCo-X simulator env (bench-dexjoco-x: MuJoCo 3.4.0 from conda-forge, Python 3.11),
# read by hexashell.benches.run (\$HEXASHELL_DEXJOCO_X_ENV): export lines with literal values.
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
# the repository's reference numerical environment (README "Get started"; also set as conda env vars of the env)
export OPENBLAS_CORETYPE=Haswell
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
# the official camera profile (evaluation/runtime.py sets the same value)
export BENCHMARK7_CAMERA_PROFILE=dexjoco_official_allegro60_hanoi_front_v1
# the release's folder (the simulator server imports its evaluation/runtime.py and sim/ from here)
export DEXJOCO_X_ROOT=$SRC
EOF

"$ENV/bin/python" -c 'import sys, mujoco, numpy; print(sys.version.split()[0], "mujoco", mujoco.__version__, "numpy", numpy.__version__)'
echo "[done] next: the smoke test (see the top of this file)"
