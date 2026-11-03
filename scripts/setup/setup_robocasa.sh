#!/bin/bash
# The RoboCasa simulator environment of hexashell.benches.robocasa: a venv (CPython 3.12) with the packages
# of robocasa-requirements.txt, the submodules third_party/robocasa and third_party/robosuite on its path
# (a .pth file: pip-installing robocasa would pull in its own pins), HexaShell's changes to RoboCasa
# (scripts/patch_third_party.py), and bench-env.sh next to the venv. Idempotent: rerun it after a failure.
#
#   bash scripts/setup/setup_robocasa.sh
#
# Settings (environment):
#   VENV    the venv to build (default .venvs/bench-robocasa in this checkout)
#   PY      a CPython 3.12 (default python3.12; e.g. `uv python install 3.12`)
#   INDEX   the pip index (default PyPI)
#   ASSETS  RoboCasa's kitchen assets (about 25 GB). Default: the submodule's own folder
#           third_party/robocasa/robocasa/models/assets, where RoboCasa's download script puts them:
#             $VENV/bin/python -m robocasa.scripts.download_kitchen_assets --type all
#           (it asks before downloading). Or a complete asset folder elsewhere, as that script leaves it.
#
# Then in config/env.sh: HEXASHELL_ROBOCASA_PYTHON=$VENV/bin/python, HEXASHELL_ROBOCASA_ENV=$VENV/bench-env.sh.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
VENV=${VENV:-$ROOT/.venvs/bench-robocasa}
PY=${PY:-python3.12}
INDEX=${INDEX:-https://pypi.org/simple}
OWN_ASSETS=$ROOT/third_party/robocasa/robocasa/models/assets
ASSETS=${ASSETS:-$OWN_ASSETS}
cd "$ROOT"

echo "== submodules"
if [ ! -f third_party/robocasa/setup.py ] || [ ! -f third_party/robosuite/setup.py ]; then
    git submodule update --init third_party/robocasa third_party/robosuite
fi
python3 scripts/patch_third_party.py

echo "== venv $VENV"
[ -x "$VENV/bin/python" ] || "$PY" -m venv "$VENV"
"$VENV/bin/python" -m pip install -q -i "$INDEX" --upgrade pip
"$VENV/bin/python" -m pip install -q -i "$INDEX" -r scripts/setup/robocasa-requirements.txt
SP=$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
printf '%s\n%s\n' "$ROOT/third_party/robocasa" "$ROOT/third_party/robosuite" > "$SP/hexashell_bench_casa.pth"

mkdir -p "$ASSETS"
ASSETS=$(cd "$ASSETS" && pwd)
cat > "$VENV/bench-env.sh" <<EOF
# source this before starting the RoboCasa env server (hexashell.benches reads it too: one export per line)
export HEXA_ROBOCASA_ASSETS=$ASSETS
export PYTHONHASHSEED=0
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
# small BLAS thread pools: a machine runs many simulator servers at once
export OMP_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export MKL_NUM_THREADS=4
# a host without glvnd's libEGL.so.1 (packages libegl1, libopengl0): copy those libraries into a folder and add
# export LD_LIBRARY_PATH=/path/to/that/folder
# export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
EOF

echo "== check"
HEXA_ROBOCASA_ASSETS=$ASSETS "$VENV/bin/python" -c "import mujoco, numpy, robosuite, robocasa
print(f'mujoco {mujoco.__version__}, numpy {numpy.__version__}, robosuite {robosuite.__version__}, robocasa {robocasa.__version__}')"
missing=""
for d in scenes fixtures textures objects/objaverse objects/lightwheel; do [ -d "$ASSETS/$d" ] || missing+=" $d"; done
if [ -n "$missing" ]; then
    echo "assets incomplete in $ASSETS (missing:$missing)"
    [ "$ASSETS" = "$(cd "$OWN_ASSETS" && pwd)" ] &&
        echo "download them: $VENV/bin/python -m robocasa.scripts.download_kitchen_assets --type all"
fi
echo "venv ready: $VENV (bench-env.sh next to it)"
