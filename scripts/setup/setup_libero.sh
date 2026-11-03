#!/usr/bin/env bash
# Reproducible, idempotent setup of the LIBERO-PRO simulator venv ("bench-libero") that HexaShell's
# LIBERO env-server process runs in. Same simulator stack as RPent[libero-pro] (RPent 886b3b2):
#   rpent-liberopro 0.2.0  = RLinf/LIBERO-PRO @ rpent  (d1e11fb)
#   rpent-libero    0.2.0  = RLinf/LIBERO     @ rpent  (a832307)
#   robosuite 1.5.2, mujoco 3.3.0, numpy 1.26.4, gym 0.26.2, CPU-only torch 2.7.1 (liberopro.benchmark
#   imports torch and torch.load()s the init states; nothing else needs it). No VLA / CUDA toolkit.
# Assets: HF dataset RLinf/LIBERO-PRO-assets @ e4b10c9 (617 MB, 865 files), symlinked into the package.
#
# Re-running is safe: every step checks whether it is already done (FORCE_REINSTALL=1 reinstalls).
# Every path / index below can be overridden from the environment, e.g.:
#   VENV=/path/envs/bench-libero BASE_PY=/path/python3.11 DATA_DIR=/path/libero-pro bash setup_libero.sh
# Defaults: the venv in .venvs/bench-libero and the data in .bench-data/libero-pro of this checkout.
# Afterwards:  source "$DATA_DIR/env.sh"; "$LIBERO_BENCH_PY" smoke_libero.py   (headless EGL smoke test)
set -euo pipefail

# ---- paths ---------------------------------------------------------------------------------------
SETUP_DIR=${SETUP_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}
ROOT=$(cd "$SETUP_DIR/../.." && pwd)
DATA_DIR=${DATA_DIR:-$ROOT/.bench-data/libero-pro}             # assets, config, sources, logs
VENV=${VENV:-$ROOT/.venvs/bench-libero}
BASE_PY=${BASE_PY:-python3.11}                                 # any CPython 3.11
SRC_TARBALL_DIR=${SRC_TARBALL_DIR:-$DATA_DIR/src-tarballs}   # LIBERO-PRO-src.tar.gz, RLinf-LIBERO-src.tar.gz
SRC_DIR=${SRC_DIR:-$DATA_DIR/src}                            # extracted sources (pip builds from here)
ASSETS_DIR=${ASSETS_DIR:-$DATA_DIR/assets}                   # read-only at run time
CONFIG_DIR=${CONFIG_DIR:-$DATA_DIR/config}                   # LIBERO_CONFIG_PATH: writable, one per venv
LOG_DIR=${LOG_DIR:-$DATA_DIR/logs}
EGL_LIB_DIR=${EGL_LIB_DIR:-$(cd "$SETUP_DIR/.." && pwd)/runtime/egl}  # glvnd's EGL libraries, for hosts
                                                             # without libEGL.so.1
EGL_VENDOR_JSON=${EGL_VENDOR_JSON:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}

# ---- indexes / network ---------------------------------------------------------------------------
PIP_INDEX=${PIP_INDEX:-https://pypi.org/simple}
TORCH_INDEX=${TORCH_INDEX:-https://download.pytorch.org/whl/cpu}
HF_ENDPOINT=${HF_ENDPOINT:-https://huggingface.co}
UNSET_PROXY=${UNSET_PROXY:-0}        # 1: run network tools without HTTP(S)_PROXY
FETCH_FROM_GIT=${FETCH_FROM_GIT:-1}  # 1: build missing source tarballs from GitHub; 0: they must be in SRC_TARBALL_DIR

# ---- pins ----------------------------------------------------------------------------------------
TORCH_VER=2.7.1+cpu
LIBERO_PRO_GIT=https://github.com/RLinf/LIBERO-PRO.git
LIBERO_PRO_REV=d1e11fb181b8544487d27742c0caa3a4d46452ad      # branch rpent
RLINF_LIBERO_GIT=https://github.com/RLinf/LIBERO.git
RLINF_LIBERO_REV=a8323074d93a09e32bd898630a70531b1f51bc77    # branch rpent
ASSETS_REPO=RLinf/LIBERO-PRO-assets                          # HF dataset
ASSETS_REV=e4b10c9adc5735c9c8776917527178e693ef10b3
CONSTRAINTS=$SETUP_DIR/constraints-libero.txt                # hand-written pins (mujoco, numpy, gym, ...)
LOCK=$SETUP_DIR/bench-libero.lock.txt                        # full pip freeze of the reference venv
ASSETS_MD5=$SETUP_DIR/libero-pro-assets.md5                  # md5 manifest of the assets tree
FORCE_REINSTALL=${FORCE_REINSTALL:-0}
RUN_SMOKE=${RUN_SMOKE:-0}            # 1: run smoke_libero.py --cases object_swap at the end (needs a GPU)

# ---------------------------------------------------------------------------------------------------
log() { printf '\n[setup_libero %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[setup_libero] ERROR: %s\n' "$*" >&2; exit 1; }
net() {
  if [ "$UNSET_PROXY" = 1 ]; then env -u HTTPS_PROXY -u HTTP_PROXY -u https_proxy -u http_proxy "$@"; else "$@"; fi
}
tar_rev() { zcat "$1" 2>/dev/null | git get-tar-commit-id 2>/dev/null || true; }

export PIP_DISABLE_PIP_VERSION_CHECK=1 HF_ENDPOINT HF_HUB_DISABLE_TELEMETRY=1
export LIBERO_CONFIG_PATH=$CONFIG_DIR
mkdir -p "$DATA_DIR" "$SRC_TARBALL_DIR" "$SRC_DIR" "$ASSETS_DIR" "$CONFIG_DIR" "$LOG_DIR"
command -v git >/dev/null || die "git is needed (to read/verify the source tarball revisions)"
PY=$VENV/bin/python
PIP=("$PY" -m pip)
PIN_ARGS=(-c "$CONSTRAINTS")
[ -f "$LOCK" ] && PIN_ARGS+=(-c "$LOCK")

# ---- 1. source tarballs (git archive of the pinned commits, without assets/images/notebooks) -------
fetch_src() {  # NAME GIT_URL REV ASSETS_SUBDIR
  local name=$1 url=$2 rev=$3 assets=$4 tmp
  tmp=$(mktemp -d)
  git -C "$tmp" init -q
  net git -C "$tmp" fetch -q --depth 1 "$url" "$rev"
  git -C "$tmp" archive --prefix="$name/" --format=tar FETCH_HEAD -- . ":(exclude)$assets" \
      ":(exclude)images" ":(exclude)notebooks" | gzip -9 > "$SRC_TARBALL_DIR/$name-src.tar.gz.new"
  mv "$SRC_TARBALL_DIR/$name-src.tar.gz.new" "$SRC_TARBALL_DIR/$name-src.tar.gz"
  rm -rf "$tmp"
}
for spec in "LIBERO-PRO|$LIBERO_PRO_GIT|$LIBERO_PRO_REV|liberopro/liberopro/assets" \
            "RLinf-LIBERO|$RLINF_LIBERO_GIT|$RLINF_LIBERO_REV|libero/libero/assets"; do
  IFS='|' read -r name url rev assets <<<"$spec"
  tb=$SRC_TARBALL_DIR/$name-src.tar.gz
  if [ ! -f "$tb" ]; then
    [ "$FETCH_FROM_GIT" = 1 ] || die "$tb missing: copy it from another host or re-run with FETCH_FROM_GIT=1"
    log "fetching $name @ $rev from $url"
    fetch_src "$name" "$url" "$rev" "$assets"
  fi
  got=$(tar_rev "$tb")
  [ "$got" = "$rev" ] || die "$tb is commit '$got', expected $rev"
done
log "source tarballs OK ($LIBERO_PRO_REV, $RLINF_LIBERO_REV)"

# ---- 2. venv ---------------------------------------------------------------------------------------
if [ ! -x "$PY" ]; then
  "$BASE_PY" -c 'import sys; assert sys.version_info[:2] == (3, 11), sys.version' \
    || die "BASE_PY=$BASE_PY is not CPython 3.11"
  log "creating venv $VENV from $BASE_PY"
  "$BASE_PY" -m venv "$VENV"
fi
"$PY" -c 'import sys; assert sys.version_info[:2] == (3, 11), sys.version' || die "$VENV is not Python 3.11"

# ---- 3. packages -------------------------------------------------------------------------------------
stamp_file=$VENV/.bench-libero-installed
lock_md5=none
[ -f "$LOCK" ] && lock_md5=$(md5sum < "$LOCK" | cut -c1-32)
want_stamp="liberopro=$LIBERO_PRO_REV libero=$RLINF_LIBERO_REV torch=$TORCH_VER"
want_stamp+=" constraints=$(md5sum < "$CONSTRAINTS" | cut -c1-32) lock=$lock_md5"
if [ "$FORCE_REINSTALL" != 1 ] && [ "$(cat "$stamp_file" 2>/dev/null)" = "$want_stamp" ] \
   && "$PY" -I -c 'import liberopro.liberopro.benchmark, robosuite' >/dev/null 2>&1; then
  log "packages already installed (stamp matches) - skipping pip"
else
  log "installing base tools (pip, huggingface_hub)"
  net "${PIP[@]}" install -q -i "$PIP_INDEX" "${PIN_ARGS[@]}" --upgrade pip huggingface_hub
  # torch: the CPU wheel only from the PyTorch index (--no-deps: its small dependencies come from
  # PIP_INDEX below).
  if ! "$PY" -c "import importlib.metadata as m, sys; sys.exit(m.version('torch') != '$TORCH_VER')" 2>/dev/null; then
    log "installing torch==$TORCH_VER (no deps) from $TORCH_INDEX"
    net "${PIP[@]}" install -q --no-deps --index-url "$TORCH_INDEX" "torch==$TORCH_VER"
  fi
  log "extracting sources into $SRC_DIR"
  rm -rf "$SRC_DIR/LIBERO-PRO" "$SRC_DIR/RLinf-LIBERO"
  tar -C "$SRC_DIR" -xzf "$SRC_TARBALL_DIR/LIBERO-PRO-src.tar.gz"
  tar -C "$SRC_DIR" -xzf "$SRC_TARBALL_DIR/RLinf-LIBERO-src.tar.gz"
  log "installing rpent-libero + rpent-liberopro (+ torch's deps) with pins; log: $LOG_DIR/pip_install.log"
  if ! net "${PIP[@]}" install -i "$PIP_INDEX" "${PIN_ARGS[@]}" "torch==$TORCH_VER" \
         "$SRC_DIR/RLinf-LIBERO" "$SRC_DIR/LIBERO-PRO" > "$LOG_DIR/pip_install.log" 2>&1; then
    tail -30 "$LOG_DIR/pip_install.log"
    die "pip install failed, see $LOG_DIR/pip_install.log"
  fi
  grep -E "^(Successfully|WARNING|ERROR)" "$LOG_DIR/pip_install.log" || true
  "${PIP[@]}" check
  printf '%s\n' "$want_stamp" > "$stamp_file"
fi

# ---- 4. assets (HF snapshot into a plain directory; resumable) ---------------------------------------
done_marker=$DATA_DIR/.assets-complete
if [ "$(cat "$done_marker" 2>/dev/null)" = "$ASSETS_REPO@$ASSETS_REV" ] && [ -d "$ASSETS_DIR/scenes" ]; then
  log "assets present ($ASSETS_REPO@$ASSETS_REV) - skipping download"
else
  log "downloading $ASSETS_REPO@$ASSETS_REV via $HF_ENDPOINT into $ASSETS_DIR; log: $LOG_DIR/assets_download.log"
  if ! net "$PY" -I "$SETUP_DIR/hf_snapshot.py" "$ASSETS_REPO" "$ASSETS_REV" "$ASSETS_DIR" dataset \
         > "$LOG_DIR/assets_download.log" 2>&1; then
    tail -c 2000 "$LOG_DIR/assets_download.log"
    die "asset download failed (re-run to resume), see $LOG_DIR/assets_download.log"
  fi
  grep -E "snapshot ready|attempt" "$LOG_DIR/assets_download.log" || true
  if [ -f "$ASSETS_MD5" ]; then
    log "verifying assets against $ASSETS_MD5"
    (cd "$ASSETS_DIR" && md5sum -c --quiet "$ASSETS_MD5") || die "asset md5 mismatch"
  fi
  printf '%s\n' "$ASSETS_REPO@$ASSETS_REV" > "$done_marker"
fi

# ---- 5. LIBERO config + asset link --------------------------------------------------------------------
# liberopro reads LIBERO_CONFIG_PATH/config.yaml (paths into the installed package) and takes a FileLock
# there at import, so the dir must be writable. Regenerate it every run so it matches this venv.
log "writing $CONFIG_DIR/config.yaml and linking the package assets dir to $ASSETS_DIR"
tmpcfg=$(mktemp -d "$CONFIG_DIR/.regen.XXXXXX")
LIBERO_CONFIG_PATH=$tmpcfg "$PY" -I -c 'import liberopro.liberopro' >/dev/null
mv -f "$tmpcfg/config.yaml" "$CONFIG_DIR/config.yaml"
rm -rf "$tmpcfg"
touch "$CONFIG_DIR/.config_lock"
"$VENV/bin/liberopro-download-assets" --link "$ASSETS_DIR"

# ---- 6. env.sh for the env-server process -------------------------------------------------------------
cat > "$DATA_DIR/env.sh.new" <<EOF
# Environment for the LIBERO-PRO simulator venv (bench-libero). Generated by setup_libero.sh.
# Usage: source this file, then run "\$LIBERO_BENCH_PY" your_env_server.py ...
export LIBERO_BENCH_VENV=$VENV
export LIBERO_BENCH_PY=$PY
export LIBERO_CONFIG_PATH=$CONFIG_DIR
export LIBERO_PRO_ASSET_PATH=$ASSETS_DIR
export LIBERO_TYPE=pro   # read only by RLinf's routing, not by liberopro itself; kept for parity with RPent
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
# Per process, pick the render GPU: unset CUDA_VISIBLE_DEVICES and set MUJOCO_EGL_DEVICE_ID to
# the EGL index of that GPU (see smoke_libero.py select_render_device()).
EOF
# Some hosts have NVIDIA's EGL (libEGL_nvidia.so.0) but not glvnd's libEGL.so.1, which PyOpenGL loads:
# then a copy of glvnd's libEGL.so.1 put in $EGL_LIB_DIR by hand (e.g. from the libglvnd0 package), with
# NVIDIA's EGL behind it.
system_libs=$(ldconfig -p 2>/dev/null || true)   # not piped into grep -q: SIGPIPE under pipefail
if ! grep -q 'libEGL\.so\.1 ' <<<"$system_libs"; then
  if [ -e "$EGL_LIB_DIR/libEGL.so.1" ]; then
    log "no libEGL.so.1 on this system: env.sh uses the copy in $EGL_LIB_DIR"
    cat >> "$DATA_DIR/env.sh.new" <<EOF
# no glvnd libEGL.so.1 on this system: the copy in runtime/egl, NVIDIA's EGL behind it
export LD_LIBRARY_PATH=$EGL_LIB_DIR
export __EGL_VENDOR_LIBRARY_FILENAMES=$EGL_VENDOR_JSON
EOF
  else
    log "WARNING: no libEGL.so.1 on this system and none in EGL_LIB_DIR=$EGL_LIB_DIR; EGL rendering will fail"
  fi
fi
mv "$DATA_DIR/env.sh.new" "$DATA_DIR/env.sh"

# ---- 7. record + verify -------------------------------------------------------------------------------
"${PIP[@]}" freeze > "$LOG_DIR/pip-freeze.txt"
if [ -f "$LOCK" ] && ! diff <(grep -v '^#' "$LOCK") <(grep -v ' @ file://' "$LOG_DIR/pip-freeze.txt") >/dev/null; then
  log "WARNING: installed packages differ from $LOCK:"
  diff <(grep -v '^#' "$LOCK") <(grep -v ' @ file://' "$LOG_DIR/pip-freeze.txt") || true
fi
log "verifying imports, suites and init states"
"$PY" -I - <<'EOF'
import importlib.metadata as md
import os
for p in ["rpent-liberopro", "rpent-libero", "robosuite", "mujoco", "numpy", "gym", "bddl", "torch"]:
    print(f"  {p}=={md.version(p)}")
import torch
assert torch.version.cuda is None, "expected the CPU-only torch wheel"
from liberopro.liberopro import benchmark, get_libero_path
assets = get_libero_path("assets")
assert os.path.isdir(os.path.join(assets, "scenes")), assets
suites = ["libero_spatial_task", "libero_spatial_swap", "libero_object_task", "libero_object_swap",
          "libero_goal_task", "libero_goal_swap", "libero_10_task", "libero_10_swap"]
runnable = 0
for s in suites:
    b = benchmark.get_benchmark(s)()
    n = [len(b.get_task_init_states(i)) for i in range(b.n_tasks)]
    runnable += sum(x > 0 for x in n)
    print(f"  {s}: {b.n_tasks} tasks, init states per task {sorted(set(n))}")
print(f"  runnable tasks: {runnable}/80; assets -> {os.path.realpath(assets)}")
EOF
if [ "$RUN_SMOKE" = 1 ]; then
  log "running the EGL smoke test (object_swap)"
  # shellcheck disable=SC1091
  (source "$DATA_DIR/env.sh" && cd "$SETUP_DIR" && "$PY" smoke_libero.py --cases object_swap)
fi
log "done. source $DATA_DIR/env.sh before starting the env server."
