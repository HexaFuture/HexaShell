#!/usr/bin/env bash
# setup_robotwin.sh -- reproducible, idempotent setup of a standalone RoboTwin 2.0 simulator env.
#
# The simulator stack only (no policy model), as RPent runs RoboTwin (see NOTICE):
#   rlinf-robotwin-runtime 0.1.1 (PyPI; SAPIEN 3.0.0b1, mplib 0.2.1, numpy 1.26.4, warp-lang 1.11.1, ...)
#   torch 2.7.1+cu128, cuRobo v0.7.8 (planner_backend="curobo"), asset snapshot
#   TianxingChen/RoboTwin2.0 @ c15cc97 (the revision pinned by the runtime package).
#
# Usage:   bash setup_robotwin.sh [STAGE ...]        (no argument = all stages, in this order)
#   venv     create $VENV from $BASE_PY with `python -m venv` (the base interpreter is not modified)
#   torch    torch 2.7.1+cu128 (wheel fetched once, sha256-checked, pip-installed; nvidia-* deps from $PIP_INDEX)
#   runtime  rlinf-robotwin-runtime==0.1.1 and its pinned deps
#   cudatk   CUDA 12.8 *build* toolchain (NVIDIA redist nvcc/cudart/cccl + torch's nvidia-* wheel headers) in $CUDA_HOME_DIR
#            (no system CUDA toolkit needed; only used to compile cuRobo, not at run time)
#   curobo   build + install cuRobo v0.7.8 from the GitHub tag tarball (sha256-checked), arch $TORCH_CUDA_ARCH_LIST
#   assets   resumable download + sha256 check + safe extraction of the asset snapshot into $ASSETS_ROOT,
#            plus the runtime's manifest (.robotwin-assets.json), i.e. what `robotwin-download-assets` produces
#   check    import / CUDA-extension / asset-root sanity check
#   env      write $DATA/env.sh, the run-time environment below (config/env.sh: HEXASHELL_ROBOTWIN_ENV)
# Each stage is skipped when its result is already in place, so the script can simply be re-run.
# All pip installs use $VENV/robotwin-constraints.txt = core pins + robotwin-lock.txt (next to this script; the
# pip freeze of the tested venv), so another host resolves exactly the tested versions.
#
# Source patches: none. RoboTwin's own _install.sh patches (a) sapien/wrapper/urdf_loader.py to open URDFs with
# encoding="utf-8" and (b) mplib/planner.py plan_screw() to drop the `or collide` early exit. RPent applies neither:
# (a) only matters under a non-UTF-8 locale (run the env server with a UTF-8 locale / PYTHONUTF8=1),
# (b) only affects mplib screw planning, which aloha-agilex never uses (curobo, or mplib RRT, + mplib TOPP only).
#
# Run-time environment for the simulator process (stage env writes it; see smoke_robotwin.py):
#   ROBOTWIN_ASSETS_ROOT=$ASSETS_ROOT  (ROBOTWIN_ASSETS_PATH / ASSETS_PATH = same value)
#   VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json (or /usr/share/vulkan/icd.d/nvidia_icd.json)
#   CUDA_VISIBLE_DEVICES=<one gpu>; no CUDA_HOME needed at run time.
#   Hosts without glvnd's libEGL.so.1, which NVIDIA's Vulkan driver loads (else SAPIEN: "failed to find a
#   rendering device"): LD_LIBRARY_PATH=<data>/runtime/egl and
#   __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json (as setup_libero.sh writes them).
set -euo pipefail

# ----------------------------------------------------------------------------- paths / knobs (override via env)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE_PY="${BASE_PY:-$(command -v python3.11 || true)}"  # any CPython 3.11
VENV="${VENV:-$ROOT/.venvs/bench-robotwin}"
DATA="${DATA:-$ROOT/.bench-data/robotwin}"
ASSETS_ROOT="${ASSETS_ROOT:-$DATA/assets}"            # = ROBOTWIN_ASSETS_ROOT at run time
ASSET_DL="${ASSET_DL:-$DATA/dl-assets-c15cc97}"       # downloaded asset zips (untrusted input)
WHEEL_DL="${WHEEL_DL:-$DATA/dl-wheels}"               # downloaded torch wheel (untrusted input)
SRC_DL="${SRC_DL:-$DATA/src-dl}"                      # downloaded cuRobo tarball (untrusted input)
SRC="${SRC:-$DATA/src}"                               # extracted cuRobo source tree (build dir)
CUDA_HOME_DIR="${CUDA_HOME_DIR:-$VENV/cuda-12.8}"     # assembled CUDA 12.8 build toolkit (build only)
CUDA_DL="${CUDA_DL:-$DATA/dl-cuda}"                   # downloaded CUDA redist archives (untrusted input)
LOGS="${LOGS:-$DATA/logs}"
PIP_INDEX="${PIP_INDEX:-https://pypi.org/simple}"
TORCH_WHEEL_BASE="${TORCH_WHEEL_BASE:-https://download.pytorch.org/whl/cu128}"  # or a mirror of it
HF_ENDPOINT="${HF_ENDPOINT:-https://huggingface.co}"  # or a mirror of it
UNSET_PROXY="${UNSET_PROXY:-0}"                       # 1: download without HTTP(S)_PROXY
CODELOAD="${CODELOAD:-https://codeload.github.com}"
CUDA_REDIST="${CUDA_REDIST:-https://developer.download.nvidia.com/compute/cuda/redist}"  # redirects to .cn in China
TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-}"      # empty = detect with nvidia-smi (e.g. RTX 4090: 8.9)
MAX_JOBS="${MAX_JOBS:-8}"
KEEP_ASSET_ZIPS="${KEEP_ASSET_ZIPS:-1}"               # 0 = delete the 15 GB of zips after a verified extraction
LOCK="${LOCK-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/robotwin-lock.txt}"  # tested pip freeze, used as
                                                      # constraints; LOCK= (empty) = only the core pins below

# ----------------------------------------------------------------------------- pinned versions / hashes
PY_MM="3.11"
TORCH_VER="2.7.1+cu128"
TORCH_WHL="torch-2.7.1+cu128-cp311-cp311-manylinux_2_28_x86_64.whl"
TORCH_SHA256="c301dc280458afd95450af794924c98fe07522dd148ff384739b810e3e3179f2"   # = download.pytorch.org index
RUNTIME_VER="0.1.1"
CUROBO_TAG="v0.7.8"                                   # commit d64c4b005459db10c5dd867d8b30a87d5bda9bdb
CUROBO_VER="0.7.8"
CUROBO_SHA256="702415790acf030aaf48c49f3381b8963d24b80bc4ab40d4a10808244ad5ee19"  # codeload tar.gz of the tag
declare -A CUDA_ARCHIVES=(                             # CUDA 12.8.1 (torch 2.7.1+cu128 is built with 12.8)
  [cuda_nvcc/linux-x86_64/cuda_nvcc-linux-x86_64-12.8.93-archive.tar.xz]=9961b3484b6b71314063709a4f9529654f96782ad39e72bf1e00f070db8210d3
  [cuda_cudart/linux-x86_64/cuda_cudart-linux-x86_64-12.8.90-archive.tar.xz]=8d566b5fe745c46842dc16945cf36686227536decd2302c372be86da37faca68
  [cuda_cccl/linux-x86_64/cuda_cccl-linux-x86_64-12.8.90-archive.tar.xz]=0740e9e01e4f15e17c5ab8d68bba4f8ec0eb6b84edccba4ac45112d2d2174e4b
)
ASSET_REPO="TianxingChen/RoboTwin2.0"
ASSET_REV="c15cc97be71e35244b6605d2d84c187f8565cc4d"  # = robotwin.assets.ASSET_REVISION in runtime 0.1.1
ASSET_ZIPS=(background_texture.zip embodiments.zip objects.zip)
declare -A ASSET_SHA256=(                              # = HF LFS oids at $ASSET_REV
  [background_texture.zip]=54ede0fb5b783e0faa2bc98720d3affd6ca3bb9280b225b48c1aafaf31473070
  [embodiments.zip]=6b87d7d55e106d8ff25917e0538eb1e177fc549280e8a742a8cec3cb9f953fc6
  [objects.zip]=6aa56b3cf1e1064f7c809308144da36b00815f8b137fef2d7e4de856f8becf27
)

NOPROXY=()
[ "$UNSET_PROXY" = 1 ] && NOPROXY=(env -u HTTPS_PROXY -u HTTP_PROXY -u https_proxy -u http_proxy)
PY="$VENV/bin/python"
PIP=("${NOPROXY[@]}" "$PY" -m pip --no-input --disable-pip-version-check)
CONSTRAINTS="$VENV/robotwin-constraints.txt"

log() { echo "[$(date +%T)] [$1] ${*:2}"; }
die() { echo "ERROR: $*" >&2; exit 1; }
sha_ok() { [ -f "$2" ] && echo "$1  $2" | sha256sum -c --status; }

write_constraints() {
  { printf 'torch==%s\nnumpy==1.26.4\nwarp-lang==1.11.1\nsapien==3.0.0b1\nmplib==0.2.1\n' "$TORCH_VER"
    if [ -n "$LOCK" ] && [ -f "$LOCK" ]; then
      grep -vE '^[[:space:]]*(#|$)' "$LOCK" | grep -viE '^(torch|numpy|warp-lang|sapien|mplib|nvidia[-_]curobo)==' || true
    fi
  } > "$CONSTRAINTS"
}

# ----------------------------------------------------------------------------- stages
stage_venv() {
  if [ -x "$PY" ] && "$PY" -c "import sys; sys.exit(sys.version[:4] != '$PY_MM')"; then
    log venv "exists: $VENV ($("$PY" -V))"; return
  fi
  [ -x "$BASE_PY" ] || die "base interpreter $BASE_PY not found"
  "$BASE_PY" -c "import sys, ssl, venv, ensurepip; sys.exit(sys.version[:4] != '$PY_MM')" || die "$BASE_PY is not a usable CPython $PY_MM"
  "$BASE_PY" -m venv "$VENV"
  log venv "created $VENV from $BASE_PY"
}

stage_torch() {
  if "$PY" -c "import importlib.metadata as m, sys; sys.exit(m.version('torch') != '$TORCH_VER')" 2>/dev/null; then
    log torch "torch $TORCH_VER present"; return
  fi
  mkdir -p "$WHEEL_DL"
  local whl="$WHEEL_DL/$TORCH_WHL"
  if ! sha_ok "$TORCH_SHA256" "$whl"; then
    "${NOPROXY[@]}" curl -L --fail --retry 5 --retry-delay 5 -C - -o "$whl" "$TORCH_WHEEL_BASE/${TORCH_WHL/+/%2B}" \
      || "${NOPROXY[@]}" curl -L --fail --retry 5 --retry-delay 5 -o "$whl" "$TORCH_WHEEL_BASE/${TORCH_WHL/+/%2B}"
    sha_ok "$TORCH_SHA256" "$whl" || { rm -f "$whl"; die "torch wheel sha256 mismatch (deleted, re-run)"; }
  fi
  "${PIP[@]}" install -i "$PIP_INDEX" "$whl"
  log torch "installed $("$PY" -c 'import torch; print(torch.__version__, torch.version.cuda)')"
}

stage_runtime() {
  write_constraints
  if "$PY" -c "import importlib.metadata as m, sys; sys.exit(m.version('rlinf-robotwin-runtime') != '$RUNTIME_VER')" 2>/dev/null; then
    log runtime "rlinf-robotwin-runtime $RUNTIME_VER present"; return
  fi
  "${PIP[@]}" install -i "$PIP_INDEX" -c "$CONSTRAINTS" "rlinf-robotwin-runtime==$RUNTIME_VER"
  log runtime "installed rlinf-robotwin-runtime $RUNTIME_VER"
}

stage_cudatk() {
  if [ -x "$CUDA_HOME_DIR/bin/nvcc" ] && "$CUDA_HOME_DIR/bin/nvcc" --version 2>/dev/null | grep -q "release 12.8" \
     && [ -e "$CUDA_HOME_DIR/include/cublas_v2.h" ] && [ -e "$CUDA_HOME_DIR/lib64/libcudart.so" ]; then
    log cudatk "present: $CUDA_HOME_DIR"; return
  fi
  # nvcc/cudart/cccl: NVIDIA's official CUDA 12.8.1 redistributable archives (the pip wheel
  # nvidia-cuda-nvcc-cu12 only ships ptxas, not the nvcc driver/cicc/cudafe++), sha256 from redistrib_12.8.1.json.
  # Headers + libs of cuBLAS/cuSPARSE/cuSOLVER/cuRAND/cuFFT/NVTX come from the nvidia-* wheels torch installed.
  rm -rf "$CUDA_HOME_DIR"
  mkdir -p "$CUDA_HOME_DIR" "$CUDA_DL"
  local rel f
  for rel in "${!CUDA_ARCHIVES[@]}"; do
    f="$CUDA_DL/$(basename "$rel")"
    if ! sha_ok "${CUDA_ARCHIVES[$rel]}" "$f"; then
      "${NOPROXY[@]}" curl -L --fail --retry 5 --retry-delay 5 -o "$f" "$CUDA_REDIST/$rel"
      sha_ok "${CUDA_ARCHIVES[$rel]}" "$f" || die "sha256 mismatch: $f"
    fi
    tar xJf "$f" -C "$CUDA_HOME_DIR" --strip-components=1 --no-same-owner --exclude='*/LICENSE'
  done
  ln -sfn lib "$CUDA_HOME_DIR/lib64"
  "$PY" - "$CUDA_HOME_DIR" <<'PY'
import os, sys, sysconfig
home = sys.argv[1]
nv = os.path.join(sysconfig.get_paths()["purelib"], "nvidia")
for pkg in ("cublas", "cusparse", "cusolver", "curand", "cufft", "nvtx", "nvjitlink"):
    for sub, dst in (("include", "include"), ("lib", "lib")):
        src = os.path.join(nv, pkg, sub)
        if not os.path.isdir(src):
            continue
        for name in sorted(os.listdir(src)):
            if name.startswith("__"):                 # python wrapper files of the wheels
                continue
            aliases = [name]
            if sub == "lib" and ".so." in name:       # also provide the unversioned link-time name
                aliases.append(name.split(".so.")[0] + ".so")
            for alias in aliases:
                d = os.path.join(home, dst, alias)
                if not os.path.lexists(d):
                    os.symlink(os.path.join(src, name), d)
need = ("bin/nvcc", "bin/ptxas", "bin/cudafe++", "bin/fatbinary", "nvvm/bin/cicc", "include/cuda_runtime.h",
        "include/crt/host_config.h", "include/thrust", "include/cub", "include/cublas_v2.h", "include/cusparse.h",
        "include/cusolverDn.h", "include/curand.h", "lib64/libcudart.so")
missing = [p for p in need if not os.path.exists(os.path.join(home, p))]
if missing:
    sys.exit(f"assembled CUDA toolkit is incomplete, missing: {missing}")
PY
  local t; t=$(mktemp -d)
  printf '__global__ void k(float *x) { x[threadIdx.x] *= 2.f; }\n' > "$t/t.cu"
  "$CUDA_HOME_DIR/bin/nvcc" -c "$t/t.cu" -o "$t/t.o" -gencode arch=compute_80,code=sm_80 || die "nvcc test compile failed"
  rm -rf "$t"
  log cudatk "ready: $("$CUDA_HOME_DIR/bin/nvcc" --version | tail -1) at $CUDA_HOME_DIR"
}

detect_arch() {
  if [ -z "$TORCH_CUDA_ARCH_LIST" ]; then
    TORCH_CUDA_ARCH_LIST=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | sort -u | paste -sd';')
    [ -n "$TORCH_CUDA_ARCH_LIST" ] || die "could not detect GPU compute capability; set TORCH_CUDA_ARCH_LIST"
  fi
  echo "$TORCH_CUDA_ARCH_LIST"
}

stage_curobo() {
  write_constraints
  if "$PY" -c "
import importlib.metadata as m, sys
assert m.version('nvidia_curobo') == '$CUROBO_VER'
import torch  # the compiled extensions resolve libc10/libtorch through the already-loaded torch
from curobo.curobolib import lbfgs_step_cu, kinematics_fused_cu, line_search_cu, tensor_step_cu, geom_cu
" 2>/dev/null; then
    log curobo "cuRobo $CUROBO_VER with compiled CUDA extensions present"; return
  fi
  [ -x "$CUDA_HOME_DIR/bin/nvcc" ] || die "run the cudatk stage first"
  mkdir -p "$SRC_DL" "$SRC" "$LOGS"
  local tgz="$SRC_DL/curobo-$CUROBO_TAG.tar.gz"
  if ! sha_ok "$CUROBO_SHA256" "$tgz"; then
    "${NOPROXY[@]}" curl -L --fail --retry 5 --retry-delay 5 -o "$tgz" "$CODELOAD/NVlabs/curobo/tar.gz/refs/tags/$CUROBO_TAG"
    sha_ok "$CUROBO_SHA256" "$tgz" || die "cuRobo tarball sha256 mismatch: $tgz"
  fi
  local tree="$SRC/curobo-$CUROBO_VER"
  rm -rf "$tree"
  tar xzf "$tgz" -C "$SRC" --no-same-owner
  [ -f "$tree/setup.py" ] || die "unexpected tarball layout (no $tree/setup.py)"
  "${PIP[@]}" install -i "$PIP_INDEX" -c "$CONSTRAINTS" ninja "setuptools_scm>=6.2" wheel
  local arch; arch=$(detect_arch)
  log curobo "building cuRobo $CUROBO_TAG for TORCH_CUDA_ARCH_LIST=$arch (log: $LOGS/curobo_build.log)"
  # non-editable install: compiled extensions + package data are copied into site-packages
  ( cd "$tree" && CUDA_HOME="$CUDA_HOME_DIR" PATH="$CUDA_HOME_DIR/bin:$VENV/bin:$PATH" TORCH_CUDA_ARCH_LIST="$arch" \
      MAX_JOBS="$MAX_JOBS" SETUPTOOLS_SCM_PRETEND_VERSION="$CUROBO_VER" \
      "${PIP[@]}" install -v -i "$PIP_INDEX" -c "$CONSTRAINTS" --no-build-isolation . ) > "$LOGS/curobo_build.log" 2>&1 \
    || { tail -40 "$LOGS/curobo_build.log"; die "cuRobo build failed (see $LOGS/curobo_build.log)"; }
  "$PY" -c "import torch, curobo; from curobo.curobolib import geom_cu, kinematics_fused_cu; print('curobo', curobo.__version__)"
}

assets_valid() {   # managed snapshot with the expected revision and archive hashes
  "$PY" -I - "$ASSETS_ROOT" "$ASSET_REV" "${ASSET_SHA256[background_texture.zip]}" "${ASSET_SHA256[embodiments.zip]}" \
      "${ASSET_SHA256[objects.zip]}" <<'PY' 2>/dev/null
import json, os, sys
root, rev, bg, emb, obj = sys.argv[1:]
for d in ("assets/background_texture", "assets/embodiments/aloha-agilex", "assets/objects"):
    if not os.path.isdir(os.path.join(root, d)):
        sys.exit(1)
m = json.load(open(os.path.join(root, ".robotwin-assets.json"), encoding="utf-8"))
want = {"background_texture.zip": bg, "embodiments.zip": emb, "objects.zip": obj}
sys.exit(0 if m.get("revision") == rev and m.get("archives") == want else 1)
PY
}

stage_assets() {
  if assets_valid; then log assets "valid snapshot at $ASSETS_ROOT"; return; fi
  if [ -e "$ASSETS_ROOT" ] && [ -n "$(ls -A "$ASSETS_ROOT" 2>/dev/null)" ]; then
    die "$ASSETS_ROOT exists but is not a valid snapshot of $ASSET_REV; move it away first"
  fi
  mkdir -p "$ASSET_DL" "$LOGS"
  local f
  for f in "${ASSET_ZIPS[@]}"; do
    if sha_ok "${ASSET_SHA256[$f]}" "$ASSET_DL/$f"; then log assets "$f verified"; continue; fi
    log assets "downloading $f from $HF_ENDPOINT (resumable)"
    local try
    for try in 1 2 3 4 5 6 7 8 9 10; do
      "${NOPROXY[@]}" curl -L -sS --fail --retry 5 --retry-delay 5 -C - -o "$ASSET_DL/$f" \
        "$HF_ENDPOINT/datasets/$ASSET_REPO/resolve/$ASSET_REV/$f" && break
      log assets "$f attempt $try failed, retrying"; sleep 10
    done
    sha_ok "${ASSET_SHA256[$f]}" "$ASSET_DL/$f" || { mv -f "$ASSET_DL/$f" "$ASSET_DL/$f.bad"; die "$f sha256 mismatch (moved to $f.bad)"; }
    log assets "$f verified"
  done
  # Safe extraction (same checks + layout + manifest as robotwin.assets.download_assets) into a staging dir,
  # then an atomic rename. `python -I`, cwd=/ : nothing from the download dir is importable.
  local staging="$DATA/.assets-staging"
  rm -rf "$staging"
  ( cd / && "$PY" -I - "$ASSET_DL" "$staging" "$ASSETS_ROOT" "$ASSET_REPO" "$ASSET_REV" \
      "${ASSET_SHA256[background_texture.zip]}" "${ASSET_SHA256[embodiments.zip]}" "${ASSET_SHA256[objects.zip]}" <<'PY'
import json, os, sys, zipfile
from pathlib import Path
dl, staging, root, repo, rev, bg, emb, obj = sys.argv[1:]
dl, staging, root = Path(dl), Path(staging), Path(root)
hashes = {"background_texture.zip": bg, "embodiments.zip": emb, "objects.zip": obj}
assets = staging / "assets"
assets.mkdir(parents=True)
dest = assets.resolve()
for name in sorted(hashes):          # same order as robotwin.assets.ASSET_ARCHIVES
    with zipfile.ZipFile(dl / name) as zf:
        for info in zf.infolist():
            target = (assets / info.filename).resolve()
            if target != dest and dest not in target.parents:
                sys.exit(f"unsafe archive member in {name}: {info.filename}")
            if (info.external_attr >> 16) & 0o170000 == 0o120000:
                sys.exit(f"symlink member in {name}: {info.filename}")
        zf.extractall(assets)
    print("extracted", name, flush=True)
manifest = {"repository": repo, "revision": rev, "archives": hashes}
(staging / ".robotwin-assets.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
for d in ("assets/background_texture", "assets/embodiments/aloha-agilex", "assets/objects"):
    if not (staging / d).is_dir():
        sys.exit(f"extracted snapshot lacks {d}")
if root.exists():
    root.rmdir()                     # only an empty directory can be here (checked by the caller)
os.replace(staging, root)
print("assets ready at", root)
PY
  )
  assets_valid || die "asset root failed validation after extraction"
  if [ "$KEEP_ASSET_ZIPS" = "0" ]; then
    for f in "${ASSET_ZIPS[@]}"; do rm -f "$ASSET_DL/$f"; done
  fi
  log assets "valid snapshot at $ASSETS_ROOT"
}

stage_check() {
  local icd=""
  for c in /etc/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json; do [ -f "$c" ] && { icd=$c; break; }; done
  [ -n "$icd" ] || log check "WARNING: no NVIDIA Vulkan ICD json found; SAPIEN rendering will fail"
  ( cd / && ROBOTWIN_ASSETS_ROOT="$ASSETS_ROOT" VK_ICD_FILENAMES="$icd" "$PY" - <<'PY'
import importlib.metadata as m, os
import torch, numpy, sapien, mplib, warp, toppra, transforms3d, curobo
from curobo.curobolib import lbfgs_step_cu, kinematics_fused_cu, line_search_cu, tensor_step_cu, geom_cu
from robotwin.assets import validate_root
from robotwin.envs.robot.planner import CuroboPlanner
assert CuroboPlanner is not None, "robotwin could not import cuRobo"
for p in ("torch", "numpy", "sapien", "mplib", "warp-lang", "nvidia_curobo", "rlinf-robotwin-runtime", "toppra", "open3d"):
    print(f"{p:24s} {m.version(p)}")
print("torch cuda", torch.version.cuda, "available:", torch.cuda.is_available())
print("assets", validate_root(os.environ["ROBOTWIN_ASSETS_ROOT"]))
print("vulkan icd", os.environ.get("VK_ICD_FILENAMES"))
PY
  )
}

stage_env() {
  local icd=/etc/vulkan/icd.d/nvidia_icd.json
  for c in /etc/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json; do [ -f "$c" ] && { icd=$c; break; }; done
  mkdir -p "$DATA"
  cat > "$DATA/env.sh.new" <<EOF
# Environment for the RoboTwin 2.0 simulator venv ($VENV), read by hexashell.benches.run
# (\$HEXASHELL_ROBOTWIN_ENV): export lines with literal values. Generated by setup_robotwin.sh.
export ROBOTWIN_ASSETS_ROOT=$ASSETS_ROOT
export ROBOTWIN_ASSETS_PATH=$ASSETS_ROOT
export ASSETS_PATH=$ASSETS_ROOT
export VK_ICD_FILENAMES=$icd
export OMP_NUM_THREADS=2
export MKL_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
EOF
  mv "$DATA/env.sh.new" "$DATA/env.sh"
  log env "wrote $DATA/env.sh"
}

ALL=(venv torch runtime cudatk curobo assets check env)
STAGES=("$@"); [ ${#STAGES[@]} -gt 0 ] || STAGES=("${ALL[@]}")
for s in "${STAGES[@]}"; do
  case "$s" in
    venv|torch|runtime|cudatk|curobo|assets|check|env) "stage_$s" ;;
    *) die "unknown stage '$s' (stages: ${ALL[*]})" ;;
  esac
done
log done "stages: ${STAGES[*]}"
