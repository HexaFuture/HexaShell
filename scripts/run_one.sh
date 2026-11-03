#!/bin/bash
# One HexaShell evaluation run on this machine (RoboDojo's own client and policy server).
#   bash scripts/run_one.sh TASK SEED LAYOUTS GPU [EVAL_NUM] [NAME]
# LAYOUTS: e.g. "0" or "0,1" (exact layout ids; needs scripts/patches/robodojo-layout-ids.patch in RoboDojo).
# GPU: the simulator's GPU (the policy server needs none).  HEXASHELL_* variables override deploy.yml
# (e.g. HEXASHELL_MAX_TURNS=6).
# The log goes to logs/NAME.log; per-episode records to results/ (or $HEXASHELL_RESULT_DIR).
set -euo pipefail
task=$1; seed=$2; layouts=$3; gpu=$4; eval_num=${5:-1}; name=${6:-${task}_s${seed}_l${layouts//,/-}_$(date +%m%d_%H%M%S)}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file=${HEXASHELL_ENV:-$here/config/env.sh}   # machine settings: see config/env.example.sh
[ -f "$env_file" ] || { echo "no machine settings: copy config/env.example.sh to config/env.sh (or set HEXASHELL_ENV)" >&2; exit 2; }
source "$env_file"
mkdir -p "$here/logs"
log="$here/logs/$name.log"
cd "$ROBODOJO_ROOT"
echo "[run_one] $name -> $log"
ROBODOJO_LAYOUT_IDS=$layouts bash scripts/robodojo.sh eval --policy-dir XPolicyLab/policy/HexaShell \
    --task "$task" --ckpt no-checkpoint --policy-env "$POLICY_ENV" --eval-env "$EVAL_ENV" --env-cfg arx_x5 \
    --action-type joint --seed "$seed" --policy-gpu "$gpu" --env-gpu "$gpu" --eval-num "$eval_num" \
    > "$log" 2>&1
