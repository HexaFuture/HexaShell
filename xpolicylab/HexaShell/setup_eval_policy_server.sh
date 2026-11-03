#!/bin/bash
# Policy server of HexaShell (XPolicyLab ws).  Run by RoboDojo with this directory as CWD.
set -euo pipefail

bench_name=${1}
task_name=${2}
ckpt_name=${3}
env_cfg_type=${4}
action_type=${5}
seed=${6}
policy_gpu_id=${7}
policy_conda_env=${8}
policy_server_port=${9}
policy_server_host=${10:-"localhost"}

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
xpl_root="$(cd "${here}/../.." && pwd)"
bench_root="$(cd "${xpl_root}/.." && pwd)"
policy_name="$(basename "${here}")"

echo "[HexaShell server] task=${task_name} seed=${seed} port=${policy_server_port}"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "${policy_conda_env}"

# the policy itself needs no GPU (SAM3 is a shared service); the variable only keeps libraries
# that look for one on the GPU this run was given
exec env \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH="${bench_root}${PYTHONPATH:+:${PYTHONPATH}}" \
    CUDA_VISIBLE_DEVICES="${policy_gpu_id}" \
    python "${xpl_root}/setup_policy_server.py" \
        --config_path "${here}/deploy.yml" \
        --overrides \
            port="${policy_server_port}" \
            host="${policy_server_host}" \
            bench_name="${bench_name}" \
            task_name="${task_name}" \
            ckpt_name="${ckpt_name}" \
            env_cfg_type="${env_cfg_type}" \
            seed="${seed}" \
            policy_name="${policy_name}" \
            action_type="${action_type}" \
            gpu_id="${policy_gpu_id}"
