#!/bin/bash
# Simulator client of HexaShell: RoboDojo's own client with this policy's deploy.py episode loop.
set -euo pipefail

bench_name=${1}
task_name=${2}
ckpt_name=${3}
env_cfg_type=${4}
action_type=${5}
seed=${6}
env_gpu_id=${7}
eval_env_conda_env=${8}
additional_info=${9}
policy_server_port=${10}
policy_server_ip=${11:-"localhost"}

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
xpl_root="$(cd "${here}/../.." && pwd)"
bench_root="$(cd "${xpl_root}/.." && pwd)"
policy_name="$(basename "${here}")"

echo "[HexaShell client] task=${task_name} seed=${seed} server=${policy_server_ip}:${policy_server_port}"

bash "${xpl_root}/utils/setup_env_client.sh" \
    "${xpl_root}/utils" \
    "${here}/deploy.yml" \
    "${eval_env_conda_env}" \
    "${policy_server_port}" \
    "${bench_name}" \
    "${task_name}" \
    "${env_cfg_type}" \
    "${policy_name}" \
    "${additional_info}" \
    "${bench_root}" \
    "${seed}" \
    "${env_gpu_id}" \
    "${policy_server_ip}"
