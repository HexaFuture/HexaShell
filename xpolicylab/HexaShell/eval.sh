#!/bin/bash
# One evaluation on this machine: the policy server in the background, then the simulator client.
set -euo pipefail

bench_name=${1}
task_name=${2}
ckpt_name=${3}
env_cfg_type=${4}
action_type=${5}
seed=${6}
policy_gpu_id=${7}
env_gpu_id=${8}
policy_conda_env=${9}
eval_env_conda_env=${10}

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
utils="$(cd "${here}/../.." && pwd)/utils"
port=$(bash "${utils}/get_free_port.sh")

stop_server() {
    if [[ -n "${server_pid:-}" ]]; then
        kill "${server_pid}" 2>/dev/null || true
    fi
}
trap stop_server EXIT

(cd "${here}" && exec bash setup_eval_policy_server.sh "${bench_name}" "${task_name}" "${ckpt_name}" \
    "${env_cfg_type}" "${action_type}" "${seed}" "${policy_gpu_id}" "${policy_conda_env}" "${port}") &
server_pid=$!

bash "${utils}/wait_for_policy_server.sh" localhost "${port}" "${server_pid}" "Policy server" 600

bash "${here}/setup_eval_env_client.sh" "${bench_name}" "${task_name}" "${ckpt_name}" "${env_cfg_type}" \
    "${action_type}" "${seed}" "${env_gpu_id}" "${eval_env_conda_env}" \
    "ckpt_name=${ckpt_name},action_type=${action_type}" "${port}" localhost
