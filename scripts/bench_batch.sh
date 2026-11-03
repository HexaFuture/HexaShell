#!/usr/bin/env bash
# Work through a list of benchmark episodes (python -m hexashell.benches.run) on this machine.
#   bash scripts/bench_batch.sh NAME [MAX_RUNNING]
# List: logs/NAME.todo, one episode per line: "BENCH TASK SEED [ATTEMPT]" (e.g. "robocasa CloseFridge 20").
# Lines may be added or removed while it runs.  Records: results/NAME/<TASK>_s<SEED>-<time>/ (the simulator
# server's log is server.log in there); each launch's output: logs/NAME/<tag>.log; one line per launch and
# exit: logs/NAME.events.  HEXASHELL_* variables pass through to the episodes; SETS adds --set NAME=VALUE
# words to every launch (e.g. SETS="sam3_url=http://127.0.0.1:18301 skills=yes").
# While it runs: a number in logs/NAME.max replaces MAX_RUNNING; while logs/NAME.pause exists nothing new
# starts.  An episode whose record has no result.json (killed), ends in "error" or "gateway_error", or whose
# simulator failed, is listed again, up to ATTEMPTS times.  A runner started again under the same NAME does
# not take over the episodes the earlier one launched: they finish on their own, without retries.
# The simulator renders on the allowed GPU (GPUS) with the most free memory, at least NEED MiB; a launch
# younger than SETTLE seconds counts as holding HOLD MiB there.
# Machine settings: $HEXASHELL_ENV or config/env.sh (HEXASHELL_PYTHON, HEXASHELL_<BENCH>_PYTHON and
# HEXASHELL_<BENCH>_ENV; see config/env.example.sh).
# scripts/bench_pool.py works through the same list with warm simulator servers (no start-up per episode).
set -uo pipefail
name=$1 max_running=${2:-4}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file=${HEXASHELL_ENV:-$here/config/env.sh}
[ -f "$env_file" ] || { echo "no machine settings: copy config/env.example.sh to config/env.sh (or set HEXASHELL_ENV)" >&2; exit 2; }
source "$env_file"
: "${HEXASHELL_PYTHON:?set HEXASHELL_PYTHON in $env_file (the python of the policy environment)}"
GPUS=${GPUS:-$(nvidia-smi --query-gpu=index --format=csv,noheader | paste -sd, -)}   # the GPUs we may use
NEED=${NEED:-3000} HOLD=${HOLD:-2000} SETTLE=${SETTLE:-120} SPACING=${SPACING:-20} ATTEMPTS=${ATTEMPTS:-3}
SETS=${SETS:-}
todo="$here/logs/$name.todo" events="$here/logs/$name.events" logdir="$here/logs/$name"
results="$here/results/$name"
mkdir -p "$logdir" "$results"; touch "$todo" "$events"
exec 9>"$here/logs/$name.runner.lock"
flock -n 9 || { echo "another runner of $name is running here" >&2; exit 1; }
cd "$here" || exit 1
declare -A started=() gpu_of=() line_of=() tag_of=()
note() { echo "$(date '+%F %T') $*" | tee -a "$events"; }

take() {    # remove and print the first episode line
    local line; exec 8>"$todo.lock"; flock 8
    line=$(grep -m1 -v '^[[:space:]]*$' "$todo")
    [ -n "$line" ] && awk -v l="$line" '!gone && $0 == l {gone = 1; next} {print}' "$todo" > "$todo.new" && mv "$todo.new" "$todo"
    flock -u 8; echo "$line"
}
add() { exec 8>"$todo.lock"; flock 8; echo "$1" >> "$todo"; flock -u 8; }

outcome() { # "ok", or why the episode should be played again: its newest record since the launch
    local task=$1 seed=$2 since=$3 rec
    rec=$(find "$results" -maxdepth 1 -type d -name "${task}_s${seed}-*" -newermt "@$since" 2>/dev/null | sort | tail -n 1)
    [ -z "$rec" ] && { echo "no record"; return; }
    [ -f "$rec/result.json" ] || { echo "no result.json"; return; }
    "$HEXASHELL_PYTHON" -I -c '
import json, sys
r = json.load(open(sys.argv[1]))
v = r.get("verdict") or {}
print("simulator failed" if v.get("failure") else r.get("end") if r.get("end") in ("error", "gateway_error") else "ok")
' "$rec/result.json" 2>/dev/null || echo "unreadable result.json"
}

choose() {  # the allowed GPU with the most memory left after young launches, if it has NEED
    local now best="" most=0 g f p; now=$(date +%s)
    while read -r g f; do
        [[ ",$GPUS," == *",$g,"* ]] || continue
        for p in "${!started[@]}"; do
            [ "${gpu_of[$p]}" = "$g" ] && (( now - started[$p] < SETTLE )) && f=$((f - HOLD))
        done
        (( f > most )) && { most=$f; best=$g; }
    done < <(nvidia-smi --query-gpu=index,memory.free --format=csv,noheader,nounits | tr -d ' ' | tr ',' ' ')
    (( most >= NEED )) && echo "$best"
}

cap() { local m=""; [ -r "$here/logs/$name.max" ] && m=$(tr -dc 0-9 < "$here/logs/$name.max"); echo "${m:-$max_running}"; }

sets=(); for s in $SETS; do sets+=(--set "$s"); done
last=0
note "begin $name max_running=$max_running gpus=$GPUS host=$(hostname) sets=[$SETS]"
while :; do
    for p in "${!started[@]}"; do
        kill -0 "$p" 2>/dev/null && continue
        wait "$p"; code=$?
        read -r bench task seed attempt <<<"${line_of[$p]}"
        why=$(outcome "$task" "$seed" "${started[$p]}")
        note "exit ${tag_of[$p]} gpu=${gpu_of[$p]} code=$code after $(( $(date +%s) - started[$p] ))s: $why"
        if [ "$why" != ok ]; then
            if (( attempt < ATTEMPTS )); then add "$bench $task $seed $((attempt + 1))"; note "listed ${tag_of[$p]} again"
            else note "given up on $bench $task s$seed after $attempt attempts"; fi
        fi
        unset "started[$p]" "gpu_of[$p]" "line_of[$p]" "tag_of[$p]"
    done
    running=${#started[@]}
    if ! grep -q -v '^[[:space:]]*$' "$todo"; then (( running == 0 )) && break; sleep 30 9>&-; continue; fi
    if (( running < $(cap) )) && [ ! -e "$here/logs/$name.pause" ] && (( $(date +%s) - last >= SPACING )) \
        && gpu=$(choose) && [ -n "$gpu" ]; then
        line=$(take); [ -z "$line" ] && continue
        read -r bench task seed attempt <<<"$line"; attempt=${attempt:-1}
        if [ -z "$seed" ] || ! [[ $seed =~ ^[0-9]+$ ]]; then note "skipped a bad line: $line"; continue; fi
        tag=${bench}_${task}_s${seed}; (( attempt > 1 )) && tag=${tag}_a$attempt
        nohup "$HEXASHELL_PYTHON" -m hexashell.benches.run --bench "$bench" --task "$task" --seed "$seed" \
            --out "$results" --egl "$gpu" "${sets[@]}" > "$logdir/$tag.log" 2>&1 < /dev/null 9>&- &
        p=$!; started[$p]=$(date +%s); last=${started[$p]}
        gpu_of[$p]=$gpu line_of[$p]="$bench $task $seed $attempt" tag_of[$p]=$tag
        note "launch $tag pid=$p gpu=$gpu running=$((running + 1))"
    fi
    sleep 10 9>&-   # children must not keep the runner lock
done
note "end $name"
