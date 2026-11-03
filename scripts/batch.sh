#!/usr/bin/env bash
# Work through a list of HexaShell episodes on this machine's free GPU memory, beside other users' jobs.
#   bash scripts/batch.sh NAME [MAX_RUNNING]
#   bash scripts/batch.sh --gpus     (what each GPU holds as counted below, and where the next launch would go)
# List: logs/NAME.todo, one episode per line: "TASK SEED LAYOUT [ATTEMPT]".  Lines may be added or removed
# while it runs.  Records: results/NAME/<episode>/ (HEXASHELL_RESULT_DIR), simulator logs: logs/NAME/<episode>.log,
# one line per launch and exit: logs/NAME.events.  HEXASHELL_* variables pass through to the policy.
# POLICY names the policy folder under RoboDojo's XPolicyLab/policy (default HexaShell, this checkout);
# a folder linked to another copy of the code runs that copy.
# While it runs: a number in logs/NAME.max replaces MAX_RUNNING; while logs/NAME.pause exists nothing new starts.
# Started again under the same NAME (after the earlier runner was stopped), it takes over the episodes that
# runner launched and that still run.
# An episode that never started (no record folder: a simulator crash at boot; stuck on an error dialog) or
# that the model gateway cut short (its record ends with "gateway_error") is listed again, up to ATTEMPTS
# times.  A layout RoboDojo drops as PhysX-unstable is replaced by layout + SHIFT (never 10..19: the test
# layouts).  The policy needs no GPU.  A RoboDojo simulator shows nothing on its GPU for a minute or so after
# its launch and then grows to 7-9.5 GB, so every RoboDojo episode on the machine, anyone's, counts from its
# launch on (a scripts/robodojo.sh eval process with its --env-gpu, or a src/eval_client/main.py with its
# --device_id) as holding what its simulator shows or SIM, whichever is more; any other python process on a GPU
# younger than RAMP seconds as holding OTHER_FULL until it shows more.  A GPU gets a launch when NEED is left
# after that: two simulators on a 24 GB card, also beside a 3 GB SAM3.  The runners of every user and checkout on
# the machine choose a GPU and launch one at a time (GLOCK: a file in /tmp anyone may lock), so two of them never
# take the same memory at once.
# Beside another runner on this machine whose launches take memory the count above misses (a policy on a GPU,
# e.g. a sweep runner):
# - OTHERS names its status logs (comma-separated). Its launches younger than OTHER_SETTLE seconds count as
#   holding memory too, on top of the count above: OTHER_POLICY_HOLD on the policy GPU and HOLD on the simulator
#   GPU (a sweep runner's lines), or HOLD on the one GPU of a batch.sh line ("launch TAG pid=P gpu=G ...").
# - THROTTLE names its max file. Before each launch of ours it is set to 0 for WINDOW seconds, so the other
#   runner starts nothing on memory our simulator is still taking. logs/NAME.throttle keeps the value to put
#   back, and a detached timer puts it back even if this runner stops.
# MAX_PER_GPU (0: no limit): a GPU that holds this many RoboDojo episodes (anyone's, counted as above) gets no launch
# of ours.
set -uo pipefail
name=$1 max_running=${2:-6}
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file=${HEXASHELL_ENV:-$here/config/env.sh}   # machine settings: see config/env.example.sh
[ -f "$env_file" ] || { echo "no machine settings: copy config/env.example.sh to config/env.sh (or set HEXASHELL_ENV)" >&2; exit 2; }
source "$env_file"
GPUS=${GPUS:-$(nvidia-smi --query-gpu=index --format=csv,noheader | paste -sd, -)}   # the GPUs we may use
NEED=${NEED:-10000} SIM=${SIM:-9500}        # MiB left to launch; MiB a simulator counts as until it shows more
SPACING=${SPACING:-45} ATTEMPTS=${ATTEMPTS:-3} SHIFT=${SHIFT:-20} QUIET=${QUIET:-1800}
FRESH=${FRESH:-1200}                        # an episode whose record changed this recently is not stuck
POLL=${POLL:-15}                            # seconds between rounds (exits, stuck checks, launches)
POLICY=${POLICY:-HexaShell}
OTHERS=${OTHERS:-} OTHER_SETTLE=${OTHER_SETTLE:-180} OTHER_POLICY_HOLD=${OTHER_POLICY_HOLD:-7500} HOLD=${HOLD:-8500}
RAMP=${RAMP:-1500} OTHER_FULL=${OTHER_FULL:-7600}   # young GPU processes other than simulators, still growing
GLOCK=${GLOCK:-/tmp/robodojo-batch.lock}            # one GPU choice at a time on this machine, every user's runners
THROTTLE=${THROTTLE:-} WINDOW=${WINDOW:-120}
MAX_PER_GPU=${MAX_PER_GPU:-0}
absolute() { local f out=""; for f in ${1//,/ }; do [[ $f == /* ]] || f="$PWD/$f"; out+="${out:+,}$f"; done; echo "$out"; }
OTHERS=$(absolute "$OTHERS") THROTTLE=$(absolute "$THROTTLE")   # relative to where it was started (it cd's below)

episodes() {  # "gpu sim root user seconds task" for each RoboDojo episode on this machine, anyone's: a scripts/robodojo.sh
              # eval (root, on its --env-gpu) and the src/eval_client/main.py under it (sim: "-" until it starts), or a
              # main.py alone (root "-", on its --device_id); not their subshells
    ps -ww -eo pid=,ppid=,user:32=,etimes=,args= | awk '
        function top(p, set,   k, t) { t = ""; for (k = 0; k < 20 && p + 0 > 1; k++) { p = up[p]; if (p in set) t = p }; return t }
        { up[$1] = $2; who[$1] = $3 " " $4
          if ($0 ~ /scripts\/robodojo\.sh eval /) {
              for (i = 5; i < NF; i++) { if ($i == "--env-gpu") root[$1] = $(i + 1); if ($i == "--task") task[$1] = $(i + 1) } }
          else if ($0 ~ /src\/eval_client\/main\.py/) {
              for (i = 5; i < NF; i++) { if ($i == "--device_id") sim[$1] = $(i + 1); if ($i == "--task_name") task[$1] = $(i + 1) } } }
        END { for (s in sim) if (top(s, sim) == "") { r = top(s, root); if (r == "") print sim[s], s, "-", who[s], task[s] "?"; else of[r] = s }
              for (r in root) if (top(r, root) == "") print root[r], (r in of ? of[r] : "-"), r, who[r], task[r] "?" }' | sed 's/?$//'
}

ledger() {  # per GPU "gpu total used episodes counted": counted = what is not free now, plus for each RoboDojo episode
            # what its simulator may still take up to SIM, plus for other python processes younger than RAMP up to OTHER_FULL
    local i u t m f p g s age comm
    local -A at=() tot=() used=() of=() eps=() cnt=()
    while IFS=', ' read -r i u t m f; do at[$u]=$i tot[$i]=$t used[$i]=$m cnt[$i]=$(( t - f )) eps[$i]=0; done \
        < <(nvidia-smi --query-gpu=index,uuid,memory.total,memory.used,memory.free --format=csv,noheader,nounits)
    while IFS=', ' read -r p m u; do [ -n "${at[$u]:-}" ] && [[ $m =~ ^[0-9]+$ ]] && of[$p]="${at[$u]} $m"; done \
        < <(nvidia-smi --query-compute-apps=pid,used_memory,gpu_uuid --format=csv,noheader,nounits)
    while read -r g s _; do   # (on the GPU its simulator is on, once it is there)
        m=0; [ -n "${of[$s]:-}" ] && { read -r g m <<<"${of[$s]}"; unset "of[$s]"; }
        [ -n "${cnt[$g]:-}" ] || continue
        eps[$g]=$(( eps[$g] + 1 )); (( m < SIM )) && cnt[$g]=$(( cnt[$g] + SIM - m ))
    done < <(episodes)
    for p in "${!of[@]}"; do
        read -r g m <<<"${of[$p]}"
        read -r age comm < <(ps -o etimes=,comm= -p "$p" 2>/dev/null) || continue
        [[ $comm == python* && $age =~ ^[0-9]+$ ]] && (( age < RAMP && m < OTHER_FULL )) && cnt[$g]=$(( cnt[$g] + OTHER_FULL - m ))
    done
    for g in "${!tot[@]}"; do echo "$g ${tot[$g]} ${used[$g]} ${eps[$g]} ${cnt[$g]}"; done | sort -n
}

if [ "$name" = --gpus ]; then
    echo "MiB; counted = not free + what each RoboDojo episode may still take up to SIM=$SIM + young python up to OTHER_FULL=$OTHER_FULL"
    ledger | awk -v need="$NEED" -v gpus=",$GPUS," 'BEGIN { printf "%3s %6s %6s %8s %7s %6s\n", "gpu", "total", "used", "episodes", "counted", "left" }
        { printf "%3s %6s %6s %8s %7s %6s  %s\n", $1, $2, $3, $4, $5, $2 - $5,
          (index(gpus, "," $1 ",") ? ($2 - $5 >= need ? "launch ok (NEED=" need ")" : "") : "(not in GPUS)") }'
    declare -A mib=()
    while IFS=', ' read -r p m; do mib[$p]=$m; done < <(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits)
    echo "RoboDojo episodes:"; printf "%3s %-16s %7s %6s  %s\n" gpu user seconds MiB task
    episodes | sort -n -k1,1 -k5,5nr | while read -r g s r user age task; do
        printf "%3s %-16s %7s %6s  %s\n" "$g" "$user" "$age" "${mib[$s]:--}" "$task"; done
    exit 0
fi
[ -n "$THROTTLE" ] && (( SPACING < WINDOW + 15 )) && SPACING=$((WINDOW + 15))   # one window at a time
todo="$here/logs/$name.todo" events="$here/logs/$name.events" logdir="$here/logs/$name"
marker="$here/logs/$name.throttle"
export HEXASHELL_RESULT_DIR="$here/results/$name"
mkdir -p "$logdir" "$HEXASHELL_RESULT_DIR"; touch "$todo" "$events"
exec 9>"$here/logs/$name.runner.lock"
flock -n 9 || { echo "another runner of $name is running here" >&2; exit 1; }
exec 6>"$here/logs/launch.lock"     # held from choosing a GPU until the launch is in the events
[ -e "$GLOCK" ] || (umask 022; : >> "$GLOCK") 2>/dev/null   # read-only use: in /tmp a file of another user's
if { exec 5<"$GLOCK"; } 2>/dev/null; then glock=1; else exec 5</dev/null; glock=0; fi   # cannot be opened to write
cd "$ROBODOJO_ROOT" || exit 1
declare -A started=() gpu_of=() line_of=() log_of=() tag_of=() adopted=()
note() { echo "$(date '+%F %T') $*" | tee -a "$events"; }

take() {    # remove and print the first episode line
    local line; exec 8>"$todo.lock"; flock 8
    line=$(grep -m1 -v '^[[:space:]]*$' "$todo")
    [ -n "$line" ] && awk -v l="$line" '!gone && $0 == l {gone = 1; next} {print}' "$todo" > "$todo.new" && mv "$todo.new" "$todo"
    flock -u 8; echo "$line"
}
add() { exec 8>"$todo.lock"; flock 8; echo "$1" >> "$todo"; flock -u 8; }

again() {   # list an episode once more, unless it has had its attempts
    local task seed layout attempt
    read -r task seed layout attempt <<<"$1"
    if (( attempt < ATTEMPTS )); then add "$task $seed $layout $((attempt + 1))"; note "$2 $task s$seed l$layout: attempt $((attempt + 1)) listed"
    else note "$2 $task s$seed l$layout: given up after $attempt attempts"; fi
}

record() {  # the policy's record folder of a running or finished episode (newest after its launch)
    local task seed layout attempt
    read -r task seed layout attempt <<<"${line_of[$1]}"
    find "$HEXASHELL_RESULT_DIR" -maxdepth 1 -type d -name "${task}_s${seed}_l${layout}-*" -newermt "@${started[$1]}" \
        2>/dev/null | sort | tail -n 1
}

retry() {   # after an episode's simulator exits: list it again when no episode was played, or the gateway failed
    local task seed layout attempt lg=$2 since=$3 rec dir
    read -r task seed layout attempt <<<"$1"; attempt=${attempt:-1}
    rec=$(find "$HEXASHELL_RESULT_DIR" -maxdepth 2 -name result.json -path "*/${task}_s${seed}_l${layout}-*" \
        -newermt "@$since" 2>/dev/null | head -n 1)
    dir=$(find "$HEXASHELL_RESULT_DIR" -maxdepth 1 -type d -name "${task}_s${seed}_l${layout}-*" \
        -newermt "@$since" 2>/dev/null | head -n 1)
    if grep -aq "Unstable nums: 1" "$lg"; then
        local next=$((layout + SHIFT)); (( next >= 10 && next <= 19 )) && next=$((next + 10))
        add "$task $seed $next 1"; note "unstable $task s$seed l$layout -> layout $next"
    elif [ -z "$dir" ] || grep -aq -e "status boot_error" -e "Success nums: 0, Fail nums: 0, Unstable nums: 0" \
        -e "^\[batch\] stuck" "$lg"; then
        again "$task $seed $layout $attempt" "no episode"
    elif [ -n "$rec" ] && grep -q '"end": "gateway_error"' "$rec"; then
        again "$task $seed $layout $attempt" "gateway error"
    fi
}

family() { local c; for c in $(ps -o pid= --ppid "$1"); do family "$c"; done; echo "$1"; }
stuck() {   # an error dialog retried every 2 min with nothing else logged, or a silent log - unless the
            # policy still writes its record (it may be waiting for the model, which the simulator log does not show)
    local p=$1 lg=${log_of[$1]} why="" rec
    rec=$(record "$p")
    [ -n "$rec" ] && [ -n "$(find "$rec" -maxdepth 1 -name events.jsonl -newermt "@$(( $(date +%s) - FRESH ))")" ] && return 0
    # a dialog line is the error message alone; healthy runs print it now and then after step progress
    if (( $(tail -n 40 "$lg" | grep -av '^[[:space:]]*$' | tail -n 6 | grep -a zenity | grep -acv 'step:') == 6 )); then
        why="error dialog"
    elif (( $(date +%s) - $(stat -c %Y "$lg") > QUIET )); then why="no output for ${QUIET}s"; fi
    [ -z "$why" ] && return 0
    local all; all=$(family "$p"); note "stuck ${tag_of[$p]} ($why): stopping $(wc -w <<<"$all") processes"
    kill $all 2>/dev/null; sleep 10 5>&- 6>&- 9>&-; kill -9 $all 2>/dev/null
    echo "[batch] stuck: $why" >> "$lg"
}

others() {  # "gpu MiB" for each launch younger than OTHER_SETTLE in the OTHERS logs
    local f d t word tag pid gpus rest since    # sweep lines: "DATE TIME launch TAG pid=P gpus=PG:EG ..."
    since=$(date -d "@$(( $1 - OTHER_SETTLE ))" '+%F %T')       # batch.sh lines: "... launch TAG pid=P gpu=G ..."
    for f in ${OTHERS//,/ }; do
        [ -r "$f" ] || continue
        while read -r d t word tag pid gpus rest; do
            [[ $word == launch && "$d $t" > "$since" ]] || continue
            if [[ $gpus =~ ^gpus=([0-9]+):([0-9]+)$ ]]; then
                echo "${BASH_REMATCH[1]} $OTHER_POLICY_HOLD"; echo "${BASH_REMATCH[2]} $HOLD"
            elif [[ $gpus =~ ^gpu=([0-9]+)$ ]]; then echo "${BASH_REMATCH[1]} $HOLD"; fi
        done < <(tail -n 200 "$f")
    done
}

choose() {  # the allowed GPU with the most memory left as ledger counts it (and OTHERS' young launches), if NEED is
            # left and it holds fewer than MAX_PER_GPU episodes
    local best="" most=0 g t u n c f
    local -A held=()
    while read -r g m; do held[$g]=$(( ${held[$g]:-0} + m )); done < <(others "$(date +%s)")
    while read -r g t u n c; do
        [[ ",$GPUS," == *",$g,"* ]] || continue
        (( MAX_PER_GPU > 0 && n >= MAX_PER_GPU )) && continue
        f=$(( t - c - ${held[$g]:-0} ))
        (( f > most )) && { most=$f; best=$g; }
    done < <(ledger)
    (( most >= NEED )) && echo "$best"
}

unthrottle() {  # put back the value throttle() replaced, once its window has passed ("now": at once)
    local orig until
    exec 7>"$marker.lock"; flock 7
    if [ -s "$marker" ] && read -r orig until < "$marker" && { [ "${1:-}" = now ] || (( $(date +%s) >= until )); }; then
        [ "$(tr -dc 0-9 < "$THROTTLE" 2>/dev/null)" = 0 ] && echo "$orig" > "$THROTTLE.tmp" && mv "$THROTTLE.tmp" "$THROTTLE"
        rm -f "$marker"; echo "throttle lifted: $THROTTLE back to $orig"
    fi
    flock -u 7
}

throttle() {    # set THROTTLE to 0 for WINDOW seconds; false when it holds no number to put back
    local cur orig until
    exec 7>"$marker.lock"; flock 7
    cur=$(tr -dc 0-9 < "$THROTTLE" 2>/dev/null)
    if [ -s "$marker" ]; then read -r orig until < "$marker"
    elif [ "$cur" = 0 ]; then flock -u 7; note "throttle: $THROTTLE is 0 already (set by someone else)"; return 0
    elif [[ $cur =~ ^[1-9][0-9]*$ ]]; then orig=$cur
    else flock -u 7; return 1; fi
    echo "$orig $(( $(date +%s) + WINDOW ))" > "$marker"
    echo 0 > "$THROTTLE.tmp" && mv "$THROTTLE.tmp" "$THROTTLE"
    flock -u 7
    # puts it back even if this runner stops; a later window's marker keeps it from ending that one early
    setsid bash -c "sleep $((WINDOW + 30)); marker='$marker' THROTTLE='$THROTTLE'; $(declare -f unthrottle); unthrottle" \
        > /dev/null 2>&1 < /dev/null 5>&- 6>&- 7>&- 8>&- 9>&- &
    note "throttle: $THROTTLE set to 0 for ${WINDOW}s (was $orig)"
}

cap() { local m=""; [ -r "$here/logs/$name.max" ] && m=$(tr -dc 0-9 < "$here/logs/$name.max"); echo "${m:-$max_running}"; }

adopt() {   # episodes an earlier runner of this batch launched and that still run
    local d t word tag p g rest
    while read -r d t word tag p g rest; do
        p=${p#pid=} g=${g#gpu=}
        [ -r "/proc/$p/cmdline" ] && tr '\0' ' ' < "/proc/$p/cmdline" | grep -q "scripts/robodojo.sh eval" || continue
        grep -aq " exit $tag gpu=" "$events" && continue
        [[ $tag =~ ^(.+)_s([0-9]+)_l([0-9]+)(_a([0-9]+))?$ ]] || continue
        started[$p]=$(date -d "$d $t" +%s) gpu_of[$p]=$g tag_of[$p]=$tag log_of[$p]="$logdir/$tag.log" adopted[$p]=1
        line_of[$p]="${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[5]:-1}"
        note "adopt $tag pid=$p gpu=$g"
    done < <(grep -a " launch " "$events")
}

last=0
note "begin $name max_running=$max_running gpus=$GPUS max_per_gpu=$MAX_PER_GPU sim=$SIM need=$NEED policy=$POLICY host=$(hostname) others=${OTHERS:-none}"
(( glock )) || note "GLOCK: cannot open $GLOCK; choosing without the machine-wide lock"
for f in ${OTHERS//,/ }; do [ -r "$f" ] || note "OTHERS: cannot read $f (yet)"; done
adopt
while :; do
    flock -u 6; flock -u 5
    for p in "${!started[@]}"; do
        kill -0 "$p" 2>/dev/null && stuck "$p"
        kill -0 "$p" 2>/dev/null && continue
        if [ -n "${adopted[$p]:-}" ]; then code="?"; else wait "$p"; code=$?; fi
        note "exit ${tag_of[$p]} gpu=${gpu_of[$p]} code=$code after $(( $(date +%s) - started[$p] ))s"
        retry "${line_of[$p]}" "${log_of[$p]}" "${started[$p]}"
        unset "started[$p]" "gpu_of[$p]" "line_of[$p]" "log_of[$p]" "tag_of[$p]" "adopted[$p]"
    done
    running=${#started[@]}
    if [ -n "$THROTTLE" ] && [ -s "$marker" ]; then msg=$(unthrottle); [ -n "$msg" ] && note "$msg"; fi
    if ! grep -q -v '^[[:space:]]*$' "$todo"; then (( running == 0 )) && break; sleep 30 5>&- 6>&- 9>&-; continue; fi
    gpu=""
    if (( running < $(cap) )) && [ ! -e "$here/logs/$name.pause" ] && (( $(date +%s) - last >= SPACING )); then
        flock -w 300 6 || note "launch lock still held after 300s: choosing anyway"
        (( glock )) && { flock -w 300 5 || note "$GLOCK still held after 300s: choosing anyway"; }
        gpu=$(choose 5>&- 6>&-); [ -z "$gpu" ] && { flock -u 6; flock -u 5; }
    fi
    if [ -n "$gpu" ]; then
        if [ -n "$THROTTLE" ]; then
            # a launch the other runner was making as we set it shows in its log within seconds
            last=$(date +%s)
            throttle || { note "throttle: $THROTTLE holds no number; not launching"; continue; }
            sleep 20 5>&- 6>&- 9>&-
            gpu=$(choose 5>&- 6>&-)
            if [ -z "$gpu" ]; then note "no GPU left after all; $(unthrottle now)"; continue; fi
        fi
        line=$(take); [ -z "$line" ] && { [ -n "$THROTTLE" ] && note "$(unthrottle now)"; continue; }
        read -r task seed layout attempt <<<"$line"; attempt=${attempt:-1}
        tag=${task}_s${seed}_l${layout}; (( attempt > 1 )) && tag=${tag}_a$attempt
        ROBODOJO_LAYOUT_IDS=$layout nohup bash scripts/robodojo.sh eval --policy-dir "XPolicyLab/policy/$POLICY" \
            --task "$task" --ckpt no-checkpoint --policy-env "$POLICY_ENV" --eval-env "$EVAL_ENV" --env-cfg arx_x5 \
            --action-type joint --seed "$seed" --policy-gpu "$gpu" --env-gpu "$gpu" --eval-num 1 \
            > "$logdir/$tag.log" 2>&1 < /dev/null 5>&- 6>&- 9>&- &
        p=$!; started[$p]=$(date +%s); last=${started[$p]}
        gpu_of[$p]=$gpu line_of[$p]="$task $seed $layout $attempt" log_of[$p]="$logdir/$tag.log" tag_of[$p]=$tag
        note "launch $tag pid=$p gpu=$gpu running=$((running + 1))"
        for i in 1 2 3 4 5 6 7 8 9 10; do   # other runners count it once ps shows it
            tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -q 'scripts/robodojo\.sh eval ' && break; sleep 0.5 5>&- 6>&- 9>&-
        done
        flock -u 6; flock -u 5
    fi
    sleep "$POLL" 5>&- 6>&- 9>&-   # children must not keep the runner or launch locks
done
[ -n "$THROTTLE" ] && [ -s "$marker" ] && note "$(unthrottle now)"
note "end $name"
