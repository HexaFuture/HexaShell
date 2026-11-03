#!/usr/bin/env bash
# Hold a pool batch while the model gateway is down, and let it go on when the gateway answers again.
#
#   setsid nohup bash scripts/gateway_guard.sh DIR NAME [HOURS] \
#       >> DIR/logs/NAME.guard.log 2>&1 < /dev/null &
#
# DIR: the HexaShell folder the batch runs in (the one holding logs/NAME.todo).
# Every EVERY seconds (60) one tiny request (scripts/gateway_ping.sh, the key on stdin). After FAILS
# (2) failures in a row it writes DIR/logs/NAME.pause with "gateway_guard" in it: the pool then
# starts no episode (running ones go on). The first answer after that removes the file again - only
# a pause file the guard wrote itself, never one somebody else made. It ends after HOURS (12).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIR=${1:?DIR}
NAME=${2:?NAME}
HOURS=${3:-12}
EVERY=${EVERY:-60}
FAILS=${FAILS:-2}
PING=${PING:-"bash $ROOT/scripts/gateway_ping.sh"}
PAUSE="$DIR/logs/$NAME.pause"
[[ -d "$DIR/logs" ]] || { echo "no logs folder in $DIR" >&2; exit 2; }

note() { echo "$(date '+%F %T') $*"; }
ours() { [[ -f "$PAUSE" ]] && grep -q '^gateway_guard' "$PAUSE" 2> /dev/null; }

note "guard $NAME in $DIR for ${HOURS} h, every ${EVERY} s, pause after $FAILS failures (pid $$)"
end=$(awk -v h="$HOURS" -v now="$(date +%s)" 'BEGIN { printf "%d", now + h * 3600 }')
failed=0
last_beat=$(date +%s)
while (( $(date +%s) < end )); do
    answer=$($PING 2>&1 | head -n 2 | tr '\n' ' ')
    if [[ "$answer" == "http 200 "* ]]; then
        if (( failed >= FAILS )); then
            note "the gateway answers again: $answer"
        fi
        failed=0
        if ours; then
            rm -f "$PAUSE" && note "removed the pause file (the pool may start episodes again)"
        fi
    else
        failed=$(( failed + 1 ))
        note "no answer ($failed in a row): ${answer:0:200}"
        if (( failed >= FAILS )) && [[ ! -e "$PAUSE" ]]; then
            echo "gateway_guard $(date '+%F %T'): the gateway did not answer $failed times" > "$PAUSE"
            note "wrote the pause file (no new episodes until the gateway answers)"
        fi
    fi
    if (( $(date +%s) - last_beat >= 3600 )); then
        last_beat=$(date +%s)
        note "still guarding; last answer: ${answer:0:80}"
    fi
    sleep "$EVERY"
done
if ours; then
    note "time is up with the gateway still down: the pause file stays"
fi
note "guard ends"
