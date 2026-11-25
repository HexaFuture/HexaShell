#!/usr/bin/env bash
# Start K online agents (tunnel.py) for one submitted job, each in its own session, detached:
#   online.sh BOARD K OUT [STATE]
# BOARD   the job's board (instruction / spatial / manip / robust)
# K       how many agents (slots 0..K-1); at most the job's PARALLELISM
# OUT     a new directory for the records and the logs (slot_<k>.out, pids)
# STATE   the job's state file (default ~/.hexashell/robocoliseum1/jobs/BOARD.env, as `challenge submit` writes it)
# Runs the code of the checkout this script is in, with the skills on (HEXASHELL_SKILLS=1), without proxies and
# without an X display.  Only PARALLELISM and JOB_BOARD are read from the state file here; nothing of it is
# printed.  Stop: kill -TERM the pids listed in OUT/pids (each agent then ends its case and exits 130).
set -euo pipefail
[ $# -ge 3 ] || { sed -n '2,10p' "$0"; exit 1; }
board=$1; k=$2; out=$3; state=${4:-$HOME/.hexashell/robocoliseum1/jobs/$board.env}
py=${HEXASHELL_PYTHON:-python3}
root=$(cd "$(dirname "$0")/../../.." && pwd)

[ -f "$state" ] || { echo "no state file $state" >&2; exit 1; }
mode=$(stat -c %a "$state")
[ "${mode:1}" = "00" ] || { echo "state file $state is readable by others ($mode)" >&2; exit 1; }
val() { sed -n "s/^$1=//p" "$state" | head -1 | tr -d "'\""; }
jb=$(val JOB_BOARD); par=$(val PARALLELISM)
[ "$jb" = "$board" ] || { echo "the state file's job is on board '$jb', not '$board'" >&2; exit 1; }
[[ "$k" =~ ^[1-9][0-9]*$ ]] || { echo "K must be a positive number" >&2; exit 1; }
if [ -n "$par" ] && [ "$k" -gt "$par" ]; then echo "K=$k is over the job's parallelism $par" >&2; exit 1; fi
[ -e "$out" ] && { echo "$out exists; give a new directory" >&2; exit 1; }
mkdir -p "$out"

cd "$root"
for ((s = 0; s < k; s++)); do
  setsid nohup env -u DISPLAY -u XAUTHORITY -u HTTPS_PROXY -u HTTP_PROXY -u https_proxy -u http_proxy \
    -u ALL_PROXY -u all_proxy PYTHONPATH=. HEXASHELL_SKILLS=1 \
    "$py" -m hexashell.benches.robocoliseum1.tunnel --board "$board" --slot "$s" --state "$state" --out "$out" \
    > "$out/slot_$s.out" 2>&1 < /dev/null &
  echo "$!" >> "$out/pids"
done
echo "started $k agent(s) for $board from $root (parallelism ${par:-unknown}); pids in $out/pids"
