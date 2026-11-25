#!/usr/bin/env bash
# Start K online agents (tunnel.py) for the submitted Season II job, each in its own session, detached:
#   online.sh K OUT [STATE]
# K       how many agents (slots 0..K-1); at most the job's PARALLELISM
# OUT     a new directory for the records and the logs (slot_<k>.out, pids)
# STATE   the job's state file (default ~/.hexashell/robocoliseum/jobs/v2.env, as `challenge submit --yes` writes it)
# Extra tunnel.py options go in TUNNEL_ARGS (e.g. TUNNEL_ARGS="--depth every_motion").
# Runs the code of the checkout this script is in, with the skills on (HEXASHELL_SKILLS=1), without proxies and
# without an X display.  Only PARALLELISM, JOB_BOARD and JOB_SEASON are read from the state file here; nothing
# of it is printed.  Stop: kill -TERM the pids listed in OUT/pids (each agent then ends its case and exits 130).
set -euo pipefail
[ $# -ge 2 ] || { sed -n '2,11p' "$0"; exit 1; }
k=$1; out=$2; state=${3:-$HOME/.hexashell/robocoliseum/jobs/v2.env}
py=${HEXASHELL_PYTHON:-python3}
root=$(cd "$(dirname "$0")/../../.." && pwd)

[ -f "$state" ] || { echo "no state file $state" >&2; exit 1; }
mode=$(stat -c %a "$state")
[ "${mode: -2}" = "00" ] || { echo "state file $state is readable by others ($mode)" >&2; exit 1; }
val() { sed -n "s/^$1=//p" "$state" | head -1 | tr -d "'\""; }
jb=$(val JOB_BOARD); season=$(val JOB_SEASON); par=$(val PARALLELISM)
[ "$jb" = "v2" ] || [ "$season" = "2" ] || { echo "the state file's job is not of Season II (board '$jb', season '$season')" >&2; exit 1; }
[ -z "$jb" ] || [ "$jb" = "v2" ] || { echo "the state file's job is on board '$jb', not v2" >&2; exit 1; }
[[ "$k" =~ ^[1-9][0-9]*$ ]] || { echo "K must be a positive number" >&2; exit 1; }
if [ -n "$par" ] && [ "$k" -gt "$par" ]; then echo "K=$k is over the job's parallelism $par" >&2; exit 1; fi
[ -e "$out" ] && { echo "$out exists; give a new directory" >&2; exit 1; }
mkdir -p "$out"
out=$(cd "$out" && pwd); state=$(cd "$(dirname "$state")" && pwd)/$(basename "$state")  # both used after the cd

cd "$root"
read -r -a extra <<< "${TUNNEL_ARGS:-}"
for ((s = 0; s < k; s++)); do
  setsid nohup env -u DISPLAY -u XAUTHORITY -u HTTPS_PROXY -u HTTP_PROXY -u https_proxy -u http_proxy \
    -u ALL_PROXY -u all_proxy PYTHONPATH=. HEXASHELL_SKILLS=1 \
    "$py" -m hexashell.benches.robocoliseum.tunnel --slot "$s" --state "$state" --out "$out" "${extra[@]}" \
    > "$out/slot_$s.out" 2>&1 < /dev/null &
  echo "$!" >> "$out/pids"
done
echo "started $k agent(s) for the Season II job from $root (parallelism ${par:-unknown}); pids in $out/pids"
