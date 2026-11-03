#!/usr/bin/env bash
# One tiny chat request to the model gateway of config/benches.yml: prints "http CODE in Ts" and the
# start of the answer (e.g. 503 node_draining while the gateway is under maintenance).
#
#   bash scripts/gateway_ping.sh
#
# The key (~/.config/hexashell/api-key, or $HEXASHELL_API_KEY_FILE) reaches curl on stdin, so it is
# never on a command line that other users of the machine could see.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE=$(awk '/^api_base:/ {print $2}' "$ROOT/config/benches.yml")
MODEL=$(awk '/^model:/ {print $2}' "$ROOT/config/benches.yml")
KEY_FILE=${HEXASHELL_API_KEY_FILE:-$HOME/.config/hexashell/api-key}
[[ -r "$KEY_FILE" ]] || { echo "no key file: $KEY_FILE" >&2; exit 2; }
body=$(mktemp)
trap 'rm -f "$body"' EXIT
printf 'Authorization: Bearer %s\n' "$(cat "$KEY_FILE")" |
    env -u HTTPS_PROXY -u HTTP_PROXY -u https_proxy -u http_proxy -u ALL_PROXY -u all_proxy \
    curl -s --max-time 60 -o "$body" -w "http %{http_code} in %{time_total}s\n" -H @- \
    -H "Content-Type: application/json" \
    -d "{\"model\": \"$MODEL\", \"messages\": [{\"role\": \"user\", \"content\": \"Reply with the word ok.\"}], \"max_tokens\": 16}" \
    "$BASE/chat/completions"
head -c 300 "$body"
echo
