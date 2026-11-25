#!/usr/bin/env bash
# A copy of genie_sim v3.5.0's benchmark/config for native (no docker) local runs of RoboColiseum Season I.
#
# The llm_task scene files name their objects by the docker mount point (@/geniesim_assets/...), which does not
# exist outside the container: natively those prims load empty (no table, no objects) and the run still goes on.
# This copy differs from the original only in that prefix, replaced by the local GenieSimAssets path;
# hexashell/benches/robocoliseum1/sim.py links each run's benchmark/config to it.
#
#   bash scripts/setup/geniesim35_native_config.sh
#
# $HEXASHELL_GENIESIM35 (default <this checkout>/.bench-data/geniesim-3.5) holds genie_sim/ ($GENIESIM_ROOT),
# GenieSimAssets/ ($GENIESIM_ASSETS) and the copy, benchmark_config/ ($GENIESIM_NATIVE_CONFIG).
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ROOT=${HEXASHELL_GENIESIM35:-$HERE/.bench-data/geniesim-3.5}
GS=${GENIESIM_ROOT:-$ROOT/genie_sim}
ASSETS=${GENIESIM_ASSETS:-$ROOT/GenieSimAssets}
DST=${GENIESIM_NATIVE_CONFIG:-$ROOT/benchmark_config}
SRC=$GS/source/geniesim_benchmark/src/geniesim_benchmark/benchmark/config
[ -d "$SRC" ] || { echo "genie_sim v3.5.0 not found: $SRC" >&2; exit 2; }
[ -d "$ASSETS" ] || { echo "assets not found: $ASSETS" >&2; exit 2; }
[ -e "$DST" ] && { echo "exists: $DST" >&2; exit 2; }
TMP=$DST.tmp.$$
mkdir -p "$(dirname "$DST")"
cp -a "$SRC" "$TMP"
find "$TMP" -name __pycache__ -type d -prune -exec rm -rf {} +
n=$( { grep -rl --include=scene.usda "@/geniesim_assets/" "$TMP" || true; } | wc -l)
{ grep -rl --include=scene.usda "@/geniesim_assets/" "$TMP" || true; } | xargs -r sed -i "s#@/geniesim_assets/#@$ASSETS/#g"
left=$( { grep -rl --include=scene.usda "/geniesim_assets/" "$TMP" || true; } | wc -l)
[ "$left" = 0 ] || { echo "still $left files with /geniesim_assets/" >&2; exit 3; }
# every changed file differs from its original only on lines with the prefix
bad=0
while IFS= read -r f; do
  rel=${f#"$TMP"/}
  d=$(diff <(sed "s#@/geniesim_assets/#@$ASSETS/#g" "$SRC/$rel") "$f" | wc -l)
  [ "$d" = 0 ] || { echo "unexpected diff: $rel" >&2; bad=1; }
done < <(find "$TMP" -name scene.usda)
[ "$bad" = 0 ] || exit 4
{
  echo "source: $SRC"
  echo "genie_sim VERSION: $(cat "$GS/VERSION" 2>/dev/null || true)"
  echo "scene.usda files patched: $n  (@/geniesim_assets/ -> @$ASSETS/)"
} > "$TMP/NATIVE_COPY.txt"
mv "$TMP" "$DST"
cat "$DST/NATIVE_COPY.txt"
