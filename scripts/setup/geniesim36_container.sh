#!/usr/bin/env bash
# Start one headless Genie Sim 3.6 container for the local runs of RoboColiseum Season II
# (hexashell/benches/robocoliseum/sim.py starts it when needed).  The mounts and the environment are those of
# genie_sim's own docker/start.sh, but without --privileged, without X11, and with one GPU per container.
#
#   bash scripts/setup/geniesim36_container.sh NAME GPU
#
# NAME must start with hexashell-geniesim- (sim.py: hexashell-geniesim-gpu<GPU>); GPU is the host's GPU index.
# $HEXASHELL_GENIESIM36 (default <this checkout>/.bench-data/geniesim-3.6) holds genie_sim/ ($GENIESIM_SOURCE),
# GenieSimAssets/ ($GENIESIM_ASSETS) and the containers' caches; $GENIESIM_IMAGE names the image.
# Then run commands with: docker exec -u isaac-sim -w /workspace NAME bash -lc "..." (a login shell: the
# entrypoint's /etc/profile.d/geniesim.sh sets PATH, the ROS middleware and CycloneDDS).
set -euo pipefail
NAME=${1:?usage: geniesim36_container.sh NAME GPU}
GPU=${2:?usage: geniesim36_container.sh NAME GPU}
case "$NAME" in hexashell-geniesim-*) ;; *) echo "NAME must start with hexashell-geniesim-" >&2; exit 2 ;; esac
case "$GPU" in ''|*[!0-9]*) echo "GPU must be a number" >&2; exit 2 ;; esac
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ROOT=${HEXASHELL_GENIESIM36:-$HERE/.bench-data/geniesim-3.6}
IMAGE=${GENIESIM_IMAGE:-geniesim4-local:3.6.0}
WORKSPACE=${GENIESIM_SOURCE:-$ROOT/genie_sim}
ASSETS=${GENIESIM_ASSETS:-$ROOT/GenieSimAssets}
CACHE=$ROOT/container_cache/$NAME
[ -f "$ASSETS/pyproject.toml" ] || { echo "assets not found: $ASSETS" >&2; exit 1; }
[ -f "$WORKSPACE/docker/entrypoint.sh" ] || { echo "genie_sim not found: $WORKSPACE" >&2; exit 1; }
mkdir -p "$CACHE/cache/main/ov" "$CACHE/cache/main/warp" "$CACHE/cache/computecache" "$CACHE/config" \
         "$CACHE/data/documents" "$CACHE/data/Kit" "$CACHE/logs" "$CACHE/pkg"
ARGS=(
    -d
    --name "$NAME"
    --init
    --gpus "device=$GPU"
    --network=host
    -e ACCEPT_EULA=Y
    -e OMNI_KIT_ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -e "HOST_UID=$(id -u)"
    -e "HOST_GID=$(id -g)"
    -e ROS_DISTRO=jazzy
    -e GENIESIM_PY_CMD=python3
    -e GENIESIM_BREAK_SYSTEM_PKGS=1
    -v "$CACHE/cache/main:/home/isaac-sim/.cache:rw"
    -v "$CACHE/cache/computecache:/home/isaac-sim/.nv/ComputeCache:rw"
    -v "$CACHE/logs:/home/isaac-sim/.nvidia-omniverse/logs:rw"
    -v "$CACHE/config:/home/isaac-sim/.nvidia-omniverse/config:rw"
    -v "$CACHE/data:/home/isaac-sim/.local/share/ov/data:rw"
    -v "$CACHE/pkg:/home/isaac-sim/.local/share/ov/pkg:rw"
    -v "$WORKSPACE:/workspace:rw"
    -v "$ASSETS:/geniesim_assets:rw"
    -v "$WORKSPACE/docker/entrypoint.sh:/usr/local/bin/geniesim-entrypoint:ro"
    -w /workspace
)
if [ -f /usr/share/nvidia/nvoptix.bin ]; then
    ARGS+=(-v /usr/share/nvidia/nvoptix.bin:/usr/share/nvidia/nvoptix.bin:ro)
fi
docker run "${ARGS[@]}" "$IMAGE" tail -f /dev/null
echo "started $NAME on GPU $GPU; waiting for GENIESIM_READY"
for _ in $(seq 1 600); do
    if docker logs "$NAME" 2>&1 | grep -q GENIESIM_READY; then
        echo "ready"; exit 0
    fi
    [ "$(docker inspect -f '{{.State.Status}}' "$NAME")" = running ] || { docker logs --tail 40 "$NAME"; exit 1; }
    sleep 1
done
echo "not ready after 600 s"; docker logs --tail 40 "$NAME"; exit 1
