#!/usr/bin/env bash
# Bring up the ROS 2 simulator that ros-mcp controls, on this Apple Silicon Mac.
#
# Why a custom image instead of examples/5_docker_turtlesim: that image is
# amd64-only (runs under QEMU here, too slow for real-time work) and its rosapi
# never advertised its services, which breaks ros-mcp's discovery tools.
# See sim/Dockerfile.sim for the full rationale.
#
# Ports:
#   9090  rosbridge WebSocket - what ros-mcp / any MCP client connects to
#   5900  VNC view of the simulated robot (macOS Screen Sharing: vnc://localhost:5900)
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER=contoro-sim
IMAGE=contoro-sim:jazzy
BRIDGE_PORT="${BRIDGE_PORT:-9090}"
VNC_PORT="${VNC_PORT:-5900}"

log() { printf '  %s\n' "$*"; }

echo "Contoro simulator launcher"
echo "=========================="

if ! docker info >/dev/null 2>&1; then
    echo "ERROR: Docker is not running." >&2
    exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image $IMAGE not found - building it (first run takes a few minutes)..."
    docker build --platform linux/arm64 -f "$WORKSPACE/sim/Dockerfile.sim" -t "$IMAGE" "$WORKSPACE/sim"
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Removing previous container..."
    docker rm -f "$CONTAINER" >/dev/null
fi

# Free the host ports if something else grabbed them.
for p in "$BRIDGE_PORT" "$VNC_PORT"; do
    if lsof -nP -iTCP:"$p" -sTCP:LISTEN >/dev/null 2>&1; then
        echo "ERROR: host port $p is already in use:" >&2
        lsof -nP -iTCP:"$p" -sTCP:LISTEN >&2
        exit 1
    fi
done

echo "Starting container ($CONTAINER)..."
docker run -d \
    --name "$CONTAINER" \
    --platform linux/arm64 \
    -p "${BRIDGE_PORT}:9090" \
    -p "${VNC_PORT}:5900" \
    -e ROS_DISTRO=jazzy \
    -e QT_QPA_PLATFORM=vnc \
    -e QT_VNC_PORT=5900 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -v "$WORKSPACE/sim/launch_turtlesim.launch.py:/ros2_ws/launch/launch_turtlesim.launch.py:ro" \
    "$IMAGE" \
    ros2 launch /ros2_ws/launch/launch_turtlesim.launch.py >/dev/null

echo "Waiting for rosbridge to accept connections..."
for _ in $(seq 1 40); do
    if python3 - "$BRIDGE_PORT" <<'PY' 2>/dev/null
import socket, sys
s = socket.socket()
s.settimeout(1)
try:
    s.connect(("127.0.0.1", int(sys.argv[1])))
except OSError:
    raise SystemExit(1)
finally:
    s.close()
PY
    then
        break
    fi
    sleep 1
done

echo
echo "Simulator is up:"
log "rosbridge : ws://127.0.0.1:${BRIDGE_PORT}"
log "VNC view  : vnc://localhost:${VNC_PORT}   (open with: open vnc://localhost:${VNC_PORT})"
log "logs      : docker logs -f ${CONTAINER}"
echo
echo "Verify with:"
log "$WORKSPACE/.venv/bin/python $WORKSPACE/scripts/probe_rosbridge.py --port ${BRIDGE_PORT}"
