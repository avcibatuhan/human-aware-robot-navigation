#!/usr/bin/env bash
# Build (if needed) and enter the dev container with GUI forwarding for
# Gazebo and RViz. Works with podman (Fedora default) or docker.
#
#   docker/run.sh            interactive shell
#   docker/run.sh <command>  run one command and exit
#   docker/run.sh --build    force an image rebuild first
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="human-aware-nav:jazzy"
NAME="human-aware-nav"

if command -v podman >/dev/null 2>&1; then
  ENGINE=podman
elif command -v docker >/dev/null 2>&1; then
  ENGINE=docker
else
  echo "Neither podman nor docker found." >&2
  exit 1
fi

if [[ "${1:-}" == "--build" ]]; then
  shift
  "$ENGINE" build -t "$IMAGE" -f "$REPO_DIR/docker/Dockerfile" "$REPO_DIR"
elif ! "$ENGINE" image inspect "$IMAGE" >/dev/null 2>&1; then
  "$ENGINE" build -t "$IMAGE" -f "$REPO_DIR/docker/Dockerfile" "$REPO_DIR"
fi

# A second call attaches to the running container (extra terminal).
if [[ -n "$("$ENGINE" ps -q -f "name=^${NAME}$")" ]]; then
  if [[ $# -eq 0 ]]; then
    exec "$ENGINE" exec -it "$NAME" bash
  fi
  exec "$ENGINE" exec -i "$NAME" bash -ic "$*"
fi

ARGS=(
  --rm --name "$NAME"
  --network host --ipc host
  # SELinux would otherwise block the X11/Wayland sockets and the bind mount.
  --security-opt label=disable
  -v "$REPO_DIR:/ws"
  -e TURTLEBOT3_MODEL=waffle
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
)

# X11 (XWayland on a Wayland session): what Gazebo and RViz use.
if [[ -n "${DISPLAY:-}" ]]; then
  ARGS+=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e QT_X11_NO_MITSHM=1)
  if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
    ARGS+=(-v "$XAUTHORITY:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
  fi
fi

# Native Wayland socket, for apps that want it.
if [[ -n "${WAYLAND_DISPLAY:-}" && -S "${XDG_RUNTIME_DIR:-}/$WAYLAND_DISPLAY" ]]; then
  ARGS+=(
    -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY"
    -e XDG_RUNTIME_DIR=/tmp/xdg
    -v "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/xdg/$WAYLAND_DISPLAY"
  )
fi

# Hardware-accelerated rendering through the host's Mesa drivers.
[[ -d /dev/dri ]] && ARGS+=(--device /dev/dri)

if [[ -t 0 ]]; then
  ARGS+=(-it)
fi

if [[ $# -eq 0 ]]; then
  exec "$ENGINE" run "${ARGS[@]}" "$IMAGE" bash
fi
exec "$ENGINE" run "${ARGS[@]}" "$IMAGE" bash -ic "$*"
