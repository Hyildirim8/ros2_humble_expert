#!/usr/bin/env bash
set -euo pipefail

# start.sh - helper to build and start the ros2-humble-dev container with
# sensible defaults (passes host UID/GID to image and enables X11 forwarding).

# Determine docker compose command (prefer `docker compose` over legacy `docker-compose`).
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
	COMPOSE_CMD="docker compose"
elif command -v docker-compose >/dev/null 2>&1; then
	COMPOSE_CMD="docker-compose"
else
	echo "ERROR: docker compose or docker-compose not found. Install Docker Compose." >&2
	exit 1
fi

# Export UID/GID/USER so docker-compose build can use them as build args
: "${UID:=$(id -u)}"
: "${GID:=$(id -g)}"
: "${USER:=$(whoami)}"
export UID GID USER

echo "Using compose command: $COMPOSE_CMD"
echo "Host user: $USER (UID=$UID GID=$GID)"

# If X11 is available, allow local docker containers to connect to X server.
if [ -n "${DISPLAY:-}" ]; then
	echo "Enabling X11 access for local docker containers..."
	xhost +local:docker || true
fi

echo "Building container image (this may take a few minutes)..."
# Pass UID/GID/USERNAME as build args so the image can create/match the host user
$COMPOSE_CMD build --build-arg USER_ID="$UID" --build-arg GROUP_ID="$GID" --build-arg USERNAME="$USER"

echo "Starting container(s) in background..."
$COMPOSE_CMD up -d

cat <<EOF
ROS 2 Humble geliştirme ortamı başlatıldı!

Konteynere bağlanmak için:
	$COMPOSE_CMD exec ros2-humble-dev bash
veya
	docker exec -it ros2-humble-dev bash

Not: Eğer GUI uygulamaları (rviz2 vb.) kullanacaksanız, ilk seferde
		xhost +local:docker komutunu host'ta çalıştırmanız gerekebilir.
EOF
