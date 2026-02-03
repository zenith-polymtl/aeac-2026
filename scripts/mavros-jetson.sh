#!/usr/bin/env bash
set -euo pipefail

# Path to your repo (must contain compose/mavros.yml)
REPO_DIR="/home/zenith/aeac-2026"
COMPOSE_FILE="compose/mavros.yml"
DEVICE="/dev/ttyTHS1"

# MAVROS args
FCU_URL="serial://${DEVICE}:921600"
FCU_PROTOCOL="v2.0"

cd "$REPO_DIR"

echo "[mavros-jetson] Waiting for ${DEVICE} to exist..."
until [ -c "$DEVICE" ]; do
  sleep 1
done
echo "[mavros-jetson] ${DEVICE} is present."

#Start container 
echo "[mavros-jetson] Starting docker compose..."
docker compose -f "$COMPOSE_FILE" up -d --build

#wait a moment for container to be ready
sleep 1

echo "[mavros-jetson] Launching MAVROS in container..."
# Run MAVROS in foreground so systemd can supervise it
exec docker compose -f "$COMPOSE_FILE" exec -T mavros bash -lc "
  source /opt/ros/humble/setup.bash
  ros2 launch mavros apm.launch fcu_url:=${FCU_URL} fcu_protocol:=${FCU_PROTOCOL}
"
