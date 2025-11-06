#!/usr/bin/env bash
# Usage: scripts/build_ws.sh workspaces/ws_heavy_task_2
set -e
WS_DIR="$1"
[ -z "$WS_DIR" ] && { echo "Usage: $0 <workspace_dir>"; exit 1; }

pushd "$WS_DIR" >/dev/null
source /opt/ros/humble/setup.bash || true
rosdep update || true
rosdep install --from-paths src -r -y
colcon build --symlink-install
popd >/dev/null
