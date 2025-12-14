#!/bin/bash
set -euo pipefail

# Fast build: parallel workers default (colcon decides), skip neupan_nav2_controller
# Usage: ./scripts/build_skip_neupan_fast.sh [additional colcon args]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
OVERLAY_SETUP=${OVERLAY_SETUP:-$WS_DIR/install/setup.bash}

if [[ -f "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [[ -f "$OVERLAY_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$OVERLAY_SETUP"
fi

echo "[build_skip_neupan_fast] Building workspace (skipping neupan_nav2_controller)..."
colcon build --packages-skip neupan_nav2_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"

echo "[build_skip_neupan_fast] Done."
