#!/usr/bin/env bash
# Build the workspace (symlink install: python edits take effect without rebuilding).
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --symlink-install "$@"
