#!/usr/bin/env bash
# Build the workspace (symlink install: python edits take effect without rebuilding).
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$(ls /opt/ros/{humble,jazzy}/setup.bash 2>/dev/null | head -1)"
cd "$WS"
colcon build --symlink-install "$@"
