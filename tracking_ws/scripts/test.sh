#!/usr/bin/env bash
# Unit tests for the algorithms (paths, tracker, camera motion, geolocation, estimator, guidance).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$(ls /opt/ros/{humble,jazzy}/setup.bash 2>/dev/null | head -1)"
source "$WS/install/setup.bash"
cd "$WS" && python3 -m pytest -q -p no:cacheprovider src/*/test "$@"
