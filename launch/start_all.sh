#!/bin/bash
# Bring up the whole Maritime GCS: web server (:8080) + SITL + chase cam + bridge.
# Run as a FILE (not inline) so the pkill -f patterns below don't match this
# launcher's own command line and kill it.
set +u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$ROOT/external/PX4-Autopilot/Tools/simulation/gz/setup.sh" 2>/dev/null
cd "$ROOT/gcs" || exit 2

# clean stale instances
pkill -f "gcs/server.py" 2>/dev/null
pkill -KILL -f "gz sim" 2>/dev/null; pkill -KILL -f "bin/px4" 2>/dev/null
pkill -f "ruby" 2>/dev/null
pkill -f "cam_bridge.py" 2>/dev/null; pkill -f "chase_cam.py" 2>/dev/null
pkill -f "mavlink2rest" 2>/dev/null
sleep 2

# 1) GCS web server on :8080, fully detached so it outlives this launcher
setsid bash -c "exec python3 '$ROOT/gcs/server.py' 8080" \
  > "$HOME/gcs_server.log" 2>&1 < /dev/null &

sleep 2

# 2) SITL + chase_cam + cam_bridge + mavlink2rest (launcher detaches its children)
setsid bash "$ROOT/launch/launch_chattogram_clean.sh" \
  > "$HOME/launch_out.log" 2>&1 < /dev/null &

sleep 1
echo "started: gcs server (:8080) + SITL/chase/bridge booting"
