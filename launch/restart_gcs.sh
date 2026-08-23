#!/bin/bash
# Restart just the GCS web server + chase cam (leaves PX4/gz/bridge running).
set +u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash 2>/dev/null
pkill -f "gcs/server.py" 2>/dev/null
pkill -f "chase_cam.py" 2>/dev/null
sleep 1
setsid bash -c "exec python3 '$ROOT/gcs/server.py' 8080" > "$HOME/gcs_server.log" 2>&1 < /dev/null &
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
  python3 "$ROOT/gcs/chase_cam.py" > "$HOME/chase_cam.log" 2>&1 < /dev/null &
sleep 2
echo "restarted gcs server + chase cam"
tail -2 "$HOME/gcs_server.log"
