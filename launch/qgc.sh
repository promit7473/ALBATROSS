#!/bin/bash
# Launch QGroundControl for the Chattogram SITL stack.
#
# Zero configuration required. PX4's default GCS link sends telemetry to
# 127.0.0.1:14550, which is exactly the port QGC's factory AutoConnect-UDP
# listens on -- so QGC connects to the drone on its own. The web GCS runs
# independently via mavlink2rest on :14555 (fed by a separate PX4 push link),
# so the two never contend for a port.
#
# Usage: launch/qgc.sh
set -u

QGC="${QGC_APPIMAGE:-$HOME/QGroundControl.AppImage}"

if [ ! -x "$QGC" ]; then
  echo "QGroundControl not found/executable at: $QGC" >&2
  echo "Set QGC_APPIMAGE=/path/to/QGroundControl.AppImage or chmod +x it." >&2
  exit 1
fi

echo "Launching QGroundControl ($QGC)"
echo "  -> auto-connects to PX4 on :14550 (web GCS is independent on :14555)"
setsid "$QGC" > "$HOME/qgc.log" 2>&1 &
echo "  started (log: $HOME/qgc.log)"
