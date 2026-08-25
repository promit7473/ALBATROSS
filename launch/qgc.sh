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

# Pick a runnable AppImage. The generic "QGroundControl.AppImage" is a newer build
# that needs GLIBC 2.38 (fails on Ubuntu 22.04 / glibc 2.35), so prefer the stable
# v4.4.5 build which runs here. Override with QGC_APPIMAGE=/path if needed.
QGC="${QGC_APPIMAGE:-}"
if [ -z "$QGC" ]; then
  for c in "$HOME/QGroundControl-v4.4.5.AppImage" "$HOME/QGroundControl.AppImage"; do
    [ -x "$c" ] && { QGC="$c"; break; }
  done
fi

if [ -z "$QGC" ] || [ ! -x "$QGC" ]; then
  echo "QGroundControl AppImage not found/executable (looked for v4.4.5, then generic)." >&2
  echo "Set QGC_APPIMAGE=/path/to/QGroundControl.AppImage or chmod +x it." >&2
  exit 1
fi

echo "Launching QGroundControl ($QGC)"
echo "  -> auto-connects to PX4 on :14550 (web GCS is independent on :14555)"
setsid "$QGC" > "$HOME/qgc.log" 2>&1 &
echo "  started (log: $HOME/qgc.log)"
