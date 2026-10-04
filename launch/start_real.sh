#!/bin/bash
# Real-hardware launcher for the Pixhawk 6C + SIYI HM30 stack -- NO SITL, NO Gazebo,
# NO SITL-only arming bypasses.
#
#   1) web GCS on :8080  (map, mission, Manual<->Auto, FPV pane)
#   2) mavlink2rest on UDP :14555 / REST :8088
#   3) mav_hub.py  -- SIYI UDP (192.168.144.12:19856) or Pixhawk USB
#                    -> :14550 (QGC) and :14555 (web GCS)
#   4) rtsp_bridge.py -- HM30 FPV  rtsp://192.168.144.25:8554/main.264 -> :8091
#   5) QGroundControl (v5.1.4) auto-connects on :14550
#
# RC transmitter -> receiver on the Pixhawk is native PX4. This launcher does
# NOT inject sticks. The web Manual<->Auto toggle only changes flight mode.
#
# Env knobs:
#   PIXHAWK_DEV     force a serial device (otherwise auto-detect / SIYI UDP)
#   PIXHAWK_BAUD    serial baud (default 115200; ignored on USB CDC)
#   FPV_RTSP_URL    camera URL (default rtsp://192.168.144.25:8554/main.264)
#   SIYI_HOST       HM30 air-unit IP (default 192.168.144.12)
#   START_QGC       1 = launch QGroundControl (default 1)
#   OPEN_BROWSER    1 = open http://localhost:8080 (default 1)
set -u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
python3 "$ROOT/launch/session_preflight.py" 3 || exit 1
GCS="$ROOT/gcs"
M2R="${M2R_BIN:-$ROOT/external/mavlink2rest}"
[ -x "$M2R" ] || M2R="${HOME}/mavlink2rest"
FPV_RTSP_URL="${FPV_RTSP_URL:-rtsp://192.168.144.25:8554/main.264}"
LOG="$HOME/real_launch.log"

: > "$LOG"
# real-flight processes only (leave any SITL session alone)
pkill -f "gcs/server.py" 2>/dev/null || true
pkill -f "gcs/mav_hub.py" 2>/dev/null || true
pkill -f "gcs/rtsp_bridge.py" 2>/dev/null || true
pkill -f "mavlink2rest" 2>/dev/null || true
pkill -f "mavlink-routerd" 2>/dev/null || true
pkill -f "QGroundControl" 2>/dev/null || true
sleep 1

export PYTHONUNBUFFERED=1

# 1) mavlink2rest first so the web GCS does not spawn a second copy
setsid "$M2R" -c udpin:0.0.0.0:14555 -s 0.0.0.0:8088 --send-initial-heartbeats \
  > "$HOME/mavlink2rest.log" 2>&1 &
sleep 0.4

# 2) GCS web server. Both feed panes proxy the FPV MJPEG bridge.
setsid env GCS_MODE=real \
  CAM_BRIDGE_URL=http://127.0.0.1:8091/video.mjpeg \
  CAM_BRIDGE2_URL=http://127.0.0.1:8091/video.mjpeg \
  bash -c "exec python3 '$GCS/server.py' 8080" \
  > "$HOME/gcs_server.log" 2>&1 < /dev/null &

# 3) vehicle <-> QGC / web GCS  (PIXHAWK_DEV, if set, is inherited)
setsid python3 "$GCS/mav_hub.py" \
  > "$HOME/mav_hub.log" 2>&1 < /dev/null &
echo "mav_hub: SIYI UDP ${SIYI_HOST:-192.168.144.12}:19856 (or Pixhawk USB) -> :14550 / :14555"

# 4) FPV camera
setsid env FPV_RTSP_URL="$FPV_RTSP_URL" CAM_BRIDGE_PORT=8091 \
  python3 "$GCS/rtsp_bridge.py" \
  > "$HOME/rtsp_bridge.log" 2>&1 < /dev/null &
echo "rtsp_bridge: $FPV_RTSP_URL -> :8091"

# 5) QGroundControl -- AutoConnect UDP :14550 (fed by mav_hub)
if [ "${START_QGC:-1}" = "1" ]; then
  sleep 1
  bash "$ROOT/launch/qgc.sh" 2>/dev/null || true
fi

if [ "${OPEN_BROWSER:-1}" = "1" ]; then
  sleep 1
  (xdg-open "http://localhost:8080" >/dev/null 2>&1 || true) &
fi

echo "started: GCS :8080 + FPV + mavlink hub + QGC"
echo "open http://localhost:8080  -- FPV starts automatically; RC flies the aircraft natively."
echo "REMINDER (real flight): set these in QGC, NOT from this script:"
echo "  NAV_RCL_ACT (RC-loss action), NAV_DLL_ACT (datalink-loss), COM_RCL_EXCEPT"
echo "logs: ~/gcs_server.log ~/mav_hub.log ~/rtsp_bridge.log ~/qgc.log"
echo "hub/fpv: $LOG"
