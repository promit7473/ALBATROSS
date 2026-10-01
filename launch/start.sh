#!/bin/bash
# Unified 3-stage launcher for the Maritime GCS.
#
#   bash launch/start.sh 1          # Stage 1: SITL planning sim (autonomous, no RC)
#   bash launch/start.sh 2          # Stage 2: SITL + manual RC control (fly the sim)
#   bash launch/start.sh 3          # Stage 3: real Pixhawk (no sim)
#
#   also accepts: planning | manual | real
#
# Stages
# ------
#  1 planning  SITL + Gazebo + web GCS. COM_RC_IN_MODE=4 (manual disabled).
#              Pure autonomous mission planning — the "earlier version".
#  2 manual    Same sim + web GCS, but COM_RC_IN_MODE=1 + rc_to_sitl.py bridge
#              so the real RC controller flies the simulated drone.
#              The receiver stays on the Pixhawk; the bridge reads RC_CHANNELS
#              over USB and forwards MANUAL_CONTROL to SITL via mavlink2rest.
#  3 real      Real Pixhawk 6C (no Gazebo). GCS + mavlink2rest + mav_hub
#              (SIYI UDP or USB) + FPV RTSP + QGC. RC is native PX4.
set -u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
STAGE="${1:-}"
case "$STAGE" in
  1|planning) SCRIPT="start_all.sh" ;;
  2|manual)   SCRIPT="start_sitl_manual.sh" ;;
  3|real)     SCRIPT="start_real.sh" ;;
  *)
    echo "Usage: bash launch/start.sh <1|2|3>" >&2
    echo "  1 / planning  Stage 1: SITL planning sim (autonomous, no RC)" >&2
    echo "  2 / manual    Stage 2: SITL + manual RC control (fly the sim)" >&2
    echo "  3 / real      Stage 3: real Pixhawk (no sim)" >&2
    exit 1 ;;
esac
echo "=== Stage $STAGE -> launch/$SCRIPT ==="
exec bash "$ROOT/launch/$SCRIPT"
