#!/bin/bash
# Log-safe Chattogram SITL launcher.
# PX4's interactive console is fed from a held-open FIFO so it BLOCKS at the
# prompt (no pxh> spam -> no multi-GB log runaway). We can also send pxh
# commands by writing to the FIFO (e.g. `commander takeoff`).
set -u
# self-locating project root (this script lives in <root>/launch)
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
python3 "$ROOT/launch/session_preflight.py" 2 || exit 1
GCS="$ROOT/gcs"
PX4_DIR="$ROOT/external/PX4-Autopilot"
M2R="$ROOT/external/mavlink2rest"
FIFO="$HOME/.px4_stdin"
LOG="$HOME/sitl_clean.log"

# clean any stragglers
pkill -KILL -f "gz sim" 2>/dev/null
pkill -KILL -f "bin/px4" 2>/dev/null
pkill -KILL -f "px4_sitl gz" 2>/dev/null
sleep 1

# fresh FIFO + keep-open holder (never closes, never sends -> px4 blocks quietly)
rm -f "$FIFO"; mkfifo "$FIFO"
setsid bash -c "exec tail -f /dev/null > '$FIFO'" &
echo $! > "$HOME/.px4_fifo_holder.pid"

# ensure mavlink2rest is running for the web GCS. It listens on a DEDICATED
# :14555 (fed by a pushed PX4 mavlink instance below) so QGroundControl can own
# PX4's default GCS port :14550 with zero configuration. REST stays on :8088.
pkill -f "mavlink2rest" 2>/dev/null
"$M2R" -c udpin:0.0.0.0:14555 -s 0.0.0.0:8088 --send-initial-heartbeats > /dev/null 2>&1 &

: > "$LOG"
cd "$PX4_DIR"
# GUI: leave HEADLESS UNSET so PX4 launches its own gz gui (which auto-follows the
# model via /gui/track). Force xcb so the gui doesn't silently crash on Wayland.
setsid env PX4_GZ_WORLD="${PX4_GZ_WORLD:-chattogram}" DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM=xcb \
  PX4_HOME_LAT="${PX4_HOME_LAT:-22.318725}" PX4_HOME_LON="${PX4_HOME_LON:-91.813156}" PX4_HOME_ALT="${PX4_HOME_ALT:-5.0}" \
  PX4_GZ_FOLLOW_OFFSET_X=-2 PX4_GZ_FOLLOW_OFFSET_Y=-2 PX4_GZ_FOLLOW_OFFSET_Z=1.5 \
  PX4_SIM_SPEED_FACTOR=4 \
  bash -c "exec make px4_sitl gz_x500 < '$FIFO' >> '$LOG' 2>&1" &
echo $! > "$HOME/sitl.pgid"

# Cameras: two invisible followers repositioned in the world (both static).
#  - chase (third-person, behind+above)  -> /chase_cam
#  - 2nd feed: TOP-DOWN by default, live-switchable to FPV from the web UI via
#    $HOME/.cam2_view (written by /api/command cam_view)  -> /topdown_cam
pkill -f "chase_cam.py" 2>/dev/null
echo topdown > "$HOME/.cam2_view"
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python CAM_MODE=chase \
  GZ_WORLD="${PX4_GZ_WORLD:-chattogram}" \
  python3 "$GCS/chase_cam.py" > "$HOME/chase_cam.log" 2>&1 &
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python CAM_MODE=topdown \
  GZ_WORLD="${PX4_GZ_WORLD:-chattogram}" \
  CAM_VIEW_FILE="$HOME/.cam2_view" \
  python3 "$GCS/chase_cam.py" > "$HOME/topdown_cam.log" 2>&1 &

# Gazebo cameras -> web MJPEG bridges (replace the old x11grab screen-scrape).
#  chase feed on :8091 (/video.mjpeg), top-down feed on :8092 (/video2.mjpeg)
pkill -f "cam_bridge.py" 2>/dev/null
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python CAM_TOPIC=/chase_cam CAM_BRIDGE_PORT=8091 \
  python3 "$GCS/cam_bridge.py" > "$HOME/cam_bridge.log" 2>&1 &
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python CAM_TOPIC=/topdown_cam CAM_BRIDGE_PORT=8092 \
  python3 "$GCS/cam_bridge.py" > "$HOME/cam_bridge2.log" 2>&1 &

# Auto-configure PX4 once it's up: arming bypass (SITL) + nose along the
# flight path (MPC_YAW_MODE=3, "along trajectory") + smooth takeoff/landing.
# This firmware has no MIS_YAWMODE; the ramp parameter is MPC_TKO_RAMP_T.
# MPC_LAND_SPEED cannot go below 0.6 m/s, and a slower value breaks the
# land detector (LNDMC_Z_VEL_MAX must stay under the landing descent).
#
# COM_RC_IN_MODE (manual-control source):
#   4 = disable manual control  -> Stage 1 (planning sim, autonomous only)
#   1 = MAVLink only            -> Stage 2 (manual sim flight via rc_to_sitl.py)
# Override with SITL_RC_IN_MODE env. Default 4 keeps Stage 1 behavior.
RC_MODE="${SITL_RC_IN_MODE:-4}"
(
  sleep 12
  for p in "COM_RC_IN_MODE $RC_MODE" "CBRK_SUPPLY_CHK 894281" "NAV_DLL_ACT 0" \
           "NAV_RCL_ACT 0" "COM_RCL_EXCEPT 7" "MPC_YAW_MODE 3" \
           "MPC_TKO_RAMP_T 3.0" "MPC_TKO_SPEED 1.0" "MPC_LAND_SPEED 0.6" \
           "MPC_LAND_ALT1 15.0" "MPC_LAND_ALT2 5.0" "MPC_Z_VEL_MAX_DN 1.0" \
           "MPC_JERK_AUTO 3.0" "MPC_ACC_UP_MAX 2.5" "MPC_ACC_DOWN_MAX 2.0" \
           "MPC_ACC_HOR 2.0" "RTL_RETURN_ALT 60.0" "RTL_DESCEND_ALT 30.0"; do
    echo "param set $p" > "$FIFO"; sleep 0.4
  done
  echo "param save" > "$FIFO"

  # Dedicated web-GCS link. PX4's default GCS link already targets :14550, which
  # we leave for QGroundControl (zero-config AutoConnect). Here we start a SECOND
  # full-GCS mavlink instance that pushes to 127.0.0.1:14555, where mavlink2rest
  # listens. -x enables FTP (param/mission download). The two GCSs are fully
  # independent: QGC on :14550, web GCS on :14555.
  sleep 1
  echo "mavlink start -x -u 14556 -o 14555 -r 4000000 -f" > "$FIFO"
) &

# Auto-lock Gazebo GUI 3D camera onto the drone (for anyone watching the GUI)
(
  for i in {1..35}; do
    sleep 1
    gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: FOLLOW, follow_target: {name: "x500_0"}, follow_offset: {x: -3.5, y: -3.5, z: 2.2}, follow_pgain: 0.8' >/dev/null 2>&1
  done
) &

echo "launched: sitl pgid=$(cat "$HOME/sitl.pgid")  fifo_holder=$(cat "$HOME/.px4_fifo_holder.pid")"
echo "log: $LOG   send cmds: echo 'commander takeoff' > $FIFO"
