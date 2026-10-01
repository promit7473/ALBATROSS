#!/bin/bash
# Stage 2: SITL + Gazebo WITH manual RC control.
#
# Same sim as start_all.sh (Stage 1) but enables manual stick input so the
# pilot can FLY the simulated drone with the real RC transmitter + receiver.
#
# The receiver stays wired to the real Pixhawk (CRSF on its RC port). This
# script starts rc_to_sitl.py, which reads RC_CHANNELS from the Pixhawk over
# USB (/dev/ttyACM0) and forwards them as MANUAL_CONTROL to PX4 SITL via
# mavlink2rest. PX4 SITL is set to COM_RC_IN_MODE=1 (MAVLink manual control).
#
# In the web UI: toggle to "Manual" (POSCTL) to fly with sticks, or "Auto" to
# resume an uploaded mission. The toggle we built picks which PX4 mode SITL is
# in; MANUAL_CONTROL sticks are only honoured in a manual mode.
#
# Env knobs:
#   RC_PIXHAWK_DEV   Pixhawk serial device (default /dev/ttyACM0)
#   RC_PIXHAWK_BAUD  serial baudrate       (default 115200)
#   LAUNCH_QGC       1 = also launch QGC (default 1; it connects to the SITL on
#                    :14550, NOT the real Pixhawk — the RC bridge owns /dev/ttyACM0)
set +u
ROOT="${MARITIME_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$ROOT/external/PX4-Autopilot/Tools/simulation/gz/setup.sh" 2>/dev/null
cd "$ROOT/gcs" || exit 2

# clean stale instances
pkill -f "gcs/server.py" 2>/dev/null
pkill -f "rc_to_sitl.py" 2>/dev/null
pkill -KILL -f "gz sim" 2>/dev/null; pkill -KILL -f "bin/px4" 2>/dev/null
pkill -f "ruby" 2>/dev/null
pkill -f "cam_bridge.py" 2>/dev/null; pkill -f "chase_cam.py" 2>/dev/null
pkill -f "mavlink2rest" 2>/dev/null
sleep 2

# 1) GCS web server on :8080
setsid bash -c "exec python3 '$ROOT/gcs/server.py' 8080" \
  > "$HOME/gcs_server.log" 2>&1 < /dev/null &
sleep 2

# 2) SITL + Gazebo + mavlink2rest  (RC_IN_MODE=1 => accept MANUAL_CONTROL via MAVLink)
setsid env SITL_RC_IN_MODE=1 \
  bash "$ROOT/launch/launch_chattogram_clean.sh" \
  > "$HOME/launch_out.log" 2>&1 < /dev/null &
echo "started: gcs server (:8080) + SITL (manual-control ENABLED) booting"

# 3) RC bridge: real Pixhawk (receiver) -> SITL MANUAL_CONTROL.
#    mavlink2rest starts early in launch_chattogram_clean.sh, so give it 3s.
#    The bridge owns /dev/ttyACM0; QGC (below) connects to the SITL via UDP only.
sleep 3
RC_DEV="${RC_PIXHAWK_DEV:-/dev/ttyACM0}"
if [ -e "$RC_DEV" ]; then
  setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    python3 "$ROOT/gcs/rc_to_sitl.py" "$RC_DEV" "${RC_PIXHAWK_BAUD:-115200}" \
    > "$HOME/rc_bridge.log" 2>&1 < /dev/null &
  echo "rc_to_sitl.py started: reading RC from $RC_DEV -> SITL (log: ~/rc_bridge.log)"
else
  echo "WARNING: $RC_DEV not found — RC bridge NOT started." >&2
  echo "  Plug in the Pixhawk (with receiver) and rerun, OR start manually:" >&2
  echo "    python3 '$ROOT/gcs/rc_to_sitl.py' /dev/ttyACM0" >&2
  echo "  The SITL + web GCS are still up; you can plan/upload autonomous missions," >&2
  echo "  and the Manual<->Auto toggle still works (Manual = centered idle sticks)." >&2
fi

# 4) QGroundControl — connects to the SITL on :14550 (NOT the real Pixhawk).
if [ "${LAUNCH_QGC:-1}" = "1" ]; then
  bash "$ROOT/launch/qgc.sh"
fi

echo ""
echo "Stage 2 ready: fly the SIM drone with the RC controller."
echo "  - Open http://localhost:8080, click the Manual<->Auto toggle -> Manual"
echo "  - Arm via the web GCS Arm button (or QGC), then fly with sticks"
echo "  - rc_bridge log: ~/rc_bridge.log  (watch for 'RC link LIVE')"
