#!/usr/bin/env bash
# Track with the real drone: RTSP video in, velocity setpoints out through the Intelis bridge.
# Take off from the Intelis dashboard first; then lock a target and press takeoff/engage in the console.
# Extra args go to the launch file, e.g.  scripts/run_real.sh console:=false
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP=$(ls /opt/ros/{humble,jazzy}/setup.bash 2>/dev/null | head -1)
source "$ROS_SETUP"
source "$WS/install/setup.bash"
export TRACKING_WS="$WS"

if pgrep -f "[t]racking.launch.py" >/dev/null; then
  echo "The simulation stack is running. Stop it first: scripts/stop.sh" >&2
  exit 1
fi
if [ -z "$GCS_TOKEN" ]; then
  read -rsp "Intelis operator key: " GCS_TOKEN; echo
  export GCS_TOKEN
fi
exec ros2 launch tracker_bringup real.launch.py "$@"
