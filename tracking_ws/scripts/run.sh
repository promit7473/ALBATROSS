#!/usr/bin/env bash
# Start everything: Gazebo city + PX4 SITL + MAVROS + perception + follow controller + console.
# Extra args go to the launch file, e.g.  scripts/run.sh gui:=false weights:=assets/weights/yolo11s.pt
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$(ls /opt/ros/{humble,jazzy}/setup.bash 2>/dev/null | head -1)"
source "$WS/install/setup.bash"
export TRACKING_WS="$WS"

if timeout 5 gz topic -l 2>/dev/null | grep -q "^/world/" || pgrep -x px4 >/dev/null; then
  echo "Another Gazebo / PX4 is running (e.g. the ALBATROSS SITL). Stop it first: scripts/stop.sh" >&2
  exit 1
fi
exec ros2 launch tracker_bringup tracking.launch.py "$@"
