#!/usr/bin/env bash
# Kill any leftover simulation processes (Gazebo, PX4, MAVROS, tracker nodes).
PATTERNS=("[t]racking.launch.py" "[r]eal.launch.py" "[g]z sim" "[b]in/px4" "[m]avros_node" "install/tracker_[a-z]*/lib/")

for p in "${PATTERNS[@]}"; do pkill -f "$p"; done
for _ in $(seq 10); do                       # give them up to 5 s to exit cleanly
  alive=0
  for p in "${PATTERNS[@]}"; do pgrep -f "$p" >/dev/null && alive=1; done
  [ $alive = 0 ] && break
  sleep 0.5
done
for p in "${PATTERNS[@]}"; do pkill -9 -f "$p"; done
echo stopped
