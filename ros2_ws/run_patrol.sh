#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/mhpromit7473/maritime_ws/install/setup.bash

echo "=== Starting MAVROS (Pixhawk 6C via HM30) ==="
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14540@127.0.0.1:14580 &
MAVROS_PID=$!
sleep 5

echo "=== Starting Patrol Node ==="
ros2 run maritime_autonomy patrol_node --ros-args \
  -p kml_path:='/home/mhpromit7473/Chattogram Port — Drone Operating Area & Exclusion Zones.kml' \
  -p loops:=2
PATROL_EXIT=$?

echo "=== Patrol exited (code $PATROL_EXIT), disarming ==="
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}" 2>/dev/null
kill $MAVROS_PID 2>/dev/null
wait $MAVROS_PID 2>/dev/null
echo "=== Done ==="
