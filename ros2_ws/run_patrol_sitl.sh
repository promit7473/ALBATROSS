#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/mhpromit7473/maritime_ws/install/setup.bash

echo "=== Killing stale processes ==="
pkill -9 -f "px4" 2>/dev/null
pkill -9 -f "mavros" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
sleep 2

echo "=== Starting PX4 SITL + Gazebo ==="
cd /home/mhpromit7473/PX4-Autopilot
export DISPLAY=:1
make px4_sitl gz_x500 > /tmp/px4_sitl.log 2>&1 &
PX4_PID=$!

echo "=== Waiting for PX4 to boot (30s) ==="
sleep 30

echo "=== Starting MAVROS ==="
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14540@127.0.0.1:14580 &
MAVROS_PID=$!
sleep 5

echo "=== Disabling PX4 pre-arm checks for SITL ==="
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'COM_ARM_WO_GPS', value: {integer: 1}}" 2>/dev/null
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'COM_LOW_BAT_ACT', value: {integer: 0}}" 2>/dev/null
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'COM_RCL_EXCEPT', value: {integer: 4}}" 2>/dev/null
sleep 2

echo "=== Starting Patrol Node ==="
ros2 run maritime_autonomy patrol_node --ros-args \
  -p kml_path:='/dev/null' \
  -p loops:=1
PATROL_EXIT=$?

echo "=== Patrol exited (code $PATROL_EXIT), cleaning up ==="
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}" 2>/dev/null
sleep 1
kill $MAVROS_PID 2>/dev/null
kill $PX4_PID 2>/dev/null
pkill -9 -f "px4" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
echo "=== Done ==="
