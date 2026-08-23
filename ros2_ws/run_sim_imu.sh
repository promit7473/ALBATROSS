#!/bin/bash
# Launch IMU-driven Gazebo simulation with real Pixhawk 6C
# Tilt the real Pixhawk -> watch the sim drone follow
# Also launches QGroundControl for mission monitoring
set -e
source /opt/ros/humble/setup.bash
source /home/mhpromit7473/maritime_ws/install/setup.bash

FCU_URL="${1:-/dev/ttyACM0:921600}"
WORLD=$(ros2 pkg prefix maritime_autonomy)/share/maritime_autonomy/worlds/chattogram.sdf
MODEL=$(ros2 pkg prefix maritime_autonomy)/share/maritime_autonomy/models/drone/model.sdf
RVIZ_CFG=$(ros2 pkg prefix maritime_autonomy)/share/maritime_autonomy/rviz/sim.rviz
QGC="/home/mhpromit7473/QGroundControl.AppImage"

echo "=== Maritime IMU Sim ==="
echo "FCU: $FCU_URL"
echo ""

cleanup() {
    echo "Shutting down..."
    kill -- -$$ 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[1/5] Starting MAVROS..."
setsid ros2 run mavros mavros_node --ros-args \
  -p fcu_url:="$FCU_URL" \
  -p plugin_blacklist:="[sys, companion_process_status]" &>/tmp/mavros.log &
sleep 12

echo "[2/5] Starting Gazebo (server + GUI)..."
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix maritime_autonomy)/share/maritime_autonomy/models:/home/mhpromit7473/PX4-Autopilot/Tools/simulation/gz/models
setsid gz sim "$WORLD" -r &>/tmp/gazebo.log &
sleep 10

echo "[3/5] Spawning drone model..."
# First delete old drone if it exists (from previous run)
gz service -s /world/chattogram_sim/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: 'drone'" 2>/dev/null || true
sleep 1
gz service -s /world/chattogram_sim/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 \
  --req "sdf_filename: '$MODEL', name: 'drone', pose: {position: {x: 0, y: 0, z: 2}, orientation: {w: 1}}"

echo "[4/5] Starting IMU bridge + RViz..."
ros2 run maritime_autonomy imu_sim_bridge &
setsid rviz2 -d "$RVIZ_CFG" &>/tmp/rviz.log &

echo "[5/5] Starting QGroundControl (Mission Planner)..."
if [ -x "$QGC" ]; then
    setsid "$QGC" &>/tmp/qgc.log &
    echo "  QGroundControl launched (connects to Pixhawk via USB)"
else
    echo "  QGC not found at $QGC - skipping"
fi

echo ""
echo "=== SIM RUNNING ==="
echo "Tilt/rotate your Pixhawk 6C -> sim drone follows orientation"
echo "Topics: /drone/pose  /drone/imu  /mavros/imu/data"
echo "Windows: Gazebo | RViz | QGroundControl"
echo "Press Ctrl+C to stop"
echo ""

wait
