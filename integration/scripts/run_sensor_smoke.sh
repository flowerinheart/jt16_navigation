#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$TASK_ROOT/results"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export GZ_PARTITION=xle_jt16 GZ_IP=127.0.0.1
/usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py"
gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
/usr/bin/python3 "$TASK_ROOT/scripts/build_world.py"
gz sim -s -r --headless-rendering "$TASK_ROOT/worlds/jt16_smoke.sdf" > "$TASK_ROOT/results/gazebo-smoke.log" 2>&1 &
SIM_PID=$!
ros2 run ros_gz_bridge parameter_bridge '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > "$TASK_ROOT/results/bridge-isolated.log" 2>&1 &
BRIDGE_PID=$!
trap 'kill "$SIM_PID" "$BRIDGE_PID" 2>/dev/null || true; wait || true' EXIT
/usr/bin/python3 "$TASK_ROOT/scripts/check_sensors.py"
