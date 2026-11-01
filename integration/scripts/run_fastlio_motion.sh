#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
REPO_ROOT=$(cd "$TASK_ROOT/.." && pwd)
OUTPUT=${1:-"$TASK_ROOT/results/$(date +%Y%m%d-%H%M%S)-fastlio-motion"}
PARTITION="jt16_fastlio_$$"
ROS_LOG_DIR_TASK=$(mktemp -d /tmp/jt16-fastlio-log.XXXXXX)
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
export GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 ROS_LOG_DIR="$ROS_LOG_DIR_TASK"
export ROS_DOMAIN_ID=11 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export LD_LIBRARY_PATH="$REPO_ROOT/integration/deps/root/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p "$OUTPUT"
/usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py"
gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
/usr/bin/python3 "$TASK_ROOT/scripts/build_world.py" --dynamic
gz sim -s -r --headless-rendering "$TASK_ROOT/worlds/jt16_smoke.sdf" > "$OUTPUT/gazebo.log" 2>&1 & SIM_PID=$!
ros2 run ros_gz_bridge parameter_bridge \
 '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
 '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
 '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' --ros-args -r /lidar_imu:=/lidar_imu_raw > "$OUTPUT/bridge.log" 2>&1 & BRIDGE_PID=$!
/usr/bin/python3 "$TASK_ROOT/scripts/instant_cloud_adapter.py" > "$OUTPUT/adapter.log" 2>&1 & ADAPTER_PID=$!
/usr/bin/python3 "$TASK_ROOT/scripts/sim_imu_adapter.py" > "$OUTPUT/imu-adapter.log" 2>&1 & IMU_ADAPTER_PID=$!
ros2 launch fast_lio mapping_jt16.launch.py use_sim_time:=true rviz:=false config_file:="$TASK_ROOT/config/jt16_instant.yaml" > "$OUTPUT/fastlio.log" 2>&1 & FASTLIO_PID=$!
cleanup(){ kill "$SIM_PID" "$BRIDGE_PID" "$ADAPTER_PID" "$IMU_ADAPTER_PID" "$FASTLIO_PID" 2>/dev/null || true; wait "$SIM_PID" "$BRIDGE_PID" "$ADAPTER_PID" "$IMU_ADAPTER_PID" "$FASTLIO_PID" 2>/dev/null || true; }
trap cleanup EXIT
/usr/bin/python3 "$TASK_ROOT/scripts/validate_fastlio_motion.py" --output "$OUTPUT" | tee "$OUTPUT/validation.log"
ros2 service call /map_save std_srvs/srv/Trigger '{}' | tee "$OUTPUT/map-save.log"
if [ -f "$REPO_ROOT/external/runtime/FAST_LIO_Hesai/PCD/scans.pcd" ]; then
  cp "$REPO_ROOT/external/runtime/FAST_LIO_Hesai/PCD/scans.pcd" "$OUTPUT/fastlio-map.pcd"
fi
echo "$OUTPUT"
