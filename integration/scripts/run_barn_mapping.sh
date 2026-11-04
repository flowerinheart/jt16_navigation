#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
REPO_ROOT=$(cd "$TASK_ROOT/.." && pwd)
WORLD_ID=${1:?world id required}
OUTPUT=${2:?output directory required}
if [ -e "$OUTPUT" ] && { [ ! -d "$OUTPUT" ] || [ -n "$(find "$OUTPUT" -mindepth 1 -maxdepth 1 -print -quit)" ]; };then echo "Refusing to overwrite nonempty output: $OUTPUT" >&2;exit 2;fi
if [ -e "$OUTPUT/mapping-result.json" ] || [ -e "$OUTPUT/map.pcd" ];then echo "Use a new output directory; historical mapping evidence is read-only" >&2;exit 2;fi
MAP_MAX_LINEAR=${MAP_MAX_LINEAR:-0.25}
MAP_MAX_ANGULAR=${MAP_MAX_ANGULAR:-0.7}
MAP_TIMEOUT=${MAP_TIMEOUT:-180}
WORLD="$TASK_ROOT/benchmarks/generated/barn_${WORLD_ID}.sdf"
PATH_FILE="$REPO_ROOT/external/benchmarks/the-barn-challenge/jackal_helper/worlds/BARN/path_files/path_${WORLD_ID}.npy"
PARTITION="jt16_barn_map_${WORLD_ID}_$$"
ROS_LOG_DIR_TASK=$(mktemp -d /tmp/jt16-barn-map-log.XXXXXX)
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
export GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 ROS_LOG_DIR="$ROS_LOG_DIR_TASK"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_OVERRIDE:-$((1+WORLD_ID*7%100))} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export LD_LIBRARY_PATH="$REPO_ROOT/integration/deps/root/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p "$OUTPUT"
PCD="$REPO_ROOT/external/runtime/FAST_LIO_Hesai/PCD/scans.pcd"
rm -f "$PCD" "$OUTPUT/mapping-result.json" "$OUTPUT/map.pcd" "$OUTPUT/map-bundle-metadata.json" "$OUTPUT/map-metrics.json"
rm -f "$OUTPUT/nav-map/map.yaml" "$OUTPUT/nav-map/map.pgm"
/usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py"
gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
/usr/bin/python3 "$TASK_ROOT/scripts/build_barn_world.py" --world-id "$WORLD_ID" --output "$WORLD"
setsid gz sim -s -r --headless-rendering "$WORLD" > "$OUTPUT/gazebo.log" 2>&1 & P1=$!
setsid ros2 run ros_gz_bridge parameter_bridge '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' --ros-args -r /lidar_imu:=/lidar_imu_raw > "$OUTPUT/bridge.log" 2>&1 & P2=$!
setsid /usr/bin/python3 "$TASK_ROOT/scripts/instant_cloud_adapter.py" > "$OUTPUT/cloud-adapter.log" 2>&1 & P3=$!
setsid /usr/bin/python3 "$TASK_ROOT/scripts/sim_imu_adapter.py" > "$OUTPUT/imu-adapter.log" 2>&1 & P4=$!
setsid ros2 launch fast_lio mapping_jt16.launch.py use_sim_time:=true rviz:=false config_file:="$TASK_ROOT/config/jt16_instant.yaml" > "$OUTPUT/fastlio.log" 2>&1 & P5=$!
/usr/bin/python3 "$TASK_ROOT/scripts/resource_monitor.py" --pids "$P1" "$P2" "$P3" "$P4" "$P5" --output "$OUTPUT/resources.json" > "$OUTPUT/resources.log" 2>&1 & PM=$!
cleanup(){
  kill -USR1 "$PM" 2>/dev/null || true
  wait "$PM" 2>/dev/null || true
  local pids=("$P1" "$P2" "$P3" "$P4" "$P5")
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..20}; do
    local alive=0
    for pid in "${pids[@]}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then alive=1; break; fi
    done
    (( alive == 0 )) && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
/usr/bin/python3 "$TASK_ROOT/scripts/follow_barn_path.py" --world-id "$WORLD_ID" --path "$PATH_FILE" --simulation-world "$WORLD" --output "$OUTPUT" --max-linear "$MAP_MAX_LINEAR" --max-angular "$MAP_MAX_ANGULAR" --timeout "$MAP_TIMEOUT"
timeout -k 5s 30s ros2 service call /map_save std_srvs/srv/Trigger '{}' > "$OUTPUT/map-save.log"
if [ -s "$PCD" ]; then
  cp "$PCD" "$OUTPUT/map.pcd"
  /usr/bin/python3 "$TASK_ROOT/scripts/pcd_to_occupancy.py" --pcd "$OUTPUT/map.pcd" --trajectory "$OUTPUT/mapping-trajectories.npz" --output "$OUTPUT/nav-map" > "$OUTPUT/map-conversion.log"
  /usr/bin/python3 "$TASK_ROOT/scripts/score_barn_map.py" --world "$REPO_ROOT/external/benchmarks/the-barn-challenge/jackal_helper/worlds/BARN/world_${WORLD_ID}.world" --map "$OUTPUT/nav-map/map.yaml" --output "$OUTPUT/map-metrics.json" > "$OUTPUT/map-scoring.log"
  /usr/bin/python3 "$TASK_ROOT/scripts/write_map_bundle_metadata.py" --run "$OUTPUT" --world-id "$WORLD_ID" > "$OUTPUT/map-metadata.log"
else
  echo "Missing or empty saved point cloud: $PCD" >&2
  exit 1
fi
kill -USR1 "$PM" 2>/dev/null || true;wait "$PM" 2>/dev/null || true
