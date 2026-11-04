#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd);REPO_ROOT=$(cd "$TASK_ROOT/.." && pwd)
WORLD_ID=${1:?world id required};SEED=${2:?seed required};MAP_YAML=$(realpath "${3:?map yaml required}");OUTPUT=${4:?output directory required}
if [ -e "$OUTPUT" ] && { [ ! -d "$OUTPUT" ] || [ -n "$(find "$OUTPUT" -mindepth 1 -maxdepth 1 -print -quit)" ]; };then echo "Refusing to overwrite nonempty output: $OUTPUT" >&2;exit 2;fi
NAV_TIMEOUT=${NAV_TIMEOUT:-150}
USE_MAPPING_WAYPOINTS=${USE_MAPPING_WAYPOINTS:-0}
PROCESS_TIMEOUT=${PROCESS_TIMEOUT:-$((NAV_TIMEOUT * 10 + 60))}
WORLD="$TASK_ROOT/benchmarks/generated/barn_${WORLD_ID}.sdf";SOURCE_WORLD="$REPO_ROOT/external/benchmarks/the-barn-challenge/jackal_helper/worlds/BARN/world_${WORLD_ID}.world";PATH_FILE="$REPO_ROOT/external/benchmarks/the-barn-challenge/jackal_helper/worlds/BARN/path_files/path_${WORLD_ID}.npy"
SHORTEST=$(/usr/bin/python3 "$TASK_ROOT/scripts/barn_path_length.py" "$PATH_FILE")
MAPPING_RESULT="$(dirname "$(dirname "$MAP_YAML")")/mapping-result.json"
MAPPING_TRAJECTORY="$(dirname "$MAPPING_RESULT")/mapping-trajectories.npz"
NAV_GOAL=$(jq -er 'select(.terminal_state == "COMPLETE" and .map_valid != false)|select(.lio_final_xyyaw|type=="array" and length>=2)|.lio_final_xyyaw[0:2]|@tsv' "$MAPPING_RESULT" 2>/dev/null || true)
if [ -z "$NAV_GOAL" ]; then
  mkdir -p "$OUTPUT"
  /usr/bin/python3 - "$WORLD_ID" "$SEED" "$OUTPUT" <<'PY'
import json,sys
from pathlib import Path
world,seed,out=int(sys.argv[1]),int(sys.argv[2]),Path(sys.argv[3])
(out/'nav2-goal.json').write_text(json.dumps({'world_id':world,'seed':seed,'result':'MISSING_LIO_ODOMETRY','scored_success':False,'barn_score':0.0},indent=2)+'\n')
PY
  exit 1
fi
read -r _MAPPING_GOAL_X _MAPPING_GOAL_Y <<< "$NAV_GOAL"
# BARN starts at (-2.25, 3.0) facing +Y and scores (-2.25, 13.0), so the
# saved-map navigation goal is fixed in the initial local frame.
NAV_GOAL_X=10.0;NAV_GOAL_Y=0.0
PARTITION="jt16_barn_nav_${WORLD_ID}_${SEED}_$$";ROS_LOG_DIR_TASK=$(mktemp -d /tmp/jt16-barn-nav-log.XXXXXX)
set +u;source /opt/ros/jazzy/setup.bash;source "$REPO_ROOT/install/setup.bash";set -u
export GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 ROS_LOG_DIR="$ROS_LOG_DIR_TASK" LD_LIBRARY_PATH="$REPO_ROOT/integration/deps/root/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_OVERRIDE:-$((1+(WORLD_ID*7+SEED*31)%100))} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
mkdir -p "$OUTPUT"
PARAMS="${NAV2_PARAMS_FILE:-$OUTPUT/nav2-params.yaml}"
if [ -z "${NAV2_PARAMS_FILE:-}" ];then /usr/bin/python3 "$TASK_ROOT/scripts/build_mppi_parameters.py" --base "$REPO_ROOT/install/jt16_bringup/share/jt16_bringup/config/nav2_params.yaml" --output "$PARAMS";fi
if [ ! -s "$WORLD" ];then /usr/bin/python3 "$TASK_ROOT/scripts/build_barn_world.py" --world-id "$WORLD_ID" --output "$WORLD";fi
setsid gz sim -s -r --headless-rendering --seed "$SEED" "$WORLD" > "$OUTPUT/gazebo.log" 2>&1 & P1=$!
setsid ros2 run ros_gz_bridge parameter_bridge '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' --ros-args -r /lidar_imu:=/lidar_imu_raw > "$OUTPUT/bridge.log" 2>&1 & P2=$!
PYTHON=/usr/bin/python3;LIB="$REPO_ROOT/install/jt16_bringup/lib/jt16_bringup"
setsid "$PYTHON" "$LIB/instant_cloud_adapter.py" > "$OUTPUT/cloud-adapter.log" 2>&1 & P3=$!;setsid "$PYTHON" "$LIB/sim_imu_adapter.py" > "$OUTPUT/imu-adapter.log" 2>&1 & P4=$!;setsid "$PYTHON" "$LIB/project_nav_scan.py" > "$OUTPUT/nav-scan.log" 2>&1 & P5=$!
setsid "$PYTHON" "$LIB/predict_obstacles.py" --ros-args -p use_sim_time:=true > "$OUTPUT/obstacle-prediction.log" 2>&1 & PP=$!
setsid ros2 launch fast_lio mapping_jt16.launch.py use_sim_time:=true rviz:=false config_file:="$TASK_ROOT/config/jt16_instant.yaml" > "$OUTPUT/fastlio.log" 2>&1 & P6=$!;setsid "$PYTHON" "$LIB/lio_odom_bridge.py" > "$OUTPUT/lio-bridge.log" 2>&1 & P7=$!
setsid ros2 launch nav2_bringup localization_launch.py map:="$MAP_YAML" params_file:="$PARAMS" use_sim_time:=true > "$OUTPUT/localization.log" 2>&1 & P8=$!;setsid ros2 launch nav2_bringup navigation_launch.py params_file:="$PARAMS" use_sim_time:=true > "$OUTPUT/navigation.log" 2>&1 & P9=$!
/usr/bin/python3 "$TASK_ROOT/scripts/resource_monitor.py" --pids "$P1" "$P2" "$P3" "$P4" "$P5" "$P6" "$P7" "$P8" "$P9" "$PP" --output "$OUTPUT/resources.json" > "$OUTPUT/resources.log" 2>&1 & PM=$!
cleanup(){
  kill -USR1 "$PM" 2>/dev/null || true; wait "$PM" 2>/dev/null || true
  local pids=("$P1" "$P2" "$P3" "$P4" "$P5" "$P6" "$P7" "$P8" "$P9" "$PP")
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for _ in {1..20}; do
    local alive=0
    for pid in "${pids[@]}"; do if kill -0 -- "-$pid" 2>/dev/null; then alive=1; break; fi; done
    (( alive == 0 )) && break
    sleep 0.1
  done
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done; wait "${pids[@]}" 2>/dev/null || true
};trap cleanup EXIT
sleep 10
"$PYTHON" "$TASK_ROOT/scripts/audit_tf_publishers.py" --output "$OUTPUT/tf-audit.json" --duration 15 > "$OUTPUT/tf-audit.log" 2>&1 & TF_AUDIT=$!
WAYPOINT_ARGS=(); if [ "$USE_MAPPING_WAYPOINTS" = 1 ] && [ -s "$MAPPING_TRAJECTORY" ]; then WAYPOINT_ARGS=(--waypoints-npz "$MAPPING_TRAJECTORY"); fi
set +e;timeout -k 5s "${PROCESS_TIMEOUT}s" "$PYTHON" "$TASK_ROOT/scripts/run_nav2_goal.py" --output "$OUTPUT" --nav2-params-file "$PARAMS" --map-yaml "$MAP_YAML" --simulation-world "$WORLD" --x "$NAV_GOAL_X" --y "$NAV_GOAL_Y" "${WAYPOINT_ARGS[@]}" --score-goal-x -2.25 --score-goal-y 13 --ground-truth-is-world --ground-truth-origin-x -2.25 --ground-truth-origin-y 3 --ground-truth-origin-yaw 1.5707963267948966 --timeout "$NAV_TIMEOUT" --goal-tolerance 1.0 --barn-world "$SOURCE_WORLD" --shortest-path "$SHORTEST" > "$OUTPUT/goal.log" 2>&1;STATUS=$?;set -e
if [ ! -s "$OUTPUT/nav2-goal.json" ];then "$PYTHON" - "$WORLD_ID" "$SEED" "$STATUS" "$OUTPUT" <<'PY'
import json,sys
from pathlib import Path
world,seed,status,out=int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]),Path(sys.argv[4]);state='TIMEOUT' if status==124 else 'PROCESS_FAILURE';(out/'nav2-goal.json').write_text(json.dumps({'world_id':world,'seed':seed,'result':state,'scored_success':False,'barn_score':0.0},indent=2)+'\n')
PY
fi
[ -z "$TF_AUDIT" ] || wait "$TF_AUDIT" || true
kill -USR1 "$PM" 2>/dev/null || true;wait "$PM" 2>/dev/null || true
exit "$STATUS"
