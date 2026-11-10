#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd);REPO_ROOT=$(cd "$TASK_ROOT/.." && pwd)
RESULTS_ROOT=$(/usr/bin/python3 "$TASK_ROOT/scripts/artifact_paths.py" evaluations)
WORLD_NAME=${1:?world required};OBSTACLES=${2:?obstacle count required};SEED=${3:?seed required};OUTPUT=${4:-"$RESULTS_ROOT/$(date +%Y%m%d-%H%M%S)-arena-${WORLD_NAME}-${OBSTACLES}-${SEED}"}
if [ -e "$OUTPUT" ] && { [ ! -d "$OUTPUT" ] || [ -n "$(find "$OUTPUT" -mindepth 1 -maxdepth 1 -print -quit)" ]; };then echo "Refusing to overwrite nonempty output: $OUTPUT" >&2;exit 2;fi
NAV_TIMEOUT=${NAV_TIMEOUT:-150};PROCESS_TIMEOUT=${PROCESS_TIMEOUT:-$((NAV_TIMEOUT*10+60))}
SCENARIO="$REPO_ROOT/external/benchmarks/arena-bench/simulator_setup/scenarios/${WORLD_NAME}_obs$(printf '%02d' "$OBSTACLES").json";MAP=$(realpath "$REPO_ROOT/external/benchmarks/arena-bench/simulator_setup/maps/$WORLD_NAME/map.yaml");WORLD="$TASK_ROOT/benchmarks/generated/arena_${WORLD_NAME}_${OBSTACLES}_seed$(printf '%02d' "$SEED").sdf"
read -r SX SY GX GY YAW < <(/usr/bin/python3 - "$SCENARIO" <<'PY'
import json,math,sys
s=json.load(open(sys.argv[1]));a=s['robot_position'];b=s['robot_goal'];print(a[0],a[1],b[0],b[1],math.atan2(b[1]-a[1],b[0]-a[0]))
PY
)
SHORTEST=$(/usr/bin/python3 "$TASK_ROOT/scripts/map_shortest_path.py" "$MAP" --start "$SX" "$SY" --goal "$GX" "$GY")
PARTITION="jt16_arena_${WORLD_NAME}_${OBSTACLES}_${SEED}_$$";LOG_DIR=$(mktemp -d /tmp/jt16-arena-log.XXXXXX)
set +u;source /opt/ros/jazzy/setup.bash;source "$REPO_ROOT/install/setup.bash";set -u
export GZ_PARTITION="$PARTITION" GZ_IP=127.0.0.1 ROS_LOG_DIR="$LOG_DIR" LD_LIBRARY_PATH="$REPO_ROOT/integration/deps/root/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
case "$WORLD_NAME" in small_warehouse) WORLD_CODE=1;;map2) WORLD_CODE=2;;map5) WORLD_CODE=3;;esac
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_OVERRIDE:-$((1+(WORLD_CODE*47+OBSTACLES*11+SEED*31)%100))} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
if [ "${ROBOT_PREBUILT:-0}" != 1 ];then
 /usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py"
 gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
fi
mkdir -p "$OUTPUT";/usr/bin/python3 "$TASK_ROOT/scripts/build_arena_world.py" --world "$WORLD_NAME" --obstacles "$OBSTACLES" --seed "$SEED" --output "$WORLD" > "$OUTPUT/world-build.log"
PARAMS="${NAV2_PARAMS_FILE:-$OUTPUT/nav2-params.yaml}"
if [ -z "${NAV2_PARAMS_FILE:-}" ];then /usr/bin/python3 "$TASK_ROOT/scripts/build_mppi_parameters.py" --base "$REPO_ROOT/install/jt16_bringup/share/jt16_bringup/config/nav2_params.yaml" --output "$PARAMS";fi
setsid ros2 run ros_gz_sim gzserver --ros-args -p world_sdf_file:="$WORLD" > "$OUTPUT/gazebo.log" 2>&1 & P1=$!
setsid ros2 run ros_gz_bridge parameter_bridge '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' --ros-args -r /lidar_imu:=/lidar_imu_raw > "$OUTPUT/bridge.log" 2>&1 & P2=$!
PY=/usr/bin/python3;LIB="$REPO_ROOT/install/jt16_bringup/lib/jt16_bringup";setsid "$PY" "$TASK_ROOT/scripts/sim_state_adapter.py" --scenario "$SCENARIO" --world-name "arena_${WORLD_NAME}_${OBSTACLES}_${SEED}" --seed "$SEED" --no-ground-truth --ros-args -p use_sim_time:=true > "$OUTPUT/sim-state.log" 2>&1 & P10=$!;setsid "$PY" "$LIB/instant_cloud_adapter.py" > "$OUTPUT/cloud-adapter.log" 2>&1 & P3=$!;setsid "$PY" "$LIB/sim_imu_adapter.py" > "$OUTPUT/imu-adapter.log" 2>&1 & P4=$!;setsid "$PY" "$LIB/project_nav_scan.py" > "$OUTPUT/nav-scan.log" 2>&1 & P5=$!
setsid ros2 launch fast_lio mapping_jt16.launch.py use_sim_time:=true rviz:=false config_file:="$TASK_ROOT/config/jt16_instant.yaml" > "$OUTPUT/fastlio.log" 2>&1 & P6=$!;setsid "$PY" "$LIB/lio_odom_bridge.py" > "$OUTPUT/lio-bridge.log" 2>&1 & P7=$!;setsid ros2 launch nav2_bringup localization_launch.py map:="$MAP" params_file:="$PARAMS" use_sim_time:=true > "$OUTPUT/localization.log" 2>&1 & P8=$!;setsid ros2 launch nav2_bringup navigation_launch.py params_file:="$PARAMS" use_sim_time:=true > "$OUTPUT/navigation.log" 2>&1 & P9=$!
setsid "$PY" "$LIB/predict_obstacles.py" --ros-args -p use_sim_time:=true -p "nav2_params_file:=$PARAMS" > "$OUTPUT/obstacle-prediction.log" 2>&1 & PP=$!
"$PY" "$TASK_ROOT/scripts/resource_monitor.py" --pids "$P1" "$P2" "$P3" "$P4" "$P5" "$P6" "$P7" "$P8" "$P9" "$P10" "$PP" --output "$OUTPUT/resources.json" > "$OUTPUT/resources.log" 2>&1 & PM=$!
cleanup(){ kill -USR1 "$PM" 2>/dev/null||true;wait "$PM" 2>/dev/null||true;local pids=("$P1" "$P2" "$P3" "$P4" "$P5" "$P6" "$P7" "$P8" "$P9" "$P10" "$PP");for pid in "${pids[@]}";do kill -TERM -- "-$pid" 2>/dev/null||true;done;for _ in {1..20};do local alive=0;for pid in "${pids[@]}";do if kill -0 -- "-$pid" 2>/dev/null;then alive=1;break;fi;done;((alive==0))&&break;sleep .1;done;for pid in "${pids[@]}";do kill -KILL -- "-$pid" 2>/dev/null||true;done;wait "${pids[@]}" 2>/dev/null||true;};trap cleanup EXIT;sleep 10
"$PY" "$TASK_ROOT/scripts/audit_tf_publishers.py" --output "$OUTPUT/tf-audit.json" --duration 15 > "$OUTPUT/tf-audit.log" 2>&1 & TF_AUDIT=$!
set +e;timeout -k 5s "${PROCESS_TIMEOUT}s" "$PY" "$TASK_ROOT/scripts/run_nav2_goal.py" --output "$OUTPUT" --nav2-params-file "$PARAMS" --map-yaml "$MAP" --simulation-world "$WORLD" --initial-x "$SX" --initial-y "$SY" --initial-yaw "$YAW" --x "$GX" --y "$GY" --ground-truth-is-world --localization-reference-is-world --timeout "$NAV_TIMEOUT" --goal-tolerance 1.0 --arena-scenario "$SCENARIO" --arena-world "$WORLD" --arena-seed "$SEED" --shortest-path "$SHORTEST" > "$OUTPUT/goal.log" 2>&1;STATUS=$?;set -e
if [ ! -s "$OUTPUT/nav2-goal.json" ];then "$PY" - "$WORLD_NAME" "$OBSTACLES" "$SEED" "$STATUS" "$OUTPUT" <<'PY'
import json,sys
from pathlib import Path
w,o,s,status,out=sys.argv[1],int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]),Path(sys.argv[5]);state='TIMEOUT' if status==124 else 'PROCESS_FAILURE';(out/'nav2-goal.json').write_text(json.dumps({'world':w,'obstacle_count':o,'seed':s,'result':state,'scored_success':False},indent=2)+'\n')
PY
fi
[ -z "$TF_AUDIT" ] || wait "$TF_AUDIT" || true
kill -USR1 "$PM" 2>/dev/null||true;wait "$PM" 2>/dev/null||true
exit "$STATUS"
