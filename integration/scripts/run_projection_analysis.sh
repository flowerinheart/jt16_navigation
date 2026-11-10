#!/usr/bin/env bash
set -euo pipefail

TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
RESULTS_ROOT=$(/usr/bin/python3 "$TASK_ROOT/scripts/artifact_paths.py" evaluations)
OUTPUT=${1:-"$RESULTS_ROOT/$(date +%Y%m%d-%H%M%S)-projection-analysis"}
PITCH_UP=${2:-0.0}
WORLD_MODE=${3:-smoke}
PARTITION="jt16_projection_$$"
ROS_LOG_DIR_TASK=$(mktemp -d /tmp/jt16-ros-log.XXXXXX)

set +u
source /opt/ros/jazzy/setup.bash
set -u

export GZ_PARTITION="$PARTITION"
export GZ_IP=127.0.0.1
export ROS_LOG_DIR="$ROS_LOG_DIR_TASK"

/usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py" --pitch-up "$PITCH_UP"
gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
if [[ "$WORLD_MODE" == "coverage" ]]; then
  /usr/bin/python3 "$TASK_ROOT/scripts/build_world.py" --coverage
else
  /usr/bin/python3 "$TASK_ROOT/scripts/build_world.py"
fi
mkdir -p "$OUTPUT"

gz sim -s -r --headless-rendering "$TASK_ROOT/worlds/jt16_smoke.sdf" > "$OUTPUT/gazebo.log" 2>&1 &
SIM_PID=$!
ros2 run ros_gz_bridge parameter_bridge \
  '/jt16/points_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
  '/lidar_imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/top_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  > "$OUTPUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
/usr/bin/python3 "$TASK_ROOT/scripts/instant_cloud_adapter.py" > "$OUTPUT/adapter.log" 2>&1 &
ADAPTER_PID=$!
/usr/bin/python3 "$TASK_ROOT/scripts/project_nav_scan.py" --ros-args -p pitch_up:="$PITCH_UP" -p input_cloud:=/lidar_points > "$OUTPUT/nav-scan.log" 2>&1 &
SCAN_PID=$!

cleanup() {
  kill "$SIM_PID" "$BRIDGE_PID" "$ADAPTER_PID" "$SCAN_PID" 2>/dev/null || true
  wait "$SIM_PID" "$BRIDGE_PID" "$ADAPTER_PID" "$SCAN_PID" 2>/dev/null || true
}
trap cleanup EXIT

/usr/bin/python3 "$TASK_ROOT/scripts/capture_sensor_bundle.py" --output "$OUTPUT" --pitch-up "$PITCH_UP" | tee "$OUTPUT/capture.log"
/usr/bin/python3 "$TASK_ROOT/scripts/analyze_2d_projection.py" --input "$OUTPUT" --output "$OUTPUT" | tee "$OUTPUT/analysis.log"
if [[ "$WORLD_MODE" == "coverage" ]]; then
  /usr/bin/python3 "$TASK_ROOT/scripts/analyze_coverage_matrix.py" --input "$OUTPUT" --manifest "$TASK_ROOT/worlds/jt16_coverage_manifest.json" | tee "$OUTPUT/coverage.log"
fi

echo "$OUTPUT"
