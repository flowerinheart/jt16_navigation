#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
LOCAL_ROS_PREFIX="$ROOT/integration/deps/root/opt/ros/jazzy"
if [ -d "$LOCAL_ROS_PREFIX/share/pcl_ros" ]; then
  export CMAKE_PREFIX_PATH="$LOCAL_ROS_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$LOCAL_ROS_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
# Reference workspace is intentionally excluded to avoid duplicate ROS packages.
if [ "${1:-scaffold}" = runtime ]; then
  colcon build --base-paths src external/runtime/FAST_LIO_Hesai external/runtime/HesaiLidar_ROS_2.0 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
elif [ "${1:-scaffold}" = scaffold ]; then
  colcon build --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
else echo "usage: $0 scaffold|runtime" >&2; exit 2; fi
