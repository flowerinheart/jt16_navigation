#!/usr/bin/env bash
set -euo pipefail
PROFILE=${1:?profile required}
BACKEND=${2:?backend required}
ROOT=/workspace/jt16
mkdir -p "$ROOT"
# Keep the host's paused evaluation, vendored sources and build caches isolated.
tar -C /source --exclude='./.git' --exclude='./external' --exclude='./build' --exclude='./install' \
  --exclude='./log' --exclude='./results' --exclude='./.setup' --exclude='./integration/results' \
  --exclude='./integration/deps' --exclude='./integration/archive' --exclude='./integration/build' \
  --exclude='./integration/install' --exclude='./integration/log' --exclude='./integration/models' \
  --exclude='./integration/worlds' --exclude='./integration/benchmarks/generated' \
  --exclude='__pycache__' -cf - . | tar -C "$ROOT" -xf -
cd "$ROOT"
mkdir -p deployment-evidence
set +u
source /opt/ros/jazzy/setup.bash
set -u
bash scripts/fetch_dependencies.sh deploy
ROLES=(deploy)
if [[ "$PROFILE" == benchmark ]]; then
  bash scripts/fetch_dependencies.sh benchmark
  ROLES+=(benchmark)
  /usr/bin/python3 integration/scripts/build_robot.py
  gz sdf -p integration/models/xlerobot_jt16.urdf > integration/models/xlerobot_jt16.sdf
  /usr/bin/python3 integration/scripts/build_barn_world.py --world-id 22 --output integration/benchmarks/generated/barn_022.sdf
  /usr/bin/python3 integration/scripts/build_arena_world.py --world small_warehouse --obstacles 5 --seed 0 --output integration/benchmarks/generated/arena_small_warehouse_5_seed00.sdf
fi
/usr/bin/python3 scripts/check_dependencies.py "${ROLES[@]}"
USE_LOCAL_ROS_FALLBACK=0 bash scripts/build.sh runtime
set +u
source install/setup.bash
set -u
/usr/bin/python3 -m unittest discover -s scripts/tests -p 'test_*.py'
if [[ "$PROFILE" == benchmark ]]; then
  /usr/bin/python3 -m unittest discover -s integration/tests -p 'test_*.py'
else
  /usr/bin/python3 -m unittest discover -s integration/tests -p 'test_motion_prediction.py'
fi
colcon test --executor sequential --base-paths src --packages-select jt16_bringup --event-handlers console_direct+
colcon test-result --verbose
/usr/bin/python3 integration/tests/ros_interface_smoke.py
/usr/bin/python3 scripts/check_deployment.py "$PROFILE" "$BACKEND" > deployment-evidence/runtime.json
cp /opt/jt16-setup/evidence/system-packages.tsv deployment-evidence/
cp /opt/jt16-setup/evidence/ubuntu.sources deployment-evidence/
echo "Runtime and installed-node checks passed: $ROOT/deployment-evidence"
