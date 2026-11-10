#!/usr/bin/env bash
set -euo pipefail
PROFILE=${1:?profile required}
BACKEND=${2:?backend required}
ROOT=/workspace/jt16
export JT16_DATA_HOME="${JT16_DATA_HOME:-/workspace/.jt16-navigation}"
mkdir -p "$ROOT"
# Keep the host's paused evaluation, vendored sources and build caches isolated.
tar -C /source --exclude='./.git' --exclude='./external' --exclude='./build' --exclude='./install' \
  --exclude='./log' --exclude='./results' --exclude='./.setup' --exclude='./integration/results' \
  --exclude='./integration/deps' --exclude='./integration/archive' --exclude='./integration/build' \
  --exclude='./integration/install' --exclude='./integration/log' --exclude='./integration/models' \
  --exclude='./integration/worlds' --exclude='./integration/benchmarks/generated' \
  --exclude='./integration/diagnostics' --exclude='./docs/reviews' --exclude='./docs/v1/local' \
  --exclude='./docs/v1/assets' --exclude='./docs/v0/assets' --exclude='./docs/v0/evidence.json' \
  --exclude='./docs/v0/report.html' --exclude='./demos/v0' --exclude='./demos/v1-candidate-g' \
  --exclude='./demos/*.json' --exclude='./demos/*.png' --exclude='./docs/v1/diagnosis-plan.md' \
  --exclude='./docs/v1/progress.md' --exclude='./docs/v1/pause-checkpoint.md' \
  --exclude='./docs/deployment-evidence/*.json' --exclude='./deployment-evidence' \
  --exclude='./.aws' --exclude='./.codex/sessions' --exclude='./.codex/log' \
  --exclude='./.codex/history.jsonl' --exclude='./.claude/projects' \
  --exclude='./.claude_code/sessions' --exclude='./.cursor/workspaceStorage' \
  --exclude='./.jt16-navigation' --exclude='__pycache__' -cf - . | tar -C "$ROOT" -xf -
cd "$ROOT"
EVIDENCE=$(/usr/bin/python3 integration/scripts/artifact_paths.py reports --label deployment)
mkdir -p "$EVIDENCE"
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
/usr/bin/python3 scripts/check_deployment.py "$PROFILE" "$BACKEND" > "$EVIDENCE/runtime.json"
cp /opt/jt16-setup/evidence/system-packages.tsv "$EVIDENCE/"
cp /opt/jt16-setup/evidence/ubuntu.sources "$EVIDENCE/"
echo "Runtime and installed-node checks passed: $EVIDENCE"
