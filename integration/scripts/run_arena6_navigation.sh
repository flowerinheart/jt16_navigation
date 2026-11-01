#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.."&&pwd);RESULTS=${1:-"$TASK_ROOT/results/v0/arena6"};JOBS=${JOBS:-2};QUEUE=$(mktemp /tmp/jt16-arena-queue.XXXXXX);trap 'rm -f "$QUEUE"' EXIT
jq -r '.entries[]|.world as $w|.obstacle_count as $o|.navigation_seeds[]|[$w,$o,.]|@tsv' "$TASK_ROOT/benchmarks/manifests/arena6.json">"$QUEUE";export TASK_ROOT RESULTS
/usr/bin/python3 "$TASK_ROOT/scripts/build_robot.py"
gz sdf -p "$TASK_ROOT/models/xlerobot_jt16.urdf" > "$TASK_ROOT/models/xlerobot_jt16.sdf"
export ROBOT_PREBUILT=1
run_one(){ w=$1;o=$2;s=$3;out=$(printf '%s/%s_obs%02d/seed_%02d' "$RESULTS" "$w" "$o" "$s");if [ -s "$out/nav2-goal.json" ];then /usr/bin/python3 "$TASK_ROOT/scripts/run_provenance.py" "$out" || exit 2;return;fi;mkdir -p "$out";bash "$TASK_ROOT/scripts/run_arena_navigation.sh" "$w" "$o" "$s" "$out"||true;};export -f run_one
xargs -P "$JOBS" -n 3 bash -c 'run_one "$@"' _ <"$QUEUE"
/usr/bin/python3 "$TASK_ROOT/scripts/score_arena.py" --results "$RESULTS"
