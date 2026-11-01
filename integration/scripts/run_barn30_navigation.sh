#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
RESULTS=${1:-"$TASK_ROOT/results/v0/barn30"}
JOBS=${JOBS:-2}
MANIFEST="$TASK_ROOT/benchmarks/manifests/barn30.json"
QUEUE=$(mktemp /tmp/jt16-barn-nav-queue.XXXXXX)
trap 'rm -f "$QUEUE"' EXIT
jq -r '.entries[]|.world_id as $w|.navigation_seeds[]|[$w,.]|@tsv' "$MANIFEST" > "$QUEUE"
export TASK_ROOT RESULTS
run_one(){
  world_id=$1;seed=$2;map=$(printf '%s/world_%03d/mapping/nav-map/map.yaml' "$RESULTS" "$world_id");out=$(printf '%s/world_%03d/navigation/seed_%02d' "$RESULTS" "$world_id" "$seed")
  if [ -s "$out/nav2-goal.json" ];then /usr/bin/python3 "$TASK_ROOT/scripts/run_provenance.py" "$out" || exit 2;return;fi
  mkdir -p "$out"
  if [ ! -s "$map" ];then /usr/bin/python3 - "$world_id" "$seed" "$out" <<'PY'
import json,sys
from pathlib import Path
w,s,o=int(sys.argv[1]),int(sys.argv[2]),Path(sys.argv[3]);(o/'nav2-goal.json').write_text(json.dumps({'world_id':w,'seed':s,'result':'MISSING_MAP','scored_success':False,'barn_score':0.0},indent=2)+'\n')
PY
else bash "$TASK_ROOT/scripts/run_barn_navigation.sh" "$world_id" "$seed" "$map" "$out" || true;fi
}
export -f run_one
xargs -P "$JOBS" -n 2 bash -c 'run_one "$@"' _ < "$QUEUE"
/usr/bin/python3 "$TASK_ROOT/scripts/score_v0.py" --results "$RESULTS"
