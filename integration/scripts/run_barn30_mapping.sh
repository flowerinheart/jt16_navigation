#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUTPUT_ROOT=${1:-"$TASK_ROOT/results/v0/barn30"}
MANIFEST="$TASK_ROOT/benchmarks/manifests/barn30.json"
mkdir -p "$OUTPUT_ROOT"
mapfile -t WORLD_IDS < <(jq -r '.entries[].world_id' "$MANIFEST")
for world_id in "${WORLD_IDS[@]}"; do
  run_dir=$(printf '%s/world_%03d/mapping' "$OUTPUT_ROOT" "$world_id")
  if [ -s "$run_dir/mapping-result.json" ];then
    /usr/bin/python3 "$TASK_ROOT/scripts/run_provenance.py" "$run_dir"
    if [ "$(jq -r '.terminal_state' "$run_dir/mapping-result.json")" = COMPLETE ] && [ -s "$run_dir/nav-map/map.yaml" ] && [ -s "$run_dir/map-bundle-metadata.json" ];then
      continue
    fi
    echo "Incomplete mapping bundle: $run_dir; use a new output root" >&2
    exit 1
  fi
  mkdir -p "$run_dir"
if ! bash "$TASK_ROOT/scripts/run_barn_mapping.sh" "$world_id" "$run_dir"; then
    if [ ! -s "$run_dir/mapping-result.json" ];then /usr/bin/python3 - "$world_id" "$run_dir" <<'PY'
import json,sys
from pathlib import Path
world_id=int(sys.argv[1]); output=Path(sys.argv[2])
(output/'mapping-result.json').write_text(json.dumps({'world_id':world_id,'terminal_state':'PROCESS_FAILURE'},indent=2)+'\n')
PY
    fi
    exit 1
  fi
done
/usr/bin/python3 "$TASK_ROOT/scripts/score_v0.py" --results "$OUTPUT_ROOT" --mapping-only
