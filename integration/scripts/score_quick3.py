#!/usr/bin/env python3
"""Aggregate the frozen quick-3 physical navigation gate."""
import argparse
import json
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--goal-tolerance', type=float, default=1.0)
args = parser.parse_args()
if args.goal_tolerance != 1.0:
    raise ValueError('Quick-3 goal tolerance is frozen at 1.0 m')

runs = []
manifest=json.loads((Path(__file__).resolve().parents[1]/'benchmarks/manifests/quick3.json').read_text())
expected={(entry['world_id'],entry['seed']) for entry in manifest['entries']}
for result_path in sorted(args.root.glob('world_*/navigation/seed_00/nav2-goal.json')):
    result = json.loads(result_path.read_text())
    physical_success = (
        result.get('final_goal_distance_m', float('inf')) <= args.goal_tolerance
        and result.get('collision') is False
        and result.get('scoring_valid') is True
        and result.get('scored_success') is True
        and result.get('input_failure') is None
        and result.get('localization_success') is True
        and result.get('gt_samples',0)>1
        and result.get('scoring_schema')==2
        and result.get('navigation_timeout_s')==manifest['navigation_timeout_s']
    )
    runs.append({
        'world_id': int(result_path.parts[-4].split('_')[1]),
        'action_result': result.get('result'),
        'physical_success': physical_success,
        'final_goal_distance_m': result.get('final_goal_distance_m'),
        'collision': result.get('collision'),
        'barn_score': result.get('barn_score', 0.0) if physical_success else 0.0,
    })

count = len(runs)
summary = {
    'scoring_schema': 2,
    'navigation_timeout_s': manifest['navigation_timeout_s'],
    'goal_tolerance_m': args.goal_tolerance,
    'runs': runs,
    'run_count': count,
    'physical_successes': sum(run['physical_success'] for run in runs),
    'success_rate': sum(run['physical_success'] for run in runs) / count if count else 0.0,
    'process_failure_rate': sum(run['action_result'] == 'PROCESS_FAILURE' for run in runs) / count if count else 0.0,
}
summary['promotion_gate_pass'] = (
    {(run['world_id'],0) for run in runs} == expected and count == len(expected) and summary['success_rate'] == 1.0
    and summary['process_failure_rate'] == 0.0
    and all(run['action_result']=='SUCCEEDED' for run in runs)
)
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
