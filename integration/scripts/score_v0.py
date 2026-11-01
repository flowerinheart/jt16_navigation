#!/usr/bin/env python3
"""Aggregate V0 terminal states and available mapping/navigation metrics."""
import argparse
import json
from pathlib import Path
import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument('--results', type=Path, required=True)
parser.add_argument('--mapping-only', action='store_true')
args = parser.parse_args()
mapping = [json.loads(path.read_text()) for path in sorted(args.results.glob('world_*/mapping/mapping-result.json'))]
map_metrics = [json.loads(path.read_text()) for path in sorted(args.results.glob('world_*/mapping/map-metrics.json'))]
resources = [json.loads(path.read_text()) for path in sorted(args.results.glob('world_*/mapping/resources.json'))]
terminal = {}
for run in mapping:
    terminal[run.get('terminal_state', 'MISSING')] = terminal.get(run.get('terminal_state', 'MISSING'), 0) + 1


def values(name):
    return np.asarray([run[name] for run in mapping if name in run], dtype=float)


summary = {'mapping_runs_scored': len(mapping), 'mapping_terminal_states': terminal}
for metric in ('ate_rmse_m', 'translation_p95_m', 'yaw_rmse_rad','initialization_time_s','odometry_update_rate_hz','rpe_translation_rmse_m_1s','rpe_yaw_rmse_rad_1s','static_drift_final_m','static_drift_p95_m','static_yaw_drift_rad'):
    data = values(metric)
    if len(data):
        summary[metric] = {'mean': float(data.mean()), 'median': float(np.median(data)),
                           'p95': float(np.percentile(data, 95)), 'count': int(len(data))}
for metric in ('occupied_precision','occupied_recall','occupied_f1','occupied_iou','wall_distance_mean_m','wall_distance_p95_m'):
    data=np.asarray([run[metric] for run in map_metrics if run.get(metric) is not None],dtype=float)
    if len(data):summary[metric]={'mean':float(data.mean()),'median':float(np.median(data)),'p95':float(np.percentile(data,95)),'count':int(len(data))}
for metric in ('max_rss_mb','mean_cpu_percent','real_time_factor'):
    data=np.asarray([run[metric] for run in resources if run.get(metric) is not None],dtype=float)
    if len(data):summary[metric]={'mean':float(data.mean()),'median':float(np.median(data)),'p95':float(np.percentile(data,95)),'count':int(len(data))}
navigation = [json.loads(path.read_text()) for path in sorted(args.results.glob('world_*/navigation/seed_*/nav2-goal.json'))]
if navigation:
    nav_terminal = {}
    for run in navigation:
        nav_terminal[run.get('result', 'MISSING')] = nav_terminal.get(run.get('result', 'MISSING'), 0) + 1
    successes = sum(bool(run.get('scored_success')) for run in navigation)
    collisions = sum(bool(run.get('collision')) for run in navigation)
    timeouts = sum(run.get('result') in ('TIMEOUT', 'CANCELED') for run in navigation)
    summary.update({'navigation_runs_scored': len(navigation), 'navigation_terminal_states': nav_terminal,
                    'success_rate': successes / len(navigation),
                    'collision_free_success_rate': sum(bool(run.get('scored_success')) and not run.get('collision', False) for run in navigation) / len(navigation),
                    'collision_rate': collisions / len(navigation), 'timeout_rate': timeouts / len(navigation),
                    'barn_score_mean': float(np.mean([run.get('barn_score', 0.0) for run in navigation]))})
    summary['localization_success_rate']=sum(bool(run.get('localization_success')) for run in navigation)/len(navigation)
    for metric in ('elapsed_s', 'final_goal_distance_m', 'path_ratio', 'minimum_obstacle_clearance_m','localization_initialization_latency_s','localization_translation_error_m','localization_yaw_error_rad','map_odom_max_translation_jump_m','map_odom_max_yaw_jump_rad'):
        data=np.asarray([run[metric] for run in navigation if run.get(metric) is not None],dtype=float)
        if len(data):summary[metric]={'mean':float(data.mean()),'median':float(np.median(data)),'p95':float(np.percentile(data,95)),'count':int(len(data))}
(args.results / 'mapping-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2))
