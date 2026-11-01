#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);a=p.parse_args();runs=[json.loads(x.read_text()) for x in sorted(a.results.glob('*_obs*/seed_*/nav2-goal.json'))];states={}
resources=[json.loads(x.read_text()) for x in sorted(a.results.glob('*_obs*/seed_*/resources.json'))]
for run in runs:states[run.get('result','MISSING')]=states.get(run.get('result','MISSING'),0)+1
s={'runs_scored':len(runs),'terminal_states':states}
s['unscorable_runs']=sum(x.get('scoring_valid') is not True for x in runs)
if runs:
 successes=sum(x.get('result')=='SUCCEEDED' and x.get('scored_success') is True and x.get('scoring_valid') is True and x.get('localization_success') is True and x.get('collision') is False and x.get('input_failure') is None for x in runs)
 s.update({'success_rate':successes/len(runs),'collision_free_success_rate':successes/len(runs),'collision_rate':sum(bool(x.get('collision')) for x in runs)/len(runs),'timeout_rate':sum(x.get('result') in ('TIMEOUT','CANCELED') for x in runs)/len(runs)})
 s['localization_success_rate']=sum(bool(x.get('localization_success')) for x in runs)/len(runs)
 for metric in ('elapsed_s','final_goal_distance_m','path_ratio','minimum_obstacle_clearance_m','mean_obstacle_clearance_m','mean_acceleration_m_s2','mean_jerk_m_s3','mean_curvature_rad_m','roughness','localization_initialization_latency_s','localization_translation_error_m','localization_yaw_error_rad','map_odom_max_translation_jump_m','map_odom_max_yaw_jump_rad'):
  values=np.asarray([x[metric] for x in runs if x.get(metric) is not None],dtype=float)
  if len(values):s[metric]={'mean':float(values.mean()),'median':float(np.median(values)),'p95':float(np.percentile(values,95)),'count':int(len(values))}
 for metric in ('max_rss_mb','mean_cpu_percent','real_time_factor'):
  values=np.asarray([x[metric] for x in resources if x.get(metric) is not None],dtype=float)
  if len(values):s[metric]={'mean':float(values.mean()),'median':float(np.median(values)),'p95':float(np.percentile(values,95)),'count':int(len(values))}
(a.results/'arena-summary.json').write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s,indent=2))
