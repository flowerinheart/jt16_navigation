#!/usr/bin/env python3
"""Audit corrected V0 evidence; historical presence-only PASS is not acceptance."""
import argparse,json
import hashlib
import math
from benchmark_geometry import tf_valid
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();repo=Path(__file__).resolve().parents[2];barn_manifest=json.loads((repo/'integration/benchmarks/manifests/barn30.json').read_text());arena_manifest=json.loads((repo/'integration/benchmarks/manifests/arena6.json').read_text())
checks=[]
quick=a.root/'quick-summary.json'
check_quick=json.loads(quick.read_text()) if quick.exists() else {}
def check(name,passed,evidence):
 checks.append({'name':name,'pass':bool(passed),'evidence':evidence})
expected_quick=json.loads((repo/'integration/benchmarks/manifests/quick3.json').read_text())
check('Frozen Quick-3 promotion',check_quick.get('promotion_gate_pass') is True and check_quick.get('scoring_schema')==2 and check_quick.get('navigation_timeout_s')==expected_quick['navigation_timeout_s'] and check_quick.get('goal_tolerance_m')==1.0 and {item.get('world_id') for item in check_quick.get('runs',[])}=={item['world_id'] for item in expected_quick['entries']},str(quick))
calibration=repo/'integration/config/hardware-calibration.json'
cal=json.loads(calibration.read_text()) if calibration.exists() else {}
check('Measured hardware calibration',cal.get('measured') is True and bool(cal.get('measurement_report')) and all(len(cal.get(edge,{}).get('translation_m',[]))==3 and len(cal.get(edge,{}).get('rotation_xyzw',[]))==4 for edge in ('base_to_lidar','lidar_to_imu')),str(calibration))
mapping=[];barn_nav=[]
for entry in barn_manifest['entries']:
 w=entry['world_id'];run=a.root/f'barn30/world_{w:03d}/mapping';result=run/'mapping-result.json';data=json.loads(result.read_text()) if result.exists() else {};mapping.append(data);check(f'BARN {w} mapping terminal',data.get('terminal_state') in ('COMPLETE','TIMEOUT','PROCESS_FAILURE'),str(result));check(f'BARN {w} map bundle',all((run/x).is_file() and (run/x).stat().st_size>0 for x in ('map.pcd','nav-map/map.yaml','nav-map/map.pgm','map-bundle-metadata.json','map-metrics.json')),str(run))
 check(f'BARN {w} mapping metrics',all(isinstance(data.get(k),(int,float)) and math.isfinite(data[k]) for k in ('ate_rmse_m','rpe_translation_rmse_m_1s','initialization_time_s','odometry_update_rate_hz')),str(result))
 metadata=run/'map-bundle-metadata.json';bundle=json.loads(metadata.read_text()) if metadata.exists() else {}
 check(f'BARN {w} map hashes',len(bundle.get('files',{}))>=5 and all((repo/item['path']).is_file() and hashlib.sha256((repo/item['path']).read_bytes()).hexdigest()==item['sha256'] for item in bundle.get('files',{}).values()),str(metadata))
 for seed in entry['navigation_seeds']:
  path=a.root/f'barn30/world_{w:03d}/navigation/seed_{seed:02d}/nav2-goal.json';data=json.loads(path.read_text()) if path.exists() else {};barn_nav.append(data);check(f'BARN {w} seed {seed} terminal',data.get('result') in ('SUCCEEDED','FAILED','TIMEOUT','CANCELED','PROCESS_FAILURE','MISSING_MAP'),str(path))
arena=[]
for entry in arena_manifest['entries']:
 for seed in entry['navigation_seeds']:
  path=a.root/f'arena6/{entry["world"]}_obs{entry["obstacle_count"]:02d}/seed_{seed:02d}/nav2-goal.json';data=json.loads(path.read_text()) if path.exists() else {};arena.append(data);check(f'Arena {entry["world"]}/{entry["obstacle_count"]} seed {seed} terminal',data.get('result') in ('SUCCEEDED','FAILED','TIMEOUT','CANCELED','PROCESS_FAILURE'),str(path))
tf=list(a.root.glob('barn30/world_*/navigation/seed_*/tf-audit.json'))+list(a.root.glob('arena6/*/seed_*/tf-audit.json'));check('TF owners and static edges',len(tf)==210 and all(json.loads(x.read_text()).get('observed_topics')==['/tf','/tf_static'] and tf_valid(json.loads(x.read_text()).get('edges',{})) for x in tf),f'{len(tf)}/210 audits')
for result in list(a.root.glob('barn30/world_*/navigation/seed_*/nav2-goal.json'))+list(a.root.glob('arena6/*/seed_*/nav2-goal.json')):
 data=json.loads(result.read_text());run=result.parent;provenance=run/'run-provenance.json'
 identity=json.loads(provenance.read_text()) if provenance.exists() else {}
 hashes=identity.get('sources',{})
 check(f'{run.name} executed identity',identity.get('schema')==2 and bool(hashes) and all((run/'executed-inputs'/path).is_file() and hashlib.sha256((run/'executed-inputs'/path).read_bytes()).hexdigest()==value for path,value in hashes.items()),str(provenance))
 check(f'{run.name} scoring validity',data.get('scoring_valid') is True or (data.get('result') in ('PROCESS_FAILURE','MISSING_MAP') and data.get('scored_success') is False),str(result))
mapping_count=sum(bool(x) for x in mapping);barn_count=sum(bool(x) for x in barn_nav);arena_count=sum(bool(x) for x in arena);check('30 mapping terminal records',mapping_count==30,str(mapping_count));check('150 BARN navigation terminal records',barn_count==150,str(barn_count));check('60 Arena navigation terminal records',arena_count==60,str(arena_count))
report={'pass':all(x['pass'] for x in checks),'checks':checks,'counts':{'mapping':mapping_count,'barn_navigation':barn_count,'arena_navigation':arena_count,'tf_audits':len(tf)}};a.root.mkdir(parents=True,exist_ok=True);(a.root/'v0-audit.json').write_text(json.dumps(report,indent=2)+'\n');lines=['# V0 acceptance audit','',f'Overall: **{"PASS" if report["pass"] else "INCOMPLETE"}**','',f'- Mapping records: {mapping_count}/30',f'- BARN navigation records: {barn_count}/150',f'- Arena navigation records: {arena_count}/60',f'- TF audits: {len(tf)}/210','', '## Checks',''];lines.extend(f'- [{"x" if item["pass"] else " "}] {item["name"]}: `{item["evidence"]}`' for item in checks);(a.root/'v0-audit.md').write_text('\n'.join(lines)+'\n');print(json.dumps(report['counts'],indent=2));raise SystemExit(0 if report['pass'] else 1)
