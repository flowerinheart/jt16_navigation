#!/usr/bin/env python3
import argparse,hashlib,json,math
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--world-id',type=int,required=True);a=p.parse_args();root=Path(__file__).resolve().parents[2];run=a.run.resolve()
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
files={'raw_pcd':run/'map.pcd','occupancy_yaml':run/'nav-map/map.yaml','occupancy_image':run/'nav-map/map.pgm','fastlio_config':root/'integration/config/jt16_instant.yaml','mount':root/'src/jt16_bringup/config/jt16_mount.json'}
result=json.loads((run/'mapping-result.json').read_text())
metrics=json.loads((run/'map-metrics.json').read_text())
if (result.get('terminal_state')!='COMPLETE' or result.get('paired_samples',0)<6 or
    not all(math.isfinite(result.get(key,math.nan)) for key in ('ate_rmse_m','translation_p95_m','yaw_rmse_rad')) or
    not all(math.isfinite(metrics.get(key,math.nan)) for key in ('occupied_f1','occupied_iou')) or
    any(not path.is_file() or path.stat().st_size==0 for path in files.values())):
 raise ValueError('incomplete mapping inputs or map bundle')
metadata={'world_id':a.world_id,'frame':'FAST-LIO initial local frame','fast_lio_source':'HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai','fast_lio_commit':'2783882405484c0c852eaffc3b07b26d1fff2556','hesai_driver_source':'HesaiTechnology/HesaiLidar_ROS_2.0','hesai_driver_commit':'3d54078a3d770004ef30ec2cacc26673f4172b22','projection_height_above_estimated_floor_m':[.05,.60],'occupancy_resolution_m':.05,'files':{name:{'path':str(path.relative_to(root)),'sha256':digest(path)} for name,path in files.items() if path.exists()}}
(run/'map-bundle-metadata.json').write_text(json.dumps(metadata,indent=2)+'\n');print(run/'map-bundle-metadata.json')
result['map_valid']=True
(run/'mapping-result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
