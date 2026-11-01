#!/usr/bin/env python3
"""Compare one FAST-LIO 2D projection with the public BARN geometry."""
import argparse,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

p=argparse.ArgumentParser();p.add_argument('--world',type=Path,required=True);p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--tolerance',type=float,default=.15);a=p.parse_args()
info=yaml.safe_load(a.map.read_text());res=float(info['resolution']);origin=np.asarray(info['origin'][:2]);estimate=np.flipud(np.asarray(Image.open(a.map.parent/info['image'])))<100;truth=np.zeros_like(estimate,dtype=bool)
for model in ET.parse(a.world).getroot().find('world').findall('model'):
 if not model.get('name','').startswith('unit_cylinder_'):continue
 pose=list(map(float,model.findtext('pose').split()[:2]));dx=pose[0]+2.25;dy=pose[1]-3.0;local=np.asarray([dy,-dx]);center=np.floor((local-origin)/res).astype(int);radius=int(math.ceil((.075+.05)/res))
 y0,y1=max(0,center[1]-radius),min(truth.shape[0],center[1]+radius+1);x0,x1=max(0,center[0]-radius),min(truth.shape[1],center[0]+radius+1)
 if x0>=x1 or y0>=y1:continue
 yy,xx=np.ogrid[y0:y1,x0:x1];truth[y0:y1,x0:x1]|=(xx-center[0])**2+(yy-center[1])**2<=radius**2
tol=a.tolerance/res;to_truth=distance_transform_edt(~truth);to_estimate=distance_transform_edt(~estimate);tp_est=estimate&(to_truth<=tol);tp_truth=truth&(to_estimate<=tol);tp=min(int(tp_est.sum()),int(tp_truth.sum()));fp=int(estimate.sum())-tp;fn=int(truth.sum())-tp;precision=tp/(tp+fp) if tp+fp else 0.;recall=tp/(tp+fn) if tp+fn else 0.;f1=2*precision*recall/(precision+recall) if precision+recall else 0.;iou=tp/(tp+fp+fn) if tp+fp+fn else 0.;distances=to_truth[estimate]*res
result={'occupied_precision':precision,'occupied_recall':recall,'occupied_f1':f1,'occupied_iou':iou,'estimated_occupied_cells':int(estimate.sum()),'truth_occupied_cells':int(truth.sum()),'tolerance_m':a.tolerance,'wall_distance_mean_m':float(distances.mean()) if len(distances) else None,'wall_distance_p95_m':float(np.percentile(distances,95)) if len(distances) else None}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
