#!/usr/bin/env python3
"""Count raw JT16 returns for the fixed multi-height coverage obstacles."""
import argparse
import json
from pathlib import Path

import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True);parser.add_argument('--manifest',type=Path,required=True);args=parser.parse_args()
points=np.load(args.input/'lidar_base_xyz.npy');points=points[np.isfinite(points).all(axis=1)]
objects=json.loads(args.manifest.read_text());rows=[]
for item in objects:
 center=np.asarray(item['center_m']);size=np.asarray(item['size_m']);lower=center-size/2-.02;upper=center+size/2+.02
 count=int(((points>=lower)&(points<=upper)).all(axis=1).sum())
 rows.append({**item,'raw_return_count':count,'detected':count>0})
summary={'objects':rows,'detected':sum(v['detected'] for v in rows),'total':len(rows),'by_height':{}}
for height in sorted(set(v['height_m'] for v in rows)):
 group=[v for v in rows if v['height_m']==height];summary['by_height'][str(height)]={'detected':sum(v['detected'] for v in group),'total':len(group),'returns':sum(v['raw_return_count'] for v in group)}
(args.input/'coverage_analysis.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
