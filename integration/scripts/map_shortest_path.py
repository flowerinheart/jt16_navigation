#!/usr/bin/env python3
import argparse,heapq,math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation
import yaml
from benchmark_geometry import shortest_path

p=argparse.ArgumentParser();p.add_argument('map_yaml',type=Path);p.add_argument('--start',nargs=2,type=float,required=True);p.add_argument('--goal',nargs=2,type=float,required=True);p.add_argument('--radius',type=float,default=.347);a=p.parse_args()
info=yaml.safe_load(a.map_yaml.read_text());res=float(info['resolution']);origin=np.asarray(info['origin'][:2]);image=np.asarray(Image.open(a.map_yaml.parent/info['image']));occupied=np.flipud(image)<int(round(255*(1-float(info.get('occupied_thresh',.65)))))
cells=int(math.ceil(a.radius/res));yy,xx=np.ogrid[-cells:cells+1,-cells:cells+1];occupied=binary_dilation(occupied,structure=xx*xx+yy*yy<=cells*cells)
def cell(point):return tuple(np.floor((np.asarray(point)-origin)/res).astype(int)[::-1])
try:
 print(shortest_path(occupied, cell(a.start), cell(a.goal)) * res)
except ValueError as error:
 raise SystemExit(str(error))
