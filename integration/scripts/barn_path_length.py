#!/usr/bin/env python3
import argparse
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('path');args=parser.parse_args()
r=.075
points=np.load(args.path).astype(float)
points[:,0]=points[:,0]*2*r-r-30*r*2
points[:,1]=points[:,1]*2*r+r+5
points=np.vstack(([-2.25,3.0],points,[-2.25,13.0]))
print(float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()))
