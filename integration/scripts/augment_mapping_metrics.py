#!/usr/bin/env python3
"""Backfill timing and one-second RPE from archived mapping trajectories."""
import argparse,json,math
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();data=np.load(a.run/'mapping-trajectories.npz');gt=data['gt'];lio=data['lio'];pairs=[]
for sample in lio:
 index=int(np.argmin(np.abs(gt[:,0]-sample[0])))
 if abs(gt[index,0]-sample[0])<=.05:pairs.append((sample,gt[index]))
if not pairs:raise SystemExit('no paired samples')
estimate=np.asarray([x[0] for x in pairs]);truth=np.asarray([x[1] for x in pairs]);count=len(estimate)
def local(samples):
 xy=samples[:,1:3]-samples[0,1:3];angle=-samples[0,3];rotation=np.asarray([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);return xy@rotation.T
exy,txy=local(estimate),local(truth);rate=(count-1)/(estimate[-1,0]-estimate[0,0]);step=max(1,int(round(rate)));result=json.loads((a.run/'mapping-result.json').read_text());result.update({'initialization_time_s':float(max(0,lio[0,0]-gt[0,0])),'odometry_update_rate_hz':float(rate)})
if count>step:
 result['rpe_translation_rmse_m_1s']=float(np.sqrt(np.mean(np.linalg.norm((exy[step:]-exy[:-step])-(txy[step:]-txy[:-step]),axis=1)**2)));ey=np.unwrap(estimate[:,3]);ty=np.unwrap(truth[:,3]);result['rpe_yaw_rmse_rad_1s']=float(np.sqrt(np.mean(((ey[step:]-ey[:-step])-(ty[step:]-ty[:-step]))**2)))
(a.run/'mapping-result.json').write_text(json.dumps(result,indent=2)+'\n');print(a.run)
