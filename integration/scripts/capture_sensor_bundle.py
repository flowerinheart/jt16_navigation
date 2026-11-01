#!/usr/bin/env python3
"""Collect real ROS sensor messages from Gazebo; independently score mount geometry."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu,Image,LaserScan
from sensor_msgs_py import point_cloud2
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--pitch-up',type=float,default=0.0);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('jt16_new_mount_capture');events={'lidar':[],'adapted':[],'scan':[],'imu':[],'camera':[]};saved={}
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def cloud(m):
 events['lidar'].append(stamp(m))
 if 'lidar' not in saved:
  raw=point_cloud2.read_points(m,skip_nans=False).reshape(-1)
  xyz=np.stack([raw[n] for n in ['x','y','z']],axis=1).astype(float)
  np.save(args.output/'lidar_xyz.npy',xyz);np.save(args.output/'lidar_ring.npy',raw['ring'])
  valid=np.isfinite(xyz).all(axis=1);p=xyz[valid];dist=np.linalg.norm(p,axis=1)
  pitch=-args.pitch_up;c=np.cos(pitch);s=np.sin(pitch);rotation=np.array([[c,0,s],[0,1,0],[-s,0,c]])
  base=p@rotation.T+np.array([-.03,0,.36]);np.save(args.output/'lidar_base_xyz.npy',base)
  (args.output/'lidar_raw.bin').write_bytes(bytes(m.data))
  saved['lidar']={'width':m.width,'height':m.height,'frame':m.header.frame_id,'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype} for f in m.fields],'valid_points':len(p),'total_points':len(xyz),'range_min_m':float(dist.min()),'range_max_m':float(dist.max()),'base_z_min_m':float(base[:,2].min()),'base_z_max_m':float(base[:,2].max()),'rings':sorted(set(raw['ring'][valid].tolist()))}
def imu(m):
 events['imu'].append(stamp(m))
 if 'imu' not in saved:saved['imu']={'frame':m.header.frame_id,'acc_m_s2':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],'gyro_rad_s':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]}
def camera(m):
 events['camera'].append(stamp(m))
 if 'camera' not in saved:
  if m.encoding not in ('rgb8','bgr8'):raise RuntimeError(m.encoding)
  im=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
  if m.encoding=='bgr8':im=im[:,:,::-1]
  PILImage.fromarray(im).save(args.output/'top_camera.png');saved['camera']={'frame':m.header.frame_id,'width':m.width,'height':m.height,'encoding':m.encoding,'pixel_std':float(im.std()),'min':int(im.min()),'max':int(im.max())}
def adapted(m):
 events['adapted'].append(stamp(m))
 if 'adapted' not in saved:
  raw=point_cloud2.read_points(m,skip_nans=True).reshape(-1)
  point_times=raw['timestamp'].astype(float)
  saved['adapted']={'width':m.width,'height':m.height,'frame':m.header.frame_id,'fields':[f.name for f in m.fields],'valid_points':len(raw),'point_time_min_s':float(point_times.min()),'point_time_max_s':float(point_times.max()),'point_time_span_s':float(point_times.max()-point_times.min())}
def scan(m):
 events['scan'].append(stamp(m))
 if 'scan' not in saved:
  ranges=np.asarray(m.ranges,dtype=float);finite=np.isfinite(ranges)
  front=np.r_[np.arange(0,9),np.arange(len(ranges)-8,len(ranges))]
  front_ranges=ranges[front];front_ranges=front_ranges[np.isfinite(front_ranges)]
  saved['scan']={'frame':m.header.frame_id,'bins':len(ranges),'finite_bins':int(finite.sum()),'range_min_m':float(ranges[finite].min()) if finite.any() else None,'range_max_m':float(ranges[finite].max()) if finite.any() else None,'front_nearest_m':float(front_ranges.min()) if len(front_ranges) else None}
for typ,topic,cb in [(PointCloud2,'/jt16/points_raw/points',cloud),(PointCloud2,'/lidar_points',adapted),(LaserScan,'/scan',scan),(Imu,'/lidar_imu',imu),(Image,'/top_camera/image',camera)]:node.create_subscription(typ,topic,cb,qos_profile_sensor_data)
end=time.monotonic()+40
while time.monotonic()<end and not all(len(events[k])>=n for k,n in [('lidar',30),('adapted',30),('scan',30),('camera',30),('imu',200)]):rclpy.spin_once(node,timeout_sec=.1)
for k,ts in events.items():
 saved.setdefault(k,{})['count']=len(ts)
 saved[k]['sim_hz']=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None
 saved[k]['timestamps']=ts
pitch=-args.pitch_up;saved['mount_pitch_up_rad']=args.pitch_up;saved['expected_imu_acc_m_s2']=[-np.sin(pitch)*9.80665,0,np.cos(pitch)*9.80665]
saved['scope']='Static ideal instantaneous sensor baseline; no navigation / deskew / precision acceptance.'
(args.output/'summary.json').write_text(json.dumps(saved,indent=2))
print(json.dumps({k:{a:v for a,v in val.items() if a!='timestamps'} if isinstance(val,dict) else val for k,val in saved.items()},indent=2))
node.destroy_node();rclpy.shutdown();raise SystemExit(0 if all(len(events[k])>=n for k,n in [('lidar',30),('adapted',30),('scan',30),('camera',30),('imu',200)]) else 1)
