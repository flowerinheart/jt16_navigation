#!/usr/bin/env python3
"""Drive a bounded trajectory and compare FAST-LIO odometry with Gazebo GT."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)

def stamp(message):return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

rclpy.init();node=Node('jt16_fastlio_motion_validation');command=node.create_publisher(Twist,'/cmd_vel',10);lio=[];gt=[];imu=[]
node.create_subscription(Odometry,'/Odometry',lambda m:lio.append((stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))),qos_profile_sensor_data)
node.create_subscription(Odometry,'/ground_truth/odom',lambda m:gt.append((stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))),qos_profile_sensor_data)
node.create_subscription(Imu,'/lidar_imu',lambda m:imu.append((stamp(m),m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z)),qos_profile_sensor_data)

stages=[(4.0,0.0,0.0),(2.0,0.15,0.0),(2.0,0.0,0.4),(2.0,0.15,0.0),(2.0,0.0,0.4),(2.0,0.15,0.0),(2.0,0.0,0.0)]
for duration,linear,angular in stages:
 end=time.monotonic()+duration
 while time.monotonic()<end:
  message=Twist();message.linear.x=linear;message.angular.z=angular;command.publish(message);rclpy.spin_once(node,timeout_sec=.05)
message=Twist();command.publish(message)
for _ in range(20):rclpy.spin_once(node,timeout_sec=.05)

result={'lio_samples':len(lio),'gt_samples':len(gt),'imu_samples':len(imu),'trajectory_duration_s':sum(s[0] for s in stages)}
if imu:
 imu_array=np.asarray(imu);result['imu_max_abs_gyro_rad_s']=np.max(np.abs(imu_array[:,1:4]),axis=0).tolist()
if len(lio)>=10 and len(gt)>=10:
 lio_array=np.asarray(lio);gt_array=np.asarray(gt);pairs=[]
 for sample in lio_array:
  index=int(np.argmin(np.abs(gt_array[:,0]-sample[0])))
  if abs(gt_array[index,0]-sample[0])<=.05:pairs.append((sample,gt_array[index]))
 if pairs:
  l=np.asarray([p[0] for p in pairs]);g=np.asarray([p[1] for p in pairs])
  lxy=l[:,1:3]-l[0,1:3];gxy=g[:,1:3]-g[0,1:3]
  errors=np.linalg.norm(lxy-gxy,axis=1)
  yaw_error=np.unwrap(l[:,3]-l[0,3])-np.unwrap(g[:,3]-g[0,3])
  result.update({'paired_samples':len(pairs),'translation_rmse_m':float(np.sqrt(np.mean(errors**2))),'translation_p95_m':float(np.percentile(errors,95)),'yaw_rmse_rad':float(np.sqrt(np.mean(yaw_error**2))),'gt_path_m':float(np.linalg.norm(np.diff(gxy,axis=0),axis=1).sum()),'lio_path_m':float(np.linalg.norm(np.diff(lxy,axis=0),axis=1).sum()),'gt_final_xy_m':gxy[-1].tolist(),'lio_final_xy_m':lxy[-1].tolist(),'gt_final_yaw_rad':float(np.unwrap(g[:,3]-g[0,3])[-1]),'lio_final_yaw_rad':float(np.unwrap(l[:,3]-l[0,3])[-1])})
(args.output/'fastlio-motion.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown();raise SystemExit(0 if result.get('paired_samples',0)>=10 else 1)
