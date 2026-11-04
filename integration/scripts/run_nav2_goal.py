#!/usr/bin/env python3
"""Run one bounded Nav2 goal and record the terminal result."""
import argparse
import json
import time
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from navigation_metrics import score_navigation
from benchmark_geometry import covariance_valid,obstacle_observations_fresh
from run_provenance import write_provenance


parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--x',type=float,default=.30);parser.add_argument('--y',type=float,default=0.0);parser.add_argument('--initial-x',type=float,default=0.0);parser.add_argument('--initial-y',type=float,default=0.0);parser.add_argument('--initial-yaw',type=float,default=0.0);parser.add_argument('--ground-truth-origin-x',type=float);parser.add_argument('--ground-truth-origin-y',type=float);parser.add_argument('--ground-truth-origin-yaw',type=float);parser.add_argument('--ground-truth-is-world',action='store_true');parser.add_argument('--localization-reference-is-world',action='store_true');parser.add_argument('--goal-is-local',action='store_true');parser.add_argument('--score-goal-x',type=float);parser.add_argument('--score-goal-y',type=float);parser.add_argument('--score-goal-is-local',action='store_true');parser.add_argument('--waypoints-npz',type=Path);parser.add_argument('--timeout',type=float,default=45.0);parser.add_argument('--activation-timeout',type=float,default=60.0);parser.add_argument('--goal-tolerance',type=float,default=.25);parser.add_argument('--map-yaml',type=Path);parser.add_argument('--nav2-params-file',type=Path);parser.add_argument('--simulation-world',type=Path);parser.add_argument('--barn-world',type=Path);parser.add_argument('--shortest-path',type=float);parser.add_argument('--arena-scenario',type=Path);parser.add_argument('--arena-world',type=Path);parser.add_argument('--arena-seed',type=int,default=0);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)

class BoundedNavigator(BasicNavigator):
 def __init__(self,timeout):super().__init__();self.activation_deadline=time.monotonic()+timeout
 def remaining(self):return self.activation_deadline-time.monotonic()
 def _waitForNodeToActivate(self,node_name):
  client=self.create_client(GetState,f'{node_name}/get_state')
  while self.remaining()>0:
   if not client.wait_for_service(timeout_sec=min(1.0,self.remaining())):continue
   future=client.call_async(GetState.Request());rclpy.spin_until_future_complete(self,future,timeout_sec=min(1.0,self.remaining()))
   if future.done() and future.result() and future.result().current_state.label=='active':return
  raise TimeoutError(f'{node_name} did not become active')
 def _waitForInitialPose(self):
  next_publish=0.
  while not self.initial_pose_received and self.remaining()>0:
   if time.monotonic()>=next_publish:
    self._setInitialPose();next_publish=time.monotonic()+1.
   rclpy.spin_once(self,timeout_sec=min(.1,self.remaining()))
  if not self.initial_pose_received:raise TimeoutError('AMCL initial pose was not received')
 def refreshLocalization(self,poses,truth):
  client=self.create_client(Empty,'/request_nomotion_update')
  if not client.wait_for_service(timeout_sec=1.):return
  previous=poses[-1][3] if poses else -1.
  future=client.call_async(Empty.Request())
  rclpy.spin_until_future_complete(self,future,timeout_sec=1.)
  if not future.done():return
  deadline=time.monotonic()+1.
  while time.monotonic()<deadline and (not poses or poses[-1][3]<=previous or not truth or truth[-1][3]<poses[-1][3]):
   rclpy.spin_once(self,timeout_sec=.05)

import yaml
static_prediction_enabled=False;timed_prediction_enabled=False
if args.nav2_params_file:
 nav_parameters=yaml.safe_load(args.nav2_params_file.read_text())
 local_parameters=nav_parameters['local_costmap']['local_costmap']['ros__parameters']
 global_parameters=nav_parameters['global_costmap']['global_costmap']['ros__parameters']
 static_prediction_enabled=any('motion_prediction' in parameters['plugins'] and parameters.get('motion_prediction',{}).get('enabled',True) for parameters in (local_parameters,global_parameters))
 follow_path=nav_parameters['controller_server']['ros__parameters']['FollowPath']
 timed_prediction_enabled='TimedObstacleCritic' in follow_path.get('critics',[]) and follow_path.get('TimedObstacleCritic',{}).get('enabled',True)
prediction_enabled=static_prediction_enabled or timed_prediction_enabled
input_paths=[path for path in (args.barn_world,args.arena_world,args.arena_scenario,args.map_yaml,args.simulation_world,args.nav2_params_file) if path]
if args.map_yaml:
 map_info=yaml.safe_load(args.map_yaml.read_text())
 input_paths.append(args.map_yaml.parent/map_info['image'])
write_provenance(args.output,input_paths,{key:str(value) if isinstance(value,Path) else value for key,value in vars(args).items()})
rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);navigator=BoundedNavigator(args.activation_timeout);gt=[];lio=[];fused=[];plans=[];amcl=[];map_odom=[];dynamic=[];commands={topic:[] for topic in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel')}
odom_twist=[];scans=[];odom_pose_3d=[];prediction_stamps=[];predicted_points=[];obstacle_tracks=[]
moving_point_stamps=[];moving_points=[]
def moving_sample(message):
 if message.header.frame_id!='odom':return
 stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
 moving_point_stamps.append(stamp)
 rows=point_cloud2.read_points(message,field_names=['x','y','vx','vy'],skip_nans=False)
 moving_points.extend([stamp,*[float(row[name]) for name in ('x','y','vx','vy')]] for row in rows)
navigator.create_subscription(PointCloud2,'/moving_obstacle_points',moving_sample,1)
def prediction_sample(message):
 if message.header.frame_id!='odom':return
 stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
 prediction_stamps.append(stamp)
 rows=point_cloud2.read_points(message,field_names=['x','y'],skip_nans=False)
 predicted_points.extend([stamp,float(row['x']),float(row['y'])] for row in rows)
def track_sample(message):
 stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
 rows=point_cloud2.read_points(message,field_names=['x','y','vx','vy'],skip_nans=False)
 obstacle_tracks.extend([stamp,*[float(row[name]) for name in ('x','y','vx','vy')]] for row in rows)
navigator.create_subscription(PointCloud2,'/predicted_obstacles',prediction_sample,1)
navigator.create_subscription(PointCloud2,'/obstacle_tracks',track_sample,1)
episode_publisher=navigator.create_publisher(Float64,'/benchmark/mission_start',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for topic,samples in commands.items():navigator.create_subscription(Twist,f'/{topic}',lambda message,samples=samples:samples.append([message.linear.x,message.angular.z,navigator.get_clock().now().nanoseconds*1e-9]),100)
navigator.create_subscription(Odometry,'/odom',lambda m:odom_twist.append([m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.stamp.sec+m.header.stamp.nanosec*1e-9]),qos_profile_sensor_data)
navigator.create_subscription(Odometry,'/odom',lambda m:odom_pose_3d.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w]),qos_profile_sensor_data)
navigator.create_subscription(LaserScan,'/scan',lambda m:scans.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,*m.ranges]),qos_profile_sensor_data)
def gt_sample(m):
 q=m.pose.pose.orientation
 heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 gt.append([m.pose.pose.position.x,m.pose.pose.position.y,heading,m.header.stamp.sec+m.header.stamp.nanosec*1e-9])
navigator.create_subscription(Odometry,'/ground_truth/odom',gt_sample,10)
def pose_yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
navigator.create_subscription(Odometry,'/Odometry',lambda m:lio.append([m.pose.pose.position.x,m.pose.pose.position.y,pose_yaw(m.pose.pose.orientation),m.header.stamp.sec+m.header.stamp.nanosec*1e-9]),qos_profile_sensor_data)
navigator.create_subscription(Odometry,'/odom',lambda m:fused.append([m.pose.pose.position.x,m.pose.pose.position.y,pose_yaw(m.pose.pose.orientation),m.header.stamp.sec+m.header.stamp.nanosec*1e-9]),qos_profile_sensor_data)
navigator.create_subscription(NavPath,'/plan',lambda m:plans.append([[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
navigator.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:amcl.append([m.pose.pose.position.x,m.pose.pose.position.y,pose_yaw(m.pose.pose.orientation),m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35]]),10)
obstacle_stamps={}
obstacle_count=len(json.loads(args.arena_scenario.read_text())['pedsim_agents']) if args.arena_scenario else 0
def dynamic_sample(message):
 stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
 index=int(message.child_frame_id.split('_')[-1])
 obstacle_stamps[index]=stamp
 dynamic.append([stamp,index,message.pose.pose.position.x,message.pose.pose.position.y,.30])
navigator.create_subscription(Odometry,'/dynamic_obstacles/ground_truth',dynamic_sample,100)
def tf_sample(message):
 for transform in message.transforms:
  if transform.header.frame_id=='map' and transform.child_frame_id=='odom':map_odom.append([transform.transform.translation.x,transform.transform.translation.y,pose_yaw(transform.transform.rotation)])
navigator.create_subscription(TFMessage,'/tf',tf_sample,100)
clock_deadline=time.monotonic()+5.
while navigator.get_clock().now().nanoseconds==0 and time.monotonic()<clock_deadline:
 rclpy.spin_once(navigator,timeout_sec=.1)
initial=PoseStamped();initial.header.frame_id='map';initial.header.stamp=navigator.get_clock().now().to_msg();initial.pose.position.x=args.initial_x;initial.pose.position.y=args.initial_y;initial.pose.orientation.z=math.sin(args.initial_yaw/2);initial.pose.orientation.w=math.cos(args.initial_yaw/2);initial_sent_stamp=initial.header.stamp.sec+initial.header.stamp.nanosec*1e-9;initial.header.stamp.sec=0;initial.header.stamp.nanosec=0;navigator.setInitialPose(initial)
activation_start=time.monotonic()
try:navigator.waitUntilNav2Active(localizer='amcl')
except TimeoutError as error:
 summary={'result':'PROCESS_FAILURE','scored_success':False,'nav2_action_success':False,'localization_success':False,'activation_error':str(error),'localization_initialization_latency_s':time.monotonic()-activation_start}
 (args.output/'nav2-goal.json').write_text(json.dumps(summary,indent=2));navigator.destroy_node();rclpy.shutdown();raise SystemExit(2)
activation_latency=time.monotonic()-activation_start
input_deadline=time.monotonic()+5.
while (not fused or navigator.get_clock().now().nanoseconds*1e-9-fused[-1][3]>.5) and time.monotonic()<input_deadline:
 rclpy.spin_once(navigator,timeout_sec=.05)
goal_heading=math.atan2(args.y-args.initial_y,args.x-args.initial_x)
goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=navigator.get_clock().now().to_msg();goal.pose.position.x=args.x;goal.pose.position.y=args.y;goal.pose.orientation.z=math.sin(goal_heading/2);goal.pose.orientation.w=math.cos(goal_heading/2)
goals=[goal]
if args.waypoints_npz:
 trajectory=np.load(args.waypoints_npz)['lio'];selected=[];last=trajectory[0,1:3]
 for sample in trajectory[1:]:
  if np.linalg.norm(sample[1:3]-last)>=0.05:selected.append(sample);last=sample[1:3]
 selected.append(np.asarray([0.0,args.x,args.y,0.0]))
 goals=[]
 for sample in selected:
  pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=navigator.get_clock().now().to_msg();pose.pose.position.x=float(sample[1]);pose.pose.position.y=float(sample[2]);pose.pose.orientation.z=math.sin(float(sample[3])/2);pose.pose.orientation.w=math.cos(float(sample[3])/2);goals.append(pose)
start=time.monotonic();sim_start=gt[-1][3] if gt else None
mission_start=Float64();mission_start.data=sim_start if sim_start is not None else navigator.get_clock().now().nanoseconds*1e-9;episode_publisher.publish(mission_start)
if len(goals)>1:
 path=NavPath();path.header.frame_id='map'
 for pose in goals:pose.header.stamp.sec=0;pose.header.stamp.nanosec=0
 path.poses=goals;navigator.followPath(path)
else:navigator.goToPose(goal)
timed_out=False;input_failure=None
while not navigator.isTaskComplete():
 rclpy.spin_once(navigator,timeout_sec=.05)
 if time.monotonic()-start>args.timeout*2+30:
  timed_out=True;navigator.cancelTask();break
 now=navigator.get_clock().now().nanoseconds*1e-9
 if not obstacle_observations_fresh(obstacle_stamps,obstacle_count,now):
  input_failure='Benchmark obstacle observations stale';navigator.cancelTask();break
 if not fused or now-fused[-1][3] > .5:
  input_failure='FAST-LIO odometry stale';navigator.cancelTask();break
 if static_prediction_enabled and (not prediction_stamps or now-prediction_stamps[-1]>.5):
  input_failure='LiDAR motion prediction stale';navigator.cancelTask();break
 if timed_prediction_enabled and (not moving_point_stamps or now-moving_point_stamps[-1]>.5):
  input_failure='LiDAR timed obstacle input stale';navigator.cancelTask();break
 if amcl and not covariance_valid(amcl[-1][4:7]):
  input_failure='AMCL covariance invalid';navigator.cancelTask();break
 if sim_start is None and gt:sim_start=gt[-1][3]
 if sim_start is not None and gt and gt[-1][3]-sim_start>args.timeout:timed_out=True;navigator.cancelTask();break
result=navigator.getResult();names={TaskResult.SUCCEEDED:'SUCCEEDED',TaskResult.CANCELED:'CANCELED',TaskResult.FAILED:'FAILED'}
mission_end=navigator.get_clock().now().nanoseconds*1e-9
wall_elapsed=time.monotonic()-start;elapsed=(mission_end-sim_start) if sim_start is not None else wall_elapsed
# AMCL normally emits only after motion; obtain a real fresh terminal observation.
if result==TaskResult.SUCCEEDED and not timed_out and not input_failure:navigator.refreshLocalization(amcl,gt)
# Drain delayed benchmark observations against the frozen mission end, without extending navigation.
observation_deadline=time.monotonic()+2.
while obstacle_count and time.monotonic()<observation_deadline and not all(
 obstacle_stamps.get(index,-math.inf)>=mission_end for index in range(obstacle_count)):
 rclpy.spin_once(navigator,timeout_sec=.05)
summary={'result':'PROCESS_FAILURE' if input_failure else ('TIMEOUT' if timed_out else names.get(result,str(result))),'elapsed_s':elapsed,'wall_elapsed_s':wall_elapsed,'goal_xy_m':[args.x,args.y],'navigation_waypoint_count':len(goals),'gt_samples':len(gt),'localization_success':False,'amcl_message_received':bool(amcl),'input_failure':input_failure,'nav2_activation_latency_s':activation_latency,'amcl_samples':len(amcl),'dynamic_obstacle_samples':len(dynamic),'scoring_schema':2,'navigation_timeout_s':args.timeout}
for topic,samples in commands.items():
 if samples:
  values=np.asarray(samples);moving=np.abs(values[:,0])>.001
  turning=np.abs(values[:,1])>.001
  summary[f'{topic}_samples']=len(samples);summary[f'{topic}_nonzero_linear_ratio']=float(moving.mean());summary[f'{topic}_nonzero_angular_ratio']=float(turning.mean());summary[f'{topic}_mean_abs_linear_m_s']=float(np.abs(values[:,0]).mean());summary[f'{topic}_max_abs_linear_m_s']=float(np.abs(values[:,0]).max());summary[f'{topic}_mean_abs_angular_rad_s']=float(np.abs(values[:,1]).mean());summary[f'{topic}_max_abs_angular_rad_s']=float(np.abs(values[:,1]).max())
summary['mission_start_stamp_s']=mission_start.data
summary['mission_end_stamp_s']=mission_end
summary.update(motion_prediction_enabled=prediction_enabled,prediction_message_count=len(prediction_stamps),predicted_point_count=len(predicted_points),moving_track_rows=len(obstacle_tracks))
summary.update(timed_obstacle_critic_enabled=timed_prediction_enabled,static_prediction_layer_enabled=static_prediction_enabled,
               moving_point_message_count=len(moving_point_stamps),moving_surface_point_count=len(moving_points))
summary.update(score_navigation(dict(gt=gt,lio=lio,fused=fused,amcl=amcl,map_odom=map_odom,dynamic=dynamic),args,
               initial_sent_stamp,summary))
summary['initial_pose_sent_stamp_s']=initial_sent_stamp
summary['nav2_action_success']=summary['result']=='SUCCEEDED'

np.savez_compressed(args.output/'navigation-trajectories.npz',gt=np.asarray(gt),lio=np.asarray(lio),fused=np.asarray(fused),amcl=np.asarray(amcl),map_odom=np.asarray(map_odom),dynamic=np.asarray(dynamic,dtype=float).reshape(-1,5))
np.savez_compressed(args.output/'navigation-sensors.npz',odom_twist=np.asarray(odom_twist),odom_pose_3d=np.asarray(odom_pose_3d),scans=np.asarray(scans),prediction_stamps=np.asarray(prediction_stamps),predicted_points=np.asarray(predicted_points).reshape(-1,3),obstacle_tracks=np.asarray(obstacle_tracks).reshape(-1,5),moving_point_stamps=np.asarray(moving_point_stamps),moving_obstacle_points=np.asarray(moving_points).reshape(-1,5),**{topic:np.asarray(samples) for topic,samples in commands.items()})
if plans:
 (args.output/'global-plan-initial.json').write_text(json.dumps(plans[0]))
 (args.output/'global-plan-final.json').write_text(json.dumps(plans[-1]))
(args.output/'nav2-goal.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));navigator.destroy_node();rclpy.shutdown();raise SystemExit(0 if summary['scored_success'] else 1)
