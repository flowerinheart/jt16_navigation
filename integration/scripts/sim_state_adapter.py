#!/usr/bin/env python3
"""Publish Gazebo entity ground truth and optionally drive Arena obstacles."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from std_msgs.msg import Float64
from simulation_interfaces.srv import GetEntityState
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean

p=argparse.ArgumentParser();p.add_argument('--scenario',type=Path);p.add_argument('--world-name');p.add_argument('--seed',type=int,default=0);p.add_argument('--no-ground-truth',action='store_true');a,ros_args=p.parse_known_args()
class Adapter(Node):
 def __init__(self):
  super().__init__('sim_state_adapter');self.publisher=None if a.no_ground_truth else self.create_publisher(Odometry,'/ground_truth/odom',qos_profile_sensor_data);self.get=self.create_client(GetEntityState,'/gzserver/get_entity_state');self.pending=None;self.scenario=json.loads(a.scenario.read_text()) if a.scenario else None
  if not a.no_ground_truth:self.get.wait_for_service(timeout_sec=20);self.create_timer(1/30,self.update_robot)
  if self.scenario:
   if not a.world_name:raise ValueError('--world-name required with --scenario')
   self.gz=GzNode();self.move_service=f'/world/{a.world_name}/set_pose_vector'
   deadline=time.monotonic()+20.
   while not self.gz.service_info(self.move_service):
    if time.monotonic()>=deadline:raise RuntimeError('Native batch pose service unavailable')
    time.sleep(.1)
   self.obstacle_start_time=None;self.dynamic_publisher=self.create_publisher(Odometry,'/dynamic_obstacles/ground_truth',100);self.get.wait_for_service(timeout_sec=20);self.create_timer(.2,self.update_obstacles)
   self.observation_pending={};self.create_timer(.1,self.poll_obstacles)
   self.create_subscription(Float64,'/benchmark/mission_start',self.start_episode,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 def start_episode(self,message):
  if self.obstacle_start_time is None and math.isfinite(message.data) and message.data>=0:self.obstacle_start_time=message.data
 def update_robot(self):
  if self.pending and not self.pending.done():return
  self.pending=self.get.call_async(GetEntityState.Request(entity='xlerobot'));self.pending.add_done_callback(self.publish)
 def publish(self,future):
  response=future.result()
  if not response or response.result.result!=response.result.RESULT_OK:return
  message=Odometry();message.header=response.state.header;message.header.frame_id='world';message.child_frame_id='base_link';message.pose.pose=response.state.pose;message.twist.twist=response.state.twist;self.publisher.publish(message)
 def update_obstacles(self):
  if not self.scenario:return
  now=self.get_clock().now();current=now.nanoseconds*1e-9
  if current==0.:return
  elapsed=0. if self.obstacle_start_time is None else max(0.,current-self.obstacle_start_time)
  request=Pose_V()
  for index,agent in enumerate(self.scenario['pedsim_agents']):
   points=[agent['pos'],*agent.get('waypoints',[])];points=[points[(i+a.seed)%len(points)] for i in range(len(points))]
   if a.seed%2:points=list(reversed(points))
   speed=max(float(agent.get('vmax',.3)),.1);lengths=[math.dist(x,y) for x,y in zip(points,points[1:]+points[:1])];period=sum(lengths)/speed;phase=elapsed%period if period else 0.;position=points[0]
   for x,y,length in zip(points,points[1:]+points[:1],lengths):
    duration=length/speed
    if phase<=duration:
     ratio=phase/duration if duration else 0.;position=[x[0]+ratio*(y[0]-x[0]),x[1]+ratio*(y[1]-x[1])];break
    phase-=duration
   pose=request.pose.add();pose.name=f'dynamic_{index}';pose.position.x=position[0];pose.position.y=position[1];pose.position.z=.85;pose.orientation.w=1.
  self.move_obstacles(request)
 def move_obstacles(self,request):
  # Use native batch pose commands, not the ROS interface's full-state synchronization.
  ok,response=self.gz.request(self.move_service,request,Pose_V,Boolean,1000)
  if not ok or not response.data:
   message=f'Obstacle move failed: native batch pose service (transport={ok}, ack={response.data})'
   self.get_logger().error(message)
   raise RuntimeError(message)
 def poll_obstacles(self):
  for index in range(len(self.scenario['pedsim_agents'])):
   if self.observation_pending.get(index):continue
   self.observation_pending[index]=True
   self.get.call_async(GetEntityState.Request(entity=f'dynamic_{index}')).add_done_callback(lambda future,index=index:self.publish_polled(future,index))
 def publish_polled(self,future,index):
  self.observation_pending[index]=False;self.publish_observation(future,index)
 def publish_observation(self,future,index):
  response=future.result()
  if not response or response.result.result!=response.result.RESULT_OK:return
  message=Odometry();message.header=response.state.header;message.header.frame_id='world';message.child_frame_id=f'dynamic_{index}';message.pose.pose=response.state.pose
  self.dynamic_publisher.publish(message)
rclpy.init(args=ros_args);node=Adapter()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
