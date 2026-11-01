#!/usr/bin/env python3
"""Bounded sensor smoke check; records raw message metadata, not navigation SR."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu
rclpy.init();node=Node('jt16_sensor_check');result={'points':[],'imu':[]}
def cloud(m):
 result['points'].append({'height':m.height,'width':m.width,'point_step':m.point_step,'frame':m.header.frame_id,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'fields':[{'name':x.name,'offset':x.offset,'datatype':x.datatype} for x in m.fields]})
def imu(m):
 result['imu'].append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,'acc':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],'gyr':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]})
node.create_subscription(PointCloud2,'/jt16/points_raw/points',cloud,qos_profile_sensor_data)
node.create_subscription(Imu,'/lidar_imu',imu,qos_profile_sensor_data)
deadline=time.monotonic()+20
while time.monotonic()<deadline and (len(result['points'])<10 or len(result['imu'])<100):rclpy.spin_once(node,timeout_sec=0.2)
result['received']={k:len(v) for k,v in result.items()}
path=Path(__file__).resolve().parents[1]/'results/sensors-smoke.json';path.write_text(json.dumps(result,indent=2))
print(result['received']);print(result['points'][:1]);print(result['imu'][:1])
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if all(result['received'].values()) else 1)
