#!/usr/bin/env python3
"""Predict short-horizon LiDAR occupancy without semantic or GT inputs."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from motion_prediction import MotionPredictor, scan_clusters


class ObstaclePredictor(Node):
    def __init__(self):
        super().__init__('jt16_obstacle_predictor')
        self.predictor = MotionPredictor()
        self.poses = deque(maxlen=50)
        self.pending = None
        self.predictions = self.create_publisher(PointCloud2,'/predicted_obstacles',1)
        self.velocities = self.create_publisher(PointCloud2,'/obstacle_tracks',1)
        self.moving = self.create_publisher(PointCloud2,'/moving_obstacle_points',1)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)

    def odom(self, message):
        if message.header.frame_id!='odom' or message.child_frame_id!='base_link':
            return
        p,q = message.pose.pose.position,message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        self.poses.append((stamp,[p.x,p.y,yaw]))
        self.process()

    def scan(self, message):
        if message.header.frame_id!='base_link':
            return
        self.pending = message
        self.process()

    def process(self):
        if self.pending is None or not self.poses:
            return
        message = self.pending
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        pose_stamp,pose = min(self.poses,key=lambda row:abs(row[0]-stamp))
        if abs(pose_stamp-stamp)>.03:
            return
        self.pending = None
        try:
            clusters = scan_clusters(message.ranges,message.angle_min,message.angle_increment,pose,message.range_min)
            predicted,velocities = self.predictor.update(clusters,stamp)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        message.header.frame_id = 'odom'
        xyz = np.column_stack((predicted,np.zeros(len(predicted),dtype=float)))
        self.predictions.publish(point_cloud2.create_cloud_xyz32(message.header,xyz.astype(np.float32)))
        fields = [PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1)
                  for i,name in enumerate(('x','y','vx','vy'))]
        self.velocities.publish(point_cloud2.create_cloud(message.header,fields,velocities.astype(np.float32)))
        self.moving.publish(point_cloud2.create_cloud(message.header,fields,self.predictor.moving_points.astype(np.float32)))


if __name__ == '__main__':
    rclpy.init()
    node = ObstaclePredictor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
