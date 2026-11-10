#!/usr/bin/env python3
"""Predict short-horizon LiDAR occupancy without semantic or GT inputs."""
from collections import deque
from copy import deepcopy
import math
from pathlib import Path
import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from motion_prediction import MotionPredictor, scan_clusters, pose_at_stamp, static_map_exclusions, static_scan_ranges


class ObstaclePredictor(Node):
    def __init__(self):
        super().__init__('jt16_obstacle_predictor')
        self.predictor = MotionPredictor()
        self.poses = deque(maxlen=50)
        self.pending = None
        self.map = None
        self.covariance = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        default_config = str(Path(get_package_share_directory('jt16_bringup')) / 'config/nav2_params.yaml')
        self.declare_parameter('nav2_params_file', default_config)
        config = yaml.safe_load(Path(self.get_parameter('nav2_params_file').value).read_text())
        self.map_margin = config['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius']
        self.predictions = self.create_publisher(PointCloud2,'/predicted_obstacles',1)
        self.velocities = self.create_publisher(PointCloud2,'/obstacle_tracks',1)
        self.moving = self.create_publisher(PointCloud2,'/moving_obstacle_points',1)
        self.slam_rejected = self.create_publisher(PointCloud2,'/slam_rejected_points',1)
        self.forecast_surfaces = self.create_publisher(PointCloud2,'/forecast_surface_points',1)
        self.static_scans = self.create_publisher(LaserScan,'/static_scan',qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.static_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localization,10)

    def static_map(self, message):
        self.map = None
        info = message.info
        origin = info.origin
        q = origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        cells = np.asarray(message.data,dtype=int)
        if (message.header.frame_id!='map' or info.width<=0 or info.height<=0 or
                cells.size!=info.width*info.height or not np.isfinite([info.resolution,origin.position.x,origin.position.y,yaw]).all() or
                info.resolution<=0 or not ((cells>=-1)&(cells<=100)).all() or not (cells>=65).any()):
            self.get_logger().error('Invalid static map for SLAM surface exclusion')
            return
        cells = cells.reshape(info.height,info.width)
        self.map = (cells,distance_transform_edt(cells<65)*info.resolution,info.resolution,
                    np.array([origin.position.x,origin.position.y,yaw]))

    def localization(self, message):
        variance = np.asarray(message.pose.covariance)[[0,7,35]]
        self.covariance = None
        if message.header.frame_id=='map' and np.isfinite(variance).all() and ((variance>=-1e-9)&(variance<=1)).all():
            self.covariance = (Time.from_msg(message.header.stamp).nanoseconds*1e-9,variance)

    def unmapped_surfaces(self, message, stamp, pose):
        if self.map is None or self.covariance is None or abs(stamp-self.covariance[0])>.5:
            return np.empty((0,4))
        try:
            transform = self.tf.lookup_transform('map','odom',Time(seconds=stamp)).transform
        except TransformException:
            return np.empty((0,4))
        clusters = scan_clusters(message.ranges,message.angle_min,message.angle_increment,pose,
                                 message.range_min,message.range_max,minimum_points=1)
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c,s],[-s,c]])
        offset = np.array([transform.translation.x,transform.translation.y])
        map_clusters = [points@rotation+offset for points in clusters]
        excluded = static_map_exclusions(map_clusters,*self.map,pose[:2]@rotation+offset,
                                         self.covariance[1],self.map_margin)
        points = [cluster for cluster,reject in zip(clusters,excluded) if reject]
        return np.column_stack((np.concatenate(points),np.zeros((sum(map(len,points)),2)))) if points else np.empty((0,4))

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
        if self.pending is None:
            return
        message = self.pending
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        pose = pose_at_stamp(self.poses, stamp)
        if pose is None:
            self.static_scans.publish(deepcopy(message))
            return
        self.pending = None
        try:
            clusters = scan_clusters(message.ranges,message.angle_min,message.angle_increment,pose,message.range_min,message.range_max)
            predicted,velocities = self.predictor.update(clusters,stamp)
        except ValueError as error:
            self.static_scans.publish(deepcopy(message))
            self.get_logger().error(str(error))
            return
        static_scan = deepcopy(message)
        static_scan.ranges = static_scan_ranges(message.ranges,message.angle_min,message.angle_increment,
                                               pose,self.predictor.tracks,stamp).tolist()
        self.static_scans.publish(static_scan)
        message.header.frame_id = 'odom'
        xyz = np.column_stack((predicted,np.zeros(len(predicted),dtype=float)))
        self.predictions.publish(point_cloud2.create_cloud_xyz32(message.header,xyz.astype(np.float32)))
        fields = [PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1)
                  for i,name in enumerate(('x','y','vx','vy'))]
        self.velocities.publish(point_cloud2.create_cloud(message.header,fields,velocities.astype(np.float32)))
        self.moving.publish(point_cloud2.create_cloud(message.header,fields,self.predictor.moving_points.astype(np.float32)))
        unmapped = self.unmapped_surfaces(message,stamp,pose)
        if len(unmapped) and len(self.predictor.moving_points):
            confirmed = (unmapped[:,None,:2]==self.predictor.moving_points[None,:,:2]).all(axis=2).any(axis=1)
            unmapped = unmapped[~confirmed]
        rejected = np.concatenate((self.predictor.moving_points,unmapped))
        self.slam_rejected.publish(point_cloud2.create_cloud(message.header,fields,rejected.astype(np.float32)))
        forecasts = np.concatenate((self.predictor.forecast_points,unmapped))
        self.forecast_surfaces.publish(point_cloud2.create_cloud(message.header,fields,forecasts.astype(np.float32)))


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
