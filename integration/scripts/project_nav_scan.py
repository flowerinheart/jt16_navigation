#!/usr/bin/env python3
"""Project FAST-LIO's deskewed body cloud into a self-masked Nav2 scan."""
import json
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_geometry import inside_self


class ProjectNavScan(Node):
    def __init__(self):
        super().__init__('jt16_nav_scan_projection')
        config = json.loads((Path(get_package_share_directory('jt16_bringup')) / 'config/jt16_mount.json').read_text())
        self.declare_parameter('pitch_up', float(config['pitch_up_rad']))
        self.declare_parameter('min_height', 0.02)
        self.declare_parameter('max_height', 0.35)
        self.declare_parameter('self_min_x', float(config['self_filter_min_m'][0]))
        self.declare_parameter('self_max_x', float(config['self_filter_max_m'][0]))
        self.declare_parameter('self_min_y', float(config['self_filter_min_m'][1]))
        self.declare_parameter('self_max_y', float(config['self_filter_max_m'][1]))
        self.declare_parameter('self_max_z', float(config['self_filter_max_m'][2]))
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('bins', 300)
        self.declare_parameter('input_cloud', '/cloud_registered_body')
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.self_min_x = float(self.get_parameter('self_min_x').value)
        self.self_max_x = float(self.get_parameter('self_max_x').value)
        self.self_min_y = float(self.get_parameter('self_min_y').value)
        self.self_max_y = float(self.get_parameter('self_max_y').value)
        self.self_max_z = float(self.get_parameter('self_max_z').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.bins = int(self.get_parameter('bins').value)
        pitch = -float(self.get_parameter('pitch_up').value)
        c, s = np.cos(pitch), np.sin(pitch)
        self.rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        self.translation = np.asarray(config['position_m'], dtype=float)
        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        input_cloud = str(self.get_parameter('input_cloud').value)
        self.create_subscription(PointCloud2, input_cloud, self.convert,
                                 1 if input_cloud == '/cloud_registered_body' else qos_profile_sensor_data)

    def convert(self, message):
        raw = point_cloud2.read_points(message, field_names=['x', 'y', 'z'], skip_nans=True).reshape(-1)
        sensor = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(float)
        base = sensor @ self.rotation.T + self.translation
        planar_range = np.linalg.norm(base[:, :2], axis=1)
        inside_robot = inside_self(base,[self.self_min_x,self.self_min_y,-np.inf],
                                   [self.self_max_x,self.self_max_y,self.self_max_z])
        valid = (
            (base[:, 2] >= self.min_height)
            & (base[:, 2] <= self.max_height)
            & ~inside_robot
            & (planar_range >= self.range_min)
            & (planar_range <= self.range_max)
        )
        angles = np.mod(np.arctan2(base[:, 1], base[:, 0]), 2 * np.pi)
        indices = np.floor(angles / (2 * np.pi) * self.bins).astype(int) % self.bins
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        for index, distance in zip(indices[valid], planar_range[valid]):
            if distance < ranges[index]:
                ranges[index] = distance

        scan = LaserScan()
        scan.header = message.header
        scan.header.frame_id = 'base_link'
        scan.angle_min = 0.0
        scan.angle_max = float(2 * np.pi * (self.bins - 1) / self.bins)
        scan.angle_increment = float(2 * np.pi / self.bins)
        # FAST-LIO expresses all points at lidar_end_time after ego-motion deskew.
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


rclpy.init()
node = ProjectNavScan()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
