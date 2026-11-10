#!/usr/bin/env python3
"""Assemble a 10 Hz rolling JT16 scan from 100 Hz Gazebo snapshots."""
import numpy as np
import json
from copy import deepcopy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from sensor_geometry import self_returns
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class Adapter(Node):
    def __init__(self):
        super().__init__('jt16_rolling_cloud_adapter')
        self.declare_parameter('segments', 10)
        self.declare_parameter('minimum_z', -100.0)
        self.segments = int(self.get_parameter('segments').value)
        self.minimum_z = float(self.get_parameter('minimum_z').value)
        self.segment_index = 0
        self.parts = []
        self.mount = json.loads((Path(get_package_share_directory('jt16_bringup'))/'config/jt16_mount.json').read_text())
        self.first_header = None
        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/jt16/points_raw/points', self.convert, qos_profile_sensor_data)

    def convert(self, message):
        raw = point_cloud2.read_points(
            message, field_names=['x', 'y', 'z', 'intensity', 'ring'], skip_nans=False
        ).reshape(message.height, message.width)
        if self.segment_index == 0:
            self.first_header = message.header
        columns_per_segment = message.width // self.segments
        start = self.segment_index * columns_per_segment
        end = message.width if self.segment_index == self.segments - 1 else start + columns_per_segment
        segment = raw[:, start:end].reshape(-1).copy()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        # A snapshot has one capture time; synthetic offsets misalign IMU deskew.
        timestamps = np.full(len(segment), stamp, dtype=np.float64)
        self.parts.append((segment, timestamps))
        self.segment_index += 1
        if self.segment_index < self.segments:
            return

        dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
            ('ring', '<u2'), ('timestamp', '<f8')
        ])
        total = sum(len(part) for part, _ in self.parts)
        points = np.empty(total, dtype=dtype)
        cursor = 0
        for part, point_times in self.parts:
            count = len(part)
            for name in ['x', 'y', 'z', 'intensity', 'ring']:
                points[name][cursor:cursor + count] = part[name]
            points['timestamp'][cursor:cursor + count] = point_times
            cursor += count
        valid = (np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
                 & (points['z'] >= self.minimum_z))
        xyz = np.stack([points[name] for name in ('x','y','z')], axis=1)
        valid &= ~self_returns(xyz, self.mount)
        points = points[valid]
        fields = [
            PointField(name=name, offset=dtype.fields[name][1], datatype=datatype, count=1)
            for name, datatype in [('x', 7), ('y', 7), ('z', 7), ('intensity', 7), ('ring', 4), ('timestamp', 8)]
        ]
        output = point_cloud2.create_cloud(deepcopy(self.first_header), fields, points)
        if len(points):
            # HESAI preprocessing measures point offsets from the first retained point.
            output.header.stamp.sec, output.header.stamp.nanosec = divmod(
                round(float(points['timestamp'][0]) * 1e9), 1000000000)
        self.publisher.publish(output)
        self.parts.clear()
        self.segment_index = 0
        self.first_header = None


rclpy.init()
node = Adapter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
