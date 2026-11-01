#!/usr/bin/env python3
"""Expose FAST-LIO base pose and velocity to Nav2 with its required reliable QoS."""
import copy
import json
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException
import numpy as np
from scipy.spatial.transform import Rotation
from benchmark_geometry import base_pose
from sensor_geometry import pose_twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class LioOdomBridge(Node):
    def __init__(self):
        super().__init__('lio_odom_bridge')
        self.mount = json.loads((Path(get_package_share_directory('jt16_bringup')) / 'config/jt16_mount.json').read_text())
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.previous = None
        self.create_subscription(Odometry, '/Odometry', self.convert, qos_profile_sensor_data)

    def convert(self, message):
        message = copy.deepcopy(message)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        position, quaternion = base_pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], self.mount)
        p.x, p.y, p.z = map(float, position)
        q.x, q.y, q.z, q.w = map(float, quaternion)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        current = (position, quaternion, stamp)
        if self.previous is not None:
            if stamp <= self.previous[2]:
                return
            linear, angular = pose_twist(self.previous, current)
            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = map(float,linear)
            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = map(float,angular)
        self.previous = current
        # Translate covariance from the IMU origin to the base origin.
        lever = Rotation.from_quat(quaternion).apply(self.mount['position_m'])
        lx, ly, lz = lever
        jacobian = np.eye(6)
        jacobian[:3, 3:] = [[0, -lz, ly], [lz, 0, -lx], [-ly, lx, 0]]
        covariance = np.asarray(message.pose.covariance).reshape(6, 6)
        message.pose.covariance = (jacobian @ covariance @ jacobian.T).reshape(-1).tolist()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        self.publisher.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = p.x
        transform.transform.translation.y = p.y
        transform.transform.translation.z = p.z
        transform.transform.rotation = q
        self.tf.sendTransform(transform)


rclpy.init()
node = LioOdomBridge()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
