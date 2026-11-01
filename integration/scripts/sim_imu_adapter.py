#!/usr/bin/env python3
"""Generate pose-consistent simulated IMU acceleration, including contact tilt."""
import json
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from sensor_geometry import sensor_position, world_specific_force
from rclpy.executors import ExternalShutdownException

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class SimImuAdapter(Node):
    def __init__(self):
        super().__init__('jt16_sim_imu_adapter')
        self.acceleration = np.zeros(3)
        mount = json.loads((Path(get_package_share_directory('jt16_bringup'))/'config/jt16_mount.json').read_text())
        self.chassis_offset = np.asarray(mount['position_m']) - [0., 0., .38]
        self.previous_stamp = None
        self.previous_velocity = None
        self.previous_pose = None
        self.publisher = self.create_publisher(Imu, '/lidar_imu', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/lidar_imu_raw', self.imu, qos_profile_sensor_data)

    def odom(self, message):
        quaternion = message.pose.pose.orientation
        current_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        position = message.pose.pose.position
        current_pose = sensor_position([position.x, position.y, position.z],
                                      [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
                                      self.chassis_offset)
        if self.previous_stamp is not None and current_stamp <= self.previous_stamp:
            return
        if self.previous_stamp is not None:
            dt = current_stamp - self.previous_stamp
            velocity = (current_pose - self.previous_pose) / dt
            if self.previous_velocity is not None:
                self.acceleration = (velocity - self.previous_velocity) / ((dt + self.previous_dt) * .5)
            self.previous_velocity = velocity
            self.previous_dt = dt
        self.previous_pose = current_pose
        self.previous_stamp = current_stamp

    def imu(self, message):
        # Kinematic control impulses are not the acceleration of observed motion.
        # Include all three axes and the sensor lever arm during contact tilt.
        q=message.orientation
        force=world_specific_force(self.acceleration,[q.x,q.y,q.z,q.w])
        message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z=map(float,force)
        self.publisher.publish(message)


rclpy.init()
node = SimImuAdapter()
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
