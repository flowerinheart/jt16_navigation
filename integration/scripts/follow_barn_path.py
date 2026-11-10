#!/usr/bin/env python3
"""Physically drive a BARN exploration path and score FAST-LIO against GT."""
import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from run_provenance import write_provenance
from benchmark_geometry import mapping_input_failure
from sensor_geometry import map_points_with_base_height


parser = argparse.ArgumentParser()
parser.add_argument('--world-id', type=int, required=True)
parser.add_argument('--path', type=Path, required=True)
parser.add_argument('--simulation-world', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--timeout', type=float, default=180.0)
parser.add_argument('--warmup', type=float, default=5.0)
parser.add_argument('--max-linear', type=float, default=.25)
parser.add_argument('--max-angular', type=float, default=.7)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
write_provenance(args.output, inputs=[args.path, args.simulation_world],
                 parameters={key:str(value) if isinstance(value,Path) else value
                             for key,value in vars(args).items()})


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def barn_xy(point):
    radius = .075
    return np.asarray([point[0] * radius * 2 - radius - 30 * radius * 2,
                       point[1] * radius * 2 + radius + 5], dtype=float)


path = [np.asarray([-2.25, 3.0])]
path.extend(barn_xy(point) for point in np.load(args.path))
path.append(np.asarray([-2.25, 13.0]))
# Remove adjacent grid points while retaining corners; this keeps the controller bounded.
waypoints = [path[0]]
for index in range(1, len(path) - 1):
    before = path[index] - path[index - 1]
    after = path[index + 1] - path[index]
    if abs(np.cross(before, after)) > 1e-6:
        waypoints.append(path[index])
waypoints.append(path[-1])

rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
node = Node('barn_mapping_path_follower')
publisher = node.create_publisher(Twist, '/cmd_vel', 10)
gt, lio = [], []
poses_3d, pending_clouds = deque(maxlen=50), deque(maxlen=5)
map_clouds, cloud_stamps = [], []
mount = json.loads((Path(__file__).resolve().parents[2]/'src/jt16_bringup/config/jt16_mount.json').read_text())

def process_clouds():
    while pending_clouds and poses_3d:
        message = pending_clouds[0]
        capture = stamp(message)
        pose = min(poses_3d, key=lambda p:abs(p[0]-capture))
        if abs(pose[0]-capture)>1e-6:
            if poses_3d[-1][0]<capture:
                return
            pending_clouds.popleft()
            continue
        pending_clouds.popleft()
        rows = point_cloud2.read_points(message,field_names=['x','y','z'],skip_nans=True).reshape(-1)
        xyz = np.column_stack([rows[name] for name in ('x','y','z')])
        if len(xyz):
            map_clouds.append(map_points_with_base_height(xyz,pose[1],pose[2],mount).astype(np.float32))
            cloud_stamps.append(capture)

def pose_3d_sample(message):
    p,q = message.pose.pose.position,message.pose.pose.orientation
    poses_3d.append((stamp(message),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    process_clouds()

def cloud_sample(message):
    if message.header.frame_id=='body':
        pending_clouds.append(message)
        process_clouds()

node.create_subscription(Odometry,'/Odometry',pose_3d_sample,qos_profile_sensor_data)
node.create_subscription(PointCloud2,'/cloud_registered_body',cloud_sample,qos_profile_sensor_data)
raw_imu, adapted_imu, lio_attitude, reported_twist = [], [], [], []
def imu_sample(message):
    q=message.orientation
    return [stamp(message),message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z,
            message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z,q.x,q.y,q.z,q.w]
def lio_orientation(message):
    q=message.pose.pose.orientation
    return [stamp(message),message.pose.pose.position.z,*Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')]
node.create_subscription(Imu,'/lidar_imu_raw',lambda m:raw_imu.append(imu_sample(m)),qos_profile_sensor_data)
node.create_subscription(Imu,'/lidar_imu',lambda m:adapted_imu.append(imu_sample(m)),qos_profile_sensor_data)
node.create_subscription(Odometry,'/Odometry',lambda m:lio_attitude.append(lio_orientation(m)),qos_profile_sensor_data)
node.create_subscription(Odometry,'/ground_truth/odom',lambda m:reported_twist.append([stamp(m),m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]),qos_profile_sensor_data)
node.create_subscription(Odometry, '/ground_truth/odom', lambda m: gt.append(
    [stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)]), qos_profile_sensor_data)
node.create_subscription(Odometry, '/Odometry', lambda m: lio.append(
    [stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)]), qos_profile_sensor_data)

warmup_deadline = time.monotonic() + args.warmup
while time.monotonic() < warmup_deadline:
    publisher.publish(Twist())
    rclpy.spin_once(node, timeout_sec=.03)
input_deadline = time.monotonic() + 15.
input_failure = mapping_input_failure(gt, lio, node.get_clock().now().nanoseconds * 1e-9)
while time.monotonic() < input_deadline:
    rclpy.spin_once(node, timeout_sec=.03)
    input_failure = mapping_input_failure(gt, lio, node.get_clock().now().nanoseconds * 1e-9)
    if input_failure is None:
        break
    publisher.publish(Twist())
motion_start_stamp = gt[-1][0] if gt else None
deadline = time.monotonic() + args.timeout
target = 1
while input_failure is None and time.monotonic() < deadline and target < len(waypoints):
    rclpy.spin_once(node, timeout_sec=.03)
    input_failure = mapping_input_failure(gt, lio, node.get_clock().now().nanoseconds * 1e-9)
    if input_failure is not None:
        break
    position=np.asarray(gt[-1][1:3])
    delta = waypoints[target] - position
    distance = float(np.linalg.norm(delta))
    if distance < .16:
        target += 1
        continue
    heading_error = wrap(math.atan2(delta[1], delta[0]) - gt[-1][3])
    command = Twist()
    command.angular.z = float(np.clip(1.8 * heading_error, -args.max_angular, args.max_angular))
    command.linear.x = min(args.max_linear, .7 * distance) if abs(heading_error) < .55 else 0.0
    publisher.publish(command)
publisher.publish(Twist())
for _ in range(15):
    rclpy.spin_once(node, timeout_sec=.03)

result = {
    'world_id': args.world_id,
    'terminal_state': 'INPUT_FAILURE' if input_failure else ('COMPLETE' if target == len(waypoints) else 'TIMEOUT'),
    'input_failure': input_failure,
    'path_complete': target == len(waypoints),
    'map_valid': False,
    'waypoints_total': len(waypoints), 'waypoints_reached': target,
    'gt_samples': len(gt), 'lio_samples': len(lio),
}
if gt:
    result['gt_final_xyyaw'] = gt[-1][1:4]
if lio:
    result['lio_final_xyyaw'] = lio[-1][1:4]
if motion_start_stamp is not None:
    stationary=np.asarray([sample for sample in lio if sample[0]<=motion_start_stamp])
    if len(stationary)>1:
        displacement=np.linalg.norm(stationary[:,1:3]-stationary[0,1:3],axis=1);result['static_drift_final_m']=float(displacement[-1]);result['static_drift_p95_m']=float(np.percentile(displacement,95));result['static_yaw_drift_rad']=float(abs(np.unwrap(stationary[:,3])[-1]-stationary[0,3]))
if len(gt) > 5 and len(lio) > 5:
    ground = np.asarray(gt); estimate = np.asarray(lio); pairs = []
    for sample in estimate:
        index = int(np.argmin(np.abs(ground[:, 0] - sample[0])))
        if abs(ground[index, 0] - sample[0]) <= .05:
            pairs.append((sample, ground[index]))
    if pairs:
        estimated = np.asarray([pair[0] for pair in pairs])
        truth = np.asarray([pair[1] for pair in pairs])
        estimated_xy = estimated[:, 1:3] - estimated[0, 1:3]
        truth_xy = truth[:, 1:3] - truth[0, 1:3]
        estimated_rotation = np.asarray([[math.cos(-estimated[0, 3]), -math.sin(-estimated[0, 3])],
                                         [math.sin(-estimated[0, 3]), math.cos(-estimated[0, 3])]])
        truth_rotation = np.asarray([[math.cos(-truth[0, 3]), -math.sin(-truth[0, 3])],
                                     [math.sin(-truth[0, 3]), math.cos(-truth[0, 3])]])
        estimated_xy = estimated_xy @ estimated_rotation.T
        truth_xy = truth_xy @ truth_rotation.T
        count = min(len(estimated_xy), len(truth_xy))
        error = np.linalg.norm(estimated_xy[:count] - truth_xy[:count], axis=1)
        yaw_error = np.unwrap(estimated[:count, 3] - estimated[0, 3]) - np.unwrap(truth[:count, 3] - truth[0, 3])
        result.update({'paired_samples': count,
                       'ate_rmse_m': float(np.sqrt(np.mean(error ** 2))),
                       'translation_p95_m': float(np.percentile(error, 95)),
                       'yaw_rmse_rad': float(np.sqrt(np.mean(yaw_error ** 2))),
                       'initialization_time_s': float(max(0.0, lio[0][0] - gt[0][0])),
                       'odometry_update_rate_hz': float((len(estimated)-1)/(estimated[-1,0]-estimated[0,0])) if estimated[-1,0]>estimated[0,0] else 0.0})
        step=max(1,int(round(result['odometry_update_rate_hz'])))
        if count>step:
            estimated_delta=estimated_xy[step:]-estimated_xy[:-step];truth_delta=truth_xy[step:]-truth_xy[:-step]
            result['rpe_translation_rmse_m_1s']=float(np.sqrt(np.mean(np.linalg.norm(estimated_delta-truth_delta,axis=1)**2)))
            estimated_yaw=np.unwrap(estimated[:count,3]);truth_yaw=np.unwrap(truth[:count,3]);rpe_yaw=(estimated_yaw[step:]-estimated_yaw[:-step])-(truth_yaw[step:]-truth_yaw[:-step]);result['rpe_yaw_rmse_rad_1s']=float(np.sqrt(np.mean(rpe_yaw**2)))
if result['terminal_state'] == 'COMPLETE' and (
        result.get('paired_samples', 0) < 6 or
        not all(math.isfinite(result.get(key, math.nan)) for key in
                ('ate_rmse_m', 'translation_p95_m', 'yaw_rmse_rad'))):
    result.update(terminal_state='INPUT_FAILURE', input_failure='invalid_paired_trajectory')
(args.output / 'mapping-result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
np.savez_compressed(args.output / 'mapping-trajectories.npz', gt=np.asarray(gt), lio=np.asarray(lio))
np.savez_compressed(args.output/'mapping-scan-points.npz',
                    map_xy_height=np.concatenate(map_clouds) if map_clouds else np.empty((0,3),dtype=np.float32),
                    cloud_stamps=np.asarray(cloud_stamps))
np.savez_compressed(args.output/'input-diagnostics.npz',raw_imu=np.asarray(raw_imu),adapted_imu=np.asarray(adapted_imu),lio_attitude=np.asarray(lio_attitude),reported_twist=np.asarray(reported_twist))
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if result['terminal_state'] == 'COMPLETE' else 1)
