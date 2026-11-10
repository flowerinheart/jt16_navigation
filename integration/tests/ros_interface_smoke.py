"""Run after sourcing ROS and install/setup.bash, in an unused ROS_DOMAIN_ID."""
import json
import math
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage


def check_outputs(poses, scans, transforms):
    assert poses and scans and transforms, 'missing installed-node outputs'
    assert abs(poses[-1].pose.pose.position.x-.03)<1e-6
    assert abs(poses[-1].pose.pose.position.z+.36)<1e-6
    assert any(t.header.frame_id=='odom' and t.child_frame_id=='base_link' for t in transforms)
    assert scans[-1].header.frame_id=='base_link'
    assert scans[-1].time_increment==0., 'deskewed cloud must not acquire fictional beam times'
    assert any(.9<distance<1.1 for distance in scans[-1].ranges)


def stop_children(children):
    for child in children:
        child.send_signal(signal.SIGINT)
    for child in children:
        try:
            output = child.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            child.kill()
            output = child.communicate()[0]
        assert child.returncode == 0, output


def check_derived_velocity(node, publisher, message, poses):
    before=len(poses)
    deadline=time.monotonic()+1.
    while time.monotonic()<deadline and not any(abs(p.twist.twist.linear.x)>.01 for p in poses[before:]):
        message.header.stamp=node.get_clock().now().to_msg()
        message.pose.pose.position.x=.1
        publisher.publish(message)
        rclpy.spin_once(node,timeout_sec=.05)
    assert any(abs(p.twist.twist.linear.x)>.01 for p in poses[before:]), 'bridge dropped derived velocity'


def check_motion_forecast(node, publisher, clouds, message, fields, predictions, tracks, moving, forecasts, static_scans):
    def static_return_kept():
        return any(sum(math.isfinite(r) for r in scan.ranges)==1 and
                   any(3.1<r<3.3 for r in scan.ranges) for scan in static_scans)
    start=time.monotonic();deadline=start+3.;next_publish=start
    while time.monotonic()<deadline and not (all(any(cloud.width for cloud in samples) for samples in (predictions,tracks,moving,forecasts)) and static_return_kept()):
        if time.monotonic()>=next_publish:
            message.header.stamp=node.get_clock().now().to_msg()
            publisher.publish(message)
            distance=1.+.2*(time.monotonic()-start)
            clouds.publish(point_cloud2.create_cloud(message.header,fields,
                           [(distance,.022*i,-.2) for i in range(6)]+[(3.,1.,-.2)]))
            next_publish=time.monotonic()+.1
        rclpy.spin_once(node,timeout_sec=.01)
    assert any(cloud.width for cloud in predictions), 'moving cluster has no forecast'
    velocities=[row['vx'] for cloud in tracks for row in point_cloud2.read_points(cloud,field_names=['vx'])]
    assert any(.1<float(speed)<.3 for speed in velocities), 'missing measured LiDAR velocity'
    assert any(cloud.width for cloud in moving), 'missing time-matched moving surfaces'
    assert any(cloud.width for cloud in forecasts), 'missing separate forecast surfaces'
    assert forecasts[-1].header.frame_id=='odom'
    assert [field.name for field in forecasts[-1].fields]==['x','y','vx','vy']
    assert static_return_kept(), 'static marking lost its wall or retained confirmed mover returns'
    assert static_scans[-1].header.frame_id=='base_link'
    assert static_scans[-1].time_increment==0.


def main():
    prefix = Path(get_package_prefix('jt16_bringup'))/'lib/jt16_bringup'
    children = [subprocess.Popen(['/usr/bin/python3', str(prefix/name)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for name in ('lio_odom_bridge.py', 'project_nav_scan.py','predict_obstacles.py')]
    rclpy.init()
    node = Node('repair_interface_check')
    poses, scans, transforms = [], [], []
    predictions, tracks = [], []
    moving=[]
    forecasts=[]
    static_scans=[]
    node.create_subscription(PointCloud2,'/predicted_obstacles',predictions.append,1)
    node.create_subscription(PointCloud2,'/obstacle_tracks',tracks.append,1)
    node.create_subscription(PointCloud2,'/moving_obstacle_points',moving.append,1)
    node.create_subscription(PointCloud2,'/forecast_surface_points',forecasts.append,1)
    node.create_subscription(LaserScan,'/static_scan',static_scans.append,qos_profile_sensor_data)
    publisher = node.create_publisher(Odometry, '/Odometry', 10)
    clouds = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
    node.create_subscription(Odometry, '/odom', poses.append, 10)
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', lambda msg: transforms.extend(msg.transforms), 10)
    try:
        deadline = time.monotonic()+8
        while time.monotonic()<deadline and not (poses and scans and transforms and predictions):
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            # FAST-LIO's initial IMU pose is zero; the ground base is below that origin.
            message.pose.pose.position.x = 0.
            message.pose.pose.position.z = 0.
            message.pose.pose.orientation.w = 1.
            publisher.publish(message)
            fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(('x', 'y', 'z'))]
            clouds.publish(point_cloud2.create_cloud(message.header, fields, [(1., 0., -.2)]))
            rclpy.spin_once(node, timeout_sec=.1)
        check_outputs(poses, scans, transforms)
        assert predictions and predictions[-1].header.frame_id=='odom'
        body_reader=next(info for info in node.get_subscriptions_info_by_topic('/cloud_registered_body')
                         if info.node_name=='jt16_nav_scan_projection')
        assert body_reader.qos_profile.reliability==ReliabilityPolicy.RELIABLE
        check_derived_velocity(node, publisher, message, poses)
        check_motion_forecast(node, publisher, clouds, message, fields, predictions, tracks, moving, forecasts, static_scans)
        print('PASS installed bridge/TF/raw/static scans and LiDAR motion forecast interfaces')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_children(children)


if __name__ == '__main__':
    main()
