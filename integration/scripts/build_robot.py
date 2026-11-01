#!/usr/bin/env python3
"""Generate a JT16-mounted upstream XLeRobot without changing vendor files."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def rigid_planar_model(robot):
    """Keep the body rigid while gravity restores floor contact after impacts."""
    for joint in robot.findall('joint'):
        joint.set('type','fixed')
    for link in robot.findall('link'):
        gazebo = ET.SubElement(robot,'gazebo',reference=link.get('name'))
        ET.SubElement(gazebo,'gravity').text='true'
        if link.get('name') == 'chassis':
            # Kinematic VelocityControl has no wheel torque to overcome box-floor friction.
            ET.SubElement(gazebo,'mu1').text='0'
            ET.SubElement(gazebo,'mu2').text='0'

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
mount = json.loads((ROOT.parent/'src/jt16_bringup/config/jt16_mount.json').read_text())
parser.add_argument('--height', type=float, default=mount['position_m'][2])
parser.add_argument('--pitch-up', type=float, default=mount['pitch_up_rad'])
args = parser.parse_args()
source = ROOT.parent/'external/reference/XLeRobot/simulation/xlerobot/xlerobot.urdf'
robot = ET.parse(source).getroot()
# Upstream chassis origin is 0.38 m above floor. Remove world anchor for Gazebo.
robot.remove(robot.find("joint[@name='joint_0']"))
robot.remove(robot.find("link[@name='world']"))
# Zero-mass upstream decorative links yield NaN during fixed-joint reduction.
# Use documented tiny positive inertias in this simulation derivative only.
for item in robot.findall('link/inertial'):
    if float(item.find('mass').get('value')) == 0:
        item.find('mass').set('value', '0.000001')
        tensor=item.find('inertia')
        for axis in ['ixx','iyy','izz']: tensor.set(axis,'0.000000001')
for mesh in robot.findall('.//mesh'):
    mesh.set('filename', str((source.parent/mesh.get('filename')).resolve()))
for joint_name in ['left_wheel_joint','right_wheel_joint']:
    limit=robot.find(f"joint[@name='{joint_name}']/limit")
    limit.set('effort','20');limit.set('velocity','20')
# Navigation uses a ground robot footprint. The upstream articulated model has
# mesh collisions for arm and decorative links that protrude beyond the mobile
# base and can pin the model in BARN corridors even when the base footprint is
# clear. V0 is planar kinematic simulation, so replace them with one measured
# navigation envelope; wheel dynamics are validated separately in later versions.
for source_link in robot.findall('link'):
    for collision in list(source_link.findall('collision')):
        source_link.remove(collision)
chassis = robot.find("link[@name='chassis']")
# sdformat applies link friction to the default collision name, not a custom name.
chassis_collision = ET.SubElement(chassis, 'collision')
# Bottom must coincide with base_link; settling otherwise projects floor as an obstacle.
ET.SubElement(chassis_collision, 'origin', xyz='0 0 -0.33', rpy='0 0 0')
chassis_geometry = ET.SubElement(chassis_collision, 'geometry')
ET.SubElement(chassis_geometry, 'box', size='0.3913 0.459 0.10')
link = ET.SubElement(robot, 'link', name='jt16_link')
inertial = ET.SubElement(link, 'inertial')
ET.SubElement(inertial, 'mass', value='0.1997')
ET.SubElement(inertial, 'inertia', ixx='0.00015', iyy='0.00015', izz='0.00015', ixy='0', ixz='0', iyz='0')
# Cylinder is a placeholder envelope, not manufacturer CAD or calibrated inertia.
for kind in ['visual','collision']:
    item=ET.SubElement(link,kind)
    geo=ET.SubElement(item,'geometry')
    ET.SubElement(geo,'cylinder',radius='0.035',length='0.065')
# Explicit ground-origin base_link, aligned with upstream chassis axes.
ET.SubElement(robot, 'link', name='base_link')
base_joint=ET.SubElement(robot,'joint',name='base_link_ground',type='fixed')
ET.SubElement(base_joint,'parent',link='chassis')
ET.SubElement(base_joint,'child',link='base_link')
ET.SubElement(base_joint,'origin',xyz='0 0 -0.38',rpy='0 0 0')
joint=ET.SubElement(robot,'joint',name='jt16_mount',type='fixed')
ET.SubElement(joint,'parent',link='base_link')
ET.SubElement(joint,'child',link='jt16_link')
# REP-103 +Y rotation tips +X down; pitch-up is negative URDF pitch.
mount_rpy = [mount['roll_rad'], -args.pitch_up, mount['yaw_rad']]
ET.SubElement(joint,'origin',xyz=f"{mount['position_m'][0]} {mount['position_m'][1]} {args.height:.6f}",rpy=' '.join(str(v) for v in mount_rpy))
gazebo=ET.SubElement(robot,'gazebo',reference='jt16_mount')
ET.SubElement(gazebo,'preserveFixedJoint').text='true'
sensors=ET.SubElement(robot,'gazebo',reference='jt16_link')
lidar=ET.SubElement(sensors,'sensor',name='jt16',type='gpu_lidar')
ET.SubElement(lidar,'always_on').text='true'
ET.SubElement(lidar,'update_rate').text='100'
ET.SubElement(lidar,'topic').text='/jt16/points_raw'
ray=ET.SubElement(lidar,'lidar');scan=ET.SubElement(ray,'scan')
vertical_fov = [v * 3.141592653589793 / 180.0 for v in mount['sim_vertical_fov_deg']]
# Provisional 0.3-degree sampling resolves BARN's 0.15 m poles at 10 m.
for axis,n,lo,hi in [('horizontal',1200,0,2*3.141592653589793*1199/1200),('vertical',16,vertical_fov[0],vertical_fov[1])]:
    e=ET.SubElement(scan,axis)
    for k,v in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:ET.SubElement(e,k).text=str(v)
r=ET.SubElement(ray,'range')
for k,v in [('min',0.15),('max',30),('resolution',0.01)]:ET.SubElement(r,k).text=str(v)
imu=ET.SubElement(sensors,'sensor',name='jt16_imu',type='imu')
ET.SubElement(imu,'always_on').text='true';ET.SubElement(imu,'update_rate').text='200';ET.SubElement(imu,'topic').text='/lidar_imu'
ET.SubElement(imu,'imu')
drive_gazebo=ET.SubElement(robot,'gazebo')
drive=ET.SubElement(drive_gazebo,'plugin',filename='gz-sim-velocity-control-system',name='gz::sim::systems::VelocityControl')
ET.SubElement(drive,'topic').text='/cmd_vel'
rigid_planar_model(robot)
odom=ET.SubElement(drive_gazebo,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
for key,value in [('odom_frame','world'),('robot_base_frame','chassis'),('dimensions','3'),('odom_topic','/ground_truth/odom'),('tf_topic','/ground_truth/tf'),('odom_publish_frequency','30')]:ET.SubElement(odom,key).text=value
ET.indent(robot)
(ROOT/'models').mkdir(parents=True, exist_ok=True)
ET.ElementTree(robot).write(ROOT/'models/xlerobot_jt16.urdf',encoding='unicode',xml_declaration=True)
print(ROOT/'models/xlerobot_jt16.urdf')
