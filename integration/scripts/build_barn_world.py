#!/usr/bin/env python3
"""Convert one public BARN Gazebo Classic world to Gazebo Harmonic."""
import argparse
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--world-id', type=int, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()

source = ROOT / f'external/benchmarks/the-barn-challenge/jackal_helper/worlds/BARN/world_{args.world_id}.world'
robot_file = ROOT / 'integration/models/xlerobot_jt16.sdf'
source_root = ET.parse(source).getroot()
source_world = source_root.find('world')
output_root = ET.Element('sdf', version='1.9')
world = ET.SubElement(output_root, 'world', name=f'barn_{args.world_id}')
for filename, name in (
    ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
    ('gz-sim-sensors-system', 'gz::sim::systems::Sensors'),
    ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
):
    plugin = ET.SubElement(world, 'plugin', filename=filename, name=name)
    if name.endswith('Sensors'):
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
physics = ET.SubElement(world, 'physics', name='benchmark', type='ignored')
ET.SubElement(physics, 'max_step_size').text = '0.002'
ET.SubElement(world, 'gravity').text = '0 0 -9.80665'
for tag in ('light', 'scene'):
    item = source_world.find(tag)
    if item is not None:
        copy = deepcopy(item)
        for source_pose in copy.findall('.//pose'):
            source_pose.attrib.pop('frame', None)
        world.append(copy)
for model in source_world.findall('model'):
    copy = deepcopy(model)
    for source_pose in copy.findall('.//pose'):
        source_pose.attrib.pop('frame', None)
    for material in copy.findall('.//material'):
        script = material.find('script')
        if script is not None:
            material.remove(script)
    world.append(copy)
robot = deepcopy(ET.parse(robot_file).getroot().find('model'))
robot.set('name', 'xlerobot')
pose = robot.find('pose')
if pose is None:
    pose = ET.SubElement(robot, 'pose')
pose.text = '-2.25 3.0 0.38 0 0 1.5707963267948966'
static = robot.find('static')
if static is None:
    static = ET.SubElement(robot, 'static')
static.text = 'false'
world.append(robot)
ET.indent(output_root)
args.output.parent.mkdir(parents=True, exist_ok=True)
ET.ElementTree(output_root).write(args.output, encoding='unicode', xml_declaration=True)
print(args.output)
