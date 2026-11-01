#!/usr/bin/env python3
"""Build a lightweight Harmonic world from one official Arena-Bench scenario."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[2]
ARENA = ROOT / 'external/benchmarks/arena-bench'
parser = argparse.ArgumentParser()
parser.add_argument('--world', choices=('small_warehouse', 'map2', 'map5'), required=True)
parser.add_argument('--obstacles', choices=(5, 10), type=int, required=True)
parser.add_argument('--seed', type=int, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
scenario_path = ARENA / f'simulator_setup/scenarios/{args.world}_obs{args.obstacles:02d}.json'
scenario = json.loads(scenario_path.read_text())
map_yaml_path = ARENA / f'simulator_setup/maps/{args.world}/map.yaml'
map_info = yaml.safe_load(map_yaml_path.read_text())
image = np.asarray(Image.open(map_yaml_path.parent / map_info['image']))
occupied = np.flipud(image) < int(round(255 * (1 - float(map_info.get('occupied_thresh', .65)))))
resolution = float(map_info['resolution']); origin = np.asarray(map_info['origin'][:2], dtype=float)


def row_runs(row):
    indices = np.flatnonzero(row)
    if not len(indices):
        return []
    splits = np.where(np.diff(indices) > 1)[0] + 1
    return [(int(group[0]), int(group[-1])) for group in np.split(indices, splits)]


rectangles, active = [], {}
for y, row in enumerate(occupied):
    runs = set(row_runs(row))
    for run, start in list(active.items()):
        if run not in runs:
            rectangles.append((run[0], run[1], start, y - 1)); del active[run]
    for run in runs:
        active.setdefault(run, y)
for run, start in active.items():
    rectangles.append((run[0], run[1], start, occupied.shape[0] - 1))

root = ET.Element('sdf', version='1.9'); world = ET.SubElement(root, 'world', name=f'arena_{args.world}_{args.obstacles}_{args.seed}')
for filename, name in (('gz-sim-physics-system','gz::sim::systems::Physics'),('gz-sim-sensors-system','gz::sim::systems::Sensors'),('gz-sim-imu-system','gz::sim::systems::Imu'),('gz-sim-scene-broadcaster-system','gz::sim::systems::SceneBroadcaster'),('gz-sim-user-commands-system','gz::sim::systems::UserCommands')):
    plugin=ET.SubElement(world,'plugin',filename=filename,name=name)
    if name.endswith('Sensors'):ET.SubElement(plugin,'render_engine').text='ogre2'
physics=ET.SubElement(world,'physics',name='benchmark',type='ignored');ET.SubElement(physics,'max_step_size').text='.002'
ET.SubElement(world,'gravity').text='0 0 -9.80665'
scene=ET.SubElement(world,'scene');ET.SubElement(scene,'ambient').text='.7 .7 .7 1';ET.SubElement(scene,'background').text='.25 .3 .35 1'
light=ET.SubElement(world,'light',name='sun',type='directional');ET.SubElement(light,'pose').text='0 0 10 0 0 0';ET.SubElement(light,'direction').text='-.5 -.5 -1'
floor=ET.SubElement(world,'model',name='floor');ET.SubElement(floor,'static').text='true';link=ET.SubElement(floor,'link',name='body')
for kind in ('collision','visual'):
    item=ET.SubElement(link,kind,name=kind);geometry=ET.SubElement(item,'geometry');plane=ET.SubElement(geometry,'plane');ET.SubElement(plane,'normal').text='0 0 1';ET.SubElement(plane,'size').text='100 100'
for index,(x0,x1,y0,y1) in enumerate(rectangles):
    sx=(x1-x0+1)*resolution;sy=(y1-y0+1)*resolution;cx=origin[0]+(x0+x1+1)*resolution/2;cy=origin[1]+(y0+y1+1)*resolution/2
    model=ET.SubElement(world,'model',name=f'wall_{index}');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=f'{cx} {cy} 1 0 0 0';link=ET.SubElement(model,'link',name='body')
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name=kind);geometry=ET.SubElement(item,'geometry');box=ET.SubElement(geometry,'box');ET.SubElement(box,'size').text=f'{sx} {sy} 2'

for index, agent in enumerate(scenario['pedsim_agents'][:args.obstacles]):
    points=[agent['pos'],*agent.get('waypoints',[])];points=[points[(i+args.seed)%len(points)] for i in range(len(points))]
    if args.seed%2:points=list(reversed(points))
    model=ET.SubElement(world,'model',name=f'dynamic_{index}');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=f'{points[0][0]} {points[0][1]} .85 0 0 0';link=ET.SubElement(model,'link',name='body')
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name=kind);geometry=ET.SubElement(item,'geometry');cylinder=ET.SubElement(geometry,'cylinder');ET.SubElement(cylinder,'radius').text='.30';ET.SubElement(cylinder,'length').text='1.7'

robot=deepcopy(ET.parse(ROOT/'integration/models/xlerobot_jt16.sdf').getroot().find('model'));robot.set('name','xlerobot');start=scenario['robot_position'];goal=scenario['robot_goal'];heading=math.atan2(goal[1]-start[1],goal[0]-start[0]);pose=robot.find('pose')
if pose is None:pose=ET.SubElement(robot,'pose')
pose.text=f'{start[0]} {start[1]} .38 0 0 {heading}';static=robot.find('static')
if static is None:static=ET.SubElement(robot,'static')
static.text='false';world.append(robot)
ET.indent(root);args.output.parent.mkdir(parents=True,exist_ok=True);ET.ElementTree(root).write(args.output,encoding='unicode',xml_declaration=True)
metadata={'world':args.world,'obstacles':args.obstacles,'seed':args.seed,'robot_position':start,'robot_goal':goal,'robot_yaw':heading,'rectangles':len(rectangles),'scenario':str(scenario_path.relative_to(ROOT))}
args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n');print(json.dumps(metadata,indent=2))
