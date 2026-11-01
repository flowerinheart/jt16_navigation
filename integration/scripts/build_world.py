import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E
r=Path(__file__).resolve().parents[1]
(r/'worlds').mkdir(parents=True, exist_ok=True)
parser=argparse.ArgumentParser();parser.add_argument('--coverage',action='store_true');parser.add_argument('--dynamic',action='store_true');args=parser.parse_args()
sdf=E.Element('sdf',version='1.9');world=E.SubElement(sdf,'world',name='jt16_smoke')
for filename,name in [('gz-sim-physics-system','gz::sim::systems::Physics'),('gz-sim-sensors-system','gz::sim::systems::Sensors'),('gz-sim-imu-system','gz::sim::systems::Imu'),('gz-sim-scene-broadcaster-system','gz::sim::systems::SceneBroadcaster')]:
 p=E.SubElement(world,'plugin',filename=filename,name=name)
 if name.endswith('Sensors'):E.SubElement(p,'render_engine').text='ogre2'
physics=E.SubElement(world,'physics',name='fixed',type='ignored');E.SubElement(physics,'max_step_size').text='0.002'
E.SubElement(world,'gravity').text='0 0 -9.80665'
scene=E.SubElement(world,'scene');E.SubElement(scene,'ambient').text='0.7 0.7 0.7 1';E.SubElement(scene,'background').text='0.3 0.4 0.5 1'
robot=E.parse(r/'models/xlerobot_jt16.sdf').getroot().find('model')
# Static mode isolates sensor geometry; dynamic mode validates wheel physics.
E.SubElement(robot,'static').text='false' if args.dynamic else 'true';E.SubElement(robot,'pose').text='0 0 0.38 0 0 0';world.append(robot)
# Provisional fixed top-camera pose, independent of tilted JT16.
chassis=robot.find("link[@name='chassis']")
cam=E.SubElement(chassis,'sensor',name='top_rgb',type='camera')
E.SubElement(cam,'pose').text='0 0 0.90 0 0.35 0'
E.SubElement(cam,'always_on').text='true';E.SubElement(cam,'update_rate').text='10'
E.SubElement(cam,'topic').text='/top_camera/image'
c=E.SubElement(cam,'camera');E.SubElement(c,'horizontal_fov').text='1.221730476'
im=E.SubElement(c,'image')
for k,v in [('width','640'),('height','480'),('format','R8G8B8')]:E.SubElement(im,k).text=v
clip=E.SubElement(c,'clip');E.SubElement(clip,'near').text='0.05';E.SubElement(clip,'far').text='30'
light=E.SubElement(world,'light',name='sun',type='directional');E.SubElement(light,'pose').text='0 0 10 0 0 0';E.SubElement(light,'diffuse').text='0.8 0.8 0.8 1';E.SubElement(light,'direction').text='-0.5 -0.5 -1'
base_objects=[('floor','0 0 -0.1 0 0 0','20 20 0.2'),('front_wall','4 0 1.5 0 0 0','0.2 8 3'),('left_wall','0 4 1.5 0 0 0','8 0.2 3')]
if not args.coverage:base_objects.append(('low_box','1.5 0 0.15 0 0 0','0.4 0.4 0.3'))
for name,pose,size in base_objects:
 m=E.SubElement(world,'model',name=name);E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text=pose;l=E.SubElement(m,'link',name='body')
 for typ in ['collision','visual']:
  q=E.SubElement(l,typ,name=typ);g=E.SubElement(q,'geometry');b=E.SubElement(g,'box');E.SubElement(b,'size').text=size
  if typ=='visual':
   mat=E.SubElement(q,'material');E.SubElement(mat,'ambient').text='0.7 0.7 0.7 1';E.SubElement(mat,'diffuse').text=('0.8 0.2 0.1 1' if name=='low_box' else '0.7 0.7 0.7 1')
if args.coverage:
 coverage=[]
 for index,(angle_deg,height) in enumerate(zip(range(0,360,45),[.1,.2,.3,.5,.1,.2,.3,.5])):
  angle=math.radians(angle_deg);distance=2.5;x=distance*math.cos(angle);y=distance*math.sin(angle);name=f'coverage_{index}'
  m=E.SubElement(world,'model',name=name);E.SubElement(m,'static').text='true';E.SubElement(m,'pose').text=f'{x} {y} {height/2} 0 0 0';l=E.SubElement(m,'link',name='body')
  for typ in ['collision','visual']:
   q=E.SubElement(l,typ,name=typ);g=E.SubElement(q,'geometry');b=E.SubElement(g,'box');E.SubElement(b,'size').text=f'0.35 0.35 {height}'
  coverage.append({'name':name,'center_m':[x,y,height/2],'size_m':[.35,.35,height],'azimuth_deg':angle_deg,'range_m':distance,'height_m':height})
 (r/'worlds/jt16_coverage_manifest.json').write_text(json.dumps(coverage,indent=2))
E.indent(sdf);E.ElementTree(sdf).write(r/'worlds/jt16_smoke.sdf',encoding='unicode')
