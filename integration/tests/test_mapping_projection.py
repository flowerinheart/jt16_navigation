"""Regress scan-time map projection when wall returns dominate global height."""
import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from sensor_geometry import map_points_with_base_height


class MappingProjectionTests(unittest.TestCase):
    def test_height_filter_does_not_follow_estimated_world_tilt(self):
        xyz=np.array([[20.,0.,0.],[1.,0.,-.36]])
        mount={'pitch_up_rad':0.,'position_m':[-.03,0.,.36]}
        for pitch,z in ((0.,0.),(.03,.04)):
            q=Rotation.from_euler('y',pitch).as_quat()
            points=map_points_with_base_height(xyz,[0.,0.,z],q,mount)
            np.testing.assert_allclose(points[:,:2],Rotation.from_quat(q).apply(xyz)[:,:2])
            np.testing.assert_allclose(points[:,2],[.36,0.],atol=1e-9)

    def test_cloud_waits_for_its_exact_pose(self):
        source=ast.parse((SCRIPTS/'follow_barn_path.py').read_text())
        functions=[node for node in source.body if isinstance(node,ast.FunctionDef) and node.name in
                   ('process_clouds','pose_3d_sample','cloud_sample')]
        rows=np.array([(1.,0.,0.)],dtype=[('x','f4'),('y','f4'),('z','f4')])
        state={'deque':deque,'np':np,'stamp':lambda m:m.time,'poses_3d':deque(maxlen=50),
               'pending_clouds':deque(maxlen=5),'map_clouds':[],'cloud_stamps':[],
               'mount':{'pitch_up_rad':0.,'position_m':[0.,0.,.36]},
               'map_points_with_base_height':map_points_with_base_height,
               'point_cloud2':SimpleNamespace(read_points=lambda *a,**k:rows)}
        exec(compile(ast.Module(body=functions,type_ignores=[]),'follower','exec'),state)
        cloud=SimpleNamespace(time=1.,header=SimpleNamespace(frame_id='body'))
        state['cloud_sample'](cloud)
        self.assertEqual(len(state['map_clouds']),0)
        def pose(t,x):
            return SimpleNamespace(time=t,pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=x,y=0.,z=.1),
                orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
        state['pose_3d_sample'](pose(1.,2.))
        np.testing.assert_allclose(state['map_clouds'][0],[[3.,0.,.36]])
        state['pose_3d_sample'](pose(2.,4.))
        cloud.time=1.5
        state['cloud_sample'](cloud)
        self.assertEqual(len(state['map_clouds']),1)
        self.assertFalse(state['pending_clouds'])

    def test_wall_dominated_cloud_exports_without_inventing_floor_occupancy(self):
        from PIL import Image
        import yaml
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            raw=np.array([[0.,0.,-.36]]*10+[[.35,0.,0.]]*1000,dtype='<f4')
            header=('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                    f'WIDTH {len(raw)}\nHEIGHT 1\nPOINTS {len(raw)}\nDATA binary\n')
            pcd=root/'map.pcd';pcd.write_bytes(header.encode()+raw.tobytes())
            command=[sys.executable,str(SCRIPTS/'pcd_to_occupancy.py'),'--pcd',str(pcd),
                     '--output',str(root/'map'),'--min-hits','1']
            previous=subprocess.run(command,capture_output=True,text=True)
            self.assertNotEqual(previous.returncode,0)
            self.assertIn('no points remain after height filtering',previous.stderr)
            points=map_points_with_base_height(raw,[0.,0.,0.],[0.,0.,0.,1.],
                                               {'pitch_up_rad':0.,'position_m':[0.,0.,.36]})
            archive=root/'points.npz';np.savez(archive,map_xy_height=points)
            subprocess.run(command+['--scan-points',str(archive)],capture_output=True,check=True)
            info=yaml.safe_load((root/'map/map.yaml').read_text())
            grid=np.flipud(np.asarray(Image.open(root/'map/map.pgm')))
            def cell(x):
                cx,cy=np.floor((np.array([x,0.])-info['origin'][:2])/info['resolution']).astype(int)
                return grid[cy,cx]
            self.assertEqual(cell(.35),0)
            self.assertEqual(cell(0.),205)
            self.assertEqual(np.count_nonzero(grid==0),1)
            self.assertEqual(cell(.40),205)


if __name__=='__main__':
    unittest.main()
