"""Geometry-only motion estimates; no benchmark identities or ROS runtime."""
from pathlib import Path
import ast
from collections import deque
from types import SimpleNamespace
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_prediction import MotionPredictor, scan_clusters, pose_at_stamp, static_map_exclusions, surface_translation, static_scan_ranges
from scipy.ndimage import distance_transform_edt


class MotionPredictionTests(unittest.TestCase):

    def test_constructor_uses_selected_navigation_profile(self):
        import tempfile
        import yaml
        scripts = Path(__file__).resolve().parents[1] / 'scripts'
        tree = ast.parse((scripts / 'predict_obstacles.py').read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
        init = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
        end = next(i for i, node in enumerate(init.body) if isinstance(node, ast.Assign) and
                   any(isinstance(t, ast.Attribute) and t.attr == 'map_margin' for t in node.targets))
        init.body = init.body[:end + 1]; cls.body = [init]
        config_dir = scripts.parents[1] / 'src/jt16_bringup'
        with tempfile.TemporaryDirectory() as directory:
            selected = Path(directory) / 'alternate profile.yaml'
            config = yaml.safe_load((config_dir / 'config/nav2_params.yaml').read_text())
            config['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] = 1.5
            selected.write_text(yaml.safe_dump(config))
            for override, expected in ((None, .75), (str(selected), 1.5)):
                class Node:
                    def __init__(self, name): self.parameters = {}
                    def declare_parameter(self, name, value): self.parameters[name] = override or value
                    def get_parameter(self, name): return SimpleNamespace(value=self.parameters[name])
                namespace = {'Node': Node, 'MotionPredictor': MotionPredictor, 'deque': deque,
                             'Buffer': lambda: None, 'TransformListener': lambda *args: None,
                             'Path': Path, 'yaml': yaml, 'get_package_share_directory': lambda _: str(config_dir)}
                exec(compile(ast.Module(body=[cls], type_ignores=[]), 'predictor', 'exec'), namespace)
                self.assertEqual(namespace['ObstaclePredictor']().map_margin, expected)
            for name in ('run_arena_navigation.sh', 'run_barn_navigation.sh'):
                self.assertIn('-p "nav2_params_file:=$PARAMS"', (scripts / name).read_text())


    def setUp(self):
        self.points = np.asarray([[0.,0.],[.04,0.],[.04,.04],[0.,.04]])+[1.,1.]

    def test_scan_pose_interpolates_bracketing_odometry_and_wrapped_yaw(self):
        poses=[(54.74991666,[0.,0.,0.]),(55.02325,[.027333334,0.,.1])]
        np.testing.assert_allclose(pose_at_stamp(poses,54.97991667),
                                   [.023000001,0.,.084146345],atol=1e-8)
        poses=[(0.,[0.,0.,np.deg2rad(170)]),(.2,[.2,.4,np.deg2rad(-170)])]
        np.testing.assert_allclose(pose_at_stamp(poses,.1),[.1,.2,np.pi])

    def test_scan_pose_rejects_unbounded_gaps_and_extrapolation(self):
        self.assertIsNone(pose_at_stamp([],1.))
        poses=[(0.,[0.,0.,0.]),(1.,[1.,0.,0.])]
        self.assertIsNone(pose_at_stamp(poses,.5))
        self.assertIsNone(pose_at_stamp(poses,1.1))
        self.assertEqual(pose_at_stamp(poses,1.),[1.,0.,0.])
        self.assertEqual(pose_at_stamp(poses,1.02),[1.,0.,0.])

    def test_visible_cluster_beyond_four_meters_respects_sensor_range(self):
        ranges=np.full(300,np.inf);ranges[5:11]=6.
        self.assertEqual(len(scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.])),1)
        self.assertEqual(len(scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.],.15,10.)),1)
        self.assertEqual(scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.],.15,5.),[])
        with self.assertRaises(ValueError):
            scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.],.15,float('nan'))

    def test_sparse_map_surface_does_not_relax_motion_confirmation_geometry(self):
        ranges = np.full(300,np.inf);ranges[10] = 2.
        self.assertEqual(scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.]),[])
        sparse = scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.],minimum_points=1)
        self.assertEqual(len(sparse),1)
        self.assertEqual(sparse[0].shape,(1,2))

    def test_static_cluster_has_no_forecast(self):
        tracker = MotionPredictor()
        for i in range(10):
            predicted, velocities = tracker.update([self.points],i*.12)
            self.assertEqual(len(predicted),0)
            self.assertEqual(len(velocities),0)

    def test_static_scan_masks_only_current_confirmation_at_wrapped_bins(self):
        for angle_min in (0.,-np.pi):
            with self.subTest(angle_min=angle_min):
                pose=np.array([-1.3,2.1,.77]);increment=2*np.pi/300
                ranges=np.full(300,np.inf);indices=np.array([298,299,0,1,49,50,51,52])
                ranges[indices]=2.+np.arange(len(indices))*.01
                angles=angle_min+indices*increment;c,s=np.cos(pose[2]),np.sin(pose[2])
                points=np.column_stack((ranges[indices]*np.cos(angles),ranges[indices]*np.sin(angles)))@np.array([[c,s],[-s,c]])+pose[:2]
                tracks=[{'motion':(1.,points[:4],np.array([.2,0.])),
                         'forecast_motion':(1.,points,np.array([.2,0.]))},
                        {'motion':(.9,points[4:],np.array([.2,0.]))},
                        {'points':points[4:]}]
                before=ranges.copy();expected=ranges.copy();expected[indices[:4]]=np.inf
                np.testing.assert_array_equal(static_scan_ranges(ranges,angle_min,increment,pose,tracks,1.),expected)
                np.testing.assert_array_equal(ranges,before)
                np.testing.assert_array_equal(static_scan_ranges(ranges,angle_min,increment,pose,tracks,1.1),before)

    def test_static_map_keeps_wall_pillar_unknown_and_uncertain_surfaces(self):
        cells = np.zeros((100,100),dtype=int)
        cells[:,10] = 100
        cells[60,60] = 100
        cells[75:85,75:85] = -1
        distances = distance_transform_edt(cells<65)*.1
        clusters = [self.points+[3.,2.], self.points+[.2,2.], self.points+[5.,5.],
                    self.points+[7.,7.], self.points+[10.,10.]]
        flags = static_map_exclusions(clusters,cells,distances,.1,np.array([0.,0.,0.]),
                                      np.array([3.,3.]),np.array([.01,.01,.001]),.75)
        self.assertEqual(flags,[True,False,False,False,False])

        flags = static_map_exclusions(clusters[:1],cells,distances,.1,np.array([0.,0.,0.]),
                                      np.array([3.,3.]),np.array([1.,1.,1.]),.75)
        self.assertEqual(flags,[False])
        rotation = np.array([[0.,1.],[-1.,0.]])
        offset = np.array([5.,2.])
        flags = static_map_exclusions([points@rotation+offset for points in clusters],cells,distances,.1,
                                      np.array([5.,2.,np.pi/2]),np.array([3.,3.])@rotation+offset,
                                      np.array([.01,.01,.001]),.75)
        self.assertEqual(flags,[True,False,False,False,False])

    def test_slam_exclusion_publication_keeps_motion_and_missing_context_contract(self):
        source = ast.parse((Path(__file__).resolve().parents[1]/'scripts/predict_obstacles.py').read_text())
        cls = next(node for node in source.body if isinstance(node,ast.ClassDef))
        cls.body = [node for node in cls.body if node.name!='__init__']
        class Field:
            FLOAT32 = 7
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)
        class MissingTransform(Exception):
            pass
        state = {'Node':object,'np':np,'math':__import__('math'),'PointField':Field,
                 'deepcopy':__import__('copy').deepcopy,'static_scan_ranges':static_scan_ranges,
                 'Time':lambda seconds:seconds,'TransformException':MissingTransform,
                 'pose_at_stamp':pose_at_stamp,'static_map_exclusions':static_map_exclusions,
                 'scan_clusters':lambda *args,**kwargs:[self.points],
                 'point_cloud2':SimpleNamespace(create_cloud=lambda header,fields,rows:rows.copy(),
                                                create_cloud_xyz32=lambda header,rows:rows.copy())}
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<predictor>','exec'),state)
        cells = np.zeros((100,100),dtype=int);cells[:,0] = 100
        geometry = (cells,distance_transform_edt(cells<65)*.1,.1,np.zeros(3))
        moving = np.array([[*self.points[0],.2,0.]])
        for context in ('valid','missing_map','stale_covariance','missing_tf'):
            with self.subTest(context=context):
                node = state['ObstaclePredictor']()
                publications = {}
                for name in ('predictions','velocities','moving','slam_rejected','forecast_surfaces','static_scans'):
                    setattr(node,name,SimpleNamespace(publish=lambda rows,name=name:publications.update({name:rows})))
                node.poses = deque([(1.,[0.,0.,0.])])
                node.pending = SimpleNamespace(header=SimpleNamespace(frame_id='base_link',
                    stamp=SimpleNamespace(sec=1,nanosec=0)),ranges=[],angle_min=0.,angle_increment=.1,
                    range_min=.15,range_max=30.)
                node.predictor = SimpleNamespace(moving_points=moving,tracks=[],
                    forecast_points=np.vstack((moving,[[2.,2.,.2,0.]])),
                    update=lambda *args:(np.empty((0,2)),np.empty((0,4))))
                node.map = None if context=='missing_map' else geometry
                node.map_margin = .75
                node.covariance = (0. if context=='stale_covariance' else 1.,np.zeros(3))
                lookups = []
                def lookup(target, origin, stamp):
                    lookups.append((target,origin,stamp))
                    if context=='missing_tf':
                        raise MissingTransform()
                    return SimpleNamespace(transform=SimpleNamespace(
                        translation=SimpleNamespace(x=3.,y=2.),
                        rotation=SimpleNamespace(x=0.,y=0.,z=np.sin(np.pi/4),w=np.cos(np.pi/4))))
                node.tf = SimpleNamespace(lookup_transform=lookup)
                node.process()
                self.assertEqual(publications['static_scans'].header.frame_id,'base_link')
                self.assertEqual(publications['static_scans'].header.stamp.sec,1)
                np.testing.assert_allclose(publications['moving'],moving)
                np.testing.assert_allclose(publications['slam_rejected'][:1],moving)
                np.testing.assert_allclose(publications['forecast_surfaces'][:2],node.predictor.forecast_points)
                np.testing.assert_allclose(publications['forecast_surfaces'][2:],publications['slam_rejected'][1:])
                if context=='valid':
                    self.assertEqual(lookups,[('map','odom',1.)])
                    np.testing.assert_allclose(publications['slam_rejected'][1:,:2],self.points[1:])
                    np.testing.assert_allclose(publications['slam_rejected'][1:,2:],0.)
                else:
                    self.assertEqual(publications['slam_rejected'].shape,(1,4))
        for missing_pose in (True,False):
            publications={}
            message=SimpleNamespace(header=SimpleNamespace(frame_id='base_link',
                stamp=SimpleNamespace(sec=1,nanosec=0)),ranges=[1.,2.])
            node.pending=message;node.poses=deque([] if missing_pose else [(1.,[0.,0.,0.])])
            def invalid_update(*args):
                raise ValueError('invalid prediction')
            node.predictor.update=invalid_update
            node.get_logger=lambda:SimpleNamespace(error=lambda error:None)
            if not missing_pose:
                message.angle_min=0.;message.angle_increment=.1;message.range_min=.15;message.range_max=30.
            node.process()
            self.assertEqual(set(publications),{'static_scans'})
            self.assertEqual(publications['static_scans'].ranges,[1.,2.])
            self.assertEqual(publications['static_scans'].header.frame_id,'base_link')
            self.assertIsNot(publications['static_scans'],message)

    def test_moving_cluster_estimate_and_forward_swept_points(self):
        tracker = MotionPredictor()
        velocity = np.asarray([.15,-.2])
        for i in range(6):
            points = self.points+velocity*i*.12
            predicted, velocities = tracker.update([points],i*.12)
        np.testing.assert_allclose(velocities[0,2:],velocity,atol=1e-10)
        np.testing.assert_allclose(predicted[:4],points+velocity*.2)
        np.testing.assert_allclose(predicted[-4:],points+velocity*4.)
        self.assertEqual(len(predicted),20*len(points))
        np.testing.assert_allclose(tracker.moving_points[:,:2],points)
        np.testing.assert_allclose(tracker.moving_points[:,2:],np.tile(velocity,(len(points),1)))

    def test_small_corner_translation_ignores_partial_visible_centroid(self):
        corner=np.array([[0.,0.],[.1,0.],[.2,0.],[0.,.1],[0.,.2]])
        translation=np.array([.03,-.02])
        shift,error=surface_translation(corner,corner[[0,1,3,4]]+translation)
        np.testing.assert_allclose(shift,translation,atol=1e-10)
        self.assertLess(error,1e-10)

    def test_surface_motion_ignores_changing_visible_centroid(self):
        angles=np.linspace(-1.2,1.2,40)
        shape=np.column_stack((.3*np.cos(angles),.3*np.sin(angles)))+[1.,1.]
        velocity=np.array([.2,-.1]);tracker=MotionPredictor()
        for i in range(8):
            _,tracks=tracker.update([shape[3*i:]+velocity*i*.1],i*.1)
        self.assertEqual(len(tracks),1)
        np.testing.assert_allclose(tracks[0,2:],velocity,atol=1e-3)

    def test_static_surface_with_partial_visibility_is_not_moving(self):
        angles=np.linspace(-1.2,1.2,40)
        shape=np.column_stack((.3*np.cos(angles),.3*np.sin(angles)))+[1.,1.]
        tracker=MotionPredictor()
        for i in range(8):
            predicted,tracks=tracker.update([shape[3*i:]],i*.1)
            self.assertEqual(len(predicted),0)
            self.assertEqual(len(tracks),0)

    def test_partial_surface_keeps_confirmed_motion(self):
        angles=np.linspace(-1.2,1.2,40)
        shape=np.column_stack((.3*np.cos(angles),.3*np.sin(angles)))+[1.,1.]
        velocity=np.array([.2,-.1]);tracker=MotionPredictor()
        for i in range(6):tracker.update([shape+velocity*i*.1],i*.1)
        _,tracks=tracker.update([shape[15:25]+velocity*.6],.6)
        self.assertEqual(len(tracks),1)
        np.testing.assert_allclose(tracks[0,2:],velocity,atol=1e-3)

    def test_confirmed_surface_retains_recent_sides_then_expires_geometry(self):
        angles=np.linspace(-1.2,1.2,40)
        shape=np.column_stack((.3*np.cos(angles),.3*np.sin(angles)))+[1.,1.]
        velocity=np.array([.2,-.1]);tracker=MotionPredictor()
        for i in range(6):tracker.update([shape+velocity*i*.1],i*.1)
        _,tracks=tracker.update([shape[15:25]+velocity*.6],.6)
        self.assertEqual(len(tracks),1)
        missing_side=shape[0]+velocity*.6
        self.assertLess(np.linalg.norm(tracker.forecast_points[:,:2]-missing_side,axis=1).min(),1e-3)
        self.assertGreater(np.linalg.norm(tracker.moving_points[:,:2]-missing_side,axis=1).min(),.1)
        for i in range(7,14):
            _,tracks=tracker.update([shape[15:25]+velocity*i*.1],i*.1)
            self.assertEqual(len(tracks),1)
        expired_side=shape[0]+velocity*1.3
        self.assertGreater(np.linalg.norm(tracker.forecast_points[:,:2]-expired_side,axis=1).min(),.1)
        tracker.update([],1.81)
        self.assertEqual(len(tracker.moving_points),0)
        self.assertEqual(len(tracker.forecast_points),0)

    def test_robot_translation_and_rotation_are_compensated(self):
        ranges=np.full(300,np.inf);ranges[5:11]=1.
        original=scan_clusters(ranges,0.,2*np.pi/300,[0.,0.,0.])[0]
        translated=scan_clusters(ranges,0.,2*np.pi/300,[1.,2.,0.])[0]
        np.testing.assert_allclose(translated,original+[1.,2.])
        rotated_ranges=np.roll(ranges,-25)
        rotated=scan_clusters(rotated_ranges,0.,2*np.pi/300,[0.,0.,25*2*np.pi/300])[0]
        # Wrapping scan order may change, but the world point set is unchanged.
        for point in original:
            self.assertLess(np.linalg.norm(rotated-point,axis=1).min(),1e-12)
        tracker=MotionPredictor()
        for i in range(6):
            prediction,_=tracker.update([original if i%2==0 else rotated],i*.12)
        self.assertEqual(len(prediction),0)

    def test_disappearance_gap_and_geometry_change_do_not_reuse_velocity(self):
        tracker=MotionPredictor()
        for i in range(6):tracker.update([self.points+[i*.025,0.]],i*.12)
        self.assertEqual(len(tracker.update([], .8)[0]),20*len(self.points))
        self.assertEqual(tracker.moving_points.shape,(4,4))
        self.assertEqual(len(tracker.update([self.points],1.2)[0]),0)
        self.assertEqual(tracker.moving_points.shape,(0,4))
        changed=self.points*2
        self.assertEqual(len(tracker.update([changed],1.32)[0]),0)

    def test_temporal_motion_bridges_dropouts_without_refreshing_confirmation(self):
        tracker=MotionPredictor();velocity=np.array([.15,-.2])
        for i in range(6):tracker.update([self.points+velocity*i*.12],i*.12)
        predicted, velocities = tracker.update([], .8)
        self.assertEqual(len(velocities), 0)
        np.testing.assert_allclose(predicted[:4],self.points+velocity*1.)
        np.testing.assert_allclose(predicted[-4:],self.points+velocity*4.8)
        np.testing.assert_allclose(tracker.moving_points[:,:2],self.points+velocity*.8)
        tracker.update([], 1.)
        np.testing.assert_allclose(tracker.moving_points[:,:2],self.points+velocity)
        predicted, _ = tracker.update([], 1.11)
        self.assertEqual(predicted.shape,(0,2))
        self.assertEqual(tracker.moving_points.shape,(0,4))
        tracker.update([self.points+velocity*1.2],1.2)
        self.assertEqual(tracker.moving_points.shape,(0,4))

    def test_other_visible_mover_does_not_erase_or_duplicate_lost_track(self):
        tracker=MotionPredictor();velocity=np.array([.15,-.2])
        for i in range(6):
            points=self.points+velocity*i*.12
            tracker.update([points,points+[2.,0.]],i*.12)
        tracker.update([self.points+velocity*.72+[2.,0.]],.72)
        self.assertEqual(tracker.moving_points.shape,(8,4))
        tracker.update([self.points+velocity*.84,self.points+velocity*.84+[2.,0.]],.84)
        self.assertEqual(tracker.moving_points.shape,(8,4))

    def test_noisy_centroid_motion_is_not_a_confirmed_velocity(self):
        tracker=MotionPredictor()
        for i in range(8):
            points=self.points+[(-1)**i*.08,0.]
            self.assertEqual(len(tracker.update([points],i*.12)[0]),0)

    def test_wall_visibility_sliding_is_not_predicted_as_motion(self):
        wall=np.column_stack((np.ones(8),np.linspace(0.,.3,8)))
        tracker=MotionPredictor()
        for i in range(8):
            self.assertEqual(len(tracker.update([wall+[0.,.2*i*.12]],i*.12)[0]),0)
        tracker=MotionPredictor()
        for i in range(8):prediction,velocity=tracker.update([wall+[.2*i*.12,0.]],i*.12)
        self.assertGreater(len(prediction),0)
        np.testing.assert_allclose(velocity[0,2:],[.2,0.],atol=1e-10)

    def test_static_corner_with_changing_visibility_is_not_forecast(self):
        tracker=MotionPredictor()
        for i in range(7):
            vertical=np.column_stack((np.zeros(25),np.linspace(0.,.4-.03*i,25)))
            horizontal=np.column_stack((np.linspace(0.,.15+.03*i,25),np.zeros(25)))
            points=np.concatenate([vertical,horizontal])+[1.,1.]
            self.assertEqual(len(tracker.update([points],i*.12)[0]),0)

    def test_invalid_time_geometry_and_angles_rejected(self):
        tracker=MotionPredictor();tracker.update([self.points],1.)
        for stamp in [1.,.9,-1.,float('nan')]:
            with self.assertRaises(ValueError):tracker.update([self.points],stamp)
        with self.assertRaises(ValueError):tracker.update([[[float('nan'),0.]]],2.)
        with self.assertRaises(ValueError):scan_clusters([1.],float('nan'),.1,[0.,0.,0.])
        self.assertEqual(scan_clusters([float('inf')]*10,0.,.1,[0.,0.,0.]),[])


if __name__=='__main__':unittest.main()
