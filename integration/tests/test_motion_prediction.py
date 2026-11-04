"""Geometry-only motion estimates; no benchmark identities or ROS runtime."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_prediction import MotionPredictor, scan_clusters, pose_at_stamp


class MotionPredictionTests(unittest.TestCase):
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

    def test_static_cluster_has_no_forecast(self):
        tracker = MotionPredictor()
        for i in range(10):
            predicted, velocities = tracker.update([self.points],i*.12)
            self.assertEqual(len(predicted),0)
            self.assertEqual(len(velocities),0)

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
        self.assertEqual(len(tracker.update([], .8)[0]),0)
        self.assertEqual(tracker.moving_points.shape,(4,4))
        self.assertEqual(len(tracker.update([self.points],1.2)[0]),0)
        self.assertEqual(tracker.moving_points.shape,(0,4))
        changed=self.points*2
        self.assertEqual(len(tracker.update([changed],1.32)[0]),0)

    def test_temporal_motion_bridges_dropouts_without_refreshing_confirmation(self):
        tracker=MotionPredictor();velocity=np.array([.15,-.2])
        for i in range(6):tracker.update([self.points+velocity*i*.12],i*.12)
        tracker.update([], .8)
        np.testing.assert_allclose(tracker.moving_points[:,:2],self.points+velocity*.8)
        tracker.update([], 1.)
        np.testing.assert_allclose(tracker.moving_points[:,:2],self.points+velocity)
        tracker.update([], 1.11)
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
