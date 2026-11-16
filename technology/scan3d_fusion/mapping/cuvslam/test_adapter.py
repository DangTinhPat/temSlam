"""Frame conversion and independent trajectory evaluation regressions."""
import unittest
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import h5py
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from evaluate import metrics
from track_rgbd import RgbdSlam, body_pose, pose_matrix, MAP_FROM_START, optimized_map_poses, validate_local_tracking


class AdapterTests(unittest.TestCase):
    def test_all_zero_depth_is_rejected_before_sdk_without_advancing_timestamp(self):
        from unittest.mock import Mock
        slam=RgbdSlam.__new__(RgbdSlam)
        slam.size=(8,8);slam.last_timestamp=7;slam.last_odometry_pose=np.eye(4)
        slam.tracker=SimpleNamespace(track=Mock(return_value=(SimpleNamespace(world_from_rig=None),None)))
        with self.assertRaisesRegex(ValueError,'depth'):
            slam.track(8,np.zeros((8,8,3),np.uint8),np.zeros((8,8),np.uint16))
        slam.tracker.track.assert_not_called()
        self.assertEqual(slam.last_timestamp,7)
        self.assertIsNone(slam.last_odometry_pose)

    def test_rejected_frame_cannot_rewrite_exported_history_or_save_bad_database(self):
        import track_rgbd
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);capture=root/'capture';capture.mkdir();prediction=root/'prediction';prediction.mkdir()
            frames=[];packets=[]
            for i in (1,2,3):
                frames.append(dict(file=f'{i}.h5',time_s=float(i),body_from_camera=np.eye(4).tolist()))
                packets.append(dict(time_s=float(i),valid=1))
                with h5py.File(capture/f'{i}.h5','w') as sample:
                    for key,value in dict(rgb=np.zeros((8,8,3),np.uint8),hist_data=np.zeros((64,2)),fr=np.zeros((64,4)),mask=np.ones(64,bool)).items():sample[key]=value
                Image.fromarray(np.full((8,8),1000,np.uint16)).save(prediction/f'{i}_depth.png')
            (capture/'sequence.json').write_text(json.dumps(dict(image_size=[8,8],intrinsics=[4,4,4,4],frames=frames)))
            odometry=root/'odometry.jsonl';odometry.write_text('\n'.join(map(json.dumps,packets)))
            saved=[]
            class FakeSlam:
                def __init__(self,*a):self.tracker=self;self.index=0;self.last_odometry_pose=np.eye(4)
                def track(self,*a):
                    self.index+=1;self.last_odometry_pose=np.eye(4);self.last_odometry_pose[0,3]=[0,.01,1.][self.index-1]
                    return self.last_odometry_pose
                def get_all_slam_poses(self):
                    history=[]
                    for i in range(1,self.index+1):
                        pose=np.eye(4);pose[0,3]=(i-1)*.01
                        if self.index==3 and i==2:pose[0,3]=99
                        history.append(SimpleNamespace(timestamp_ns=i*1_000_000_000,pose=pose))
                    return history
                def get_slam_metrics(self):return None
                def get_pose_graph(self):return None
                def get_loop_closure_poses(self):return []
                def save_map(self,path,callback):saved.append(path);callback(True)
            argv=['track_rgbd.py',str(capture),str(prediction),str(root/'out'),'--odometry',str(odometry)]
            with patch('sys.argv',argv),patch.object(track_rgbd,'RgbdSlam',FakeSlam), \
                    patch.object(track_rgbd,'pose_matrix',lambda p:p), \
                    patch.object(track_rgbd,'odom_body',lambda p:np.eye(4)), \
                    patch.object(track_rgbd,'tof_support',lambda depth,*a:depth):
                self.assertEqual(track_rgbd.main(),2)
            result=json.loads((root/'out/sequence.json').read_text())
            self.assertEqual(len(result['frames']),2)
            self.assertAlmostEqual(result['frames'][1]['camera_to_world'][0][3],.01)
            self.assertAlmostEqual(result['frames'][1]['visual_camera_to_world'][0][3],.01)
            self.assertFalse(saved)
            argv[3]=str(root/'zero-out')
            with patch('sys.argv',argv),patch.object(track_rgbd,'RgbdSlam',FakeSlam), \
                    patch.object(FakeSlam,'track',side_effect=ValueError('Require nonzero measured depth')), \
                    patch.object(track_rgbd,'tof_support',lambda depth,*a:depth):
                self.assertEqual(track_rgbd.main(),2)
            zero=json.loads((root/'zero-out/tracking.json').read_text())
            self.assertEqual(zero['tracked_frames'],0)
            self.assertFalse(zero['map_saved'])
            self.assertIsNone(zero['tracker_ms_median'])

    def test_sdk_local_pose_is_kept_separate_from_corrected_slam_pose(self):
        local=SimpleNamespace(rotation=[0,0,0,1],translation=[0,0,.03])
        global_pose=SimpleNamespace(rotation=[0,0,0,1],translation=[0,0,2.])
        estimate=SimpleNamespace(world_from_rig=SimpleNamespace(pose=local))
        slam=RgbdSlam.__new__(RgbdSlam)
        slam.size=(8,8);slam.last_timestamp=-1;slam.last_odometry_pose=None
        slam.tracker=SimpleNamespace(track=lambda *a,**k:(estimate,global_pose))
        rgb=np.zeros((8,8,3),np.uint8);depth=np.ones((8,8),np.uint16)
        mapped=slam.track(1,rgb,depth)
        self.assertAlmostEqual(mapped[0,3],2.)
        self.assertAlmostEqual(slam.last_odometry_pose[0,3],.03)
        estimate.world_from_rig=None
        self.assertIsNone(slam.track(2,rgb,depth))
        self.assertIsNone(slam.last_odometry_pose)

    def test_local_tracking_check_handles_moving_head_and_rejects_bad_visual_motion(self):
        previous=np.eye(4)
        current=np.eye(4)
        current[:3,:3]=Rotation.from_euler('z',.4).as_matrix()
        current[:3,3]=[.03,.01,0.]
        validate_local_tracking(previous,current,previous,current)
        # The checker receives local visual odometry, not globally corrected SLAM poses.
        bad=current.copy();bad[0,3]+=.3
        with self.assertRaises(RuntimeError):validate_local_tracking(previous,bad,previous,current)
        bad=current.copy();bad[:3,:3]=Rotation.from_euler('z',.8).as_matrix()
        with self.assertRaises(RuntimeError):validate_local_tracking(previous,bad,previous,current)

    def test_large_historical_loop_correction_is_preserved_in_map(self):
        initial=np.eye(4);initial[:3,3]=[.1,-.2,.235]
        graph={t:np.eye(4) for t in (1,2,3)}
        graph[1][:3,:3]=Rotation.from_euler('z',.6).as_matrix()
        graph[1][:3,3]=[3.,4.,.2]
        graph[2]=graph[1].copy();graph[2][:3,3]+=[1.2,-.4,.08]
        graph[3]=graph[1].copy();graph[3][:3,3]+=[.02,0.,0.]
        mapped=optimized_map_poses(graph,[1,2,3],initial)
        np.testing.assert_allclose(mapped[0],initial,atol=1e-12)
        # Neither the 1.2m correction nor the optimized vertical component is clamped.
        np.testing.assert_allclose(np.linalg.inv(mapped[0])@mapped[1],
                                   np.linalg.inv(graph[1])@graph[2],atol=1e-12)
        self.assertGreater(np.linalg.norm(mapped[1][:2,3]-initial[:2,3]),1.)
        self.assertAlmostEqual(mapped[1][2,3],.315)
        revised={t:p.copy() for t,p in graph.items()}
        revised[2][:3,3]+=[-.8,.3,0.]
        updated=optimized_map_poses(revised,[1,2,3],initial)
        self.assertGreater(np.linalg.norm(updated[1]-mapped[1]),.8)

    def test_missing_invalid_or_unordered_optimized_poses_never_fall_back(self):
        graph={1:np.eye(4),2:np.eye(4)}
        for timestamps in ([],[1,3],[2,1],[1,1]):
            with self.assertRaises(ValueError):optimized_map_poses(graph,timestamps,np.eye(4))
        graph[2][0,0]=2
        with self.assertRaises(ValueError):optimized_map_poses(graph,[1,2],np.eye(4))

    def test_optical_axes(self):
        class Pose:
            rotation = [0, 0, 0, 1]
            translation = [0, 0, 2]
        np.testing.assert_allclose(pose_matrix(Pose)[:3, 3], [2, 0, 0])
        np.testing.assert_allclose(MAP_FROM_START[:3, :3].T @ MAP_FROM_START[:3, :3], np.eye(3))

    def test_stationary_body_with_turning_head_and_lever_arm(self):
        world_body = np.eye(4)
        world_body[:3, 3] = [1, 2, .3]
        for yaw in [-180, -40, 0, 90, 180]:
            mount = np.eye(4)
            mount[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
            mount[:3, 3] = mount[:3, :3] @ [.1, .02, .09]
            np.testing.assert_allclose(body_pose(world_body @ mount, mount), world_body, atol=1e-12)
        with self.assertRaises(ValueError):
            body_pose(world_body, np.zeros((4, 4)))

    def test_rigid_alignment_preserves_scale_error(self):
        truth = np.tile(np.eye(4), (5, 1, 1))
        truth[:, :3, 3] = [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, .3], [0, 0, .5]]
        offset = np.eye(4)
        offset[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .7]).as_matrix()
        offset[:3, 3] = [5, 2, 1]
        report, _ = metrics(offset @ truth, truth)
        self.assertLess(report['ate_se3_rmse_m'], 1e-12)
        self.assertLess(report['rpe_adjacent_translation_rmse_m'], 1e-12)
        scaled = truth.copy()
        scaled[:, :3, 3] *= 2
        report, _ = metrics(scaled, truth)
        self.assertGreater(report['ate_se3_rmse_m'], .5)


if __name__ == '__main__':
    unittest.main()
