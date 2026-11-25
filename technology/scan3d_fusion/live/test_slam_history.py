import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from workers import tracking,write_npz,FUSION_DEPTH_SOURCE,map_refresh_due,graph_update_kind


class HistoryTests(unittest.TestCase):
    def test_only_pose_graph_optimization_enables_global_corrections(self):
        self.assertIsNone(graph_update_kind(None))
        self.assertEqual(graph_update_kind(SimpleNamespace(lc_status=True,pgo_status=False)),'loop_closure')
        self.assertEqual(graph_update_kind(SimpleNamespace(lc_status=True,pgo_status=True)),'pgo')
        self.assertEqual(graph_update_kind(SimpleNamespace(lc_status=False,pgo_status=False)),None)

    def test_map_refresh_batches_keyframes_but_keeps_graph_updates_immediate(self):
        self.assertTrue(map_refresh_due(1,0,False))
        self.assertFalse(map_refresh_due(25,1,False))
        self.assertTrue(map_refresh_due(26,1,False))
        self.assertTrue(map_refresh_due(2,1,True))

    def test_hybrid_empty_depth_is_skipped_before_entering_sdk(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)
            packet=dict(time_s=1.,x=0,y=0,yaw=0,pitch=0,roll=0,axle_body_x=0,
                axle_body_z=-.1,wheel_radius=.035,valid=1,slip=False,head_yaw=0.)
            write_npz(session/'rgbd.npz',rgb=np.zeros((480,640,3),np.uint8),
                depth=np.zeros((480,640),np.uint16),depth_source=FUSION_DEPTH_SOURCE,
                t=1.,K=[580,580,319.5,239.5],mount=np.eye(4),odometry=json.dumps(packet))
            sdk=Mock(side_effect=AssertionError('Empty depth must not enter SDK'))
            def on_status(*a,**kw):(session/'STOP').touch()
            with patch('track_rgbd.RgbdSlam',sdk),patch('worker_tracking.status',side_effect=on_status):
                tracking(session,pose_policy='hybrid')
            sdk.assert_not_called()
            self.assertFalse((session/'live_sequence.json').exists())
            record=json.loads((session/'tracking_diagnostics.jsonl').read_text())
            self.assertEqual(record['reason'],'no_supported_depth')

    def test_final_non_keyframe_updates_retained_historical_poses(self):
        self.check_history(False)

    def test_rejected_non_keyframe_keeps_previous_export_and_does_not_save_database(self):
        self.check_history(True)

    def check_history(self,reject_last):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)
            batch=dict(rgb=np.zeros((480,640,3),np.uint8),depth=np.full((480,640),1000,np.uint16),
                       depth_source=FUSION_DEPTH_SOURCE,K=[580,580,319.5,239.5],mount=np.eye(4),
                       tof_hist=np.zeros((64,2)),tof_rect=np.zeros((64,4)),tof_valid=np.ones(64,bool))
            def sample(t):
                write_npz(session/'rgbd.npz',**batch,t=t,odometry=json.dumps(dict(time_s=t,head_yaw=0.)))
            sample(1.)
            saved=[]
            class FakeSlam:
                def __init__(self,*args):self.index=0;self.last_odometry_pose=np.eye(4);self.tracker=self
                def track(self,timestamp,rgb,depth):
                    self.index+=1
                    if self.index<4:sample(float(self.index+1))
                    else:(session/'STOP').touch()
                    pose=np.eye(4);pose[0,3]=min(.2*(self.index-1),.4)
                    return pose
                def get_all_slam_poses(self):
                    out=[]
                    for i in range(1,self.index+1):
                        pose=np.eye(4)
                        if i==2:pose[0,3]=1.2 if self.index==4 else .2
                        out.append(SimpleNamespace(timestamp_ns=i*1_000_000_000,pose=pose))
                    return out
                def get_slam_metrics(self):return SimpleNamespace(lc_status=False,pgo_status=False)
                def save_map(self,path,callback):saved.append(path);callback(True)
            fake=FakeSlam()
            def validate(*args):
                if reject_last and fake.index==4:raise RuntimeError('Bad local tracking')
            adapter=SimpleNamespace(RgbdSlam=FakeSlam,pose_matrix=lambda p:p,
                optimized_map_poses=lambda poses,timestamps,initial:[poses[t] for t in timestamps],
                SLAM_MAP_SOURCE='optimized graph',validate_local_tracking=validate)
            adapter.RgbdSlam=lambda *args:fake
            odometry=SimpleNamespace(odom_body=lambda packet:np.eye(4),rigid=lambda matrix:matrix)
            with patch.dict('sys.modules',{'track_rgbd':adapter,'localization':odometry}), \
                patch('worker_tracking.startup_tracking_state',side_effect=lambda previous,still_since,t,pose,max_gap:((t,pose.copy()),t,True)):
                if reject_last:
                    with self.assertRaisesRegex(RuntimeError,'Bad local tracking'):tracking(session,pose_policy='cuvslam')
                else:tracking(session,pose_policy='cuvslam')
            sequence=json.loads((session/'live_sequence.json').read_text())
            self.assertEqual(len(sequence['frames']),1 if reject_last else 3)
            if not reject_last:
                self.assertAlmostEqual(sequence['frames'][1]['camera_to_world'][0][3],1.2)
            self.assertEqual(bool(saved),not reject_last)

    def test_hybrid_rejects_visual_jump_and_keeps_synchronized_odometry(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)
            batch=dict(rgb=np.zeros((480,640,3),np.uint8),depth=np.full((480,640),1000,np.uint16),
                depth_source=FUSION_DEPTH_SOURCE,K=[580,580,319.5,239.5],mount=np.eye(4),
                tof_hist=np.zeros((64,2)),tof_rect=np.zeros((64,4)),tof_valid=np.ones(64,bool))
            def sample(i):
                packet=dict(time_s=float(i),x=(i-1)*.1,y=0,yaw=0,pitch=0,roll=0,
                    axle_body_x=0,axle_body_z=-.1,wheel_radius=.035,valid=1,slip=False,head_yaw=0.)
                write_npz(session/'rgbd.npz',**batch,t=float(i),odometry=json.dumps(packet))
            sample(1)
            saved=[]
            reads={'history':0,'metrics':0}
            class FakeSlam:
                def __init__(self,*args):self.index=0;self.last_odometry_pose=np.eye(4);self.tracker=self
                def track(self,timestamp,rgb,depth):
                    self.index+=1
                    self.last_odometry_pose=np.eye(4);self.last_odometry_pose[0,3]=(self.index-1)*.1
                    if self.index>=3:self.last_odometry_pose[0,3]+=4.54
                    if self.index<4:sample(self.index+1)
                    else:(session/'STOP').touch()
                    pose=np.eye(4);pose[0,3]=100. if self.index>=3 else (self.index-1)*.1
                    return pose
                def get_all_slam_poses(self):
                    reads['history']+=1
                    out=[]
                    for i in range(1,self.index+1):
                        pose=np.eye(4);pose[0,3]=(i-1)*.3
                        if i>=3:pose[0,3]+=4.54
                        out.append(SimpleNamespace(timestamp_ns=i*1_000_000_000,pose=pose))
                    return out
                def get_slam_metrics(self):
                    reads['metrics']+=1
                    return SimpleNamespace(lc_status=self.index>=2,pgo_status=self.index>=2,
                                           timestamp_ns=self.index*1_000_000_000)
                def save_map(self,path,callback):saved.append(path);callback(True)
            with patch('track_rgbd.RgbdSlam',FakeSlam),patch('track_rgbd.pose_matrix',lambda p:p), \
                patch('worker_tracking.startup_tracking_state',side_effect=lambda previous,still_since,t,pose,max_gap:((t,pose.copy()),t,True)):
                tracking(session,pose_policy='hybrid')
            sequence=json.loads((session/'live_sequence.json').read_text())
            self.assertEqual(len(sequence['frames']),4)
            for i,frame in enumerate(sequence['frames']):
                np.testing.assert_allclose(np.asarray(frame['camera_to_world'])[:3,3],[(i)*.1,0,.135])
            self.assertTrue(sequence['frames'][0]['visual_accepted'])
            self.assertTrue(sequence['frames'][1]['visual_accepted'])
            self.assertFalse(sequence['frames'][2]['visual_accepted'])
            self.assertFalse(sequence['frames'][3]['visual_accepted'])
            self.assertEqual([f['local_visual_accepted'] for f in sequence['frames']],
                             [True,True,False,True])
            self.assertIn('wheel/IMU',sequence['pose_source'])
            self.assertFalse(saved,'Do not publish an SDK database after a rejected visual jump')
            self.assertTrue((session/'tracking_failure.json').exists())
            self.assertEqual(reads,{'history':0,'metrics':0})
            quality=json.loads((session/'pose_quality.json').read_text())
            self.assertEqual(quality['accepted_visual_keyframes'],2)
            self.assertEqual(quality['pgo_events'],0)
            reasons=[json.loads(line)['reason'] for line in (session/'map_refresh_diagnostics.jsonl').read_text().splitlines()]
            self.assertNotIn('pgo',reasons)


if __name__=='__main__':unittest.main()
