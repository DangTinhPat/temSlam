import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from workers import tracking,write_npz,FUSION_DEPTH_SOURCE


class HistoryTests(unittest.TestCase):
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
                    return np.eye(4)
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
            with patch.dict('sys.modules',{'track_rgbd':adapter,'localization':odometry}):
                if reject_last:
                    with self.assertRaisesRegex(RuntimeError,'Bad local tracking'):tracking(session)
                else:tracking(session)
            sequence=json.loads((session/'live_sequence.json').read_text())
            self.assertEqual(len(sequence['frames']),3)
            self.assertAlmostEqual(sequence['frames'][1]['camera_to_world'][0][3],.2 if reject_last else 1.2)
            self.assertEqual(bool(saved),not reject_last)


if __name__=='__main__':unittest.main()
