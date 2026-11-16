import json
import os
import tempfile
import subprocess
import sys
import time
import unittest
from pathlib import Path

import numpy as np

from run_localization import load_measurement, run_replay, live_status


def write_fixture(root):
    reference, query = root/'map', root/'query'
    reference.mkdir(); query.mkdir()
    y, x = np.mgrid[:480, :640]
    ray_y = -(x-319.5)/200
    depth = np.minimum(1.5, np.divide(-1.5, ray_y, out=np.full_like(ray_y, np.inf), where=ray_y<0))
    depth = np.rint(depth*1000).astype(np.uint16)
    mount = np.eye(4)
    mount[:3, :3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    mount[2,3] = .4
    body = np.eye(4); body[:3,3] = [.5,.5,.185]
    intrinsics = [200,200,319.5,239.5]
    np.savez(reference/'depth.npz', depth=depth, depth_source='deltar_tof_fusion')
    (reference/'live_sequence.json').write_text(json.dumps(dict(intrinsics=intrinsics,
        frames=[dict(file='depth.npz',time_s=0,camera_to_world=(body@mount).tolist())])))
    frames=[]
    for i in range(6):
        filename=f'{i}.npz';t=i*.1
        packet=dict(time_s=t,x=.52,y=.5,yaw=0,pitch=0,roll=0,axle_body_x=.02,
            axle_body_z=-.15,wheel_radius=.035,valid=1,slip=False)
        np.savez(query/filename,depth=depth,depth_source='deltar_tof_fusion')
        frames.append(dict(file=filename,time_s=t,odometry=packet,body_from_camera=mount.tolist(),
                           camera_to_world=(np.eye(4)*999).tolist()))
    (query/'localization_input.json').write_text(json.dumps(dict(intrinsics=intrinsics,frames=frames)))
    return reference,query


class RunnerTests(unittest.TestCase):
    def test_replay_recovers_pose_and_keeps_inputs_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ref,query=write_fixture(root)
            before={p:p.read_bytes() for d in (ref,query) for p in d.iterdir()}
            result=run_replay(ref,query,root/'result',(.53,.48,np.deg2rad(1)))
            records=[json.loads(line) for line in (root/'result/poses.jsonl').read_text().splitlines()]
            self.assertEqual(result['frames'],6)
            self.assertTrue(records[-1]['usable'])
            self.assertEqual(records[-1]['state'],'tracking')
            self.assertLess(np.linalg.norm(np.asarray(records[-1]['world_from_body'])[:2,3]-[.5,.5]),.025)
            self.assertTrue((root/'result/run.json').is_file())
            self.assertEqual(json.loads((root/'result/status.json').read_text())['state'],'stopped')
            for p,data in before.items():self.assertEqual(p.read_bytes(),data)
            with self.assertRaises(FileExistsError):run_replay(ref,query,root/'result',(.5,.5,0))

    def test_rejects_depth_without_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'depth.npz'
            np.savez(path,depth=np.ones((32,32),np.uint16))
            with self.assertRaises(ValueError):load_measurement(path)
            np.savez(path,depth=np.ones((32,32),np.uint16),depth_source='deltar_tof_fusion',floor_plane=[0,0,1,0])
            with self.assertRaises(ValueError):load_measurement(path)

    def test_no_output_is_written_inside_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ref,query=write_fixture(root)
            with self.assertRaises(ValueError):run_replay(ref,query,ref/'result',(.5,.5,0))
            self.assertFalse((ref/'result').exists())

    def test_failed_replay_marks_latest_pose_unusable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ref,query=write_fixture(root)
            (query/'3.npz').unlink()
            with self.assertRaises(FileNotFoundError):run_replay(ref,query,root/'result',(.5,.5,0))
            status=json.loads((root/'result/status.json').read_text())
            self.assertFalse(status['usable'])
            self.assertEqual(status['state'],'lost')
            self.assertIn('error',status)

    def test_live_process_tracks_then_marks_frozen_input_lost(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);ref,query=write_fixture(root);out=root/'live-result'
            manifest=json.loads((query/'localization_input.json').read_text())
            with np.load(query/'0.npz') as data:depth=data['depth'].copy()
            child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('run_localization.py')),
                '--map',str(ref),'--live',str(query),'--output',str(out),'--initial-pose','.5','.5','0'],
                stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
            def wait_status(predicate):
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    if child.poll() is not None:self.fail(child.stderr.read())
                    try:status=json.loads((out/'status.json').read_text())
                    except FileNotFoundError:status={}
                    if predicate(status):return status
                    time.sleep(.03)
                self.fail('Live status did not reach expected state: '+str(status))
            try:
                wait_status(lambda s:s.get('state')=='initializing')
                for i in range(2):
                    frame=manifest['frames'][i]
                    with (query/'buffer.tmp').open('wb') as stream:
                        np.savez(stream,depth=depth,depth_source='deltar_tof_fusion',t=frame['time_s'],
                            odometry=json.dumps(frame['odometry']),mount=frame['body_from_camera'],K=manifest['intrinsics'])
                    if i == 1:
                        old=time.time()-1.5;os.utime(query/'buffer.tmp',(old,old))
                    (query/'buffer.tmp').replace(query/'rgbd.npz')
                    wait_status(lambda s:s.get('timestamp_ns')==i*100_000_000)
                tracked=wait_status(lambda s:s.get('state')=='tracking')
                self.assertTrue(tracked['usable'])
                self.assertGreaterEqual(tracked['input_age_s'],1.4)
                stale=wait_status(lambda s:s.get('reason')=='stale_rgbd')
                self.assertFalse(stale['usable'])
                (query/'STOP').touch();child.wait(timeout=5)
                self.assertEqual(child.returncode,0)
                stopped=json.loads((out/'status.json').read_text())
                self.assertFalse(stopped['usable'])
                self.assertEqual(stopped['state'],'stopped')
            finally:
                if child.poll() is None:child.terminate();child.wait(timeout=5)
                child.stderr.close()

    def test_stale_and_stopped_buffers_never_publish_usable_pose(self):
        record=dict(state='tracking',usable=True,world_from_body=np.eye(4).tolist())
        recent=live_status(record,10,10.5)
        stale=live_status(record,10,12.1)
        stopped=live_status(record,10,10.5,stopped=True)
        self.assertTrue(recent['usable'])
        self.assertFalse(stale['usable']);self.assertEqual(stale['state'],'lost')
        self.assertFalse(stopped['usable']);self.assertEqual(stopped['state'],'stopped')
        self.assertTrue(record['usable'])
        self.assertFalse(live_status(None,10,10.5)['usable'])


if __name__=='__main__':unittest.main()
