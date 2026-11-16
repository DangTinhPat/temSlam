import json,tempfile,unittest,time,subprocess
from pathlib import Path
import numpy as np
from workers import write_json,write_npz,keep_keyframe,fusion_depth,FUSION_DEPTH_SOURCE,finalize,cloud_voxel_indices
from session import LiveSession
from types import SimpleNamespace
from unittest.mock import patch,Mock

class IpcTests(unittest.TestCase):
    def test_shutdown_waits_for_map_export_and_records_timeout(self):
        for timeout in (False,True):
            with self.subTest(timeout=timeout),tempfile.TemporaryDirectory() as tmp:
                live=LiveSession.__new__(LiveSession);live.session=Path(tmp);live.logs=[]
                child=Mock(pid=999999,returncode=0)
                child.wait.side_effect=[subprocess.TimeoutExpired('slam',60),0] if timeout else [0,0]
                child.poll.return_value=None if timeout else 0
                live.children=[('slam',child)]
                with patch('session.os.killpg'):
                    live.close()
                self.assertGreaterEqual(child.wait.call_args_list[0].kwargs['timeout'],30)
                result=json.loads((live.session/'slam_shutdown.json').read_text())
                self.assertEqual(result['clean_stop'],not timeout)

    def test_live_mesh_rebuilds_when_loop_updates_poses_without_new_keyframes(self):
        import workers
        with tempfile.TemporaryDirectory() as tmp:
            session=Path(tmp)
            sequence=dict(frames=[dict(camera_to_world=np.eye(4).tolist()) for _ in range(3)])
            write_json(session/'live_sequence.json',sequence)
            exports=[]
            def export(path):
                exports.append(path);Path(path).touch()
                sequence['frames'][1]['camera_to_world'][0][3]=1.2
                write_json(session/'live_sequence.json',sequence)
            ticks=[]
            def sleep(seconds):
                ticks.append(seconds)
                if len(ticks)==3:(session/'STOP').touch()
            surface=SimpleNamespace(export=export,faces=[0])
            fake_torch=SimpleNamespace(set_num_threads=lambda n:None,cuda=SimpleNamespace(synchronize=lambda:None))
            with patch.dict('sys.modules',{'torch':fake_torch,'reconstruct':SimpleNamespace(TSDF=lambda *a,**k:SimpleNamespace(integrate=lambda *a:None))}), \
                    patch('workers.time.sleep',side_effect=sleep), \
                    patch('workers.time.monotonic',side_effect=[0,10,20,30,40,50,60]), \
                    patch('workers.build_surface',return_value=(surface,.04)) as build:
                workers.mesh(session)
            self.assertEqual(build.call_count,2)
            self.assertEqual(len(set(exports)),2,'RViz needs a new mesh URI for corrected poses')

    def test_finalize_does_not_override_globally_optimized_slam_poses(self):
        with tempfile.TemporaryDirectory() as tmp:
            session=Path(tmp)
            sequence=dict(frames=[dict(camera_to_world=np.eye(4).tolist())],
                          pose_source='cuVSLAM final optimized poses; single initial wheel/IMU alignment',
                          bounds_m=[[0,0,0],[1,1,1]])
            write_json(session/'live_sequence.json',sequence)
            mesh=SimpleNamespace(export=lambda p:Path(p).write_text('sensor mesh'),
                                 bounds=np.array([[0,0,0],[1,1,1]]),faces=[0])
            refine=Mock(side_effect=AssertionError('Sequential refinement overwrites global closure'))
            with patch.dict('sys.modules',{'torch':SimpleNamespace(set_num_threads=lambda n:None),
                    'surface_matching':SimpleNamespace(refine_sequence=refine)}), \
                    patch('workers.build_surface',return_value=(mesh,.04)) as build:
                finalize(session)
            build.assert_called_once_with(session,sequence)
            refine.assert_not_called()

    def test_fast_cloud_voxel_keys_preserve_the_same_points_and_order(self):
        rng=np.random.default_rng(3)
        cloud=rng.uniform(-4,4,(10000,3)).astype(np.float32)
        cloud=np.concatenate((cloud,cloud[:1000],np.zeros((2,3),np.float32)))
        _,expected=np.unique(np.floor(cloud/.03).astype(np.int32),axis=0,return_index=True)
        np.testing.assert_array_equal(cloud_voxel_indices(cloud),expected)
        self.assertEqual(len(cloud_voxel_indices(np.empty((0,3)))),0)

    def test_voxel_downsampling_keeps_obstacle_and_ground_in_same_voxel(self):
        cloud=np.array([[0.,0.,0.],[.001,.001,.001],[.002,.002,.002]])
        selected=cloud_voxel_indices(cloud,np.array([1,2,1],np.uint8))
        self.assertEqual(set(selected),{0,1})

    def test_scan_feedback_requires_recent_processed_head_angle(self):
        with tempfile.TemporaryDirectory() as tmp:
            live=LiveSession.__new__(LiveSession);live.session=Path(tmp)
            self.assertIsNone(live.scan_feedback())
            state=dict(wall_time=time.time(),sensor_time=10,head_yaw_rad=.5)
            write_json(live.session/'slam_status.json',state)
            self.assertEqual(live.scan_feedback()['head_yaw_rad'],.5)
            state['wall_time']-=6;write_json(live.session/'slam_status.json',state)
            self.assertIsNone(live.scan_feedback())

    def test_map_rejects_legacy_or_plane_modified_depth(self):
        depth=np.array([[0,1000],[1250,0]],np.uint16)
        sample=dict(depth=depth,depth_source=FUSION_DEPTH_SOURCE)
        np.testing.assert_array_equal(fusion_depth(sample),depth/1000)
        with self.assertRaises(ValueError):fusion_depth(dict(depth=depth))
        with self.assertRaises(ValueError):fusion_depth(dict(sample,floor_plane=np.array([0,0,1,0])))

    def test_finalize_publishes_sensor_mesh_without_floor_completion(self):
        with tempfile.TemporaryDirectory() as tmp:
            session=Path(tmp);sequence=dict(frames=[],pose_source='estimated',bounds_m=[[0,0,0],[1,1,1]])
            write_json(session/'live_sequence.json',sequence)
            mesh=SimpleNamespace(export=lambda p:Path(p).write_text('sensor mesh'),
                bounds=np.array([[0,0,0],[1,1,1]]),faces=[0])
            with patch.dict('sys.modules',{'torch':SimpleNamespace(set_num_threads=lambda n:None),
                    'surface_matching':SimpleNamespace(refine_sequence=lambda *a:(sequence,{}))}), \
                    patch('workers.build_surface',return_value=(mesh,.04)):
                finalize(session)
            result=json.loads((session/'mesh.json').read_text())
            self.assertEqual(result['file'],str(session/'final_mesh.stl'))
            self.assertEqual(result['depth_source'],FUSION_DEPTH_SOURCE)
            self.assertFalse((session/'floor_completion').exists())

    def test_idle_does_not_exhaust_keyframes_but_motion_is_kept(self):
        pose=np.eye(4);frames=[]
        for _ in range(1000):
            if keep_keyframe(pose,frames):frames.append({'camera_to_world':pose.tolist()})
        self.assertEqual(len(frames),3)
        moved=pose.copy();moved[0,3]=.06
        self.assertTrue(keep_keyframe(moved,frames))
        rotated=pose.copy();angle=.1
        rotated[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        self.assertTrue(keep_keyframe(rotated,frames))

    def test_failed_worker_is_reported_to_route(self):
        with tempfile.TemporaryDirectory() as tmp:
            live=LiveSession.__new__(LiveSession);live.session=Path(tmp)
            child=SimpleNamespace(poll=lambda:None,returncode=None)
            live.children=[('slam',child)]
            self.assertIsNone(live.failure_reason())
            write_json(live.session/'slam_status.json',{'message':'LỖI: mất tracking'})
            self.assertIn('mất tracking',live.failure_reason())
            (live.session/'slam_status.json').unlink()
            child.poll=lambda:1;child.returncode=1
            self.assertIn('đã dừng',live.failure_reason())

    def test_point_cloud_path_field_and_atomic_replacement(self):
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'cloud.npz'
            for n in (1,3):
                write_npz(target,xyz=np.zeros((n,3),np.float32),path=np.ones((n,3)))
                with np.load(target,allow_pickle=False) as f:
                    self.assertEqual(f['xyz'].shape,(n,3));self.assertEqual(f['path'].shape,(n,3))
            self.assertFalse(target.with_suffix('.tmp').exists())
            status=Path(tmp)/'status.json';write_json(status,{'message':'Đang quét'})
            self.assertEqual(json.loads(status.read_text())['message'],'Đang quét')

if __name__=='__main__': unittest.main()
