import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
import io

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

import replay_slam_probe as replay
import evaluate_slam_map as evaluator


class ProbeTests(unittest.TestCase):
    def test_rejected_frame_cannot_change_accepted_graph(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared = Path(tmp)/'prepared'; prepared.mkdir(); (prepared/'inputs').mkdir()
            frames = []
            for i in range(3):
                with h5py.File(prepared/'inputs'/f'{i}.h5', 'w') as sample:
                    sample['rgb'] = np.zeros((2,2,3), np.uint8)
                np.savez(prepared/f'{i}.npz', depth=np.ones((2,2), np.uint16)*1000,
                         depth_source='deltar_tof_fusion')
                frames.append(dict(file=f'{i}.npz', time_s=float(i), odometry=dict(time_s=float(i)), body_from_camera=np.eye(4).tolist()))
            (prepared/'localization_input.json').write_text(json.dumps(dict(frames=frames, intrinsics=[1,1,0,0])))
            class Slam:
                def __init__(self, *args): self.tracker=self; self.index=-1; self.last_odometry_pose=np.eye(4)
                def track(self, *args): self.index+=1; return np.eye(4)
                def get_all_slam_poses(self):
                    pose=np.eye(4); pose[0,3]=99 if self.index==2 else 1
                    history=[]
                    for j in range(self.index+1):
                        value=pose.copy();value[:3,3]+=[j*.1,0,j*.05]
                        history.append(SimpleNamespace(timestamp_ns=j*10**9,pose=value))
                    return history
                def get_last_observations(self, *args): return []
                def get_slam_metrics(self): return None
            with patch.object(replay, 'RgbdSlam', Slam), patch.object(replay, 'pose_matrix', side_effect=lambda p:p), \
                    patch.object(replay, 'odom_body', return_value=np.eye(4)), \
                    patch.object(replay, 'validate_local_tracking', side_effect=[None, RuntimeError('bad local delta')]):
                result=replay.run(prepared, Path(tmp)/'out', guarded=True)
            final=json.loads((Path(tmp)/'out/live_sequence.json').read_text())
            self.assertEqual(len(final['frames']),2)
            self.assertEqual(final['frames'][0]['visual_camera_to_world'][0][3],1)
            self.assertEqual(final['frames'][0]['camera_to_world'][0][3],0)
            self.assertAlmostEqual(final['frames'][1]['camera_to_world'][0][3],.1)
            self.assertAlmostEqual(final['frames'][1]['camera_to_world'][2][3],.05)
            self.assertEqual(final['frames'][0]['body_from_camera'], np.eye(4).tolist())
            self.assertFalse(result['complete'])
            self.assertFalse(result['map_saved'])
            self.assertEqual(result['failure']['index'],2)
            with patch.object(replay, 'RgbdSlam', Slam), \
                    patch.object(Slam, 'track', side_effect=ValueError('Require nonzero measured depth')):
                empty=replay.run(prepared, Path(tmp)/'empty', guarded=True)
            self.assertEqual(empty['tracked_frames'],0)
            self.assertEqual(empty['failure']['index'],0)
            with patch.object(replay, 'run', return_value=empty), \
                    patch('sys.argv',['probe',str(prepared),str(Path(tmp)/'cli'),'--guarded']):
                self.assertEqual(replay.main(),2)



class EvaluationTests(unittest.TestCase):
    def test_first_anchor_exposes_rotation_error_hidden_by_straight_path_alignment(self):
        truth=np.repeat(np.eye(4)[None], 4, axis=0); truth[:,0,3]=np.arange(4)
        predicted=truth.copy(); predicted[-1,:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()
        result,_=evaluator.trajectory_metrics(predicted,truth,[0,1,2,3])
        self.assertAlmostEqual(result['ate_se3_rmse_m'],0)
        self.assertAlmostEqual(result['endpoint_rotation_error_deg'],30)
        self.assertGreater(result['first_pose_rotation_rmse_deg'],0)
        self.assertEqual(result['rpe_1s_pairs'],3)

    def test_empty_and_short_sequences_have_no_invented_metrics(self):
        for count in [0,1]:
            poses=np.repeat(np.eye(4)[None], count, axis=0)
            result,_=evaluator.trajectory_metrics(poses,poses,list(range(count)))
            self.assertEqual(result['matched_frames'],count)
            self.assertEqual(result['rpe_1s_pairs'],0)
            self.assertIsNone(result['rpe_1s_translation_rmse_m'])
            self.assertIsNone(result['ate_se3_rmse_m'])

    def test_gate_distributions_use_first_anchor_and_actual_one_second_pairs(self):
        truth=np.repeat(np.eye(4)[None],4,axis=0);truth[:,0,3]=[0,1,2,3]
        predicted=truth.copy();predicted[:,0,3]+=[0,.1,.2,.4]
        predicted[:,:3,:3]=Rotation.from_euler('z',np.array([0,5,10,20])[:,None],degrees=True).as_matrix()
        result,_=evaluator.trajectory_metrics(predicted,truth,[0,1,2,3])
        self.assertAlmostEqual(result['first_pose_translation_p95_m'],.37)
        self.assertAlmostEqual(result['first_pose_translation_max_m'],.4)
        self.assertAlmostEqual(result['first_pose_rotation_p95_deg'],18.5)
        self.assertAlmostEqual(result['first_pose_rotation_max_deg'],20.)
        self.assertAlmostEqual(result['path_length_ratio'],3.4/3)
        self.assertAlmostEqual(result['rpe_1s_rotation_p95_deg'],9.5)
        self.assertAlmostEqual(result['rpe_1s_rotation_max_deg'],10.)
        errors=[]
        for i in range(3):
            delta=np.linalg.inv(np.linalg.inv(truth[i])@truth[i+1])@(np.linalg.inv(predicted[i])@predicted[i+1])
            errors.append(np.linalg.norm(delta[:3,3]))
        self.assertAlmostEqual(result['rpe_1s_translation_p95_m'],np.quantile(errors,.95))
        self.assertAlmostEqual(result['rpe_1s_translation_max_m'],max(errors))
        self.assertEqual(result['input_interval_max_s'],1.)
        self.assertEqual(result['rpe_1s_interval_min_s'],1.)
        self.assertEqual(result['rpe_1s_interval_max_s'],1.)

    def test_short_stationary_sequences_have_no_path_ratio_or_rpe_distribution(self):
        for count in (0,1,2):
            poses=np.repeat(np.eye(4)[None],count,axis=0)
            result,_=evaluator.trajectory_metrics(poses,poses,np.arange(count)*.1)
            self.assertIsNone(result['path_length_ratio'])
            self.assertIsNone(result['rpe_1s_translation_p95_m'])
            self.assertIsNone(result['rpe_1s_rotation_max_deg'])
            self.assertIsNone(result['rpe_1s_interval_max_s'])
            if count:
                self.assertEqual(result['first_pose_translation_max_m'],0.)
                self.assertEqual(result['first_pose_rotation_p95_deg'],0.)
                self.assertEqual(result['ground_truth_path_m'],0.)
            else:
                self.assertIsNone(result['first_pose_translation_max_m'])
                self.assertIsNone(result['ground_truth_path_m'])
            json.dumps(result,allow_nan=False)

    def test_native_rpe_rejects_mismatches_beyond_one_frame(self):
        poses=np.repeat(np.eye(4)[None],2,axis=0)
        for horizon in [.90,.91,.92]:
            result,_=evaluator.trajectory_metrics(poses,poses,[0,horizon])
            self.assertEqual(result['rpe_1s_pairs'],0)
            self.assertEqual(result['rpe_timestamp_tolerance_s'],.07)
        for tolerance in [0.,.071,float('nan')]:
            with self.assertRaises(ValueError):evaluator.trajectory_metrics(poses,poses,[0,.93],tolerance)
        result,_=evaluator.trajectory_metrics(poses,poses,[0,.93])
        self.assertEqual(result['rpe_1s_pairs'],1)
        self.assertEqual(result['rpe_1s_interval_min_s'],.93)

    def evaluate_fixture(self,root,indices,**provenance):
        prepared=root/'prepared';candidate=root/'candidate';prepared.mkdir();candidate.mkdir()
        inputs=[dict(file=f'{i}.npz',time_s=i*.06,body_from_camera=np.eye(4).tolist()) for i in range(4)]
        (prepared/'localization_input.json').write_text(json.dumps(dict(frames=inputs)))
        (prepared/'evaluator_truth.json').write_text(json.dumps(dict(frames=[dict(time_s=f['time_s'],simulator_from_body=np.eye(4).tolist()) for f in inputs])))
        (candidate/'live_sequence.json').write_text(json.dumps(dict(frames=[dict(inputs[i],camera_to_world=np.eye(4).tolist()) for i in indices])))
        if provenance:(candidate/'source.json').write_text(json.dumps(provenance))
        with redirect_stdout(io.StringIO()):return evaluator.evaluate_prepared(candidate,prepared)

    def test_missing_interior_frames_are_reported_without_hiding_prefix_metrics(self):
        for indices,status,interior in [([0,2,3],'interior_gaps',[1]),([0,1],'accepted_prefix',[]),([],'empty',[])]:
            with tempfile.TemporaryDirectory() as tmp:
                result=self.evaluate_fixture(Path(tmp),indices)
                self.assertFalse(result['source_coverage_complete'])
                self.assertEqual(result['source_coverage_status'],status)
                self.assertEqual(result['missing_interior_source_indices'],interior)
                self.assertEqual(result['coverage_fraction'],len(indices)/4)
                self.assertEqual(result['trajectories']['cuvslam_final_optimized']['matched_frames'],len(indices))

    def test_declared_oracle_depth_is_preserved_in_evaluation_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            result=self.evaluate_fixture(Path(tmp),[0,1,2,3],ground_truth_depth_used=True)
            self.assertTrue(result['ground_truth_depth_used'])
            self.assertTrue(result['not_for_mapping'])
            self.assertEqual(result['map_quality_status'],'diagnostic/oracle control; not accepted')
            self.assertIn('Oracle depth entered tracking',result['truth_scope'])
            self.assertNotIn('no truth enters',result['truth_scope'])
            self.assertNotIn('ground truth used only by evaluator',result['trajectories']['cuvslam_final_optimized']['alignment'])
            self.assertTrue(result['source_coverage_complete'])
            self.assertAlmostEqual(result['trajectories']['cuvslam_final_optimized']['rpe_timestamp_tolerance_s'],.06)

    def test_prepared_truth_requires_exact_timestamp_match_and_uses_mount(self):
        with tempfile.TemporaryDirectory() as tmp:
            prepared=Path(tmp); mount=np.eye(4); mount[2,3]=.3
            frame=dict(time_s=1., file='0.npz', body_from_camera=mount.tolist())
            (prepared/'localization_input.json').write_text(json.dumps(dict(frames=[frame])))
            (prepared/'evaluator_truth.json').write_text(json.dumps(dict(frames=[dict(time_s=1.,simulator_from_body=np.eye(4).tolist())])))
            truth=evaluator.prepared_truth(prepared,[frame])
            self.assertAlmostEqual(truth[0,2,3],.3)
            with self.assertRaises(ValueError): evaluator.prepared_truth(prepared,[dict(frame,time_s=1.001)])


if __name__=='__main__': unittest.main()
