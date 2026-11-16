import unittest
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from ground_segmentation import GroundSegmenter, GROUND, OBSTACLE, UNCERTAIN, read_labels


def scene(rotation=None, height=.3):
    K = np.array([150., 150., 79.5, 59.5])
    pose = np.eye(4)
    pose[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    if rotation is not None:
        pose[:3, :3] = rotation @ pose[:3, :3]
    pose[:3, 3] = [.2, -.1, height]
    y, x = np.indices((120, 160))
    rays = np.stack(((x-K[2])/K[0], (y-K[3])/K[1], np.ones_like(x)), -1)
    down = -(rays @ pose[:3, :3].T)[..., 2]
    depth = np.divide(height, down, out=np.zeros_like(down), where=down>.06)
    depth[depth>4] = 0
    return depth.astype(np.float32), K, pose, down


def read_ply(path):
    with Path(path).open('rb') as stream:
        header=[]
        while True:
            line=stream.readline()
            if not line:raise AssertionError('Missing PLY header')
            header.append(line)
            if line==b'end_header\n':break
        count=int(next(line for line in header if line.startswith(b'element vertex ')).split()[-1])
        points=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,3)
        assert len(points)==count
        return points


class GroundTests(unittest.TestCase):
    def test_export_keeps_all_classes_in_same_voxel_even_off_old_sampling_grid(self):
        from segment_scan import run
        with tempfile.TemporaryDirectory() as directory:
            source,output=Path(directory)/'source',Path(directory)/'out';source.mkdir()
            depth=np.zeros((8,8),np.uint16);depth[1,1:4]=1000
            np.savez(source/'frame.npz',depth=depth,depth_source='deltar_tof_fusion')
            pose=np.eye(4)
            sequence=dict(intrinsics=[1000,1000,0,0],frames=[dict(file='frame.npz',time_s=0.,camera_to_world=pose.tolist())])
            (source/'live_sequence.json').write_text(json.dumps(sequence))
            labels=np.zeros(depth.shape,np.uint8);labels[1,1:4]=[GROUND,OBSTACLE,UNCERTAIN]
            classification=dict(ground_labels=labels,ground_plane_camera=np.array([0.,0.,1.,0.]),
                                ground_confirmations=3,ground_source='gravity_ransac_labels_v3')
            with patch('segment_scan.GroundSegmenter.classify',return_value=classification):report=run(source,output)
            self.assertEqual(report['valid_depth_pixel_counts'],dict(uncertain=1,ground=1,obstacle=1))
            self.assertEqual(report['invalid_depth_pixels'],61)
            self.assertEqual(report['map_quality_status'],'diagnostic/unvalidated')
            self.assertEqual(report['source_pose_sha256'],report['output_pose_sha256'])
            for name,x in [('ground',1),('obstacle',2),('uncertain',3)]:
                filename='obstacles.ply' if name=='obstacle' else name+'.ply'
                points=read_ply(output/filename)
                np.testing.assert_allclose(points,[[x/1000,.001,1.]])
                self.assertEqual(report['layer_points'][name],len(points))
            self.assertEqual(report['classification_pose_source'],'saved estimated Z-up poses; legacy fallback')

    def test_declared_oracle_or_tracking_only_poses_cannot_export_measured_geometry(self):
        from segment_scan import run
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'source';source.mkdir()
            np.savez(source/'frame.npz',depth=np.full((8,8),1000,np.uint16),depth_source='deltar_tof_fusion')
            sequence=dict(intrinsics=[100,100,3.5,3.5],frames=[dict(file='frame.npz',time_s=0.,camera_to_world=np.eye(4).tolist())])
            for index,(filename,flag) in enumerate([('live_sequence.json','ground_truth_depth_used'),
                    ('source.json','ground_truth_pose_used_by_tracker'),('tracking.json','not_for_mapping')]):
                for name in ('source.json','tracking.json'):
                    if (source/name).exists():(source/name).unlink()
                (source/'live_sequence.json').write_text(json.dumps(sequence))
                metadata=dict(sequence) if filename=='live_sequence.json' else {}
                metadata[flag]=True
                (source/filename).write_text(json.dumps(metadata))
                output=Path(directory)/f'forbidden-{index}'
                with self.assertRaisesRegex(ValueError,'oracle|tracking-only'):
                    run(source,output)
                self.assertFalse(output.exists())

    def test_empty_layers_have_readable_zero_vertex_files(self):
        from segment_scan import run
        with tempfile.TemporaryDirectory() as directory:
            source,output=Path(directory)/'source',Path(directory)/'out';source.mkdir()
            np.savez(source/'frame.npz',depth=np.zeros((8,8),np.uint16),depth_source='deltar_tof_fusion')
            (source/'live_sequence.json').write_text(json.dumps(dict(intrinsics=[100,100,3.5,3.5],
                frames=[dict(file='frame.npz',time_s=0.,camera_to_world=np.eye(4).tolist())])))
            report=run(source,output)
            for name in ['ground.ply','obstacles.ply','uncertain.ply']:
                self.assertEqual(read_ply(output/name).shape,(0,3))
            self.assertEqual(report['invalid_depth_pixels'],64)
            self.assertEqual(sum(report['valid_depth_pixel_counts'].values()),0)

    def test_physical_ground_labels_follow_odometry_mount_while_export_follows_final_slam(self):
        from segment_scan import run
        depth,K,physical,_=scene();measured=np.rint(depth*1000).astype(np.uint16)
        packet=dict(time_s=0.,x=0.,y=0.,yaw=0.,pitch=0.,roll=0.,axle_body_x=0.,axle_body_z=-.1,
                    wheel_radius=.035,valid=1)
        body=np.eye(4);body[2,3]=.135;mount=np.linalg.inv(body)@physical
        mapped=physical.copy();mapped[2,3]+=1.
        with tempfile.TemporaryDirectory() as directory:
            source,output=Path(directory)/'source',Path(directory)/'out';source.mkdir()
            prepared=Path(directory)/'prepared';prepared.mkdir()
            frames=[]
            for i in range(3):
                np.savez(prepared/f'{i}.npz',depth=measured,depth_source='deltar_tof_fusion')
                frames.append(dict(file=f'{i}.npz',time_s=i*.2,camera_to_world=mapped.tolist(),
                                   odometry=dict(packet,time_s=i*.2),body_from_camera=mount.tolist()))
            (source/'live_sequence.json').write_text(json.dumps(dict(frames=frames,intrinsics=K.tolist(),prepared_source=str(prepared))))
            report=run(source,output)
            self.assertEqual(report['classification_pose_source'],'synchronized wheel/IMU + body_from_camera')
            self.assertGreater(report['layer_points']['ground'],0)
            np.testing.assert_allclose(read_ply(output/'ground.ply')[:,2],1.,atol=.012)
            with np.load(output/'keyframes/00002.npz') as sample:
                self.assertGreater((sample['ground_labels']==GROUND).sum(),1000)
                np.testing.assert_array_equal(sample['depth'],measured)
            broken=dict(frames[0]);broken.pop('body_from_camera')
            (source/'live_sequence.json').write_text(json.dumps(dict(frames=[broken],intrinsics=K.tolist())))
            with self.assertRaisesRegex(ValueError,'physical metadata'):
                run(source,Path(directory)/'bad')


    def test_confirmation_preserves_depth_holes_and_low_obstacles(self):
        depth, K, pose, down = scene()
        # A 3cm high horizontal object must survive, including its center.
        depth[90:110, 65:95] = (.3-.03)/down[90:110, 65:95]
        depth[90:110, 105:135] = (.3-.02)/down[90:110, 105:135]
        depth[105:115, 20:30] = 0
        original = depth.copy()
        segmenter = GroundSegmenter()
        for i in range(3):
            result = segmenter.classify(depth, K, pose, i*.2)
            if i<2:
                self.assertFalse(np.any(result['ground_labels']==GROUND))
        labels = result['ground_labels']
        np.testing.assert_array_equal(depth, original)
        self.assertGreater(np.count_nonzero(labels==GROUND), 1000)
        self.assertTrue(np.all(labels[94:106, 69:91]==OBSTACLE))
        self.assertTrue(np.all(labels[105:115, 20:30]==UNCERTAIN))
        self.assertTrue(np.all(labels[94:106,109:131]==UNCERTAIN))

    def test_roll_pitch_head_and_height_do_not_tilt_ground(self):
        segmenter = GroundSegmenter()
        for i in range(5):
            rotation = Rotation.from_euler('zyx', [i*.12, .06-i*.02, -.04+i*.015]).as_matrix()
            depth, K, pose, _ = scene(rotation, .25+i*.015)
            result = segmenter.classify(depth, K, pose, i*.2)
        self.assertGreater(np.count_nonzero(result['ground_labels']==GROUND), 1000)
        plane = result['ground_plane_camera']
        np.testing.assert_allclose(pose[:3, :3]@plane[:3], [0, 0, 1], atol=1e-5)
        self.assertAlmostEqual(plane[3]-pose[2, 3], 0., places=5)

    def test_wall_missing_floor_gap_and_bad_data_cannot_clear(self):
        depth, K, pose, _ = scene()
        segmenter = GroundSegmenter()
        for i in range(3):
            segmenter.classify(depth, K, pose, i*.2)
        for i, absent in enumerate([np.zeros_like(depth), np.ones_like(depth)]):
            result = segmenter.classify(absent, K, pose, 1+i*.2)
            self.assertFalse(np.any(result['ground_labels']==GROUND))
        # A gap invalidates temporal confirmation, as does failed fitting.
        result = segmenter.classify(depth, K, pose, 10.)
        self.assertFalse(np.any(result['ground_labels']==GROUND))
        with self.assertRaises(ValueError):
            segmenter.classify(depth, K, np.zeros((4, 4)), 10.2)
        with self.assertRaises(ValueError):
            segmenter.classify(depth, [0, 150, 80, 60], pose, 10.2)

    def test_raised_plane_without_floor_is_not_accepted_as_ground(self):
        for raised in (.02, .06):
            depth, K, pose, down = scene()
            valid = depth>0
            depth[valid] = (.3-raised)/down[valid]
            segmenter = GroundSegmenter()
            for i in range(4):
                result = segmenter.classify(depth, K, pose, i*.2)
            self.assertFalse(np.any(result['ground_labels']==GROUND), raised)

    def test_large_low_object_cannot_tilt_fitted_floor_and_be_erased(self):
        depth, K, pose, down = scene()
        depth[72:118, 40:120] = .27/down[72:118, 40:120]
        segmenter = GroundSegmenter()
        for i in range(3):
            result = segmenter.classify(depth, K, pose, i*.2)
        labels = result['ground_labels'][74:116, 42:118]
        self.assertFalse(np.any(labels==GROUND))
        self.assertTrue(np.all(labels==OBSTACLE))

    def test_label_provenance_rejects_partial_and_invalid_data(self):
        depth, K, pose, _ = scene()
        segmenter = GroundSegmenter()
        for i in range(3):
            result = segmenter.classify(depth, K, pose, i*.2)
        self.assertIsNone(read_labels({}, depth.shape))
        for bad in [dict(ground_labels=result['ground_labels']),
                    dict(result, ground_source='old'),
                    dict(result, ground_labels=np.full(depth.shape, 8, np.uint8)),
                    dict(result, ground_plane_camera=np.full(4, np.nan)),
                    dict(result, ground_confirmations=1)]:
            with self.assertRaises(ValueError):
                read_labels(bad, depth.shape)

    def test_replay_exports_layers_preserves_bytes_and_never_overwrites(self):
        from segment_scan import run
        depth, K, pose, _ = scene()
        measured = np.rint(depth*1000).astype(np.uint16)
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'source', Path(directory)/'classified'
            source.mkdir()
            frames = []
            for i in range(4):
                np.savez(source/f'{i}.npz', depth=measured, depth_source='deltar_tof_fusion')
                frames.append(dict(file=f'{i}.npz', time_s=i*.2, camera_to_world=pose.tolist()))
            (source/'live_sequence.json').write_text(json.dumps(dict(frames=frames, intrinsics=K.tolist())))
            report = run(source, output)
            self.assertEqual(report['source_depth_sha256'], report['output_depth_sha256'])
            self.assertGreater(report['layer_points']['ground'], 0)
            for i in range(4):
                with np.load(output/f'keyframes/{i:05d}.npz') as sample:
                    np.testing.assert_array_equal(sample['depth'], measured)
                    read_labels(sample, measured.shape)
            with self.assertRaises(FileExistsError):
                run(source, output)
            with self.assertRaises(ValueError):
                run(source, source/'output')


if __name__=='__main__':
    unittest.main()
