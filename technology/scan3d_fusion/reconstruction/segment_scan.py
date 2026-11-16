"""Replay an estimated-pose scan into a separate, classified navigation session."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live.workers import fusion_depth, cloud_voxel_indices
from mapping.odometry.localization import rigid, odom_body
from ground_segmentation import GroundSegmenter, SOURCE


def merge_layers(layers, points, labels):
    for name, value in [('uncertain', 0), ('ground', 1), ('obstacle', 2)]:
        combined = np.concatenate((layers[name], points[labels==value])).astype(np.float32)
        layers[name] = combined[cloud_voxel_indices(combined)]


def write_ply(path, points):
    points=np.asarray(points,dtype='<f4')
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('Require finite Nx3 points')
    header=('ply\nformat binary_little_endian 1.0\n'+f'element vertex {len(points)}\n'
            'property float x\nproperty float y\nproperty float z\nend_header\n')
    with Path(path).open('xb') as stream:
        stream.write(header.encode('ascii'));stream.write(points.tobytes())


def run(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output==source or source in output.parents:
        raise ValueError('Output must be new and outside the source scan')
    sequence_path = source/'surface_sequence.json'
    if not sequence_path.exists():
        sequence_path = source/'live_sequence.json'
    sequence = json.loads(sequence_path.read_text())
    provenance=[sequence]
    for name in ('source.json','tracking.json'):
        if (source/name).exists():provenance.append(json.loads((source/name).read_text()))
    if any(record.get(key) is True for record in provenance
           for key in ('ground_truth_depth_used','ground_truth_pose_used_by_tracker','not_for_mapping')):
        raise ValueError('Cannot export geometry using declared oracle or tracking-only poses')
    if not sequence['frames']:
        raise ValueError('Scan has no frames')
    has_physical=[('odometry' in f, 'body_from_camera' in f) for f in sequence['frames']]
    if any(a!=b for a,b in has_physical) or len(set(has_physical))!=1:
        raise ValueError('Require complete synchronized physical metadata on every frame, or none')
    physical=has_physical[0][0]
    classification_source=('synchronized wheel/IMU + body_from_camera' if physical else
                           'saved estimated Z-up poses; legacy fallback')
    before_pose=hashlib.sha256()
    for frame in sequence['frames']:
        before_pose.update(rigid(frame['camera_to_world']).astype('<f8').tobytes())
    output.mkdir(parents=True, exist_ok=False)
    (output/'keyframes').mkdir()
    segmenter = GroundSegmenter()
    layers = {k:np.empty((0, 3), np.float32) for k in ('ground', 'obstacle', 'uncertain')}
    counts = np.zeros(3, np.int64);valid_counts=np.zeros(3,np.int64);invalid_pixels=0
    before, after = hashlib.sha256(), hashlib.sha256()
    frames, pending_points, pending_labels = [], [], []
    confirmed = 0
    started = time.monotonic()
    fx, fy, cx, cy = sequence['intrinsics']
    for i, frame in enumerate(sequence['frames']):
        path = Path(frame['file'])
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Frame paths must be relative to the scan')
        depth_path=source/path
        if not depth_path.exists() and 'prepared_source' in sequence:
            depth_path=Path(sequence['prepared_source'])/path
        with np.load(depth_path, allow_pickle=False) as sample:
            values = dict(sample)
            depth = fusion_depth(sample)
        pose = rigid(frame['camera_to_world'])
        classification_pose=pose
        if physical:
            packet=frame['odometry']
            if not packet['valid'] or abs(float(packet['time_s'])-float(frame['time_s']))>.02+1e-9:
                raise ValueError('Require valid wheel/IMU packet within 20 ms of camera')
            classification_pose=odom_body(packet)@rigid(frame['body_from_camera'])
        labels = segmenter.classify(depth, sequence['intrinsics'], classification_pose, float(frame['time_s']))
        values.update(labels)
        filename = f'keyframes/{i:05d}.npz'
        np.savez_compressed(output/filename, **values)
        before.update(values['depth'].tobytes())
        with np.load(output/filename, allow_pickle=False) as saved:
            after.update(saved['depth'].tobytes())
            if not np.array_equal(values['depth'], saved['depth']):
                raise RuntimeError('Depth changed while saving labels')
        frames.append(dict(frame, file=filename))
        counts += np.bincount(labels['ground_labels'].ravel(), minlength=3)
        valid_depth=np.isfinite(depth)&(depth>.02)&(depth<=4.)
        valid_counts+=np.bincount(labels['ground_labels'][valid_depth],minlength=3)
        invalid_pixels+=int((~valid_depth).sum())
        confirmed += bool(np.isfinite(labels['ground_plane_camera']).all())
        y, x = np.nonzero(valid_depth)
        z = depth[valid_depth]
        points = np.column_stack(((x-cx)*z/fx, (y-cy)*z/fy, z))
        pending_points.append(points@pose[:3, :3].T+pose[:3, 3])
        pending_labels.append(labels['ground_labels'][valid_depth])
        if len(pending_points)>=32 or i==len(sequence['frames'])-1:
            merge_layers(layers, np.concatenate(pending_points), np.concatenate(pending_labels))
            pending_points.clear(); pending_labels.clear()
        if i%100==0:
            print(json.dumps(dict(frames=i+1, total=len(sequence['frames']),
                                  confirmed=confirmed, seconds=round(time.monotonic()-started, 1))), flush=True)
    if before.digest()!=after.digest():
        raise RuntimeError('Depth checksum mismatch')
    sequence.update(frames=frames, ground_source=SOURCE,classification_pose_source=classification_source)
    (output/'live_sequence.json').write_text(json.dumps(sequence))
    saved_sequence=json.loads((output/'live_sequence.json').read_text())
    after_pose=hashlib.sha256()
    for frame in saved_sequence['frames']:
        after_pose.update(rigid(frame['camera_to_world']).astype('<f8').tobytes())
    if before_pose.digest()!=after_pose.digest():raise RuntimeError('Saved pose checksum mismatch')
    np.savez_compressed(output/'ground_layers.npz', **layers, source=SOURCE, voxel_m=.03)
    exported={name:('obstacles.ply' if name=='obstacle' else name+'.ply') for name in layers}
    for name,path in exported.items():write_ply(output/path,layers[name])
    report = dict(source=str(source), sequence=str(sequence_path), frames=len(frames),
                  confirmed_frames=confirmed, source_depth_sha256=before.hexdigest(),
                  output_depth_sha256=after.hexdigest(), depth_modified=False,
                  source_pose_sha256=before_pose.hexdigest(),output_pose_sha256=after_pose.hexdigest(),poses_modified=False,
                  source_sequence_sha256=hashlib.sha256(sequence_path.read_bytes()).hexdigest(),
                  classification_pose_source=classification_source,map_quality_status='diagnostic/unvalidated',
                  valid_depth_pixel_counts=dict(zip(('uncertain','ground','obstacle'),map(int,valid_counts))),
                  invalid_depth_pixels=invalid_pixels,valid_depth_range_m=[.02,4.],exported_files=exported,voxel_m=.03,
                  ground_source=SOURCE, pose_source=sequence.get('pose_source', 'saved estimated poses'),
                  ground_truth_geometry=False, pixel_counts=dict(zip(('uncertain', 'ground', 'obstacle'), map(int, counts))),
                  layer_points={k:len(v) for k,v in layers.items()}, seconds=time.monotonic()-started,
                  limits='All valid measured pixels before per-class 3cm voxel sampling; no smoothing. Flat floor prior in classification frame; uncertain geometry is not free. Quality gates and navigation clearance unvalidated.')
    (output/'ground_report.json').write_text(json.dumps(report, indent=2))
    return report


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.source, args.output), indent=2))
