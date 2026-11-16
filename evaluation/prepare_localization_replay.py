"""Prepare held-out RGB/ToF localization measurements; isolate simulator truth for scoring."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import h5py
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from replay_fusion_only import ROOT, Sensors, tof_support, FUSION_DEPTH_SOURCE, write_json
from localization import odom_body


def select_samples(captures, odometry, start, duration, period):
    """Select increasing sensor times, with the nearest valid odometry within 20 ms."""
    if not np.isfinite([start, duration, period]).all() or start < 0 or min(duration, period) <= 0:
        raise ValueError('Require nonnegative start and positive finite duration/period')
    if any('reset' in packet for packet in odometry):
        raise ValueError('Replay must contain one continuous session, without resets')
    times = np.asarray([float(p['time_s']) for p in odometry])
    capture_times = np.asarray([float(c['t']) for c in captures])
    for timestamps in (times, capture_times):
        if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
            raise ValueError('Require finite strictly increasing source timestamps')
    selected = []
    next_time = start
    for capture, timestamp in zip(captures, capture_times):
        if timestamp < next_time-1e-9 or timestamp >= start+duration or not len(times):
            continue
        index = int(np.searchsorted(times, timestamp))
        index = min(range(max(0, index-1), min(len(times), index+1)), key=lambda i: abs(times[i]-timestamp))
        packet = odometry[index]
        if abs(times[index]-timestamp) > .02+1e-9 or not packet.get('valid'):
            continue
        try:
            odom_body(packet)
            values = [float(capture[k]) for k in ('x','y','z','qw','qx','qy','qz','lh','lk','lw','rh','rk','rw','hj')]
            if not np.isfinite(values).all() or np.linalg.norm(values[3:7]) < 1e-8:
                continue
        except (KeyError, ValueError, TypeError):
            continue
        selected.append((capture, packet))
        next_time = start+(int(np.floor((timestamp-start)/period+1e-9))+1)*period
    return selected


def prepare(source, output, start, duration, period):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output == source or source in output.parents:
        raise ValueError('Output must be outside the immutable raw source directory')
    if output.exists():
        raise FileExistsError(output)
    metadata = json.loads((source/'metadata.json').read_text())
    captures = [json.loads(line) for line in (source/'live_capture.jsonl').read_text().splitlines() if line.strip()]
    odometry = [json.loads(line) for line in (source/'odometry.jsonl').read_text().splitlines() if line.strip()]
    selected = select_samples(captures, odometry, start, duration, period)
    if not selected:
        raise ValueError('No valid synchronized samples in requested interval')
    command = metadata['runner_command']
    model = command[command.index('--model')+1]
    output.mkdir(parents=True, exist_ok=False)
    inputs = output/'inputs'; inputs.mkdir()
    (output/'keyframes').mkdir()
    began = time.monotonic()
    previous_model = os.environ.get('RB1_SIM_MODEL')
    os.environ['RB1_SIM_MODEL'] = model
    try:
        sensor = Sensors()
    finally:
        if previous_model is None:
            os.environ.pop('RB1_SIM_MODEL', None)
        else:
            os.environ['RB1_SIM_MODEL'] = previous_model
    frames, truth, samples = [], [], []
    try:
        for i, (capture, packet) in enumerate(selected):
            rgb, hist, rect, valid, mount = sensor.render(capture)
            filename = f'{i:05d}.h5'
            with h5py.File(inputs/filename, 'w') as sample:
                for key, value in dict(rgb=rgb, depth=np.zeros((480,640), np.float32),
                                       hist_data=hist, fr=rect, mask=valid).items():
                    sample.create_dataset(key, data=value, compression='gzip')
            samples.append(dict(filename=filename))
            timestamp = float(capture['t'])
            frames.append(dict(file=f'keyframes/{i:05d}.npz', time_s=timestamp,
                               odometry=packet, body_from_camera=mount.tolist()))
            pose = np.eye(4)
            pose[:3,:3] = Rotation.from_quat([float(capture[k]) for k in ('qx','qy','qz','qw')]).as_matrix()
            pose[:3,3] = [float(capture[k]) for k in ('x','y','z')]
            truth.append(dict(time_s=timestamp, simulator_from_body=pose.tolist()))
            if i % 50 == 0:
                print(f'sensor {i}/{len(selected)}', flush=True)
    finally:
        sensor.rgb.close(); sensor.tof.close()
    write_json(inputs/'data.json', dict(train=[], test=samples))
    subprocess.run([sys.executable, str(ROOT/'technology/scan3d_fusion/inference/run_demo.py'),
                    '--max-frames', '0', '--data-root', str(inputs), '--output', str(output/'prediction')], check=True)
    valid_pixels = 0
    for i, frame in enumerate(frames):
        depth = np.array(Image.open(output/'prediction'/f'{i:05d}_depth.png'), np.float32)/1000
        with h5py.File(inputs/f'{i:05d}.h5', 'r') as sample:
            hist, rect, valid = sample['hist_data'][:], sample['fr'][:], sample['mask'][:]
        depth = tof_support(depth, hist, rect, valid, sensor.K, sensor.cal)
        valid_pixels += int(np.count_nonzero(depth))
        np.savez_compressed(output/frame['file'], depth=np.rint(depth*1000).astype(np.uint16),
                            depth_source=FUSION_DEPTH_SOURCE, tof_hist=hist, tof_rect=rect, tof_valid=valid)
    if not valid_pixels:
        raise RuntimeError('Fusion returned no supported depth')
    write_json(output/'localization_input.json', dict(intrinsics=sensor.K, frames=frames,
                                                      depth_source=FUSION_DEPTH_SOURCE))
    write_json(output/'evaluator_truth.json', dict(source=str(source), frame='simulator',
               purpose='Offline evaluation only; never a localization measurement or initializer', frames=truth))
    report = dict(source=str(source), simulator_model=model, start_s=start, duration_s=duration, period_s=period,
                  frames=len(frames), first_time_s=frames[0]['time_s'], last_time_s=frames[-1]['time_s'],
                  depth_source=FUSION_DEPTH_SOURCE, valid_depth_pixels=valid_pixels,
                  max_odometry_timestamp_offset_s=max(abs(f['time_s']-f['odometry']['time_s']) for f in frames),
                  ground_truth_rgbd_depth_used=False, simulator_world_pose_in_measurements=False,
                  plane_depth_correction=False, floor_completion=False, wall_seconds=time.monotonic()-began)
    write_json(output/'source.json', report)
    print(json.dumps(report, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--start', type=float, required=True, help='First simulator timestamp in seconds')
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--period', type=float, default=.2)
    args = parser.parse_args()
    prepare(args.source, args.output, args.start, args.duration, args.period)


if __name__ == '__main__':
    main()
