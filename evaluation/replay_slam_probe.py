"""Replay prepared measured RGB/fusion depth without odometry substitution or truth."""
import argparse
import json
from pathlib import Path
import sys

import h5py
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/mapping/cuvslam'),
              str(ROOT/'technology/scan3d_fusion/live')]
from track_rgbd import RgbdSlam,pose_matrix,validate_local_tracking,optimized_map_poses,SLAM_MAP_SOURCE
from localization import odom_body,rigid
from workers import fusion_depth


def run(prepared, output, guarded=False):
    prepared, output = Path(prepared).resolve(), Path(output).resolve()
    sequence=json.loads((prepared/'localization_input.json').read_text())
    frames=sequence['frames'];timestamps=[round(f['time_s']*1e9) for f in frames]
    assert timestamps and min(timestamps)>=0 and all(b>a for a,b in zip(timestamps,timestamps[1:])), 'Invalid timestamps'
    output.mkdir(parents=True,exist_ok=False)
    slam=RgbdSlam([640,480],sequence['intrinsics'])
    immediate=[];local=[];events={};features=[];failure=None
    optimized={};previous_visual=previous_sensor=None
    for index,(frame,timestamp) in enumerate(zip(frames,timestamps)):
        stem=Path(frame['file']).stem
        with h5py.File(prepared/'inputs'/f'{stem}.h5') as sample:rgb=sample['rgb'][:]
        with np.load(prepared/frame['file'],allow_pickle=False) as sample:
            fusion_depth(sample)
            depth=sample['depth']
        try:
            pose=slam.track(timestamp,rgb,depth)
        except ValueError as error:
            failure=dict(index=index,time_s=frame['time_s'],reason=str(error))
            break
        if pose is None:
            failure=dict(index=index,time_s=frame['time_s'],reason='tracking lost')
            break
        if guarded:
            if abs(frame['odometry']['time_s']-frame['time_s'])>.02+1e-9:
                raise ValueError('Odometry must match camera timestamp within 20 ms')
            sensor=odom_body(frame['odometry'])@rigid(frame['body_from_camera'])
            try:
                if previous_visual is not None:
                    validate_local_tracking(previous_visual,slam.last_odometry_pose,previous_sensor,sensor)
            except RuntimeError as error:
                failure=dict(index=index,time_s=frame['time_s'],reason=str(error))
                break
            previous_visual,previous_sensor=slam.last_odometry_pose.copy(),sensor
        record=dict(frame,timestamp_ns=timestamp)
        # Snapshot only accepted SDK states: the next track() can mutate the graph before rejection.
        optimized={p.timestamp_ns:pose_matrix(p.pose).copy() for p in slam.tracker.get_all_slam_poses()}
        immediate.append(dict(record,camera_to_world=pose.tolist()))
        local.append(dict(record,camera_to_world=slam.last_odometry_pose.tolist()))
        features.append(len(slam.tracker.get_last_observations(0)))
        metrics=slam.tracker.get_slam_metrics()
        if metrics and (metrics.lc_status or metrics.pgo_status):
            events[metrics.timestamp_ns]=str(metrics)
        if index%100==0:print(f'{index+1}/{len(frames)} observations={features[-1]}',flush=True)
    assert all(f['timestamp_ns'] in optimized for f in immediate), 'Missing optimized history'
    final=[dict(f,camera_to_world=optimized[f['timestamp_ns']].tolist()) for f in immediate]
    if guarded and final:
        poses=optimized_map_poses(optimized,[f['timestamp_ns'] for f in final],
                                  odom_body(frames[0]['odometry'])@rigid(frames[0]['body_from_camera']))
        final=[dict(f,visual_camera_to_world=f['camera_to_world'],camera_to_world=p.tolist()) for f,p in zip(final,poses)]
    for filename,records in [('immediate_sequence.json',immediate),('local_sequence.json',local),('live_sequence.json',final)]:
        payload=dict(intrinsics=sequence['intrinsics'],frames=records,
                     prepared_source=str(prepared), guarded=guarded,
                     pose_source=SLAM_MAP_SOURCE if guarded and filename=='live_sequence.json' else
                     'cuVSLAM diagnostic; no odometry gate or truth; not an accepted map')
        (output/filename).write_text(json.dumps(payload))
    report=dict(input_frames=len(frames),tracked_frames=len(final),failure=failure,
                complete=len(final)==len(frames),map_saved=False,guarded=guarded,prepared_source=str(prepared),
                ground_truth_used=False,depth_modified=False,timestamps_modified=False,
                successful_metrics_by_timestamp=events,observations_per_frame=features,
                warning='Tracking coverage is not accuracy; no SDK database saved' if guarded else
                'Diagnostic bypasses wheel/IMU sanity check; tracking coverage is not accuracy')
    (output/'tracking.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--guarded',action='store_true',help='Stop on local visual/wheel-IMU disagreement')
    args=parser.parse_args()
    report=run(args.prepared,args.output,args.guarded)
    return 0 if report['complete'] else 2


if __name__=='__main__':sys.exit(main())
