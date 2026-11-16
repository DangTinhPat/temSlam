"""Score saved mapping trajectories; simulator truth stays in this evaluator."""
import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/live'),str(ROOT/'technology/scan3d_fusion/mapping/cuvslam')]
from evaluate import metrics
from scipy.spatial.transform import Rotation
from track_rgbd import rigid


def prepared_truth(prepared, frames):
    prepared=Path(prepared)
    inputs=json.loads((prepared/'localization_input.json').read_text())['frames']
    truth=json.loads((prepared/'evaluator_truth.json').read_text())['frames']
    measured={f['file']:f for f in inputs}
    by_time={f['time_s']:f for f in truth}
    if len(measured)!=len(inputs) or len(by_time)!=len(truth):
        raise ValueError('Duplicate prepared measurement/truth keys')
    poses=[]
    for frame in frames:
        sample=measured.get(frame['file'])
        if (sample is None or frame['time_s']!=sample['time_s'] or frame['time_s'] not in by_time
                or frame.get('timestamp_ns',round(frame['time_s']*1e9))!=round(frame['time_s']*1e9)):
            raise ValueError('Require exact prepared measurement/truth timestamp match')
        poses.append(rigid(by_time[frame['time_s']]['simulator_from_body'])@rigid(sample['body_from_camera']))
    return np.asarray(poses).reshape(-1,4,4)


def trajectory_metrics(estimated, truth, times, rpe_tolerance_s=.07):
    if not np.isfinite(rpe_tolerance_s) or not 0<rpe_tolerance_s<=.07:
        raise ValueError('RPE tolerance must be positive and <= one 0.07s native frame')
    estimated=np.asarray(estimated,float).reshape(-1,4,4)
    truth=np.asarray(truth,float).reshape(-1,4,4)
    times=np.asarray(times,float)
    if (estimated.shape!=truth.shape or times.shape!=(len(truth),) or not np.isfinite(times).all()
            or np.any(np.diff(times)<=0)):
        raise ValueError('Require matching poses and strictly increasing timestamps')
    for pose in np.concatenate((estimated,truth)):
        rigid(pose)
    result=dict(matched_frames=len(truth),duration_s=float(times[-1]-times[0]) if len(times)>1 else 0.,
                ate_se3_rmse_m=None,ate_first_pose_rmse_m=None,first_pose_rotation_rmse_deg=None,
                endpoint_translation_error_m=None,endpoint_rotation_error_deg=None,
                rpe_1s_pairs=0,rpe_1s_translation_rmse_m=None,rpe_1s_rotation_rmse_deg=None,
                first_pose_translation_p95_m=None,first_pose_translation_max_m=None,
                first_pose_rotation_p95_deg=None,first_pose_rotation_max_deg=None,
                ground_truth_path_m=None,estimated_path_m=None,path_length_ratio=None,
                rpe_1s_translation_p95_m=None,rpe_1s_translation_max_m=None,
                rpe_1s_rotation_p95_deg=None,rpe_1s_rotation_max_deg=None,
                input_interval_max_s=float(np.diff(times).max()) if len(times)>1 else None,
                rpe_1s_interval_min_s=None,rpe_1s_interval_max_s=None,
                rpe_target_interval_s=1.,rpe_timestamp_tolerance_s=float(rpe_tolerance_s))
    if not len(truth):return result,estimated
    aligned=truth[0]@np.linalg.inv(estimated[0])@estimated
    if len(truth)>=3:
        legacy,aligned_se3=metrics(estimated,truth)
        result.update(legacy)
        result['alignment']='SE3, no scale fitting; reference poses used for evaluation alignment'
    else:
        aligned_se3=aligned
    def rms(values):return float(np.sqrt(np.mean(np.square(values))))
    errors=np.linalg.inv(truth)@aligned
    translation=np.linalg.norm(errors[:,:3,3],axis=1)
    rotation=np.rad2deg(Rotation.from_matrix(errors[:,:3,:3]).magnitude())
    truth_path=float(np.linalg.norm(np.diff(truth[:,:3,3],axis=0),axis=1).sum())
    estimated_path=float(np.linalg.norm(np.diff(estimated[:,:3,3],axis=0),axis=1).sum())
    result.update(ate_first_pose_rmse_m=rms(translation),first_pose_rotation_rmse_deg=rms(rotation),
                  first_pose_translation_p95_m=float(np.quantile(translation,.95)),
                  first_pose_translation_max_m=float(translation.max()),
                  first_pose_rotation_p95_deg=float(np.quantile(rotation,.95)),
                  first_pose_rotation_max_deg=float(rotation.max()),
                  ground_truth_path_m=truth_path,estimated_path_m=estimated_path,
                  path_length_ratio=estimated_path/truth_path if truth_path>0 else None,
                  endpoint_translation_error_m=float(np.linalg.norm(errors[-1,:3,3])),
                  endpoint_rotation_error_deg=float(np.rad2deg(Rotation.from_matrix(errors[-1,:3,:3]).magnitude())))
    relative=[];intervals=[]
    for i,t in enumerate(times):
        j=int(np.argmin(np.abs(times-(t+1))))
        if j>i and abs(times[j]-t-1)<=rpe_tolerance_s+1e-9:
            relative.append(np.linalg.inv(np.linalg.inv(truth[i])@truth[j])@(np.linalg.inv(estimated[i])@estimated[j]))
            intervals.append(float(times[j]-t))
    if relative:
        relative=np.asarray(relative)
        translation=np.linalg.norm(relative[:,:3,3],axis=1)
        rotation=np.rad2deg(Rotation.from_matrix(relative[:,:3,:3]).magnitude())
        result.update(rpe_1s_pairs=len(relative),rpe_1s_translation_rmse_m=rms(translation),
                      rpe_1s_rotation_rmse_deg=rms(rotation),
                      rpe_1s_translation_p95_m=float(np.quantile(translation,.95)),
                      rpe_1s_translation_max_m=float(translation.max()),
                      rpe_1s_rotation_p95_deg=float(np.quantile(rotation,.95)),
                      rpe_1s_rotation_max_deg=float(rotation.max()),
                      rpe_1s_interval_min_s=min(intervals),rpe_1s_interval_max_s=max(intervals))
    return result,aligned_se3


def evaluate_prepared(candidate, prepared):
    candidate,prepared=Path(candidate),Path(prepared)
    final=json.loads((candidate/'live_sequence.json').read_text())
    inputs=json.loads((prepared/'localization_input.json').read_text())['frames']
    source_times=np.asarray([f['time_s'] for f in inputs],float)
    if not np.isfinite(source_times).all() or np.any(np.diff(source_times)<=0):
        raise ValueError('Prepared source timestamps must be finite and strictly increasing')
    rpe_tolerance=min(.07,float(np.diff(source_times).max())) if len(source_times)>1 else .07
    frames=final['frames'];truth=prepared_truth(prepared,frames)
    expected=[f['time_s'] for f in frames]
    reports={};aligned={}
    for filename,name in [('immediate_sequence.json','cuvslam_immediate'),
                          ('live_sequence.json','cuvslam_final_optimized'),('local_sequence.json','cuvslam_local_odometry')]:
        path=candidate/filename
        if not path.exists():continue
        records=json.loads(path.read_text())['frames']
        if [(f['file'],f['time_s']) for f in records]!=[(f['file'],f['time_s']) for f in frames]:
            raise ValueError('Trajectory timestamps/files differ')
        predicted=np.asarray([f['camera_to_world'] for f in records]).reshape(-1,4,4)
        reports[name],aligned[name]=trajectory_metrics(predicted,truth,expected,rpe_tolerance)
    source_indices={f['file']:i for i,f in enumerate(inputs)}
    observed=[source_indices[f['file']] for f in frames]
    missing=sorted(set(range(len(inputs)))-set(observed))
    interior=[i for i in missing if observed and observed[0]<i<observed[-1]]
    complete=bool(inputs) and not missing
    coverage_status=('complete' if complete else 'empty' if not observed else 'interior_gaps' if interior
                     else 'accepted_prefix' if observed==list(range(len(observed))) else 'truncated_window')
    provenance=[final]
    for name in ('source.json','tracking.json'):
        if (candidate/name).exists():provenance.append(json.loads((candidate/name).read_text()))
    flags={}
    for key in ('ground_truth_depth_used','ground_truth_pose_used_by_tracker','not_for_mapping'):
        if any(key in record and type(record[key]) is not bool for record in provenance):
            raise ValueError('Provenance flags must be booleans')
        flags[key]=any(record.get(key) is True for record in provenance)
    oracle_depth=flags['ground_truth_depth_used'];oracle_pose=flags['ground_truth_pose_used_by_tracker']
    if oracle_depth or oracle_pose:flags['not_for_mapping']=True
    truth_scope=('Oracle depth entered tracking; simulator truth is part of this diagnostic control, not an accepted map'
                 if oracle_depth else 'Oracle pose entered tracking; diagnostic control, not an accepted map' if oracle_pose else
                 'Reference poses used by evaluator; tracker provenance reports no oracle depth/pose input')
    result=dict(source=str(candidate.resolve()),prepared_source=str(prepared.resolve()),frames=len(frames),
                input_frames=len(inputs),coverage_fraction=len(frames)/len(inputs) if inputs else 0.,
                truth_scope=truth_scope,trajectories=reports,**flags,
                source_coverage_complete=complete,source_coverage_status=coverage_status,
                missing_source_indices=missing,missing_interior_source_indices=interior,
                map_quality_status=('diagnostic/oracle control; not accepted' if oracle_depth or oracle_pose else
                                    'diagnostic/tracking-only; not accepted' if flags['not_for_mapping'] else 'diagnostic/unvalidated'))
    (candidate/'evaluation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(candidate/'evaluator_trajectories.npz',truth=truth,**aligned,time_s=expected)
    print(json.dumps(result,indent=2,allow_nan=False))
    return result


def legacy_main():
    import mujoco
    from sensors import Sensors
    raw=ROOT/'datasets/raw/tof-20261112-094256-483079'
    baseline=ROOT/'datasets/processed/scan3d_fusion/room-floor-preserved-20261112'
    candidate=Path(sys.argv[1]).resolve() if len(sys.argv)>1 else ROOT/'datasets/processed/scan3d_fusion/room-cuvslam-diagnostic-20261114'
    original=json.loads((baseline/'live_sequence.json').read_text())
    final=json.loads((candidate/'live_sequence.json').read_text())
    immediate=json.loads((candidate/'immediate_sequence.json').read_text())
    captures=[json.loads(line) for line in (raw/'live_capture.jsonl').open()]
    times=np.array([float(v['t']) for v in captures])
    command=json.loads((raw/'metadata.json').read_text())['runner_command']
    os.environ['RB1_SIM_MODEL']=command[command.index('--model')+1]
    sensor=Sensors()
    truth=[];offsets=[]
    try:
        for frame in final['frames']:
            t=frame['time_s'];index=int(np.searchsorted(times,t))
            index=min(range(max(0,index-1),min(len(times),index+1)),key=lambda i:abs(times[i]-t))
            offset=abs(times[index]-t)
            if offset>.02:raise ValueError('No timestamp-matched evaluator truth')
            offsets.append(offset);view=captures[index];m,d=sensor.model,sensor.data
            d.qpos[:7]=[float(view[k]) for k in ('x','y','z','qw','qx','qy','qz')]
            for key,address in sensor.joints.items():d.qpos[address]=float(view[key])
            mujoco.mj_forward(m,d)
            pose=np.eye(4);pose[:3,:3]=d.cam_xmat[sensor.camera].reshape(3,3)@np.diag([1,-1,-1])
            pose[:3,3]=d.cam_xpos[sensor.camera]
            truth.append(pose)
    finally:
        sensor.rgb.close();sensor.tof.close()
    truth=np.array(truth)
    reference={round(f['time_s']*1e9):f for f in original['frames']}
    immediate_by_time={f['timestamp_ns']:f for f in immediate['frames']}
    predictions={
        'cuvslam_immediate':np.array([immediate_by_time[f['timestamp_ns']]['camera_to_world'] for f in final['frames']]),
        'cuvslam_final_optimized':np.array([f['camera_to_world'] for f in final['frames']])}
    reports={};aligned={}
    common=np.array([f['timestamp_ns'] in reference for f in final['frames']])
    if common.sum()>=3:
        baseline_poses=np.array([reference[f['timestamp_ns']]['camera_to_world'] for f in final['frames'] if f['timestamp_ns'] in reference])
        reports['old_gated_odometry_common_frames'],_=metrics(baseline_poses,truth[common])
    local_path=candidate/'local_sequence.json'
    if local_path.exists():
        local=json.loads(local_path.read_text())['frames']
        assert [f['timestamp_ns'] for f in local]==[f['timestamp_ns'] for f in final['frames']], 'Local/final timestamps differ'
        predictions['cuvslam_local_odometry']=np.array([f['camera_to_world'] for f in local])
    for name,prediction in predictions.items():
        reports[name],aligned[name]=metrics(prediction,truth)
    result=dict(source=str(candidate),frames=len(truth),max_truth_timestamp_offset_s=max(offsets),
                truth_scope='Evaluator only; no truth enters tracker or reconstruction',trajectories=reports)
    (candidate/'evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
    np.savez_compressed(candidate/'evaluator_trajectories.npz',truth=truth,**aligned,
                        time_s=[f['time_s'] for f in final['frames']])
    print(json.dumps(result,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('candidate',type=Path,nargs='?')
    parser.add_argument('--prepared',type=Path,help='Prepared measurements and isolated evaluator_truth.json')
    args=parser.parse_args()
    prepared=args.prepared
    if prepared is None and args.candidate:
        sequence=json.loads((args.candidate/'live_sequence.json').read_text())
        if sequence.get('prepared_source'):prepared=Path(sequence['prepared_source'])
    if prepared is not None:
        if args.candidate is None:parser.error('candidate is required with --prepared')
        evaluate_prepared(args.candidate,prepared)
    else:legacy_main()


if __name__=='__main__':main()
