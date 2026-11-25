"""Pose tracking and keyframe integration worker."""
import json
import os
import sys
import time

import numpy as np

from worker_common import (FUSION_DEPTH_SOURCE, ROOT, cloud_voxel_indices, fusion_depth,
                           graph_update_kind, keep_keyframe, map_refresh_due, read_labels,
                           startup_tracking_state, status, write_json, write_npz)
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/mapping/cuvslam'))
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/mapping/odometry'))
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/reconstruction'))
from ground_segmentation import FIELDS, SOURCE

def tracking(session,pose_policy='hybrid'):
    from track_rgbd import RgbdSlam,pose_matrix,optimized_map_poses,SLAM_MAP_SOURCE,validate_local_tracking
    from localization import odom_body,rigid
    if pose_policy not in ('hybrid','cuvslam'):
        raise ValueError('Unknown mapping pose policy')
    source=(SLAM_MAP_SOURCE if pose_policy=='cuvslam' else
            'wheel/IMU + gated current cuVSLAM correction; no unchecked PGO history')
    slam=None;last=-1;frames=[];points=[];point_labels=[];mounts=[];packets=[];visual_frames=[]
    startup_sensor=None;startup_still_since=None
    # Keep the startup dwell tolerant of the slower image cadence at ×5, with a bounded gap.
    startup_max_gap=min(2.5,.5*max(1.,float(os.environ.get('RB1_SIM_SPEED_FACTOR','1'))))
    previous_visual=previous_sensor=previous_batch=None
    localization=None
    if pose_policy=='hybrid':
        from localization import BodyLocalization
        localization=BodyLocalization(level_floor=True)
    visual_rejections=skipped_depth_frames=tracked_frames=0
    sample_intervals=[]
    last_refresh_keyframes=last_point_count=0
    last_graph_event=None
    loop_closure_events=pgo_events=0
    clean_stop=False
    def graph_updated():
        nonlocal last_graph_event,loop_closure_events,pgo_events
        metrics=slam.tracker.get_slam_metrics()
        kind=graph_update_kind(metrics)
        if kind is None:return None
        event=(getattr(metrics,'timestamp_ns',str(metrics)),kind)
        if event==last_graph_event:return None
        last_graph_event=event
        if kind=='pgo':
            pgo_events+=1
            return 'pgo'
        loop_closure_events+=1
        return 'loop_closure'
    def refresh_map(reason):
        started=time.monotonic()
        if pose_policy=='cuvslam':
            optimized={p.timestamp_ns:pose_matrix(p.pose) for p in slam.tracker.get_all_slam_poses()}
            mapped=optimized_map_poses(optimized,[f['timestamp_ns'] for f in frames],
                                       odom_body(packets[0])@rigid(mounts[0]))
        else:
            mapped=[np.asarray(frame['camera_to_world']) for frame in frames]
        cloud=[]
        for i,(frame,corrected) in enumerate(zip(frames,mapped)):
            frame['camera_to_world']=corrected.tolist()
            cloud.append(points[i]@corrected[:3,:3].T+corrected[:3,3])
        cloud=np.concatenate(cloud).astype(np.float32)
        if len(cloud):
            labels=np.concatenate(point_labels)
            indices=cloud_voxel_indices(cloud,labels)
            cloud=cloud[indices]
            write_npz(session/'cloud.npz',xyz=cloud,path=np.array([f['camera_to_world'] for f in frames])[:,:3,3],
                      depth_source=FUSION_DEPTH_SOURCE,labels=labels[indices],ground_source=SOURCE)
            write_json(session/'live_sequence.json',dict(frames=frames,intrinsics=intrinsics,depth_source=FUSION_DEPTH_SOURCE,
                pose_source=source,pose_policy=pose_policy,
                bounds_m=[(cloud.min(0)-.1).tolist(),(cloud.max(0)+.1).tolist()]))
        with (session/'map_refresh_diagnostics.jsonl').open('a') as log:
            log.write(json.dumps(dict(sensor_time=frames[-1]['time_s'],keyframes=len(frames),
                points=len(cloud),reason=reason,elapsed_s=time.monotonic()-started))+'\n')
        return len(cloud)
    (session/'keyframes').mkdir(exist_ok=True)
    try:
        while not (session/'STOP').exists():
            try:
                with np.load(session/'rgbd.npz',allow_pickle=False) as f: batch={k:f[k] for k in f.files}
            except FileNotFoundError: time.sleep(.05);continue
            t=float(batch['t'])
            if t<=last: time.sleep(.02);continue
            depth=fusion_depth(batch)
            intrinsics=batch['K'].tolist()
            packet=json.loads(str(batch['odometry']))
            if pose_policy=='hybrid':
                if not packet['valid'] or abs(packet['time_s']-t)>.02:
                    raise RuntimeError('Invalid or unsynchronized odometry; stop hybrid mapping')
                if not np.any(batch['depth']):
                    last=t;skipped_depth_frames+=1
                    with (session/'tracking_diagnostics.jsonl').open('a') as log:
                        log.write(json.dumps(dict(sensor_time=t,valid_depth_fraction=0.,
                            local_visual_accepted=False,reason='no_supported_depth'))+'\n')
                    status(session,'slam','Chờ depth fusion hợp lệ; bỏ ảnh không có số đo',
                        sensor_time=t,keyframes=len(frames),head_yaw_rad=packet['head_yaw'])
                    continue
            sensor_pose=odom_body(packet)@rigid(batch['mount'])
            # Standup sway can look like a large visual jump before the route's own hold starts.
            if slam is None:
                startup_sensor,startup_still_since,startup_ready=startup_tracking_state(
                    startup_sensor,startup_still_since,t,sensor_pose,startup_max_gap)
                if not startup_ready:
                    last=t
                    status(session,'slam','Chờ RB và camera ổn định trước khi khởi tạo SLAM',
                           sensor_time=t,keyframes=len(frames),head_yaw_rad=packet['head_yaw'])
                    continue
                status(session,'slam','Đang làm nóng GPU tracking')
                slam=RgbdSlam([640,480],batch['K'].tolist())

            last=t
            ts=round(t*1e9);pose=slam.track(ts,batch['rgb'],batch['depth'])
            if pose is None: raise RuntimeError('Mất tracking: map đã giữ lại. Stop SIM rồi Start SIM để quét lại')
            tracked_frames+=1
            local_visual_accepted=True
            rejection=None
            if previous_visual is not None:
                try:
                    validate_local_tracking(previous_visual,slam.last_odometry_pose,previous_sensor,sensor_pose)
                except RuntimeError as error:
                    if not (session/'tracking_failure.json').exists():
                        write_npz(session/'tracking_failure_previous.npz',**previous_batch)
                        write_npz(session/'tracking_failure_current.npz',**batch)
                        write_json(session/'tracking_failure.json',dict(reason=str(error),
                            previous_time=float(previous_batch['t']),current_time=t,
                            previous_visual=previous_visual.tolist(),current_visual=slam.last_odometry_pose.tolist(),
                            previous_sensor=previous_sensor.tolist(),current_sensor=sensor_pose.tolist()))
                    if pose_policy=='cuvslam':raise
                    local_visual_accepted=False;visual_rejections+=1;rejection=str(error)
            interval=None if previous_batch is None else t-float(previous_batch['t'])
            if interval is not None:sample_intervals.append(interval)
            visual_accepted=local_visual_accepted
            mapped_pose=pose
            if pose_policy=='hybrid':
                localized=localization.update(ts,packet,batch['mount'],
                    pose if local_visual_accepted else None)
                mapped_pose=np.asarray(localized['world_from_camera'])
                visual_accepted=localized['visual_accepted']
            with (session/'tracking_diagnostics.jsonl').open('a') as log:
                log.write(json.dumps(dict(sensor_time=t,interval_s=interval,
                    valid_depth_fraction=float(np.mean(batch['depth']>0)),
                    local_visual_accepted=local_visual_accepted,
                    visual_correction_accepted=visual_accepted,reason=rejection))+'\n')
            previous_visual,previous_sensor=slam.last_odometry_pose,sensor_pose
            previous_batch=batch
            # Bootstrap readiness, then retain views by motion; idle time must not fill the map.
            keyframe_pose=mapped_pose
            if not keep_keyframe(keyframe_pose, visual_frames):
                graph_update=graph_updated() if pose_policy=='cuvslam' else None
                if frames and map_refresh_due(len(frames),last_refresh_keyframes,graph_update is not None):
                    last_point_count=refresh_map(graph_update or 'batch')
                    last_refresh_keyframes=len(frames)
                status(session,'slam','Đang theo dõi',sensor_time=t,keyframes=len(frames),
                       points=last_point_count,head_yaw_rad=packet['head_yaw']);continue
            if len(frames)>=8192: raise RuntimeError('Đã đạt 8192 keyframe: map đã lưu; bắt đầu phiên mới')
            filename=f'keyframes/{len(frames):05d}.npz'
            visual_frames.append({'camera_to_world':keyframe_pose.copy()})
            write_npz(session/filename,depth=batch['depth'],depth_source=FUSION_DEPTH_SOURCE,
                      tof_hist=batch['tof_hist'],tof_rect=batch['tof_rect'],tof_valid=batch['tof_valid'],
                      **{key:batch[key] for key in FIELDS if key in batch})
            frames.append(dict(file=filename,time_s=t,timestamp_ns=ts,camera_to_world=mapped_pose.tolist(),
                               local_camera_pose=slam.last_odometry_pose.tolist(),
                               local_visual_accepted=local_visual_accepted,
                               visual_accepted=visual_accepted))
            mounts.append(batch['mount']);packets.append(packet)
            y,x=np.mgrid[0:480:8,0:640:8];z=depth[::8,::8];fx,fy,cx,cy=batch['K']
            valid=(z>.02)&(z<=4)
            points.append(np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[valid])
            pixel_labels=read_labels(batch,depth.shape)
            point_labels.append(np.zeros(valid.sum(),np.uint8) if pixel_labels is None else pixel_labels[::8,::8][valid])
            graph_update=graph_updated() if pose_policy=='cuvslam' else None
            if map_refresh_due(len(frames),last_refresh_keyframes,graph_update is not None):
                last_point_count=refresh_map(graph_update or 'batch')
                last_refresh_keyframes=len(frames)
            status(session,'slam','Đang vẽ bản đồ',sensor_time=t,keyframes=len(frames),
                   points=last_point_count,head_yaw_rad=packet['head_yaw'])
        clean_stop=True
    finally:
        if slam is not None and clean_stop:
            if frames:refresh_map('final')
            saved=[]
            if not visual_rejections:
                slam.tracker.save_map(str(session/'cuvslam_map'),saved.append)
                if saved!=[True]:raise RuntimeError('cuVSLAM map save failed')
            write_json(session/'pose_quality.json',dict(pose_policy=pose_policy,
                local_visual_rejections=visual_rejections,skipped_depth_frames=skipped_depth_frames,
                database_saved=saved==[True],
                accepted_visual_keyframes=sum(f.get('visual_accepted',False) for f in frames),
                keyframes=len(frames),tracked_frames=tracked_frames,
                tracking_interval_median_s=float(np.median(sample_intervals)) if sample_intervals else None,
                tracking_interval_p95_s=float(np.percentile(sample_intervals,95)) if sample_intervals else None,
                tracking_interval_max_s=max(sample_intervals) if sample_intervals else None,
                loop_closure_events=loop_closure_events,pgo_events=pgo_events,
                loop_closure_verified=False))
            status(session,'slam','Đã lưu lịch sử pose cuối',keyframes=len(frames),
                   pose_policy=pose_policy,local_visual_rejections=visual_rejections,
                   pgo_events=pgo_events,database_saved=saved==[True])
