"""Pose tracking and keyframe integration worker."""
import json
import os
import sys
import time

import numpy as np

from worker_common import (FUSION_DEPTH_SOURCE, ROOT, cloud_voxel_indices, fusion_depth,
                           keep_keyframe, map_refresh_due, read_labels,
                           startup_tracking_state, status, write_json, write_npz)
sys.path.insert(0, str(ROOT / 'simulation/demo/reconstruction'))
from ground_segmentation import FIELDS, SOURCE

def tracking(session):
    import cv2
    from vslam.tracker import (RgbdSlam, pose_matrix, optimized_map_poses, graph_is_consistent,
                               SLAM_MAP_SOURCE)
    from tracking_events import TrackingEvents
    from vslam.pose_guard import PoseGuard
    from vslam.tracker import local_motion_error
    from vslam.localization import floor_camera_pose, odom_body, rigid
    from vslam.recovery import VisualRecovery

    cv2.setNumThreads(1)
    slam = None
    last = -1
    frames, points, point_labels = [], [], []
    map_from_slam = None
    map_from_local = None
    intrinsics = None
    startup_sensor=None;startup_still_since=None
    # Keep the startup dwell tolerant of the slower image cadence at ×5, with a bounded gap.
    startup_max_gap=min(2.5,.5*max(1.,float(os.environ.get('RB1_SIM_SPEED_FACTOR','1'))))
    guard = PoseGuard()
    recovery = VisualRecovery()
    events = TrackingEvents(session)
    checkpoint = None
    localizer = None
    checkpoint_at = -16
    checkpoint_slot = 0
    stable_samples = 0
    next_localization = 0.
    original_graph = True
    restarted_odometry = False
    previous_track_time = None
    approved_sequence = None
    disagreements = skipped_depth_frames = tracked_frames = recovery_count = matched_recoveries = 0
    skipped_flat_frames = 0
    sample_intervals = []
    last_refresh_keyframes = last_point_count = 0
    last_refresh_at = 0.
    graph_dirty = False
    graph_rejections = 0
    graph_events = {'loop_closure': set(), 'pgo': set()}
    clean_stop=False
    def graph_updated():
        metrics = slam.tracker.get_slam_metrics()
        if not metrics:
            return False
        stamp = metrics.timestamp_ns
        changed = False
        for kind, enabled in (('loop_closure', metrics.lc_status), ('pgo', metrics.pgo_status)):
            if enabled and stamp not in graph_events[kind]:
                graph_events[kind].add(stamp)
                changed = True
        return changed

    def refresh_map(reason):
        nonlocal approved_sequence, last_refresh_at, graph_dirty, map_from_local, graph_rejections
        started = time.monotonic()
        last_refresh_at = started
        graph_dirty = False
        mapped = [np.asarray(frame['camera_to_world']) for frame in frames]
        graph_result = None
        graph_delta = {}
        capture_review = capture is not None and reason in ('graph', 'final')
        if capture_review:
            before = [dict(f) for f in frames]
        optimized = None
        if original_graph and reason in ('graph', 'final'):
            timestamps = [frame['timestamp_ns'] for frame in frames]
            retained = set(timestamps)
            optimized = {p.timestamp_ns: pose_matrix(p.pose) for p in slam.tracker.get_all_slam_poses()
                         if p.timestamp_ns in retained}
            if retained <= optimized.keys():
                candidate = optimized_map_poses(optimized, timestamps, map_from_slam)
                visual_history = [frame['visual_camera_to_world'] for frame in frames]
                if graph_is_consistent(visual_history, candidate):
                    graph_result = 'applied'
                    map_from_local = candidate[-1] @ np.linalg.inv(frames[-1]['local_camera_pose'])
                    mapped = [floor_camera_pose(visual, frame['odometry_body_pose'], frame['body_from_camera'])
                              for visual, frame in zip(candidate, frames)]
                    prior = np.asarray(frames[-1]['camera_to_world']) @ np.linalg.inv(frames[-1]['body_from_camera'])
                    revised = mapped[-1] @ np.linalg.inv(frames[-1]['body_from_camera'])
                    yaw_change = np.arctan2(revised[1, 0], revised[0, 0]) - np.arctan2(prior[1, 0], prior[0, 0])
                    graph_delta = dict(last_xy_change_m=float(np.linalg.norm(revised[:2, 3] - prior[:2, 3])),
                                       last_yaw_change_rad=float(np.arctan2(np.sin(yaw_change), np.cos(yaw_change))))
                    for frame, visual, pose in zip(frames, candidate, mapped):
                        frame['visual_camera_to_world'] = visual.tolist()
                        frame['camera_to_world'] = pose.tolist()
                else:
                    graph_result = 'inconsistent'
                    graph_rejections += 1
            else:
                graph_result = 'missing_poses'
                graph_rejections += 1
        if capture_review:
            ts = round(last * 1e9)
            name = 'final' if reason == 'final' else f'review_{ts}'
            capture.snapshot(slam.tracker, ts, before, name, graph_result or 'restarted', optimized)
        snapshot = [dict(frame) for frame in frames]
        cloud = [xyz @ pose[:3, :3].T + pose[:3, 3] for xyz, pose in zip(points, mapped)]
        cloud=np.concatenate(cloud).astype(np.float32)
        if len(cloud):
            labels=np.concatenate(point_labels)
            indices=cloud_voxel_indices(cloud,labels)
            cloud=cloud[indices]
            write_npz(session/'cloud.npz',xyz=cloud,path=np.array([f['camera_to_world'] for f in snapshot])[:,:3,3],
                      depth_source=FUSION_DEPTH_SOURCE,labels=labels[indices],ground_source=SOURCE)
            approved_sequence = dict(frames=snapshot,intrinsics=intrinsics,depth_source=FUSION_DEPTH_SOURCE,
                pose_source=SLAM_MAP_SOURCE,pose_policy='cuvslam',
                body_pose_constraint='wheel_imu_tilt_height',
                pose_stage='final' if reason=='final' else 'preview',
                map_from_slam=map_from_slam.tolist(),
                bounds_m=[(cloud.min(0)-.1).tolist(),(cloud.max(0)+.1).tolist()])
            write_json(session/'live_sequence.json', approved_sequence)
        with (session/'map_refresh_diagnostics.jsonl').open('a') as log:
            log.write(json.dumps(dict(sensor_time=frames[-1]['time_s'], trigger_sensor_time=last, keyframes=len(frames),
                points=len(cloud),reason=reason,graph_result=graph_result,elapsed_s=time.monotonic()-started, **graph_delta))+'\n')
        return len(cloud)
    def save_checkpoint():
        nonlocal checkpoint, checkpoint_at, checkpoint_slot
        checkpoint_at = len(frames)
        stamps = [frame['timestamp_ns'] for frame in frames]
        retained = set(stamps)
        optimized = {p.timestamp_ns: pose_matrix(p.pose) for p in slam.tracker.get_all_slam_poses()
                     if p.timestamp_ns in retained}
        if not retained <= optimized.keys():
            return
        mapped = [f['visual_camera_to_world'] for f in frames]
        if not graph_is_consistent(mapped, optimized_map_poses(optimized, stamps, map_from_slam)):
            return
        folder = session / f'checkpoint_{checkpoint_slot}'
        saved = []
        slam.tracker.save_map(str(folder), saved.append)
        if saved == [True]:
            local_from_db = np.asarray(frames[-1]['local_camera_pose']) @ np.linalg.inv(optimized[stamps[-1]])
            checkpoint = folder, local_from_db
            checkpoint_slot = 1 - checkpoint_slot

    def log_sample(t, state, translation, angle, matched, sdk_result, stats, valid_fraction):
        with (session/'tracking_diagnostics.jsonl').open('a') as log:
            log.write(json.dumps(dict(sensor_time=t, local_disagreement_m=translation,
                                      local_disagreement_rad=angle, tracking_state=state,
                                      recovery_match=matched, checkpoint_result=sdk_result,
                                      valid_depth_fraction=valid_fraction, **stats))+'\n')

    def report_hold(state, t, packet):
        message = ('Đang tìm lại vị trí · giữ map, hướng camera về vùng đã quét'
                   if state == 'lost' else 'Mất bám · đang giữ bản đồ cuối')
        status(session, 'slam', message, sensor_time=t, keyframes=last_refresh_keyframes,
               points=last_point_count, head_yaw_rad=packet['head_yaw'], tracking_usable=False,
               tracking_state=state)

    (session/'keyframes').mkdir(exist_ok=True)
    recorded = session/'tracker_input' if os.environ.get('RB1_RECORD_RGBD') == '1' else None
    capture = None
    if recorded is not None:
        from graph_capture import GraphCapture
        recorded.mkdir()
        try:
            capture = GraphCapture(session)
        except OSError as exc:
            print(f'Graph capture unavailable: {exc}', file=sys.stderr, flush=True)
    try:
        while not (session/'STOP').exists():
            try:
                with np.load(session/'rgbd.npz',allow_pickle=False) as f: batch={k:f[k] for k in f.files}
            except FileNotFoundError: time.sleep(.05);continue
            t=float(batch['t'])
            if t<=last: time.sleep(.02);continue
            depth=fusion_depth(batch)
            frame_intrinsics = batch['K'].tolist()
            packet = json.loads(str(batch['odometry']))
            if not packet['valid'] or abs(packet['time_s']-t) > .02:
                raise RuntimeError('Invalid or unsynchronized wheel/IMU sample')
            if intrinsics is not None and frame_intrinsics != intrinsics:
                raise RuntimeError('Camera intrinsics changed during tracking')
            has_depth = np.any(batch['depth'])
            thumbnail = cv2.resize(batch['rgb'], (80, 60), interpolation=cv2.INTER_AREA)
            flat_image = np.ptp(thumbnail, axis=(0, 1)).max() < 4
            if not has_depth or flat_image:
                last = t
                skipped_depth_frames += int(not has_depth)
                skipped_flat_frames += int(flat_image)
                if guard.accepted is not None or guard.hold_since is not None:
                    state, _ = guard.update(t, None, None)
                    log_sample(t, state, None, None, False, None,
                               {'input_issue': 'empty_depth' if not has_depth else 'flat_rgb'},
                               float(np.mean(batch['depth'] > 0)))
                    stable_samples = 0
                    events.add(t, batch, None, None, state, odom_body(packet) @ rigid(batch['mount']))
                    report_hold(state, t, packet)
                else:
                    status(session, 'slam', 'Chờ ảnh RGB-D đủ tương phản và có depth', sensor_time=t,
                           keyframes=0, head_yaw_rad=packet['head_yaw'], tracking_usable=False)
                continue
            measured_body = odom_body(packet)
            mount = rigid(batch['mount'])
            sensor_pose = measured_body @ mount
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
                intrinsics = frame_intrinsics
                slam = RgbdSlam([640, 480], intrinsics)

            last=t
            ts=round(t*1e9)
            if recorded is not None:
                write_npz(recorded/f'{ts:020d}.npz', rgb=batch['rgb'], depth=batch['depth'],
                          K=batch['K'], timestamp_ns=ts, mount=batch['mount'])
            # This channel order produced fewer pose jumps on the fixed-camera replay.
            if guard.lost and slam.last_odometry_pose is None and not restarted_odometry:
                if capture is not None:
                    capture.restart(slam.tracker, ts, frames)
                slam = RgbdSlam([640, 480], intrinsics)
                original_graph = False
                restarted_odometry = True
            pose = slam.track(ts, batch['rgb'][..., ::-1], batch['depth'])
            tracked_frames += 1
            if capture is not None:
                capture.observe(slam.tracker, ts, frames)
            if pose is None and guard.accepted is None:
                status(session, 'slam', 'Chờ pose thị giác đầu tiên', sensor_time=t,
                       keyframes=0, tracking_usable=False)
                continue
            raw_pose = slam.last_odometry_pose
            local_pose, matched = recovery.candidate(
                raw_pose, sensor_pose, batch['rgb'], batch['depth'], intrinsics, guard, t)
            sdk_result = None
            if guard.lost and not matched and checkpoint is not None and (t >= next_localization or guard.good_frames == 2):
                next_localization = t + 2.
                folder, local_from_db = checkpoint
                anchor, odom = guard.accepted
                guess = anchor @ np.linalg.solve(odom, sensor_pose)
                # SDK localization may change its graph even when our pose gate rejects the result.
                if localizer is None:
                    localizer = RgbdSlam([640, 480], intrinsics)
                localized = localizer.localize(folder, ts, batch['rgb'], np.linalg.solve(local_from_db, guess))
                sdk_result = 'no_match'
                if localized is not None and raw_pose is not None:
                    candidate = local_from_db @ localized
                    metres, radians = local_motion_error(anchor, candidate, odom, sensor_pose)
                    sdk_result = 'inconsistent'
                    if metres <= .04 and radians <= .06:
                        recovery.pending_alignment = candidate @ np.linalg.inv(raw_pose)
                        local_pose, matched = candidate, True
                        sdk_result = 'matched'
            state, error = guard.update(t, local_pose, sensor_pose, verified_match=matched)
            events.add(t, batch, raw_pose, local_pose, state, sensor_pose, recovery.stats)
            stable_samples = stable_samples + 1 if state == 'accepted' else 0
            translation, angle = error if error is not None else (None, None)
            disagreements += int(error is not None and not guard.plausible(error))
            if previous_track_time is not None:
                sample_intervals.append(t - previous_track_time)
            previous_track_time = t
            log_sample(t, state, translation, angle, matched, sdk_result, recovery.stats,
                       float(np.mean(batch['depth'] > 0)))
            if state in ('hold', 'lost'):
                report_hold(state, t, packet)
                continue
            if state == 'recovered':
                restarted_odometry = False
                recovery_count += 1
                matched_recoveries += int(matched)
            recovery.accept(batch['rgb'], batch['depth'], matched)
            if map_from_slam is None:
                map_from_slam = sensor_pose @ np.linalg.inv(pose)
                map_from_local = sensor_pose @ np.linalg.inv(local_pose)
            if capture is not None and capture.alignment is None and pose is not None:
                capture.alignment = (map_from_slam if original_graph else
                                     map_from_local @ local_pose @ np.linalg.inv(pose))
            graph_dirty = (original_graph and graph_updated()) or graph_dirty
            stride = 4 if checkpoint is None else 16
            if original_graph and len(frames) >= 3 and stable_samples >= 6 and len(frames) - checkpoint_at >= stride:
                save_checkpoint()
            if not keep_keyframe(local_pose, frames):
                if frames and map_refresh_due(len(frames), last_refresh_keyframes, graph_dirty,
                                              time.monotonic()-last_refresh_at):
                    last_point_count = refresh_map('graph' if graph_dirty else 'batch')
                    last_refresh_keyframes = len(frames)
                status(session, 'slam', 'Đang theo dõi', sensor_time=t,
                       keyframes=len(frames), points=last_point_count, head_yaw_rad=packet['head_yaw'],
                       tracking_usable=True, tracking_state=state)
                continue
            if len(frames)>=8192: raise RuntimeError('Đã đạt 8192 keyframe: map đã lưu; bắt đầu phiên mới')
            filename=f'keyframes/{len(frames):05d}.npz'
            write_npz(session/filename,depth=batch['depth'],depth_source=FUSION_DEPTH_SOURCE,
                      tof_hist=batch['tof_hist'],tof_rect=batch['tof_rect'],tof_valid=batch['tof_valid'],
                      **{key:batch[key] for key in FIELDS if key in batch})
            visual_camera = map_from_local @ local_pose
            camera = floor_camera_pose(visual_camera, measured_body, mount)
            frames.append(dict(file=filename, time_s=t, timestamp_ns=ts,
                               local_camera_pose=local_pose.tolist(),
                               visual_camera_to_world=visual_camera.tolist(), body_from_camera=mount.tolist(),
                               odometry_body_pose=measured_body.tolist(), camera_to_world=camera.tolist()))
            if capture is not None:
                frames[-1]['tracker_epoch'] = capture.epoch
            recovery.remember_keyframe(local_pose, sensor_pose, batch['rgb'], batch['depth'])
            y,x=np.mgrid[0:480:8,0:640:8];z=depth[::8,::8];fx,fy,cx,cy=batch['K']
            valid=(z>.02)&(z<=4)
            points.append(np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[valid])
            pixel_labels=read_labels(batch,depth.shape)
            point_labels.append(np.zeros(valid.sum(),np.uint8) if pixel_labels is None else pixel_labels[::8,::8][valid])
            if map_refresh_due(len(frames), last_refresh_keyframes, graph_dirty,
                               time.monotonic()-last_refresh_at):
                last_point_count=refresh_map('graph' if graph_dirty else 'batch')
                last_refresh_keyframes=len(frames)
            status(session,'slam','Đang vẽ bản đồ',sensor_time=t,keyframes=len(frames),
                   points=last_point_count,head_yaw_rad=packet['head_yaw'],
                   tracking_usable=True, tracking_state=state)
        clean_stop=True
    finally:
        events.close()
        if slam is not None and clean_stop:
            held = guard.hold_since is not None
            if capture is not None and (held or not frames):
                capture.snapshot(slam.tracker, round(last * 1e9), frames, 'final', 'held' if held else 'empty')
            if held and approved_sequence is not None:
                write_json(session/'live_sequence.json',
                           dict(approved_sequence, pose_stage='final', tracking_interrupted=True))
            elif frames and not held:
                refresh_map('final')
            saved=[]
            if original_graph and frames and not disagreements and not held and not recovery_count and not graph_rejections:
                slam.tracker.save_map(str(session/'cuvslam_map'),saved.append)
                if saved!=[True]:raise RuntimeError('cuVSLAM map save failed')
            write_json(session/'pose_quality.json',dict(pose_policy='cuvslam',
                local_motion_disagreements=disagreements,skipped_depth_frames=skipped_depth_frames,
                skipped_flat_rgb_frames=skipped_flat_frames,
                database_saved=saved==[True], tracking_interrupted=held,
                recovery_count=recovery_count,
                matched_recoveries=matched_recoveries,
                graph_rejections=graph_rejections,
                published_keyframes=len(approved_sequence['frames']) if approved_sequence else 0,
                keyframes=len(frames),tracked_frames=tracked_frames,
                tracking_interval_median_s=float(np.median(sample_intervals)) if sample_intervals else None,
                tracking_interval_p95_s=float(np.percentile(sample_intervals,95)) if sample_intervals else None,
                tracking_interval_max_s=max(sample_intervals) if sample_intervals else None,
                loop_closure_events=len(graph_events['loop_closure']),pgo_events=len(graph_events['pgo']),
                loop_closure_verified=False))
            status(session,'slam','Đã lưu lịch sử pose cuối',keyframes=len(frames),
                   tracking_usable=False, tracking_state='stopped',
                   pose_policy='cuvslam',local_motion_disagreements=disagreements,
                   pgo_events=len(graph_events['pgo']),database_saved=saved==[True])
