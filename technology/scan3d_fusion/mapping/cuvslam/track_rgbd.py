#!/usr/bin/env python3
"""Native cuVSLAM RGB-D replay. Ground-truth depth/poses never enter tracking."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import sys
import time

import h5py
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/reconstruction'))
from depth_support import tof_support
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "odometry"))
from localization import BodyLocalization

# OpenCV camera axes (right/down/forward) to initial horizontal map axes.
MAP_FROM_START = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                           [0., -1., 0., 0.], [0., 0., 0., 1.]])


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose.rotation).as_matrix()
    transform[:3, 3] = pose.translation
    if not np.isfinite(transform).all():
        raise ValueError('cuVSLAM returned a non-finite pose')
    return MAP_FROM_START @ transform


def body_pose(camera_to_world, body_from_camera):
    """Use time-matched neck kinematics, including the camera lever arm."""
    transform = np.asarray(body_from_camera, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(transform[:3, :3]) < .999):
        raise ValueError('body_from_camera must be a rigid transform')
    return camera_to_world @ np.linalg.inv(transform)


class RgbdSlam:
    """Reusable image/depth interface; no simulator, ground-truth or ROS inputs."""
    def __init__(self, image_size, intrinsics):
        import cuvslam
        version = importlib.metadata.version('cuvslam')
        if version.split('+')[0] != '17.0.1':
            raise RuntimeError(f'This adapter targets cuVSLAM 17.0.1, found {version}')
        self.size = tuple(map(int, image_size))
        self.last_timestamp = -1
        self.localization = BodyLocalization()
        camera = cuvslam.Camera()
        camera.size = self.size
        camera.focal = list(intrinsics[:2])
        camera.principal = list(intrinsics[2:])
        if len(intrinsics) != 4 or not np.isfinite(intrinsics).all() or min(intrinsics[:2]) <= 0:
            raise ValueError('Invalid pinhole intrinsics')
        settings = cuvslam.Tracker.OdometryRGBDSettings()
        settings.depth_camera_id = 0
        settings.depth_scale_factor = 1000.
        odom = cuvslam.Tracker.OdometryConfig(
            use_gpu=True, async_sba=False,
            odometry_mode=cuvslam.Tracker.OdometryMode.RGBD, rgbd_settings=settings)
        slam = cuvslam.Tracker.SlamConfig(
            use_gpu=True, sync_mode=True, enable_reading_internals=True,
            planar_constraints=False, gt_align_mode=False, max_map_size=0)
        self.tracker = cuvslam.Tracker(cuvslam.Rig([camera]), odom, slam)

    def track(self, timestamp_ns, rgb, depth_mm):
        width, height = self.size
        if (timestamp_ns <= self.last_timestamp or rgb.shape != (height, width, 3)
                or rgb.dtype != np.uint8 or depth_mm.shape != (height, width)
                or depth_mm.dtype != np.uint16):
            raise ValueError('Require increasing timestamps, RGB uint8 and aligned depth uint16 mm')
        self.last_timestamp = timestamp_ns
        estimate, slam = self.tracker.track(
            timestamp_ns, images=[np.ascontiguousarray(rgb)], depths=[np.ascontiguousarray(depth_mm)])
        if estimate.world_from_rig is None or slam is None:
            return None
        return pose_matrix(slam)

    def track_body(self, timestamp_ns, rgb, depth_mm, odometry, body_from_camera):
        """Track synchronized RGB-D and correct the external wheel/IMU odometry frame.

        This does not feed wheel factors or a moving body IMU into cuVSLAM.
        Use this instead of track() for that frame, never call both with the same timestamp.
        """
        if abs(timestamp_ns*1e-9-odometry['time_s']) > .02:
            raise ValueError('Odometry must match the camera timestamp within 20 ms')
        pose = self.track(timestamp_ns, rgb, depth_mm)
        return self.localization.update(timestamp_ns, odometry, body_from_camera, pose)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('prediction', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--odometry', type=Path, help='Synchronized native odometry.jsonl for this capture')
    args = parser.parse_args()
    sequence = json.loads((args.capture / 'sequence.json').read_text())
    frames = sequence['frames']
    if not frames or len({f['file'] for f in frames}) != len(frames):
        raise ValueError('Capture must contain unique frames')
    timestamps = [round(float(f['time_s']) * 1e9) for f in frames]
    if min(timestamps) < 0 or any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        raise ValueError('Capture timestamps must increase strictly')
    odometry = None
    if args.odometry:
        packets = []
        for line in args.odometry.read_text().splitlines():
            packet = json.loads(line)
            if 'reset' in packet:
                if packets:
                    raise ValueError('Split odometry at MAPRESET before replay')
            else:
                packets.append(packet)
        times_odom = np.array([p['time_s'] for p in packets])
        if not len(packets) or not np.isfinite(times_odom).all() or np.any(np.diff(times_odom)<=0):
            raise ValueError('Odometry must have strictly increasing timestamps')
        odometry = []
        for frame in frames:
            nearest = int(np.argmin(np.abs(times_odom-frame['time_s'])))
            packet = packets[nearest]
            if abs(packet['time_s']-frame['time_s'])>.02 or not packet['valid'] or 'body_from_camera' not in frame:
                raise ValueError('Missing synchronized valid odometry or camera mount')
            odometry.append(packet)
    # Refuse overwrite: save_map overwrites its target database.
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = json.loads((ROOT / 'technology/scan3d_fusion/config/sim_calibration.json').read_text())['camera_to_tof']
    slam = RgbdSlam(sequence['image_size'], sequence['intrinsics'])
    records, observations, times = [], [], []
    failure = None
    loops = 0
    fx, fy, cx, cy = sequence['intrinsics']
    for index, (frame, timestamp) in enumerate(zip(frames, timestamps)):
        with h5py.File(args.capture / frame['file']) as sample:
            rgb = sample['rgb'][:]
            depth = np.asarray(Image.open(args.prediction / (Path(frame['file']).stem + '_depth.png')), np.float32) / 1000
            depth = tof_support(depth, sample['hist_data'][:], sample['fr'][:], sample['mask'][:],
                                sequence['intrinsics'], calibration)
        depth_mm = np.rint(np.clip(depth, 0, 65.535) * 1000).astype(np.uint16)
        start = time.perf_counter()
        pose = slam.track(timestamp, rgb, depth_mm)
        times.append(time.perf_counter() - start)
        if pose is None:
            failure = {'index': index, 'file': frame['file'], 'reason': 'tracking lost; no pose substitution'}
            print(json.dumps(failure), flush=True)
            break  # A resumed tracker may start a different map frame.
        record = {'file': frame['file'], 'time_s': frame['time_s'], 'timestamp_ns': timestamp,
                  'camera_to_world': pose.tolist()}
        if 'body_from_camera' in frame:
            record['body_from_camera'] = frame['body_from_camera']
        records.append(record)
        y, x = np.mgrid[0:depth.shape[0]:12, 0:depth.shape[1]:12]
        z = depth[::12, ::12]
        valid = (z > .02) & (z <= 4.)
        points = np.stack(((x-cx)*z/fx, (y-cy)*z/fy, z), -1)[valid]
        observations.append(points)
        metrics = slam.tracker.get_slam_metrics()
        if metrics and metrics.lc_status:
            loops += 1
        if index % 24 == 0:
            print(f'track {index+1}/{len(frames)}', flush=True)

    # Recalculate all past poses after the final pose graph optimization.
    optimized = {p.timestamp_ns: pose_matrix(p.pose) for p in slam.tracker.get_all_slam_poses()}
    cloud = []
    localization = BodyLocalization()
    visual_rejected = 0
    for index, (record, points) in enumerate(zip(records, observations)):
        pose = optimized.get(record['timestamp_ns'])
        if pose is None:
            raise RuntimeError('Missing final optimized pose; refuse a mixed trajectory')
        if odometry is not None:
            record['visual_camera_to_world'] = pose.tolist()
            fused = localization.update(record['timestamp_ns'], odometry[index], record['body_from_camera'], pose)
            if fused is None or fused['frame'] != 'map':
                raise RuntimeError('Invalid fused localization')
            visual_rejected += not fused['visual_accepted']
            record['localization'] = fused
            pose = np.asarray(fused['world_from_camera'])
        record['camera_to_world'] = pose.tolist()
        if 'body_from_camera' in record:
            record['body_to_world'] = body_pose(pose, record['body_from_camera']).tolist()
        cloud.append(points @ pose[:3, :3].T + pose[:3, 3])
    result = {'pose_source': 'cuVSLAM 17.0.1 RGBD + SLAM, final optimized poses; no ground truth',
              'coordinate_frame': 'initial camera origin; X forward, Y left, Z up if initial camera level',
              'depth_source': 'DELTAR with ToF support; measured input only',
              'intrinsics': sequence['intrinsics'], 'image_size': sequence['image_size'],
              'frames': records}
    if odometry is not None:
        result['pose_source'] += '; wheel/IMU odometry with external visual map correction'
    saved = []
    if records:
        points = np.concatenate(cloud)
        result['bounds_m'] = [np.min(points, axis=0).tolist(), (np.max(points, axis=0)+.02).tolist()]
        slam.tracker.save_map(str(args.output / 'map'), saved.append)
        if saved != [True]:
            raise RuntimeError('cuVSLAM map save failed')
    report = {'input_frames': len(frames), 'tracked_frames': len(records), 'failure': failure,
              'complete': len(records) == len(frames), 'map_saved': saved == [True],
              'odometry_file': str(args.odometry) if args.odometry else None,
              'visual_corrections_rejected': visual_rejected,
              'loop_closure_success_frames': loops,
              'tracker_ms_median': float(np.median(times)*1000),
              'tracker_ms_p95': float(np.percentile(times, 95)*1000),
              'input_interval_s_median': float(np.median(np.diff(timestamps))/1e9),
              'mode': 'synchronous offline, CUDA; timing excludes DELTAR and ToF preprocessing'}
    (args.output / 'sequence.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.output / 'tracking.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    sys.exit(main())
