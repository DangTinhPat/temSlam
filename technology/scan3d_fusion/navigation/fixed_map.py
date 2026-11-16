"""Immutable measured-surface reference and wheel/IMU-assisted XY/yaw localization."""
import json
from collections import deque
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live.workers import fusion_depth
from mapping.odometry.localization import odom_body, rigid
from reconstruction.surface_matching import planar_change, vertical_points


def _depth_and_intrinsics(depth, intrinsics):
    depth = np.asarray(depth)
    intrinsics = np.asarray(intrinsics, dtype=float)
    if depth.ndim != 2 or min(depth.shape) < 16 or depth.dtype != np.uint16:
        raise ValueError('Require a 2D uint16 millimeter depth image at least 16 by 16')
    if intrinsics.shape != (4,) or not np.isfinite(intrinsics).all() or min(intrinsics[:2]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    return depth, intrinsics


class FixedReference:
    def __init__(self, points, normals):
        points, normals = np.asarray(points, dtype=float), np.asarray(normals, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or points.shape != normals.shape
                or len(points) < 100 or not np.isfinite(points).all() or not np.isfinite(normals).all()):
            raise ValueError('Reference requires at least 100 finite points and normals')
        lengths = np.linalg.norm(normals, axis=1)
        if np.any(lengths < 1e-9):
            raise ValueError('Reference normals must be nonzero')
        _, indices = np.unique(np.floor(points/.04).astype(np.int64), axis=0, return_index=True)
        # Bytes-backed arrays keep the reference immutable even if a caller toggles write flags.
        self.points = np.frombuffer(points[indices].tobytes(), dtype=float).reshape(-1, 3)
        self.normals = np.frombuffer((normals[indices]/lengths[indices, None]).tobytes(), dtype=float).reshape(-1, 3)
        if len(self.points) < 100:
            raise ValueError('Reference has fewer than 100 distinct surface voxels')
        self.tree = cKDTree(self.points)

    @classmethod
    def from_session(cls, session):
        session = Path(session)
        sequence_path = session/'surface_sequence.json'
        if not sequence_path.exists():
            sequence_path = session/'live_sequence.json'
        sequence = json.loads(sequence_path.read_text())
        points, normals = np.empty((0, 3)), np.empty((0, 3))
        frames = sequence['frames']
        for start in range(0, len(frames), 64):
            point_batches, normal_batches = [points], [normals]
            for frame in frames[start:start+64]:
                pose = rigid(frame['camera_to_world'])
                with np.load(session/frame['file'], allow_pickle=False) as sample:
                    fusion_depth(sample)
                    depth, intrinsics = _depth_and_intrinsics(sample['depth'], sequence['intrinsics'])
                    p, n = vertical_points(depth, intrinsics, pose)
                point_batches.append(p)
                normal_batches.append(n)
            points, normals = np.concatenate(point_batches), np.concatenate(normal_batches)
            _, indices = np.unique(np.floor(points/.04).astype(np.int64), axis=0, return_index=True)
            points, normals = points[indices], normals[indices]
        return cls(points, normals)


class FixedMapLocalizer:
    # shortcut: conservative fixed gates; calibrate with synchronized hardware logs before control.
    gap_seconds = .5
    loss_seconds = 1.

    def __init__(self, reference, initial_pose):
        initial_pose = np.asarray(initial_pose, dtype=float)
        if not isinstance(reference, FixedReference):
            raise ValueError('Require a fixed surface reference')
        if initial_pose.shape != (3,) or not np.isfinite(initial_pose).all():
            raise ValueError('Initial body pose must be finite x, y, yaw in radians')
        self.reference = reference
        self.initial_pose = initial_pose.copy()
        self.map_from_odom = None
        self.last_timestamp = None
        self.last_match_timestamp = None
        self.start_timestamp = None
        self.confirmations = 0
        self.state = 'initializing'
        self.views = deque(maxlen=16)
        self.pending_map_from_odom = None

    def _correspondences(self, points, normals, delta, origin):
        change = planar_change(delta, origin)
        q = points @ change[:3, :3].T + change[:3, 3]
        qn = normals @ change[:3, :3].T
        distances, indices = self.reference.tree.query(q)
        n = self.reference.normals[indices]
        residual = np.sum(n*(q-self.reference.points[indices]), axis=1)
        valid = ((distances < .2) & (np.abs(residual) < .1)
                 & (np.abs(np.sum(n*qn, axis=1)) > .85))
        n, residual, offset = n[valid], residual[valid], q[valid]-origin
        jacobian = np.column_stack((n[:, :2], -n[:, 0]*offset[:, 1]+n[:, 1]*offset[:, 0]))
        return jacobian, residual

    def _match(self, points, normals, origin):
        delta = np.zeros(3)
        metrics = dict(matches=0, overlap=0., rmse_m=None, geometry_min_eigenvalue=None,
                       correction_translation_m=0., correction_yaw_rad=0., delta=delta.tolist())
        if len(points) < 100:
            return np.eye(4), dict(metrics, accepted=False, reason='insufficient_surface')
        for _ in range(12):
            jacobian, residual = self._correspondences(points, normals, delta, origin)
            if len(residual) < 100:
                break
            weights = np.minimum(1., .025/np.maximum(np.abs(residual), 1e-9))
            prior = np.diag([2., 2., 5.])
            step = np.linalg.solve(jacobian.T @ (weights[:, None]*jacobian)+prior,
                                   -jacobian.T @ (weights*residual)-prior @ delta)
            delta += step
            if np.linalg.norm(delta[:2]) > .2 or abs(delta[2]) > .12 or np.linalg.norm(step) < 1e-4:
                break
        # Acceptance is based on the final pose, never an earlier successful iteration.
        jacobian, residual = self._correspondences(points, normals, delta, origin)
        count = len(residual)
        metrics.update(matches=count, overlap=count/len(points), delta=delta.tolist(),
                       correction_translation_m=float(np.linalg.norm(delta[:2])),
                       correction_yaw_rad=float(abs(delta[2])))
        if count:
            metrics['rmse_m'] = float(np.sqrt(np.mean(residual**2)))
            metrics['geometry_min_eigenvalue'] = float(np.linalg.eigvalsh(jacobian.T @ jacobian/count)[0])
        if metrics['correction_translation_m'] > .2 or metrics['correction_yaw_rad'] > .12:
            reason = 'correction_jump'
        elif count < 100 or metrics['overlap'] < .35:
            reason = 'overlap'
        elif metrics['rmse_m'] > .04:
            reason = 'residual'
        elif metrics['geometry_min_eigenvalue'] < .005:
            reason = 'geometry'
        else:
            reason = 'matched'
        accepted = reason == 'matched'
        return (planar_change(delta, origin) if accepted else np.eye(4)), dict(metrics, accepted=accepted, reason=reason)

    def update(self, timestamp_ns, packet, body_from_camera, depth_mm, intrinsics):
        try:
            if (isinstance(timestamp_ns, (bool, np.bool_)) or not isinstance(timestamp_ns, (int, np.integer))
                    or timestamp_ns < 0 or (self.last_timestamp is not None and timestamp_ns <= self.last_timestamp)
                    or abs(timestamp_ns*1e-9-packet['time_s']) > .02):
                raise ValueError('Require increasing timestamps and odometry within 20 ms')
            mount = rigid(body_from_camera)
            local = odom_body(packet)
            depth, intrinsics = _depth_and_intrinsics(depth_mm, intrinsics)
        except (KeyError, TypeError, OverflowError) as error:
            raise ValueError('Invalid localization input') from error
        timestamp_ns = int(timestamp_ns)
        gap = self.last_timestamp is not None and (timestamp_ns-self.last_timestamp)*1e-9 > self.gap_seconds
        self.last_timestamp = timestamp_ns
        if self.start_timestamp is None:
            self.start_timestamp = timestamp_ns
        if gap or not packet['valid']:
            self.state, self.confirmations = 'lost', 0
            self.views.clear()
            self.pending_map_from_odom = None
        result = dict(timestamp_ns=timestamp_ns, state=self.state, usable=False,
                      world_from_body=None, world_from_camera=None, accepted=False,
                      reason='invalid_odometry', matches=0, overlap=0., rmse_m=None,
                      geometry_min_eigenvalue=None, delta=[0., 0., 0.],
                      correction_translation_m=0., correction_yaw_rad=0., window_views=0,
                      consistent_views=0, worst_view_rmse_m=None, min_view_overlap=None,
                      current_matches=0, current_overlap=0., current_rmse_m=None)
        if not packet['valid']:
            return result
        if self.map_from_odom is None:
            odom_yaw = np.arctan2(local[1, 0], local[0, 0])
            self.map_from_odom = planar_change([0, 0, self.initial_pose[2]-odom_yaw], np.zeros(3))
            self.map_from_odom[:2, 3] = self.initial_pose[:2]-(self.map_from_odom @ local)[:2, 3]
        body = self.map_from_odom @ local
        camera = body @ mount
        # A short head sweep adds geometry; odometry coordinates avoid feeding earlier map corrections back in.
        while self.views and (timestamp_ns-self.views[0][0])*1e-9 > 3.:
            self.views.popleft()
        points, normals = vertical_points(depth, intrinsics, local @ mount)
        if len(points) >= 100:
            self.views.append((timestamp_ns, points, normals))
            world_views = [(p @ self.map_from_odom[:3, :3].T+self.map_from_odom[:3, 3],
                            n @ self.map_from_odom[:3, :3].T) for _, p, n in self.views]
            pooled_points = np.concatenate([p for p, _ in world_views])
            pooled_normals = np.concatenate([n for _, n in world_views])
            change, metrics = self._match(pooled_points, pooled_normals, body[:3, 3])
            if metrics['accepted']:
                overlaps, residuals, consistent = [], [], 0
                for index, (p, n) in enumerate(world_views):
                    _, residual = self._correspondences(p, n, metrics['delta'], body[:3, 3])
                    overlap = len(residual)/len(p)
                    rmse = float(np.sqrt(np.mean(residual**2))) if len(residual) else None
                    overlaps.append(overlap)
                    if rmse is not None:
                        residuals.append(rmse)
                    reason = ('overlap' if len(residual) < 100 or overlap < .35 else
                              'residual' if rmse > .04 else None)
                    if reason is None:
                        consistent += 1
                    else:
                        metrics.update(accepted=False, reason=('current_' if index == len(world_views)-1 else 'history_')+reason)
                    if index == len(world_views)-1:
                        result.update(current_matches=len(residual), current_overlap=overlap, current_rmse_m=rmse)
                result.update(consistent_views=consistent, min_view_overlap=min(overlaps),
                              worst_view_rmse_m=max(residuals) if residuals else None)
        else:
            # Old views cannot stand in for a missing current measurement.
            change, metrics = self._match(points, normals, body[:3, 3])
        result['window_views'] = len(self.views)
        if metrics['accepted']:
            candidate = change @ self.map_from_odom
            consistent = False
            if self.pending_map_from_odom is not None:
                difference = candidate @ np.linalg.inv(self.pending_map_from_odom)
                consistent = (np.linalg.norm((candidate @ local)[:2, 3]-(self.pending_map_from_odom @ local)[:2, 3]) <= .05
                              and abs(np.arctan2(difference[1, 0], difference[0, 0])) <= .04)
            self.confirmations = self.confirmations+1 if consistent else 1
            self.pending_map_from_odom = candidate
            if self.confirmations >= 2:
                self.map_from_odom = candidate
                body, camera = change @ body, change @ camera
                self.last_match_timestamp = timestamp_ns
                self.state = 'tracking'
            else:
                metrics.update(accepted=False, reason='confirming_match')
        else:
            self.confirmations = 0
            self.pending_map_from_odom = None
        if not metrics['accepted']:
            last_match = self.start_timestamp if self.last_match_timestamp is None else self.last_match_timestamp
            if (timestamp_ns-last_match)*1e-9 > self.loss_seconds:
                self.state = 'lost'
            elif self.state == 'tracking':
                self.state = 'degraded'
        result.update(metrics)
        if gap:
            result['reason'] = 'timestamp_gap'
        result.update(state=self.state, usable=self.state == 'tracking',
                      world_from_body=body.tolist(), world_from_camera=camera.tolist())
        return result
