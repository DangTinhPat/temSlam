"""Conservative labels after fusion; never changes measured depth.

Coordinates use an IMU-aligned, Z-up odometry frame whose floor is near z=0.
This flat-floor prior comes from wheel/leg calibration, not simulator geometry.
"""
from collections import deque

import numpy as np
from scipy.ndimage import binary_erosion, maximum_filter, minimum_filter

UNCERTAIN, GROUND, OBSTACLE = 0, 1, 2
SOURCE = 'gravity_ransac_labels_v3'
FIELDS = ('ground_labels', 'ground_plane_camera', 'ground_confirmations', 'ground_source')


class GroundSegmenter:
    def __init__(self):
        self.history = deque(maxlen=3)
        self.last_time = None
        self.rng = np.random.default_rng(51)

    def classify(self, depth, intrinsics, world_from_camera, timestamp):
        depth = np.asarray(depth)
        K = np.asarray(intrinsics, float)
        pose = np.asarray(world_from_camera, float)
        if (depth.ndim!=2 or min(depth.shape)<5 or K.shape!=(4,)
                or not np.isfinite(K).all() or min(K[:2])<=0
                or pose.shape!=(4, 4) or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3), atol=1e-5)
                or np.linalg.det(pose[:3, :3])<.999 or not np.isfinite(timestamp)
                or (self.last_time is not None and timestamp<=self.last_time)):
            raise ValueError('Require depth, intrinsics, rigid Z-up pose and increasing timestamps')
        if self.last_time is not None and timestamp-self.last_time>1.:
            self.history.clear()
        self.last_time = timestamp
        labels = np.zeros(depth.shape, np.uint8)
        result = dict(ground_labels=labels, ground_plane_camera=np.full(4, np.nan),
                      ground_confirmations=0, ground_source=SOURCE)
        valid = np.isfinite(depth) & (depth>.02) & (depth<=4.)
        y, x = np.indices(depth.shape)
        points = np.stack(((x-K[2])/K[0], (y-K[3])/K[1], np.ones_like(x)), -1)
        points *= np.where(valid, depth, 0.)[..., None]
        up = pose[:3, :3].T @ [0., 0., 1.]
        height = points@up+pose[2, 3]
        # Fit a measured plane near the calibrated floor, never a table top.
        candidates = valid & (np.abs(height)<.07)
        sample = points[::4, ::4][candidates[::4, ::4]]
        if len(sample)>2000:
            sample = sample[self.rng.choice(len(sample), 2000, replace=False)]
        best = None
        if len(sample)>=60:
            for _ in range(64):
                a, b, c = sample[self.rng.choice(len(sample), 3, replace=False)]
                normal = np.cross(b-a, c-a)
                norm = np.linalg.norm(normal)
                if norm<1e-8:
                    continue
                normal /= norm
                if normal@up<0:
                    normal = -normal
                distance = -normal@a
                if normal@up<np.cos(np.deg2rad(8)) or abs(distance-(pose[:3, :3]@normal)@pose[:3, 3])>.01:
                    continue
                inside = np.abs(sample@normal+distance)<.012
                if best is None or inside.sum()>best.sum():
                    best = inside
        if best is None or best.sum()<max(60, .4*len(sample)):
            self.history.clear()
            return result
        subset = sample[best]
        center = subset.mean(0)
        _, singular, vh = np.linalg.svd(subset-center, full_matrices=False)
        normal = vh[-1]
        if normal@up<0:
            normal = -normal
        distance = -normal@center
        world_normal = pose[:3, :3]@normal
        world_distance = distance-world_normal@pose[:3, 3]
        if (singular[1]/np.sqrt(len(subset))<.08
                or normal@up<np.cos(np.deg2rad(8)) or abs(world_distance)>.01):
            self.history.clear()
            return result
        plane = np.r_[world_normal, world_distance]
        if self.history and (plane[:3]@self.history[-1][:3]<np.cos(np.deg2rad(5))
                             or abs(plane[3]-self.history[-1][3])>.02):
            self.history.clear()
        self.history.append(plane)
        result['ground_confirmations'] = len(self.history)
        if len(self.history)<3:
            return result
        plane = np.mean(self.history, axis=0)
        plane /= np.linalg.norm(plane[:3])
        normal = pose[:3, :3].T@plane[:3]
        distance = plane[3]+plane[:3]@pose[:3, 3]
        result['ground_plane_camera'] = np.r_[normal, distance]
        residual = points@normal+distance
        dy, dx = np.gradient(points, axis=(0, 1))
        normals = np.cross(dx, dy)
        alignment = np.abs(normals@normal)/np.maximum(np.linalg.norm(normals, axis=2), 1e-10)
        local = np.where(valid, residual, 10.)
        smooth = maximum_filter(local, size=3)-minimum_filter(local, size=3)<.02
        # A large low object can tilt a mixed RANSAC plane; the calibrated flat-floor
        # height gate prevents that fit from swallowing the object farther from the origin.
        ground = valid & (np.abs(height)<=.012) & (np.abs(residual)<=.012)
        ground &= (alignment>=np.cos(np.deg2rad(25))) & smooth
        labels[binary_erosion(ground)] = GROUND
        # shortcut: geometry cannot distinguish sub-centimeter bumps from depth noise.
        # Keep the 1.2–2.5cm band uncertain; never increase floor tolerance to hide roughness.
        labels[valid & ((residual>=.025) | (height>=.025))] = OBSTACLE
        return result


def read_labels(sample, depth_shape):
    """Reject partial/stale label formats rather than silently reverting to height cuts."""
    present = [key in sample for key in FIELDS]
    if not any(present):
        return None
    if not all(present):
        raise ValueError('Incomplete ground labels')
    labels = np.asarray(sample['ground_labels'])
    plane = np.asarray(sample['ground_plane_camera'])
    confirmations = np.asarray(sample['ground_confirmations'])
    if (str(sample['ground_source'])!=SOURCE or labels.shape!=depth_shape
            or labels.dtype!=np.uint8 or not np.isin(labels, [UNCERTAIN, GROUND, OBSTACLE]).all()
            or plane.shape!=(4,) or confirmations.shape!=() or confirmations.dtype.kind not in 'iu'
            or confirmations not in (0, 1, 2, 3)
            or (np.any(labels!=UNCERTAIN) and (confirmations!=3 or not np.isfinite(plane).all()
                                              or abs(np.linalg.norm(plane[:3])-1)>.001))):
        raise ValueError('Invalid ground classification provenance, labels or plane')
    return labels
