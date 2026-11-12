"""Legacy plane-depth correction experiment; disabled in the fusion-only pipeline."""
import numpy as np
from scipy.ndimage import binary_erosion, gaussian_filter, label, maximum_filter, minimum_filter


class FloorSupport:
    def __init__(self):
        self.previous = None
        self.confirmations = 0
        self.rng = np.random.default_rng(51)

    def apply(self, depth, K, odom_from_camera):
        depth = np.asarray(depth, dtype=np.float32)
        K = np.asarray(K, float)
        pose = np.asarray(odom_from_camera, float)
        if (depth.ndim != 2 or K.shape != (4,) or pose.shape != (4, 4)
                or not np.isfinite(K).all() or min(K[:2]) <= 0
                or not np.isfinite(pose).all()):
            raise ValueError('Invalid floor depth, intrinsics or camera pose')
        h, w = depth.shape
        y, x = np.indices((h, w))
        rays = np.stack(((x-K[2])/K[0], (y-K[3])/K[1], np.ones_like(x)), -1)
        points = rays * depth[..., None]
        up = pose[:3, :3].T @ [0., 0., 1.]
        height = float(pose[2, 3])
        valid = np.isfinite(depth) & (depth > .02) & (depth <= 4)
        candidates = valid & (y > h*.55) & (np.abs(points @ up + height) < .10)
        sample = points[::8, ::8][candidates[::8, ::8]]
        best = None
        if len(sample) >= 80 and .10 < height < .5:
            for _ in range(80):
                a, b, c = sample[self.rng.choice(len(sample), 3, replace=False)]
                n = np.cross(b-a, c-a)
                norm = np.linalg.norm(n)
                if norm < 1e-6:
                    continue
                n /= norm
                if n @ up < 0:
                    n = -n
                distance = -n @ a
                if n @ up < np.cos(np.deg2rad(12)) or abs(distance-height) > .06:
                    continue
                inside = np.abs(sample @ n + distance) < .02
                if best is None or inside.sum() > best.sum():
                    best = inside
        if best is None or best.sum() < max(80, .45*len(sample)):
            self.previous = None
            self.confirmations = 0
            return depth.copy(), None, 0
        subset = sample[best]
        center = subset.mean(0)
        _, singular, vh = np.linalg.svd(subset-center, full_matrices=False)
        n = vh[-1]
        if n @ up < 0:
            n = -n
        distance = float(-n @ center)
        if (singular[1]/np.sqrt(len(subset)) < .10 or
                n @ up < np.cos(np.deg2rad(12)) or abs(distance-height) > .06):
            self.previous = None
            self.confirmations = 0
            return depth.copy(), None, 0
        normal_odom = pose[:3, :3] @ n
        consistent = (self.previous is not None and
                      normal_odom @ self.previous[:3] > np.cos(np.deg2rad(5)) and
                      abs(distance-self.previous[3]) < .04)
        self.confirmations = self.confirmations+1 if consistent else 1
        self.previous = np.r_[normal_odom, distance]
        if self.confirmations < 3:
            return depth.copy(), None, 0
        # Depth confirms floor presence; calibrated IMU/leg geometry supplies one consistent level plane.
        n, distance = up, height
        residual = points @ n + distance
        # Follow smooth floor-connected regions; raised islands and vertical edges are excluded.
        smooth = rays * gaussian_filter(np.where(valid, depth, 0.), 2)[..., None]
        dy, dx = np.gradient(smooth, axis=(0, 1))
        normals = np.cross(dx, dy)
        alignment = np.abs(normals @ n) / np.maximum(np.linalg.norm(normals, axis=2), 1e-9)
        region = valid & (y > h*.5) & (np.abs(residual) < .12)
        region &= alignment > np.cos(np.deg2rad(40))
        local = np.where(valid, residual, 1.)
        region &= maximum_filter(local, size=5)-minimum_filter(local, size=5) < .025
        region = binary_erosion(region, iterations=2)
        components, _ = label(region)
        seed = region & (y > h*.75) & (np.abs(residual) < .025)
        seed_counts = np.bincount(components[seed], minlength=components.max()+1)
        sizes = np.bincount(components.ravel())
        supported = (seed_counts >= 100) & (sizes >= 500)
        supported[0] = False
        # shortcut: a shallow bump connected smoothly to the floor needs semantic/temporal object evidence.
        mask = supported[components]
        denominator = rays @ n
        mask &= denominator < -.05
        out = depth.copy()
        out[mask] = -distance / denominator[mask]
        return out, np.r_[n, distance], int(mask.sum())
