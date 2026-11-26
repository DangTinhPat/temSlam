"""Verify a short RGB-D match before rebasing visual tracking."""
from collections import deque

import numpy as np

from .tracker import local_motion_error


class VisualRecovery:
    def __init__(self):
        self.anchors = deque(maxlen=6)
        self.next_search = 0.
        self.stats = {}
        self.reference = None
        self.sdk_to_local = np.eye(4)
        self.pending_alignment = None

    def candidate(self, raw_pose, sensor_pose, rgb, depth, intrinsics, guard, timestamp=None):
        self.stats = {}
        if raw_pose is None:
            self.pending_alignment = None
            return None, False
        visual = self.sdk_to_local @ raw_pose
        if self.reference is None:
            return visual, False

        last_visual, last_sensor = guard.accepted
        error = guard.disagreement(visual, sensor_pose)
        if guard.trusted(error) and not guard.lost:
            self.pending_alignment = None
            return visual, False
        if self.pending_alignment is not None and guard.good_frames < 2:
            candidate = self.pending_alignment @ raw_pose
            metres, radians = local_motion_error(last_visual, candidate, last_sensor, sensor_pose)
            if metres <= .04 and radians <= .06:
                return candidate, True
            self.pending_alignment = None
        if guard.lost and not guard.good_frames and timestamp is not None:
            if timestamp < self.next_search:
                return visual, False
            self.next_search = timestamp + .5
        candidates = [(last_visual, last_sensor, *self.reference)]
        if guard.hold_since is not None:
            nearest = sorted(self.anchors, key=lambda a: np.linalg.norm(a[1][:3, 3]-sensor_pose[:3, 3]))
            candidates.extend(nearest[:3])
        for anchor, odom, old_rgb, old_depth in candidates:
            sensor_motion = np.linalg.solve(odom, sensor_pose)
            delta = match_rgbd(old_rgb, old_depth, rgb, depth, intrinsics, sensor_motion, self.stats)
            if delta is None:
                continue
            matched = anchor @ delta
            metres, radians = local_motion_error(last_visual, matched, last_sensor, sensor_pose)
            if metres <= .04 and radians <= .06 and guard.trusted(guard.disagreement(matched, sensor_pose)):
                break
        else:
            return visual, False
        self.pending_alignment = matched @ np.linalg.inv(raw_pose)
        # Seed from the first jump while the views still overlap, but let the guard enter hold.
        if guard.hold_since is None:
            return visual, False
        return matched, True

    def remember_keyframe(self, pose, sensor, rgb, depth):
        if self.anchors:
            translation, angle = local_motion_error(self.anchors[-1][1], sensor, np.eye(4), np.eye(4))
            if translation < .12 and angle < .2:
                return
        self.anchors.append((pose, sensor, rgb, depth))

    def accept(self, rgb, depth, matched):
        if matched:
            self.sdk_to_local = self.pending_alignment
        self.pending_alignment = None
        self.reference = rgb, depth


def match_rgbd(previous_rgb, previous_depth, rgb, depth, intrinsics, sensor_motion, stats=None):
    """Return previous_camera_from_current_camera from mutual flow and depth."""
    import cv2

    old = cv2.cvtColor(previous_rgb, cv2.COLOR_RGB2GRAY)
    new = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    valid = ((previous_depth >= 100) & (previous_depth <= 4000)).astype(np.uint8)
    start = cv2.goodFeaturesToTrack(old, 300, .003, 6, mask=valid)
    if stats is not None:
        stats.update(features=0 if start is None else len(start), flow_pairs=0, inliers=0, depth_inliers=0)
    if start is None or len(start) < 20:
        return None
    end, ok, _ = cv2.calcOpticalFlowPyrLK(old, new, start, None)
    if end is None or ok is None:
        return None
    back, ok_back, _ = cv2.calcOpticalFlowPyrLK(new, old, end, None)
    if back is None or ok_back is None:
        return None
    x, y = start[:, 0, :].T
    u, v = end[:, 0, :].T
    height, width = new.shape
    good = (ok.ravel().astype(bool) & ok_back.ravel().astype(bool)
            & (np.linalg.norm(back-start, axis=(1, 2)) < 1)
            & (u >= 0) & (u < width-1) & (v >= 0) & (v < height-1))
    x, y, u, v = (values[good] for values in (x, y, u, v))
    if stats is not None:
        stats["flow_pairs"] = len(x)
    if len(x) < 20:
        return None
    z = previous_depth[np.rint(y).astype(int), np.rint(x).astype(int)].astype(float)*.001
    fx, fy, cx, cy = intrinsics
    points = np.column_stack(((x-cx)*z/fx, (y-cy)*z/fy, z))
    pixels = np.column_stack((u, v))
    K = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
    success, rotation, translation, inliers = cv2.solvePnPRansac(
        points, pixels, K, None, iterationsCount=60, reprojectionError=2.,
        flags=cv2.SOLVEPNP_EPNP)
    if not success or inliers is None or len(inliers) < max(20, .70*len(points)):
        # Reject ambiguous flow after a large image gap, then fit pose to the remaining observations.
        predicted = (points - sensor_motion[:3, 3]) @ sensor_motion[:3, :3]
        projected = predicted @ K.T
        offsets = projected[:, :2] / np.maximum(projected[:, 2:], .02) - pixels
        support = (predicted[:, 2] > .02) & (np.linalg.norm(offsets, axis=1) <= 5)
        if np.count_nonzero(support) < 20:
            return None
        points, pixels, u, v = points[support], pixels[support], u[support], v[support]
        success, rotation, translation, inliers = cv2.solvePnPRansac(
            points, pixels, K, None, iterationsCount=60, reprojectionError=3.,
            flags=cv2.SOLVEPNP_AP3P)
    if not success or inliers is None or len(inliers) < max(20, .70*len(points)):
        return None
    if stats is not None:
        stats['inliers'] = len(inliers)
    ids = inliers.ravel()
    # A match concentrated in a small patch gives weak geometry.
    hull = cv2.convexHull(pixels[ids].astype(np.float32))
    if cv2.contourArea(hull) < .015*width*height:
        return None
    rotation, translation = cv2.solvePnPRefineLM(points[ids], pixels[ids], K, None, rotation, translation)
    current = np.eye(4)
    current[:3, :3] = cv2.Rodrigues(rotation)[0]
    current[:3, 3] = translation.ravel()
    current_z = (points[ids]@current[:3, :3].T+current[:3, 3])[:, 2]
    measured = depth[np.rint(v[ids]).astype(int), np.rint(u[ids]).astype(int)]*.001
    overlap = (measured >= .1) & (measured <= 4.) & (current_z > 0.)
    supported = overlap & (np.abs(current_z-measured) <= .08)
    if supported.sum() < max(20, .6*overlap.sum()) or overlap.sum() < .5*len(ids):
        return None
    ids = ids[supported]
    if cv2.contourArea(cv2.convexHull(pixels[ids].astype(np.float32))) < .015*width*height:
        return None
    rotation, translation = cv2.solvePnPRefineLM(points[ids], pixels[ids], K, None, rotation, translation)
    current[:3, :3] = cv2.Rodrigues(rotation)[0]
    current[:3, 3] = translation.ravel()
    projected = points[ids] @ current[:3, :3].T + current[:3, 3]
    uv = projected @ K.T
    if (np.any(projected[:, 2] <= 0.)
            or np.percentile(np.abs(projected[:, 2]-measured[supported]), 90) > .08
            or np.percentile(np.linalg.norm(uv[:, :2]/uv[:, 2:] - pixels[ids], axis=1), 90) > 3.):
        return None
    if stats is not None:
        stats['depth_inliers'] = len(ids)
    return np.linalg.inv(current)
