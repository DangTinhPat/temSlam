"""Pure bounded navigation commands; callers own actuation and the stop watchdog.

The frozen inflated map protects the circular footprint, including turns and
near/rear blind areas. Fresh depth checks only the forward region. This is an
experimental static-room assumption, not all-around dynamic collision safety.
"""

import math

import numpy as np


def _angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _fresh(stamp, now, age):
    return math.isfinite(stamp) and 0 <= now-stamp <= age


class Navigator:
    """Forward-only path follower, latched until a new explicit goal after stops."""

    def __init__(self, grid):
        if not math.isfinite(grid.clearance_m) or grid.clearance_m < .25:
            raise ValueError('Navigation requires at least .25m map footprint clearance')
        self.grid = grid
        self.path, self.goal = [], None
        self.state, self.reason = 'idle', ''
        self.v = self.w = 0.
        self._goal_yaw = None
        self._index = 0
        self._last_now = self._progress_at = None
        self._best_distance = self._best_angle = math.inf

    def _result(self):
        return dict(state=self.state, reason=self.reason, v=self.v, w=self.w,
                    path=[point[:] for point in self.path],
                    goal=None if self.goal is None else self.goal[:])

    def _stop(self, state, reason):
        # Safety stops bypass the comfort ramp; a stale command must never linger.
        self.state, self.reason = state, reason
        self.v = self.w = 0.
        return self._result()

    def set_goal(self, start_xy, goal_xy, goal_yaw=None):
        self._stop('blocked', 'invalid goal')
        self.path, self.goal = [], None
        try:
            start, goal = np.asarray(start_xy, float), np.asarray(goal_xy, float)
            if (start.shape != (2,) or goal.shape != (2,) or
                    not np.isfinite(start).all() or not np.isfinite(goal).all() or
                    (goal_yaw is not None and not math.isfinite(goal_yaw))):
                raise ValueError('goal and start must be finite XY, yaw must be finite')
        except (TypeError, ValueError) as exc:
            raise ValueError('goal and start must be finite XY, yaw must be finite') from exc
        self.goal = goal.tolist()
        self._goal_yaw = None if goal_yaw is None else _angle(goal_yaw)
        self._index = 0
        self._last_now = self._progress_at = None
        self._best_distance = self._best_angle = math.inf
        try:
            self.path = [list(point) for point in self.grid.plan(start, goal)]
        except ValueError as exc:
            return self._stop('blocked', str(exc))
        self.state, self.reason = 'planning', ''
        return self._result()

    def cancel(self):
        return self._stop('cancelled', 'cancelled by operator')

    def update(self, pose, now, obstacles, dt):
        if self.state not in ('planning', 'driving', 'aligning'):
            return self._result()
        try:
            if (not math.isfinite(now) or not math.isfinite(dt) or not 0 < dt <= .3 or
                    (self._last_now is not None and
                     not 0 <= now-self._last_now <= .3+1e-9)):
                return self._stop('lost', 'invalid control clock or update gap')
            xy, yaw = np.asarray(pose['xy'], float), float(pose['yaw'])
            uncertainty = float(pose['uncertainty_m'])
            if (xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(yaw) or
                    pose['usable'] is not True or not math.isfinite(uncertainty) or
                    not 0 <= uncertainty <= .05 or not _fresh(pose['stamp'], now, .3)):
                return self._stop('lost', 'pose invalid, stale or uncertain')
        except (KeyError, TypeError, ValueError, OverflowError):
            return self._stop('lost', 'pose missing or invalid')
        self._last_now = now
        try:
            if obstacles['clear'] is not True or not _fresh(obstacles['stamp'], now, .5):
                return self._stop('blocked', str(obstacles.get('reason', 'depth blocked or stale')))
        except (KeyError, TypeError, ValueError):
            return self._stop('blocked', 'depth missing or invalid')
        if not self.grid.segment_clear(xy, xy):
            return self._stop('blocked', 'pose outside free map footprint')

        distance = float(np.linalg.norm(np.asarray(self.goal)-xy))
        if distance <= .025:
            if self._goal_yaw is None:
                return self._stop('reached', 'goal reached')
            error = _angle(self._goal_yaw-yaw)
            if abs(error) <= .04:
                return self._stop('reached', 'goal and heading reached')
            target_distance = 0.
            self.state = 'aligning'
        else:
            # Every lookahead is certified from the actual pose, never across a corner.
            target_index = None
            for index in range(len(self.path)-1, self._index-1, -1):
                if self.grid.segment_clear(xy, self.path[index]):
                    target_index = index
                    break
            if target_index is None:
                return self._stop('blocked', 'path no longer visible from pose')
            if target_index != self._index:
                self._best_distance = self._best_angle = math.inf
                self._progress_at = now
                self._index = target_index
            delta = np.asarray(self.path[self._index])-xy
            target_distance = float(np.linalg.norm(delta))
            error = _angle(math.atan2(delta[1], delta[0])-yaw)
            self.state = 'aligning' if abs(error) > .15 else 'driving'

        if (self._progress_at is None or target_distance < self._best_distance-.01 or
                abs(error) < self._best_angle-.03):
            self._progress_at = now
            self._best_distance = min(self._best_distance, target_distance)
            self._best_angle = min(self._best_angle, abs(error))
        if now-self._progress_at > 15.:
            return self._stop('blocked', 'no progress for 15 seconds')

        desired_v = min(.1, .8*max(0., target_distance-.01)) if self.state == 'driving' else 0.
        desired_w = float(np.clip(1.5*error, -.4, .4))
        # Brake translation before a large in-place turn.
        if self.state == 'aligning' and self.v > .001:
            desired_w = 0.
        self.v += float(np.clip(desired_v-self.v, -.2*dt, .2*dt))
        self.w += float(np.clip(desired_w-self.w, -.8*dt, .8*dt))
        # Check the next command plus braking travel against the same inflated map.
        horizon = dt+self.v/.2
        end = xy + self.v*horizon*np.array([math.cos(yaw+self.w*horizon/2),
                                           math.sin(yaw+self.w*horizon/2)])
        if not self.grid.segment_clear(xy, end):
            return self._stop('blocked', 'command or braking distance leaves free map')
        self.reason = ''
        return self._result()


def depth_guard(depth_mm, K, body_from_camera, stamp, now, *, body_height_m=.14305):
    """Fail closed on stale/unknown forward depth or any collision-height endpoint.

    Uses the entire image, current rotating-head transform and measured body
    height above floor. The forward x<=.65m, |y|<=.25m endpoint corridor covers
    low/overhanging objects. Visibility is required across a .36m-wide vertical
    cross-section at x=.65m (floor heights .04.. .38m); the narrow camera cannot
    observe the entire near footprint. The frozen map must protect that blind
    region and the rear; never use this guard alone for hardware safety.
    """
    result = dict(stamp=stamp, clear=False, reason='invalid depth geometry')
    try:
        if not math.isfinite(now) or not _fresh(stamp, now, .5):
            result['reason'] = 'depth stale'
            return result
        depth = np.asarray(depth_mm, float)
        intrinsics, mount = np.asarray(K, float), np.asarray(body_from_camera, float)
        if (depth.ndim != 2 or min(depth.shape) < 8 or intrinsics.shape != (4,) or
                not np.isfinite(intrinsics).all() or (intrinsics[:2] <= 0).any() or
                mount.shape != (4, 4) or not np.isfinite(mount).all() or
                not np.allclose(mount[3], [0, 0, 0, 1]) or
                not np.allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(mount[:3, :3]), 1.) or
                not math.isfinite(body_height_m) or not 0 < body_height_m < 1.):
            return result
        if mount[0, 2] < .7 or mount[0, 3] >= .25:
            result['reason'] = 'camera not facing forward corridor'
            return result
        fx, fy, cx, cy = intrinsics
        rows, cols = np.indices(depth.shape)
        rays = np.stack(((cols-cx)/fx, (rows-cy)/fy, np.ones_like(depth)), -1)
        rays = rays @ mount[:3, :3].T
        valid = np.isfinite(depth) & (depth > 0) & (depth <= 10000)
        points = rays*np.where(valid, depth*.001, 0)[..., None] + mount[:3, 3]
        floor_z = points[..., 2]+body_height_m
        collision = (valid & (points[..., 0] > 0) & (points[..., 0] <= .65) &
                     (np.abs(points[..., 1]) <= .25) & (floor_z > .025) &
                     (floor_z <= max(.4, body_height_m+.18)))
        if collision.any():
            result['reason'] = 'obstacle in forward stopping corridor'
            return result

        # All four corners must be in view; checking observed rays alone hides blind sectors.
        corners = np.array([[.65, y, z-body_height_m]
                            for y in (-.18, .18) for z in (.04, .38)])
        camera_corners = (corners-mount[:3, 3]) @ mount[:3, :3]
        if (camera_corners[:, 2] <= 0).any():
            result['reason'] = 'unknown forward corridor'
            return result
        uv = camera_corners[:, :2]/camera_corners[:, 2, None]*[fx, fy]+[cx, cy]
        if ((uv < 0).any() or (uv[:, 0] > depth.shape[1]-1).any() or
                (uv[:, 1] > depth.shape[0]-1).any()):
            result['reason'] = 'unknown forward corridor'
            return result
        with np.errstate(divide='ignore', invalid='ignore'):
            crossing_depth = (.65-mount[0, 3])/rays[..., 0]
        crossing = rays*crossing_depth[..., None]+mount[:3, 3]
        observed = ((crossing_depth > 0) & (np.abs(crossing[..., 1]) <= .18) &
                    (crossing[..., 2]+body_height_m >= .04) &
                    (crossing[..., 2]+body_height_m <= .38))
        if (observed.sum() < 20 or not valid[observed].all() or
                (depth[observed]*.001 < crossing_depth[observed]).any()):
            result['reason'] = 'unknown forward corridor'
            return result
        result.update(clear=True, reason='forward corridor observed; near/rear protected by map')
        return result
    except (TypeError, ValueError, IndexError, OverflowError):
        return result
