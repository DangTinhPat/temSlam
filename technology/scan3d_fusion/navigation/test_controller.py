import math
import unittest

import numpy as np

from controller import Navigator, depth_guard
from grid import NavigationGrid


def grid():
    return NavigationGrid(np.zeros((60, 60), np.int8), (-1.5, -1.5), .05)


def pose(now, xy=(0., 0.), yaw=0., **changes):
    return dict(dict(xy=list(xy), yaw=yaw, stamp=now, usable=True,
                     uncertainty_m=.02), **changes)


def guard(now, **changes):
    return dict(dict(stamp=now, clear=True, reason='clear'), **changes)


def camera(yaw=0.):
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[s, 0, c], [-c, 0, s], [0, -1, 0]]
    t[2, 3] = .092
    return t


K = (120., 120., 79.5, 59.5)


class ControllerTests(unittest.TestCase):
    def test_requires_full_footprint_clearance_and_rejects_blocked_goal(self):
        with self.assertRaises(ValueError):
            Navigator(NavigationGrid(np.zeros((60, 60), np.int8), (-1.5, -1.5), .05, .1))
        nav = Navigator(grid())
        result = nav.set_goal((0, 0), (3, 3))
        self.assertEqual(result['state'], 'blocked')
        self.assertEqual(result['path'], [])
        self.assertEqual(result['v'], 0.)

    def test_turn_drive_arrive_final_yaw_with_bounded_acceleration(self):
        nav = Navigator(grid())
        nav.set_goal((0, 0), (.5, .5), -math.pi/2)
        xy, yaw, previous = np.zeros(2), 0., (0., 0.)
        for i in range(1500):
            now, dt = i*.05, .05
            result = nav.update(pose(now, xy, yaw), now, guard(now), dt)
            self.assertLessEqual(abs(result['v']), .1+1e-10)
            self.assertLessEqual(abs(result['w']), .4+1e-10)
            if result['state'] not in ('reached', 'blocked', 'lost'):
                self.assertLessEqual(abs(result['v']-previous[0]), .2*dt+1e-10)
                self.assertLessEqual(abs(result['w']-previous[1]), .8*dt+1e-10)
            xy += result['v']*dt*np.array([math.cos(yaw), math.sin(yaw)])
            yaw += result['w']*dt
            previous = result['v'], result['w']
            if result['state'] == 'reached':
                break
        self.assertEqual(result['state'], 'reached', result)
        np.testing.assert_allclose(xy, [.5, .5], atol=.03)
        self.assertLess(abs(math.atan2(math.sin(yaw+math.pi/2), math.cos(yaw+math.pi/2))), .05)
        self.assertEqual(nav.update(None, now+1, None, .1)['state'], 'reached')

    def test_bad_pose_or_depth_stops_immediately_and_latches(self):
        cases = [(None, guard(1)), (pose(0), guard(1)),
                 (pose(1, usable=False), guard(1)),
                 (pose(1, uncertainty_m=float('nan')), guard(1)),
                 (pose(1, uncertainty_m=.06), guard(1)),
                 (pose(1, xy=(float('nan'), 0)), guard(1)),
                 (pose(1), None), (pose(1), guard(0)),
                 (pose(1), guard(2)), (pose(1), guard(1, clear=False))]
        for current_pose, obstacles in cases:
            nav = Navigator(grid()); nav.set_goal((0, 0), (1, 0))
            nav.update(pose(.9), .9, guard(.9), .1)
            result = nav.update(current_pose, 1, obstacles, .1)
            self.assertIn(result['state'], ('lost', 'blocked'))
            self.assertEqual((result['v'], result['w']), (0., 0.))
            self.assertEqual(nav.update(pose(1.1), 1.1, guard(1.1), .1)['state'], result['state'])
            nav.set_goal((0, 0), (.5, 0))
            self.assertGreater(nav.update(pose(1.2), 1.2, guard(1.2), .1)['v'], 0)

    def test_cancel_invalid_goal_and_invalid_clock_stop(self):
        nav = Navigator(grid()); nav.set_goal((0, 0), (1, 0))
        nav.cancel()
        self.assertEqual(nav.update(pose(0), 0, guard(0), .1)['state'], 'cancelled')
        for start, goal, yaw in [((0, 0), (float('inf'), 0), None),
                                 ((0,), (1, 0), None), ((0, 0), (1, 0), float('nan'))]:
            with self.assertRaises(ValueError):
                nav.set_goal(start, goal, yaw)
            self.assertEqual(nav.update(None, 0, None, .1)['v'], 0)
        for now, dt in [(float('nan'), .1), (0, 0), (0, float('inf')), (0, 1.)]:
            nav.set_goal((0, 0), (1, 0))
            self.assertEqual(nav.update(pose(0), now, guard(0), dt)['state'], 'lost')

    def test_map_departure_and_no_progress_stop(self):
        nav = Navigator(grid()); nav.set_goal((0, 0), (1, 0))
        result = nav.update(pose(0, (4, 4)), 0, guard(0), .1)
        self.assertEqual(result['state'], 'blocked')
        nav.set_goal((0, 0), (1, 0))
        for i in range(200):
            result = nav.update(pose(i*.1), i*.1, guard(i*.1), .1)
        self.assertEqual(result['state'], 'blocked')
        self.assertIn('progress', result['reason'])

    def test_path_around_wall_never_crosses_blocked_cells(self):
        cells = np.zeros((80, 80), np.int8)
        cells[15:52, 40] = 1
        arena = NavigationGrid(cells, (-2, -2), .05)
        nav = Navigator(arena); nav.set_goal((-.6, 0), (.6, 0))
        xy, yaw = np.array([-.6, 0.]), 0.
        for i in range(2500):
            now, dt = i*.05, .05
            result = nav.update(pose(now, xy, yaw), now, guard(now), dt)
            nxt = xy + result['v']*dt*np.array([math.cos(yaw), math.sin(yaw)])
            self.assertTrue(arena.segment_clear(xy, nxt), result)
            xy = nxt; yaw += result['w']*dt
            if result['state'] in ('reached', 'lost', 'blocked'):
                break
        self.assertEqual(result['state'], 'reached', result)


class DepthGuardTests(unittest.TestCase):
    def test_clear_forward_frame_and_unknown_or_away_fail_closed(self):
        depth = np.full((120, 160), 2000, np.uint16)
        self.assertTrue(depth_guard(depth, K, camera(), 1., 1.)['clear'])
        for image, mount, stamp in [(np.zeros_like(depth), camera(), 1.),
                                     (depth, camera(math.pi/2), 1.),
                                     (depth, camera(math.pi), 1.),
                                     (depth, camera(), 0.),
                                     (depth, np.zeros((4, 4)), 1.)]:
            self.assertFalse(depth_guard(image, K, mount, stamp, 1.)['clear'])
        missing = depth.copy(); missing[:, 70:90] = 0
        self.assertFalse(depth_guard(missing, K, camera(), 1., 1.)['clear'])

    def test_single_low_and_overhanging_endpoints_block_without_height_matching_mask(self):
        for row in (10, 59, 110):
            depth = np.full((120, 160), 2000, np.uint16)
            depth[row, 80] = 350
            result = depth_guard(depth, K, camera(), 1., 1.)
            self.assertFalse(result['clear'], (row, result))
            self.assertIn('obstacle', result['reason'])

    def test_floor_and_body_height_use_current_mount(self):
        depth = np.full((120, 160), 2000, np.uint16)
        # At this pixel: body z=.092-(119-59.5)/120*.44=-.1262.
        depth[119, 80] = 440
        self.assertTrue(depth_guard(depth, K, camera(), 1., 1., body_height_m=.14305)['clear'])
        self.assertFalse(depth_guard(depth, K, camera(), 1., 1., body_height_m=.2078)['clear'])

    def test_invalid_geometry_never_throws_or_claims_clear(self):
        for depth, intrinsics, mount in [(None, K, camera()),
                                         (np.ones((2, 2)), K, camera()),
                                         (np.full((120, 160), np.nan), K, camera()),
                                         (np.ones((120, 160)), (0, 1, 0, 0), camera()),
                                         (np.ones((120, 160)), K, np.full((4, 4), np.nan))]:
            self.assertFalse(depth_guard(depth, intrinsics, mount, 1., 1.)['clear'])


if __name__ == '__main__':
    unittest.main()
