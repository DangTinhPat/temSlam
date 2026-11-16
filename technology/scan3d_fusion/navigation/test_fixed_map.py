import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from fixed_map import FixedMapLocalizer, FixedReference, vertical_points


K = (160., 160., 159.5, 119.5)


def packet(t, **changes):
    return dict(dict(time_s=t, x=0., y=0., yaw=0., pitch=0., roll=0.,
                     axle_body_x=0., axle_body_z=-.15, wheel_radius=.035,
                     valid=1, slip=False), **changes)


def mount(yaw=np.pi/4):
    c, s = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[s, 0, c], [-c, 0, s], [0, -1, 0]]
    result[2, 3] = .615
    return result


def corner_depth(camera_mount=None, noise=False, body_xy=(0, 0)):
    camera_mount = mount() if camera_mount is None else camera_mount
    y, x = np.mgrid[:240, :320]
    rays = np.stack(((x-K[2])/K[0], (y-K[3])/K[1], np.ones_like(x)), -1)
    rays = rays @ camera_mount[:3, :3].T
    with np.errstate(divide='ignore', invalid='ignore'):
        distances = (2-np.asarray(body_xy)) / rays[..., :2]
    distances[distances <= 0] = np.inf
    depth = distances.min(axis=2)
    if noise:
        depth += np.random.default_rng(8).normal(0, .003, depth.shape)
    return np.rint(depth*1000).astype(np.uint16)


def corner_reference(single_wall=False):
    along, z = np.meshgrid(np.arange(-1, 3.01, .03), np.arange(.17, 1.61, .03))
    points = np.column_stack((np.full(along.size, 2.), along.ravel(), z.ravel()))
    normals = np.tile([1., 0, 0], (len(points), 1))
    if not single_wall:
        points = np.concatenate((points, points[:, [1, 0, 2]]))
        normals = np.concatenate((normals, normals[:, [1, 0, 2]]))
    return FixedReference(points, normals)


class FixedMapTests(unittest.TestCase):
    def update(self, localizer, i, depth=None, camera_mount=None, **packet_changes):
        return localizer.update(i*100_000_000, packet(i*.1, **packet_changes),
                                mount() if camera_mount is None else camera_mount,
                                corner_depth() if depth is None else depth, K)

    def test_noisy_corner_recovers_pose_and_requires_initial_confirmation(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(.06, -.04, .025))
        first = self.update(localizer, 0, corner_depth(noise=True))
        self.assertFalse(first['accepted'])
        np.testing.assert_allclose(np.asarray(first['world_from_body'])[:2, 3], [.06, -.04])
        self.assertEqual(first['state'], 'initializing')
        self.assertFalse(first['usable'])
        result = self.update(localizer, 1, corner_depth(noise=True))
        self.assertEqual(result['state'], 'tracking')
        self.assertTrue(result['usable'])
        pose = np.asarray(result['world_from_body'])
        np.testing.assert_allclose(pose[:2, 3], [0, 0], atol=.012)
        self.assertLess(abs(np.arctan2(pose[1, 0], pose[0, 0])), .008)
        self.assertAlmostEqual(pose[2, 3], .185)
        self.assertGreater(result['overlap'], .7)
        self.assertLess(result['rmse_m'], .01)
        json.dumps(result, allow_nan=False)

    def test_single_wall_and_no_overlap_never_become_usable(self):
        for reference, initial in [(corner_reference(True), (0, 0, 0)),
                                   (corner_reference(), (10, 10, 0))]:
            localizer = FixedMapLocalizer(reference, initial_pose=initial)
            for i in range(3):
                result = self.update(localizer, i)
                self.assertFalse(result['accepted'])
                self.assertFalse(result['usable'])
            self.assertIn(result['reason'], ('geometry', 'overlap'))

    def test_rotating_camera_does_not_rotate_body_and_reference_stays_fixed(self):
        reference = corner_reference()
        points, normals = reference.points.copy(), reference.normals.copy()
        localizer = FixedMapLocalizer(reference, initial_pose=(0, 0, 0))
        for i, yaw in enumerate([.7, .8, .6, .9]):
            camera_mount = mount(yaw)
            result = self.update(localizer, i, corner_depth(camera_mount), camera_mount)
            np.testing.assert_allclose(np.asarray(result['world_from_body'])[:3, :3], np.eye(3), atol=.005)
            np.testing.assert_allclose(np.asarray(result['world_from_camera']),
                                       np.asarray(result['world_from_body']) @ camera_mount)
        np.testing.assert_array_equal(reference.points, points)
        np.testing.assert_array_equal(reference.normals, normals)
        with self.assertRaises(ValueError):
            reference.points[0, 0] = 42

    def test_loss_recovery_and_timestamp_gap_require_reconfirmation(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(0, 0, 0))
        self.update(localizer, 0)
        self.assertEqual(self.update(localizer, 1)['state'], 'tracking')
        rejected = self.update(localizer, 2, np.zeros((240, 320), np.uint16))
        self.assertEqual(rejected['state'], 'degraded')
        self.assertFalse(rejected['usable'])
        for i in range(3, 14):
            rejected = self.update(localizer, i, np.zeros((240, 320), np.uint16))
        self.assertEqual(rejected['state'], 'lost')
        self.assertEqual(self.update(localizer, 14)['state'], 'lost')
        self.assertEqual(self.update(localizer, 15)['state'], 'tracking')
        self.assertEqual(self.update(localizer, 30)['state'], 'lost')
        self.assertEqual(self.update(localizer, 31)['state'], 'tracking')

    def test_invalid_inputs_do_not_advance_timestamp(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(0, 0, 0))
        cases = [dict(timestamp_ns=-1), dict(timestamp_ns=True),
                 dict(packet=packet(.1)), dict(depth_mm=np.ones((2, 2), np.uint16)),
                 dict(depth_mm=corner_depth().astype(float)), dict(intrinsics=(0, 1, 2, 3)),
                 dict(body_from_camera=np.zeros((4, 4))), dict(packet={})]
        arguments = dict(timestamp_ns=0, packet=packet(0), body_from_camera=mount(),
                         depth_mm=corner_depth(), intrinsics=K)
        for case in cases:
            with self.assertRaises(ValueError):
                localizer.update(**dict(arguments, **case))
        self.update(localizer, 0)
        with self.assertRaises(ValueError):
            self.update(localizer, 0)
        invalid = self.update(localizer, 1, valid=0)
        self.assertEqual(invalid['state'], 'lost')
        self.assertFalse(invalid['usable'])
        self.assertIsNone(invalid['world_from_body'])

    def test_initial_body_pose_keeps_odometry_height_and_tilt(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(3, 4, .5))
        result = self.update(localizer, 0, np.zeros((240, 320), np.uint16),
                             x=10, y=20, yaw=.2, pitch=.03, roll=-.02)
        pose = np.asarray(result['world_from_body'])
        np.testing.assert_allclose(pose[:2, 3], [3, 4], atol=1e-10)
        self.assertAlmostEqual(np.arctan2(pose[1, 0], pose[0, 0]), .5)
        self.assertAlmostEqual(pose[2, 2], np.cos(.03)*np.cos(-.02))

    def test_conflicting_fresh_depth_cannot_pull_body_away_from_recent_views(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(0, 0, 0))
        for i in range(6):
            camera_mount = mount(.65+.04*i)
            result = self.update(localizer, i, corner_depth(camera_mount), camera_mount)
        previous = np.asarray(result['world_from_body'])
        result = self.update(localizer, 6, corner_depth()+80)
        self.assertFalse(result['accepted'])
        self.assertFalse(result['usable'])
        np.testing.assert_allclose(result['world_from_body'], previous, atol=1e-10)
        self.assertEqual(result['reason'], 'current_residual')
        result = self.update(localizer, 7)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'history_residual')
        self.assertGreater(result['current_matches'], 100)
        self.assertLess(result['current_rmse_m'], .04)
        self.assertLess(result['consistent_views'], result['window_views'])
        np.testing.assert_allclose(result['world_from_body'], previous, atol=1e-10)


    def test_measurement_window_tracks_odometry_and_expires_without_fresh_depth(self):
        localizer = FixedMapLocalizer(corner_reference(), initial_pose=(0, 0, 0))
        for i in range(6):
            camera_mount = mount(.65+.04*i)
            result = self.update(localizer, i, corner_depth(camera_mount, body_xy=(.04*i, 0)),
                                 camera_mount, x=.04*i)
            np.testing.assert_allclose(np.asarray(result['world_from_body'])[:2, 3], [.04*i, 0], atol=.01)
        self.assertGreater(result['window_views'], 1)
        for i in range(6, 40):
            result = self.update(localizer, i, np.zeros((240, 320), np.uint16), x=.2)
            self.assertFalse(result['accepted'])
            self.assertFalse(result['usable'])
        self.assertEqual(result['window_views'], 0)
        self.assertEqual(result['state'], 'lost')

    def test_yaw_unobservable_circle_is_rejected_despite_two_translation_axes(self):
        theta, z = np.meshgrid(np.linspace(0, 2*np.pi, 180, endpoint=False), np.linspace(.2, 1.5, 20))
        points = np.column_stack((np.cos(theta.ravel()), np.sin(theta.ravel()), z.ravel()))
        normals = points.copy(); normals[:, 2] = 0
        localizer = FixedMapLocalizer(FixedReference(points, normals), initial_pose=(0, 0, 0))
        change, report = localizer._match(points, normals, np.zeros(3))
        self.assertFalse(report['accepted'])
        self.assertEqual(report['reason'], 'geometry')
        np.testing.assert_array_equal(change, np.eye(4))

    def test_excessive_yaw_correction_is_rejected(self):
        rng = np.random.default_rng(9)
        target = rng.uniform([0, 0, .2], [.6, .6, 1.5], (2000, 3))
        target[:1000, 0] = 0; target[1000:, 1] = 0
        normals = np.zeros_like(target); normals[:1000, 0] = 1; normals[1000:, 1] = 1
        angle = .2
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        localizer = FixedMapLocalizer(FixedReference(target, normals), initial_pose=(0, 0, 0))
        change, report = localizer._match(target @ rotation.T, normals @ rotation.T, np.zeros(3))
        self.assertFalse(report['accepted'])
        self.assertEqual(report['reason'], 'correction_jump')
        np.testing.assert_array_equal(change, np.eye(4))

    def test_low_overlap_and_large_final_residual_are_rejected(self):
        reference = corner_reference()
        localizer = FixedMapLocalizer(reference, initial_pose=(0, 0, 0))
        target, normals = reference.points, reference.normals
        sources = [np.concatenate((target, target+10, target+20)),
                   target+normals*np.random.default_rng(3).normal(0, .065, (len(target), 1))]
        for source, source_normals, reason in [(sources[0], np.tile(normals, (3, 1)), 'overlap'),
                                               (sources[1], normals, 'residual')]:
            change, report = localizer._match(source, source_normals, np.zeros(3))
            self.assertFalse(report['accepted'])
            self.assertEqual(report['reason'], reason)
            np.testing.assert_array_equal(change, np.eye(4))

    def test_reference_and_initial_pose_validate_bad_inputs(self):
        for points, normals in [(np.zeros((100, 3)), np.zeros((100, 3))),
                                (np.zeros((2, 3)), np.ones((2, 3))),
                                (np.full((100, 3), np.nan), np.ones((100, 3)))]:
            with self.assertRaises(ValueError):
                FixedReference(points, normals)
        for initial in [(0, 0), (0, 0, float('nan'))]:
            with self.assertRaises(ValueError):
                FixedMapLocalizer(corner_reference(), initial_pose=initial)

    def test_batched_reference_keeps_first_observation_per_voxel(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            frames, first = [], {}
            for i in range(70):
                pose = mount(); pose[2, 3] += .185; pose[0, 3] = .003*i
                depth = corner_depth()+i
                filename = f'frame{i}.npz'
                np.savez(path/filename, depth=depth, depth_source='deltar_tof_fusion')
                frames.append(dict(file=filename, camera_to_world=pose.tolist()))
                points, normals = vertical_points(depth, K, pose)
                for point, normal in zip(points, normals):
                    cell = tuple(np.floor(point/.04).astype(np.int64))
                    first.setdefault(cell, (point, normal/np.linalg.norm(normal)))
            (path/'live_sequence.json').write_text(json.dumps(dict(intrinsics=K, frames=frames)))
            reference = FixedReference.from_session(path)
            expected_points, expected_normals = zip(*(first[cell] for cell in sorted(first)))
            np.testing.assert_array_equal(reference.points, expected_points)
            np.testing.assert_allclose(reference.normals, expected_normals, atol=1e-15, rtol=0)

    def test_reference_load_requires_fusion_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            pose = mount(); pose[2, 3] += .185
            (path/'live_sequence.json').write_text(json.dumps(dict(intrinsics=K, frames=[
                dict(file='frame.npz', camera_to_world=pose.tolist())])))
            np.savez(path/'frame.npz', depth=corner_depth())
            with self.assertRaises(ValueError):
                FixedReference.from_session(path)
            np.savez(path/'frame.npz', depth=corner_depth(), depth_source='deltar_tof_fusion')
            reference = FixedReference.from_session(path)
            self.assertGreater(len(reference.points), 100)
            np.savez(path/'frame.npz', depth=corner_depth(), depth_source='deltar_tof_fusion',
                     floor_plane=[0., 0, 1, 0])
            with self.assertRaises(ValueError):
                FixedReference.from_session(path)


if __name__ == '__main__':
    unittest.main()
