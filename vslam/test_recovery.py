import unittest

import cv2
import numpy as np

from vslam.pose_guard import PoseGuard
from vslam.recovery import VisualRecovery, match_rgbd


class RecoveryTests(unittest.TestCase):
    def test_verified_match_rebases_a_persistent_sdk_jump(self):
        rng = np.random.default_rng(32)
        rgb = rng.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
        rgb = cv2.GaussianBlur(rgb, (5, 5), .8)
        shifted = rgb.copy()
        depth = np.zeros(rgb.shape[:2], dtype=np.uint16)
        for region, range_mm, pixels in ((slice(None, 150), 1000, -4), (slice(170, None), 2000, -2)):
            shifted[:, region] = cv2.warpAffine(
                rgb[:, region], np.array([[1., 0., pixels], [0., 1., 0.]]), (150, 240))
            depth[:, region] = range_mm
        intrinsics = (400., 400., 159.5, 119.5)
        origin, sensor, jumped = np.eye(4), np.eye(4), np.eye(4)
        sensor[0, 3], jumped[0, 3] = .01, .25
        guard, recovery = PoseGuard(), VisualRecovery()
        guard.update(0., origin, origin)
        recovery.accept(rgb, depth, False)
        pose, matched = recovery.candidate(jumped, sensor, shifted, depth, intrinsics, guard)
        self.assertFalse(matched)
        self.assertEqual(guard.update(.2, pose, sensor)[0], 'hold')
        for t in (.4, .6, .8):
            pose, matched = recovery.candidate(jumped, sensor, shifted, depth, intrinsics, guard)
            self.assertTrue(matched)
            state, _ = guard.update(t, pose, sensor)
        self.assertEqual(state, 'recovered')
        np.testing.assert_allclose(pose, sensor, atol=.002)
        recovery.accept(shifted, depth, matched)
        following, matched = recovery.candidate(jumped, sensor, shifted, depth, intrinsics, guard)
        self.assertFalse(matched)
        np.testing.assert_allclose(following, pose)

    def test_partial_depth_outliers_are_excluded_from_the_fit(self):
        rng = np.random.default_rng(12)
        rgb = rng.integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
        depth = np.full(rgb.shape[:2], 1200, np.uint16)
        depth[:, 160:] = 1800
        observed = depth.copy()
        observed[:55] += 500
        stats = {}
        K = (400., 400., 159.5, 119.5)
        delta = match_rgbd(rgb, depth, rgb, observed, K, np.eye(4), stats)
        np.testing.assert_allclose(delta, np.eye(4), atol=1e-4)
        self.assertLess(stats['depth_inliers'], stats['inliers'])
        self.assertIsNone(match_rgbd(rgb, depth, rgb, depth + 500, K, np.eye(4)))

    def test_blank_confirmation_frame_cannot_commit_a_pending_alignment(self):
        origin, jumped = np.eye(4), np.eye(4)
        jumped[0, 3] = .5
        rgb = np.zeros((100, 100, 3), np.uint8)
        depth = np.full(rgb.shape[:2], 1000, np.uint16)
        guard, recovery = PoseGuard(), VisualRecovery()
        guard.update(0, origin, origin)
        recovery.accept(rgb, depth, False)
        guard.update(.1, jumped, origin)
        guard.update(4, None, None)
        recovery.pending_alignment = np.linalg.inv(jumped)
        for t in (5., 5.2, 5.4):
            visual, matched = recovery.candidate(jumped, origin, rgb, depth, (80., 80., 50., 50.), guard, t)
            state, _ = guard.update(t, visual, origin, verified_match=matched)
        self.assertEqual(state, 'lost')
        np.testing.assert_array_equal(guard.accepted[0], origin)

    def test_textureless_reference_does_not_replace_sdk_pose(self):
        rgb = np.zeros((100, 100, 3), dtype=np.uint8)
        depth = np.full(rgb.shape[:2], 1000, dtype=np.uint16)
        origin, jumped = np.eye(4), np.eye(4)
        jumped[0, 3] = .25
        guard, recovery = PoseGuard(), VisualRecovery()
        guard.update(0., origin, origin)
        recovery.accept(rgb, depth, False)
        guard.update(.2, jumped, origin)
        pose, matched = recovery.candidate(jumped, origin, rgb, depth, (80., 80., 50., 50.), guard)
        self.assertFalse(matched)
        self.assertEqual(guard.update(.4, pose, origin)[0], 'hold')


if __name__ == '__main__':
    unittest.main()
