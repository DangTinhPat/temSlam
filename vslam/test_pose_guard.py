import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from vslam.pose_guard import PoseGuard


def pose(x, degrees=0):
    result = np.eye(4)
    result[0, 3] = x
    result[:3, :3] = Rotation.from_euler('z', degrees, degrees=True).as_matrix()
    return result


class PoseGuardTests(unittest.TestCase):
    def test_wrong_pose_with_small_followup_steps_never_recovers(self):
        guard = PoseGuard()
        self.assertEqual(guard.update(210.535, pose(0), pose(0))[0], 'accepted')
        self.assertEqual(guard.update(211.135, pose(4.88, 79), pose(.003, 1.9))[0], 'hold')
        self.assertEqual(guard.update(211.5, pose(4.90, 79), pose(.023, 1.9))[0], 'hold')
        self.assertEqual(guard.update(214.135, pose(.03), pose(.03))[0], 'lost')
        self.assertEqual(guard.update(214.2, pose(.03), pose(.03))[0], 'lost')
        self.assertEqual(guard.accepted[0][0, 3], 0)

    def test_recovers_only_after_three_consecutive_plausible_frames(self):
        guard = PoseGuard()
        guard.update(0, pose(0), pose(0))
        self.assertEqual(guard.update(.1, None, pose(.01))[0], 'hold')
        self.assertEqual(guard.update(.2, pose(.02), pose(.02))[0], 'hold')
        self.assertEqual(guard.update(.3, pose(1), pose(.03))[0], 'hold')
        for i in (4, 5):
            self.assertEqual(guard.update(i/10, pose(i/100), pose(i/100))[0], 'hold')
        self.assertEqual(guard.update(.6, pose(.06), pose(.06))[0], 'recovered')
        self.assertEqual(guard.update(.7, pose(.07), pose(.07))[0], 'accepted')

    def test_delayed_verified_match_still_requires_three_frames(self):
        guard = PoseGuard()
        guard.update(0, pose(0), pose(0))
        guard.update(.1, pose(.3), pose(0))
        for t in (4.2, 4.4):
            self.assertEqual(guard.update(t, pose(.02), pose(.02), verified_match=True)[0], 'hold')
        self.assertEqual(guard.update(4.6, pose(.03), pose(.03), verified_match=True)[0], 'recovered')

    def test_late_match_with_inconsistent_pose_still_expires(self):
        guard = PoseGuard()
        guard.update(0, pose(0), pose(0))
        guard.update(.1, pose(.3), pose(0))
        self.assertEqual(guard.update(4.2, pose(.3), pose(0), verified_match=True)[0], 'lost')

    def test_borderline_steps_do_not_replace_the_trusted_anchor(self):
        guard = PoseGuard()
        guard.update(0, pose(0), pose(0))
        for t, x in ((.2, .09), (.4, .18)):
            self.assertEqual(guard.update(t, pose(x), pose(0))[0], 'hold')
        self.assertEqual(guard.accepted[0][0, 3], 0)

    def test_slow_drift_is_checked_against_a_recent_trusted_pose(self):
        guard = PoseGuard()
        for t, x in ((0., 0.), (.2, .035), (.4, .07)):
            self.assertEqual(guard.update(t, pose(x), pose(0))[0], 'accepted')
        self.assertEqual(guard.update(.6, pose(.105), pose(0))[0], 'hold')
        self.assertAlmostEqual(guard.accepted[0][0, 3], .07)

    def test_three_verified_frames_can_leave_lost(self):
        guard = PoseGuard()
        guard.update(0, pose(0), pose(0))
        guard.update(.2, None, None)
        self.assertEqual(guard.update(4, None, None)[0], 'lost')
        for t in (6, 6.2):
            self.assertEqual(guard.update(t, pose(.02), pose(.02), verified_match=True)[0], 'hold')
        self.assertEqual(guard.update(6.4, pose(.02), pose(.02), verified_match=True)[0], 'recovered')
        self.assertFalse(guard.lost)
