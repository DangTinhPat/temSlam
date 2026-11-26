"""Regressions for evaluating the two pose histories on identical observations."""
import unittest

import numpy as np

import compare_loop as compare


class CompareLoopTests(unittest.TestCase):
    def test_bmap1_requires_both_wall_pairs_on_sufficient_common_coverage(self):
        strips = {wall: {i: (.15*i, 0.) for i in range(28)} for wall in compare.WALLS}
        self.assertTrue(compare.bmap1_wall_check(strips)['passed'])
        strips['right'] = {i: (.15*i, .015*i) for i in range(28)}
        self.assertFalse(compare.bmap1_wall_check(strips)['passed'])
        strips['right'] = {i: (.15*i, 0.) for i in range(12)}
        result = compare.bmap1_wall_check(strips)
        self.assertFalse(result['passed'])
        self.assertEqual(result['pairs']['left_right']['shared_bins'], 12)

    def test_split_at_missing_sdk_unpublished_frame_and_restart(self):
        stamps = np.arange(8)
        proposed = np.tile(np.eye(4), (8, 1, 1))
        proposed[2] = np.nan
        epochs = [0, 0, 0, 0, 1, 1, 1, 1]
        approved = [0, 1, 2, 3, 4, 6]
        self.assertEqual(compare.common_runs(stamps, proposed, epochs, approved),
                         [[0, 1], [3], [4], [6]])

    def test_same_observed_strips_exclude_extra_slanted_tail(self):
        def strips(bins, slope):
            return {i: (i * .15, i * .15 * slope) for i in bins}
        a = strips(range(6), .1)
        b = strips(range(3, 11), .1)
        b[6] = (.9, .09)
        a[6] = (.9, .09)
        b.update(strips(range(7, 11), .5))
        all_bins = sorted(a.keys() & b.keys())
        fit = compare.wall_fit(b, all_bins, 1)
        self.assertEqual(fit['bin_ids'], [3, 4, 5, 6])
        self.assertAlmostEqual(fit['axis_angle_deg'], 5.7105931375)
        self.assertIsNone(compare.wall_fit(a, [0, 1], 1)['axis_angle_deg'])

    def test_grid_keeps_displaced_candidate_reconstructions_in_bounds(self):
        bounds = np.array([[0., 0., 0.], [2., 3., 1.]])
        approved = np.tile(np.eye(4), (2, 1, 1))
        candidate = approved.copy()
        candidate[1, 0, 3] = -1.2
        np.testing.assert_allclose(compare.shared_bounds(bounds, approved, candidate, .04),
                                   [[-1.2, 0., 0.], [2., 3., 1.]])

    def test_revisit_prefers_same_heading_and_reports_reverse_view_if_only_match(self):
        bodies = np.tile(np.eye(4), (6, 1, 1))
        bodies[:, 0, 3] = [0, .3, 4, 2, .31, 1.]
        self.assertEqual(compare.revisit_pair(bodies), [1, 4])
        bodies[4, :2, :2] *= -1
        self.assertEqual(compare.revisit_pair(bodies), [1, 4])
        bodies[4, 0, 3] = 2.5
        self.assertIsNone(compare.revisit_pair(bodies))


if __name__ == '__main__':
    unittest.main()
