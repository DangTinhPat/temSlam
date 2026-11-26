"""Regression checks for the map frame and graph snapshots."""
import unittest

import numpy as np

from vslam.tracker import graph_diagnostics, graph_is_consistent, optimized_map_poses


def pose_at(x):
    pose = np.eye(4)
    pose[0, 3] = x
    return pose


class GraphSnapshotTests(unittest.TestCase):
    def test_later_graph_snapshot_preserves_initial_alignment_and_prior_snapshot(self):
        anchor = pose_at(10)
        raw = {1: pose_at(0), 2: pose_at(2)}
        preview = optimized_map_poses(raw, [1, 2], anchor)
        refined = {1: pose_at(1), 2: pose_at(4)}
        final = optimized_map_poses(refined, [1, 2], anchor)
        np.testing.assert_allclose([p[0, 3] for p in preview], [10, 12])
        np.testing.assert_allclose([p[0, 3] for p in final], [11, 14])
        self.assertEqual(anchor[0, 3], 10)

    def test_missing_pose_rejects_the_entire_snapshot(self):
        with self.assertRaisesRegex(ValueError, 'missing'):
            optimized_map_poses({1: pose_at(0)}, [1, 2], np.eye(4))

    def test_diagnostic_reports_origin_and_only_failing_edges(self):
        stamps = [17, 23, 49]
        previous = [pose_at(x) for x in (0, .50, 1)]
        for positions, pairs in [((0, .57, 1.14), []), ((0, .5, 4), [(23, 49)]),
                                 ((.2, .7, 1.2), [(17, 17)])]:
            proposed = [pose_at(x) for x in positions]
            details = graph_diagnostics(previous, proposed, stamps)
            self.assertEqual(details['consistent'], graph_is_consistent(previous, proposed))
            self.assertEqual([(e['from_ns'], e['to_ns']) for e in details['violations']], pairs)
            self.assertAlmostEqual(details['origin']['distance_m'], positions[0])

    def test_graph_accepts_distributed_drift_and_rejects_a_jump(self):
        previous = [pose_at(x) for x in (0, .50, 1)]
        self.assertTrue(graph_is_consistent(previous, [pose_at(x) for x in (0, .57, 1.14)]))
        self.assertFalse(graph_is_consistent(previous, [pose_at(x) for x in (0, 4, 1)]))
        self.assertFalse(graph_is_consistent(previous[:1], [pose_at(.2)]))


if __name__ == '__main__':
    unittest.main()
