"""Bounds and fixed-denominator checks for the evaluator-only floor scorer."""
import json
from pathlib import Path
import unittest

import numpy as np

from review_floor_quality_prepared import FRAME_INDICES, validate_frame_indices, fixed_obstacle_counts


class FloorScorerTests(unittest.TestCase):
    def test_original_defaults_and_explicit_coffee48_bounds(self):
        self.assertEqual(validate_frame_indices(None, 150), FRAME_INDICES)
        manifest = json.loads((Path(__file__).resolve().parents[1] /
            'docs/superpowers/plans/2026-11-16-deltar-pilot-split.json').read_text())
        indices = tuple(s['prepared_frame_index'] for s in manifest['samples'] if s['split'] == 'validation')
        self.assertEqual(len(indices), 48)
        self.assertEqual(validate_frame_indices(indices, 60), indices)
        self.assertEqual((indices[0], indices[-1]), (0, 59))
        for invalid in ((), (0, 0), (-1,), (60,), (1.5,), (True,)):
            with self.assertRaises(ValueError):
                validate_frame_indices(invalid, 60)
        with self.assertRaises(ValueError):
            validate_frame_indices(None, 60)

    def test_lost_obstacle_depth_remains_in_baseline_denominator(self):
        reference = np.array([True, True, True, True, False])
        support = np.array([True, True, False, True, True])
        labels = np.array([2, 0, 0, 1, 2], np.uint8)
        counts = fixed_obstacle_counts(reference, support, labels)
        self.assertEqual(counts['baseline_valid_visible_obstacle_ge_025'], 4)
        self.assertEqual(counts['candidate_valid'], 3)
        self.assertEqual(counts['candidate_non_ground_blocked'], 2)
        self.assertEqual(counts['candidate_obstacle_only'], 1)
        self.assertEqual(counts['candidate_uncertain'], 1)
        self.assertEqual(counts['candidate_invalid'], 1)
        self.assertEqual(counts['candidate_ground_removed'], 1)


if __name__ == '__main__':
    unittest.main()
