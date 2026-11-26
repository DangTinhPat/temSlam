from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch

import numpy as np

from worker_reconstruction import LiveVolume


class LiveVolumeTests(TestCase):
    def test_appends_views_but_rebuilds_on_pose_or_bounds_change(self):
        sample = dict(bounds_m=[[0, 0, 0], [1, 1, 1]], intrinsics=[1, 1, 1, 1])
        a, b = {'file': 'a', 'camera_to_world': [0]}, {'file': 'b', 'camera_to_world': [1]}
        live = LiveVolume()
        with (patch('worker_reconstruction.new_volume',
                    side_effect=lambda bounds: SimpleNamespace(bounds=np.asarray(bounds))) as build,
              patch('worker_reconstruction.integrate') as integrate):
            self.assertEqual(live.update(None, dict(sample, frames=[a])), 1)
            self.assertEqual(live.update(None, dict(sample, frames=[a, b])), 1)
            self.assertEqual(integrate.call_args.args[2], [b])
            self.assertEqual(build.call_count, 1)
            corrected = [dict(a, camera_to_world=[.1]), b]
            self.assertEqual(live.update(None, dict(sample, frames=corrected)), 2)
            self.assertEqual(integrate.call_args.args[2], corrected)
            self.assertEqual(build.call_count, 2)
            self.assertEqual(live.update(None, dict(sample, frames=corrected,
                                                   bounds_m=[[0, 0, 0], [3, 1, 1]])), 2)
            self.assertEqual(build.call_count, 3)
