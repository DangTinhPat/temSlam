"""Regression for a return to the same point with a different heading."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np

from pose_check import loop_closure_check


class EndpointCheckTests(unittest.TestCase):
    def test_reference_heading_change_is_not_an_estimation_error(self):
        with TemporaryDirectory() as directory:
            session = Path(directory)
            start, end = np.eye(4), np.diag([-1., -1., 1., 1.])
            np.savez(session / 'rgbd.npz', mount=np.eye(4))
            (session / 'live_capture.jsonl').write_text(
                '{"t":1,"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":0,"qw":1}\n'
                '{"t":2,"x":0,"y":0,"z":0,"qx":0,"qy":0,"qz":1,"qw":0}\n'
            )
            (session / 'live_sequence.json').write_text(json.dumps({'frames': [
                {'time_s': 1, 'camera_to_world': start.tolist()},
                {'time_s': 2, 'camera_to_world': end.tolist()},
            ]}))
            (session / 'pose_quality.json').write_text(
                '{"pgo_events":1,"loop_closure_events":1}')
            route = {'waypoints': [{'x': 0, 'y': 0}, {'x': 0, 'y': 0}]}
            result = loop_closure_check(route, session)
            self.assertTrue(result['passed'])
            self.assertAlmostEqual(result['endpoint_rotation_error_deg'], 0)

            evidence = dict(verified=[], ambiguous=[{'edge': [3, 12]}], return_ns=[1900000000, 2100000000])
            with patch('graph_capture.assess_return_edges', return_value=evidence):
                recorded = loop_closure_check(route, session, require_return_edge=True)
                self.assertFalse(recorded['passed'])
                self.assertFalse(recorded['return_edge_verified'])
                evidence['verified'] = [{'edge': [3, 12]}]
                self.assertFalse(loop_closure_check(route, session, require_return_edge=True)['passed'])
                evidence['verified_in_final_graph'] = True
                self.assertTrue(loop_closure_check(route, session, require_return_edge=True)['passed'])
                evidence['return_ns'] = [3000000000, 3200000000]
                self.assertFalse(loop_closure_check(route, session, require_return_edge=True)['passed'])


if __name__ == '__main__':
    unittest.main()
