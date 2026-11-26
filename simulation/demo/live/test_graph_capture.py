import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase

import numpy as np

from graph_capture import GraphCapture, return_edge_evidence, assess_return_edges
from vslam.tracker import MAP_FROM_START


def native(x):
    return SimpleNamespace(translation=[x, 0., 0.], rotation=[0., 0., 0., 1.])


class FakeSDK:
    def __init__(self):
        self.history = {}
        self.nodes = []
        self.edges = []
        self.event = None

    def get_pose_graph(self):
        return SimpleNamespace(nodes=[SimpleNamespace(id=n, node_pose=native(n)) for n in self.nodes],
                               edges=[SimpleNamespace(node_from=a, node_to=b, transform=native(b-a),
                                                      covariance=np.eye(6).ravel()) for a, b in self.edges])

    def get_slam_metrics(self):
        return self.event

    def get_all_slam_poses(self):
        return [SimpleNamespace(timestamp_ns=ts, pose=native(x)) for ts, x in self.history.items()]


class GraphCaptureTests(TestCase):
    def test_edges_use_sdk_ids_and_consecutive_observations(self):
        with TemporaryDirectory() as tmp:
            capture = GraphCapture(Path(tmp))
            sdk = FakeSDK()
            sdk.nodes, sdk.edges = [100, 901, 577], [(100, 901), (901, 577)]
            capture.observe(sdk, 5, [])
            sdk.edges.append((100, 577))
            sdk.event = SimpleNamespace(timestamp_ns=12, lc_status=True, pgo_status=True,
                                        lc_tracked_landmarks_count=21)
            capture.observe(sdk, 12, [])
            capture.observe(sdk, 20, [])  # metrics may repeat between events
            sdk.event.timestamp_ns = 24
            sdk.nodes.append(971)
            sdk.edges.extend([(577, 971), (100, 971)])
            capture.observe(sdk, 24, [])
            rows = [json.loads(s) for s in (capture.folder/'events.jsonl').read_text().splitlines()]
            topology = [r for r in rows if r['kind'] == 'topology']
            self.assertEqual(topology[0]['new_nodes'], [100, 577, 901])
            self.assertEqual(topology[1]['isolated_lc_edge'], [100, 577])
            self.assertEqual(topology[1]['previous_read_ns'], 5)
            self.assertIsNone(topology[2]['isolated_lc_edge'])
            self.assertEqual([r['metrics']['lc_tracked_landmarks_count'] for r in rows
                              if r['kind'] == 'sdk_event'], [21, 21])
            with np.load(capture.folder/'topology_12.npz') as f:
                self.assertEqual(f['endpoints'].tolist()[-1], [100, 577])
                self.assertEqual(f['covariance'].shape, (3, 36))

    def test_sdk_predecessor_and_loop_edges_are_attributable_only_with_known_head(self):
        with TemporaryDirectory() as tmp:
            capture, sdk = GraphCapture(Path(tmp)), FakeSDK()
            sdk.nodes, sdk.edges = [5], []
            capture.observe(sdk, 1, [])
            for t, node, edges in [(2, 91, [(5, 91)]), (3, 50, [(91, 50)]),
                                   (4, 3, [(50, 3), (5, 3)]), (5, 7, [(50, 7), (5, 7)])]:
                sdk.nodes.append(node)
                sdk.edges.extend(edges)
                if t >= 4:
                    sdk.event = SimpleNamespace(timestamp_ns=t, lc_status=True, pgo_status=True)
                capture.observe(sdk, t, [])
            rows = [json.loads(s) for s in (capture.folder/'events.jsonl').read_text().splitlines()]
            events = {r['timestamp_ns']: r for r in rows if r['kind'] == 'topology'}
            self.assertEqual(events[4]['isolated_lc_edge'], [5, 3])
            self.assertIsNone(events[5]['isolated_lc_edge'])

    def test_return_edge_requires_same_epoch_synchronous_attribution_and_unambiguous_first_sightings(self):
        def row(t, prev, nodes=(), edges=(), isolated=None, epoch=0):
            return dict(kind='topology', timestamp_ns=t, previous_read_ns=prev, new_nodes=nodes,
                        new_edges=edges, isolated_lc_edge=isolated, epoch=epoch)

        rows = [row(11, 10, [90]), row(65, 55, [700]), row(72, 71, [4]),
                row(75, 74, edges=[[4, 90]], isolated=[4, 90]),
                row(77, 76, edges=[[90, 700]], isolated=[90, 700]),
                row(78, 77, edges=[[90, 4], [4, 700]]),
                row(79, 78, edges=[[4, 90]], isolated=[4, 90], epoch=1)]
        evidence = return_edge_evidence(rows, (10, 15), (60, 80))
        self.assertEqual(evidence['verified'], [dict(epoch=0, timestamp_ns=75, edge=[4, 90])])
        self.assertEqual(evidence['ambiguous'], [dict(epoch=0, timestamp_ns=78, edge=[90, 4])])

    def test_delayed_metric_joins_prior_topology_and_capture_windows(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp)
            capture = GraphCapture(path)
            track_dir = path/'tracker_input'
            track_dir.mkdir()
            sdk = FakeSDK()
            route = dict(waypoints=[dict(x=0,y=0), dict(x=3,y=0), dict(x=0,y=0)])
            (path/'scan_route.json').write_text(json.dumps(route))
            (path/'headless_result.json').write_text(json.dumps(dict(route_completed=True)))
            with (path/'live_capture.jsonl').open('w') as stream:
                for t, x in enumerate((0., .45, .1, .2, 1., 3., 1., .2, .15, .1, 0.)):
                    stream.write(json.dumps(dict(t=t, x=x, y=0.))+'\n')
            for t in range(1, 11):
                ts = t * 10**9
                (track_dir/f'{ts:020d}.npz').touch()
                if t in (3, 4, 6, 8):
                    if sdk.nodes:
                        sdk.edges.append((sdk.nodes[-1], t))
                    sdk.nodes.append(t)
                if t == 8:
                    sdk.edges.append((3, 8))
                if t == 9:
                    sdk.event = SimpleNamespace(timestamp_ns=8*10**9, lc_status=True, pgo_status=True)
                capture.observe(sdk, ts, [])
            report = assess_return_edges(path)
            pair = report['attributable_edges'][0]
            self.assertEqual(pair['first_seen_ns'], [3*10**9, 8*10**9])
            self.assertAlmostEqual(pair['reference_xy_gap_m'], .05)
            self.assertEqual(report['outbound_ns'], [2*10**9, 3*10**9])
            self.assertEqual(report['return_ns'], [7*10**9, 10*10**9])
            self.assertEqual(report['verified'], [dict(epoch=0, timestamp_ns=8*10**9, edge=[3,8])])
            self.assertFalse(report['verified_in_final_graph'])
            capture.log('snapshot', 10*10**9, snapshot='final.npz', decision='applied')
            self.assertTrue(assess_return_edges(path)['verified_in_final_graph'])
            # A landmark can be visited twice without being the start or end of the run.
            landmark = assess_return_edges(path, region=(1., 0., .1))
            self.assertEqual(landmark['outbound_ns'], [4*10**9, 4*10**9])
            self.assertEqual(landmark['return_ns'], [6*10**9, 6*10**9])
            (path/'headless_result.json').write_text(json.dumps(dict(route_completed=False)))
            self.assertFalse(assess_return_edges(path)['verified'])

    def test_missing_middle_pose_still_reports_observable_errors(self):
        with TemporaryDirectory() as tmp:
            capture = GraphCapture(Path(tmp))
            sdk = FakeSDK()
            capture.alignment = np.linalg.inv(MAP_FROM_START)
            sdk.history = {1: .3, 3: 12., 4: 13.}
            frames = [dict(timestamp_ns=i, tracker_epoch=0, visual_camera_to_world=np.eye(4),
                           camera_to_world=np.eye(4)) for i in range(1, 5)]
            capture.snapshot(sdk, 5, frames, 'final', 'missing_poses')
            row = json.loads((capture.folder/'events.jsonl').read_text())
            self.assertEqual(row['missing_ns'], [2])
            self.assertIsNone(row['consistent'])
            self.assertAlmostEqual(row['origin']['distance_m'], .3)
            self.assertEqual([(v['from_ns'], v['to_ns']) for v in row['violations']], [(1, 1), (3, 4)])

    def test_sdk_history_read_failure_keeps_prior_poses_and_marks_candidates_missing(self):
        with TemporaryDirectory() as tmp:
            capture = GraphCapture(Path(tmp))
            capture.alignment = np.eye(4)
            frame = dict(timestamp_ns=777, tracker_epoch=0, visual_camera_to_world=np.eye(4),
                         camera_to_world=np.eye(4))
            capture.snapshot(object(), 900, [frame], 'final', 'held')
            with np.load(capture.folder/'final.npz') as evidence:
                self.assertTrue(np.isnan(evidence['candidate'][0]).all())
                np.testing.assert_equal(evidence['previous'][0], np.eye(4))
            rows = [json.loads(s) for s in (capture.folder/'events.jsonl').read_text().splitlines()]
            self.assertEqual(rows[-1]['missing_ns'], [777])
            self.assertEqual(rows[0]['kind'], 'capture_error')

    def test_restart_preserves_original_partial_epoch_in_final(self):
        with TemporaryDirectory() as tmp:
            capture = GraphCapture(Path(tmp))
            sdk = FakeSDK()
            template = dict(visual_camera_to_world=np.eye(4), camera_to_world=np.eye(4))
            frames = [dict(template, timestamp_ns=1, tracker_epoch=0),
                      dict(template, timestamp_ns=2, tracker_epoch=0)]
            capture.alignment = np.linalg.inv(MAP_FROM_START)
            sdk.history = {1: 3.}
            capture.restart(sdk, 9, frames)
            self.assertIsNone(capture.alignment)
            frames.append(dict(template, timestamp_ns=10, tracker_epoch=1))
            sdk.history = {1: 88., 2: 88., 10: 0.}
            capture.alignment = np.linalg.inv(MAP_FROM_START)
            capture.alignment[0, 3] = 25.
            capture.snapshot(sdk, 11, frames, 'final', 'restarted')
            with np.load(capture.folder/'final.npz') as f:
                self.assertEqual(f['tracker_epoch'].tolist(), [0, 0, 1])
                self.assertEqual(f['candidate'][:, 0, 3][[0, 2]].tolist(), [3., 25.])
                self.assertTrue(np.isnan(f['candidate'][1]).all())
            rows = [json.loads(s) for s in (capture.folder/'events.jsonl').read_text().splitlines()]
            self.assertEqual(rows[-1]['missing_ns'], [2])
            self.assertEqual([r['timestamp_ns'] for r in rows if r['kind'] == 'restart'], [9])
