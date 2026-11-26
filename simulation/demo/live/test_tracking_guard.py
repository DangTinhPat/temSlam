import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch

import numpy as np

from session import LiveSession
from worker_tracking import tracking
from vslam.test_pose_guard import pose


class TrackingGuardIntegrationTests(TestCase):
    def test_jump_keeps_published_snapshot_even_if_sdk_changes_history(self):
        self.run_tracking_case(stop_time=3.5)

    def test_accepted_graph_and_final_snapshot_keep_measured_height(self):
        self.run_tracking_case(stop_time=.3)

    def run_tracking_case(self, stop_time):
        with TemporaryDirectory() as directory:
            session = Path(directory)
            timestamps = iter(((.1, 0), (.2, 0), (.3, 0), (.35, 0), (.4, 4.88), (.5, 4.90), (3.5, 4.91)))
            packet = dict(time_s=0, valid=1, x=0, y=0, yaw=0, pitch=0, roll=0,
                          axle_body_x=0, axle_body_z=0, wheel_radius=.035, head_yaw=0)
            base = dict(depth_source=np.array('deltar_tof_fusion'),
                        depth=np.full((480, 640), 1000, np.uint16),
                        rgb=np.zeros((480, 640, 3), np.uint8), K=np.array([500, 500, 320, 240]),
                        mount=np.eye(4), tof_hist=np.zeros((64, 2)),
                        tof_rect=np.zeros((64, 4)), tof_valid=np.zeros(64))
            base['rgb'][:, :320] = 150
            pending = {}
            worker = LiveSession.__new__(LiveSession)
            worker.session = session

            class Batch(dict):
                files = tuple(base) + ('t', 'odometry')

                def __enter__(self):
                    return self

                def __exit__(self, *_):
                    pass

            def load(*_, **__):
                t, x = next(timestamps)
                pending[t] = x
                if t == stop_time:
                    (session / 'STOP').touch()
                batch = Batch(base, t=t, odometry=json.dumps(dict(packet, time_s=t)))
                if t == .35:
                    batch['rgb'] = np.full_like(base['rgb'], 255)
                return batch

            class Tracker:
                def __init__(self, *_):
                    self.history = {}
                    self.tracker = self
                    self.last_odometry_pose = None

                def track(self, ts, *args):
                    self.last_odometry_pose = pose(pending[ts / 1e9])
                    self.last_odometry_pose[2, 3] = min(.04, .02 * (len(self.history)))
                    if self.last_odometry_pose[0, 3] > 4:
                        self.history = {stamp: pose(9) for stamp in self.history}
                    self.history[ts] = self.last_odometry_pose
                    return self.last_odometry_pose

                def get_slam_metrics(self):
                    if len(self.history) == 2:
                        return SimpleNamespace(timestamp_ns=200000000, lc_status=True, pgo_status=True)
                    return None

                def save_map(self, folder, callback):
                    callback(True)

                def get_all_slam_poses(self):
                    return [SimpleNamespace(timestamp_ns=ts, pose=matrix)
                            for ts, matrix in self.history.items()]

            with (patch.dict(os.environ, RB1_RECORD_RGBD='1'),
                  patch('worker_tracking.np.load', side_effect=load),
                  patch('worker_tracking.startup_tracking_state', return_value=(None, None, True)),
                  patch('vslam.tracker.RgbdSlam', Tracker),
                  patch('vslam.tracker.pose_matrix', side_effect=lambda value: value)):
                tracking(session)

            seq = json.loads((session / 'live_sequence.json').read_text())
            self.assertEqual([f['time_s'] for f in seq['frames']], [.1, .2, .3])
            np.testing.assert_allclose([f['camera_to_world'][2][3] for f in seq['frames']], [.035]*3)
            self.assertAlmostEqual(seq['frames'][-1]['visual_camera_to_world'][2][3], .075)
            with np.load(session/'graph_snapshots/final.npz') as evidence:
                np.testing.assert_array_equal(evidence['timestamp_ns'], [100000000, 200000000, 300000000])
                np.testing.assert_allclose(evidence['mapped'][:, 2, 3], [.035]*3)
                np.testing.assert_allclose(evidence['previous'][:, 0, 3], [0.]*3)
                np.testing.assert_allclose(evidence['candidate'][:, 0, 3], [0. if stop_time == .3 else 9.]*3)
            events = [json.loads(line) for line in (session/'graph_snapshots/events.jsonl').read_text().splitlines()]
            final = next(e for e in events if e.get('snapshot') == 'final.npz')
            self.assertEqual(final['decision'], 'applied' if stop_time == .3 else 'held')
            if stop_time == .3:
                log = [json.loads(line) for line in (session / 'map_refresh_diagnostics.jsonl').read_text().splitlines()]
                self.assertEqual((log[-1]['reason'], log[-1]['graph_result']), ('final', 'applied'))
                self.assertEqual(seq['pose_stage'], 'final')
                return
            self.assertTrue(seq['tracking_interrupted'])
            rows = [json.loads(line) for line in (session / 'tracking_diagnostics.jsonl').read_text().splitlines()]
            flat = next(r for r in rows if r['sensor_time'] == .35)
            self.assertEqual((flat['tracking_state'], flat['input_issue']), ('hold', 'flat_rgb'))
            self.assertEqual([f['camera_to_world'][0][3] for f in seq['frames']], [0., 0., 0.])
            self.assertFalse(worker.ready())
