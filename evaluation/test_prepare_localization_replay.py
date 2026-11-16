import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

import prepare_localization_replay as replay


def capture(t, **changes):
    return dict(t=t, x=12., y=-3., z=.14, qw=1., qx=0., qy=0., qz=0.,
                lh=0., lk=0., lw=0., rh=0., rk=0., rw=0., hj=0., **changes)


def packet(t, valid=1):
    return dict(time_s=t, x=0., y=0., yaw=0., pitch=0., roll=0.,
                axle_body_x=0., axle_body_z=-.1, wheel_radius=.035,
                valid=valid, slip=False, head_yaw=0.)


class PreparationTests(unittest.TestCase):
    def test_selection_uses_nearest_packet_then_excludes_invalid_or_unsynchronized(self):
        captures = [capture(t) for t in [0., .1, .2, .4, .6, .8, 1.]]
        odometry = [packet(0.), packet(.2, 0), packet(.21), packet(.43), packet(.6), packet(.8)]
        selected = replay.select_samples(captures, odometry, start=0, duration=1, period=.2)
        self.assertEqual([c['t'] for c, p in selected], [0., .6, .8])
        self.assertEqual([p['time_s'] for c, p in selected], [0., .6, .8])

    def test_selection_rejects_reset_and_invalid_window_and_skips_bad_measurements(self):
        for kwargs in [dict(start=-1, duration=1, period=.2), dict(start=0, duration=0, period=.2),
                       dict(start=0, duration=1, period=float('nan'))]:
            with self.assertRaises(ValueError):
                replay.select_samples([capture(0)], [packet(0)], **kwargs)
        with self.assertRaises(ValueError):
            replay.select_samples([capture(0)], [dict(reset=True)], 0, 1, .2)
        bad = capture(.2); bad['hj'] = float('nan')
        bad_packet = packet(.4); bad_packet['wheel_radius'] = -1
        self.assertEqual(len(replay.select_samples([capture(0), bad, capture(.4)],
                                                 [packet(0), packet(.2), bad_packet], 0, 1, .2)), 1)

    def test_output_is_new_outside_source_and_measurements_exclude_world_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)/'raw'; source.mkdir()
            (source/'metadata.json').write_text(json.dumps(dict(runner_command=['sim', '--model', '/model.xml'])))
            (source/'live_capture.jsonl').write_text(json.dumps(capture(1.))+'\n')
            (source/'odometry.jsonl').write_text(json.dumps(packet(1.))+'\n')
            original = {p.name: p.read_bytes() for p in source.iterdir()}
            for output in [source, source/'generated']:
                with self.assertRaises((ValueError, FileExistsError)):
                    replay.prepare(source, output, 1, .5, .2)
            out = Path(tmp)/'prepared'
            sensor = SimpleNamespace(K=[500., 500., 319.5, 239.5],
                cal=dict(rotation_matrix=np.eye(3).tolist(), translation_m=[0, 0, 0]),
                rgb=SimpleNamespace(close=lambda: None), tof=SimpleNamespace(close=lambda: None),
                render=lambda v: (np.zeros((480,640,3), np.uint8), np.tile([1., .01], (64,1)),
                                  np.tile([0,0,480,640], (64,1)), np.ones(64, bool), np.eye(4)))
            def predict(command, **kwargs):
                target = Path(command[command.index('--output')+1]); target.mkdir()
                Image.fromarray(np.full((480,640), 1000, np.uint16)).save(target/'00000_depth.png')
            with patch.object(replay, 'Sensors', return_value=sensor), patch.object(replay.subprocess, 'run', side_effect=predict):
                replay.prepare(source, out, 1, .5, .2)
            manifest = json.loads((out/'localization_input.json').read_text())
            frame = manifest['frames'][0]
            self.assertEqual(set(frame), {'file', 'time_s', 'odometry', 'body_from_camera'})
            self.assertEqual(frame['odometry']['x'], 0.)
            self.assertEqual(frame['body_from_camera'], np.eye(4).tolist())
            truth = json.loads((out/'evaluator_truth.json').read_text())
            self.assertEqual(np.asarray(truth['frames'][0]['simulator_from_body'])[:3,3].tolist(), [12., -3., .14])
            with np.load(out/frame['file'], allow_pickle=False) as sample:
                self.assertEqual(str(sample['depth_source']), 'deltar_tof_fusion')
                self.assertGreater(np.count_nonzero(sample['depth']), 0)
                self.assertNotIn('camera_to_world', sample)
            self.assertEqual({p.name: p.read_bytes() for p in source.iterdir()}, original)
            with self.assertRaises(FileExistsError):
                replay.prepare(source, out, 1, .5, .2)


if __name__ == '__main__':
    unittest.main()
