"""Native integration regression; simulator truth is used only as a test oracle."""
import json
from pathlib import Path
import subprocess
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]


class MappingAttitudeTest(unittest.TestCase):
    def test_home_zero_reaches_mapping_with_noisy_and_ideal_imu(self):
        results = {}
        for noisy in (0, 1):
            command = [str(ROOT/'firmware/build/bin/rb1_sim'), '--model',
                       str(ROOT/'simulation/scenarios/indoor_tiny_house_living_room.xml'),
                       '--start-key', 'home', '--capture-sync', '--imu-model', str(noisy),
                       '--imu-unit', '648', '--run-for', '65', '--at', '9:STANDUP',
                       '--at', '40:HEIGHT 0.14305', '--at', '48:HEAD -3.14',
                       '--at', '55:HEAD 3.14', '--at', '62:HEAD 0']
            run = subprocess.run(command, capture_output=True, text=True, timeout=90, check=True)
            errors = []
            packet = None
            for line in run.stdout.splitlines():
                if line.startswith('TOFMAP '):
                    packet = list(map(float, line.split()[1:]))
                elif line.startswith('CAPTURE|') and packet and packet[0] >= 45 and packet[1]:
                    v = dict(p.split('=', 1) for p in line.split('|')[1:])
                    self.assertAlmostEqual(packet[0], float(v['t']), places=3)
                    truth = Rotation.from_quat([float(v[k]) for k in ('qx','qy','qz','qw')])
                    pitch_roll = truth.as_euler('ZYX')[1:]
                    errors.append(np.degrees(np.array(packet[5:7])-pitch_roll))
            self.assertGreater(len(errors), 200, 'Robot must stand and remain valid during head rotation')
            p95 = np.quantile(np.abs(errors), .95, axis=0)
            self.assertTrue(np.all(p95 < .6), f'IMU={noisy}: pitch/roll p95={p95} degrees')
            results[str(noisy)] = dict(samples=len(errors), p95_pitch_roll_deg=p95.tolist())
        print(json.dumps(results, indent=2))


if __name__ == '__main__':
    unittest.main()
