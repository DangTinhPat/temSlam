"""Regression for missed camera frames after a paused, paced physics process."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def check(runner, factor):
    scene = Path(__file__).resolve().parents[1] / 'scenarios/bmap1_demo_fixed_camera.xml'
    with subprocess.Popen([runner, '--model', str(scene), '--capture-sync', '--start-key', 'home',
                           '--speed-factor', str(factor), '--run-for', '1.2'],
                          stdout=subprocess.PIPE, text=True) as process:
        resumed_at = sim_at_pause = None
        for line in process.stdout:
            if not line.startswith('CAPTURE|t='):
                continue
            t = float(line.split('|', 2)[1][2:])
            if resumed_at is None and t >= .25:
                os.kill(process.pid, signal.SIGSTOP)
                try:
                    time.sleep(.7)
                finally:
                    resumed_at, sim_at_pause = time.monotonic(), t
                    os.kill(process.pid, signal.SIGCONT)
            elif resumed_at is not None and t - sim_at_pause >= .6:
                elapsed = time.monotonic() - resumed_at
                assert elapsed >= .40 / factor, (t, sim_at_pause, elapsed, factor)
                break
        else:
            raise AssertionError('Missing synchronized camera captures')
        process.communicate(timeout=5)
        assert process.returncode == 0


if __name__ == '__main__':
    for factor in (1, 2):
        check(sys.argv[1], factor)
