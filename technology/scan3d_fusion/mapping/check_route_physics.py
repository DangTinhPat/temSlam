"""Run the actual native controller through the fixed route, without GUI/GPU workers.

Uses an isolated simulator via stdin. Existing simulation sessions are untouched.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
from fixed_scan_route import FixedScanRoute

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--points', type=int, default=0, help='0 tests the complete route')
    parser.add_argument('--route',type=Path,default=Path(__file__).with_name('native')/'fixed_scan_route.json')
    args = parser.parse_args()
    spec=json.loads(args.route.read_text())
    command = [str(ROOT/'firmware/build/bin/rb1_sim'), '--model',
               str(ROOT/spec['scene']), '--rate', '3000',
               '--state-hz', '50', '--start-key', 'home', '--imu-model', '1', '--imu-unit', '648',
               '--at', '9:STANDUP', '--run-for', '7000']
    child = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)
    def send(command):
        child.stdin.write(command+'\n'); child.stdin.flush()
    backend = SimpleNamespace(root=ROOT, running=True, process=child, map_session=None,
                              telemetry={}, view={}, log=[], send=send,
                              command_drive=lambda v, w: send(f'DRIVE {v} {w}'))
    route = FixedScanRoute(backend, args.route)
    started = False; previous = None; samples = []; phases = []; success = False
    lowered=False
    begin = time.monotonic()
    try:
        for line in child.stdout:
            if not line.startswith(('STATE|', 'VIEW|')): continue
            fields = {k: v.strip() for k, v in (p.split('=', 1) for p in line.split('|')[1:] if '=' in p)}
            if line.startswith('STATE|'):
                backend.telemetry = fields
                continue
            backend.view = fields
            t = float(fields['t'])
            if backend.telemetry.get('mode') in ('Fault','Fallen','EmergencyStop'):
                raise RuntimeError('Controller: '+backend.telemetry.get('fault_reason','fault'))
            settled=(backend.telemetry.get('mode')=='Balancing' and
                     backend.telemetry.get('height_settled')=='1' and
                     backend.telemetry.get('height_motion')=='0')
            if not lowered:
                if settled:
                    send('HEIGHT 0.14305');lowered=True
                continue
            if not started:
                # Match the live session's warm-up before commanding motion after stand-up.
                if (t<43 or not settled or abs(float(fields.get('body_height',0))-.14305)>.005 or
                        backend.telemetry.get('drive_ready')!='1'):continue
                assert route.start(True, False), route.status
                started = True
            route.update(True, False)
            state = (route.index, route.phase)
            if state != previous:
                row = dict(t=t, index=route.index, phase=route.phase, status=route.status)
                phases.append(row); print(json.dumps(row, ensure_ascii=False), flush=True)
                previous = state
            if not samples or t-float(samples[-1]['view']['t']) >= .2:
                samples.append(dict(view=fields, mode=backend.telemetry.get('mode'),
                                    pitch=backend.telemetry.get('pitch'), index=route.index))
            if not route.active or (args.points and route.index >= args.points):
                success = route.status == 'Đã hoàn tất tuyến quét map' or (route.active and args.points and route.index >= args.points)
                break
            if time.monotonic()-begin > 900:
                raise TimeoutError('Physics check exceeded 900 wall seconds')
    finally:
        child.terminate(); child.wait(timeout=5)
        result = dict(success=bool(success), wall_seconds=time.monotonic()-begin,
                      waypoints=len(route.waypoints), final_index=route.index, status=route.status,
                      phases=phases, samples=samples)
        args.output.write_text(json.dumps(result, ensure_ascii=False))
    assert success, route.status
    assert all(s['mode'] == 'Balancing' for s in samples), 'Controller lost balance'
    print(f'PASS: {route.index}/{len(route.waypoints)} waypoints, output {args.output}')

if __name__ == '__main__': main()
