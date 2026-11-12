"""Run the living-room scan without windows, then export the fusion-only mesh."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'gui'), str(ROOT/'technology/scan3d_fusion/mapping')]
from rb1_gui.backend import NativeBackend
from fixed_scan_route import FixedScanRoute


def main():
    lock = open('/tmp/rb-live-slam.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.environ.update(RB1_HEADLESS='1', RB1_LIVE_SLAM='1', RB1_SIM_SPEED_FACTOR='5',
                      RB1_SIM_PORT='37685', RB1_SIM_IMU_UNIT='648',
                      RB1_SIM_MODEL=str(ROOT/'simulation/scenarios/indoor_tiny_house_living_room.xml'))
    backend = NativeBackend(root=ROOT)
    route = FixedScanRoute(backend, ROOT/'technology/scan3d_fusion/mapping/native/living_room_scan_route.json')
    begin = time.monotonic()
    started = lowered = standup_sent = False
    last_phase = None
    success = False
    result = {'speed_factor': 5, 'headless': True, 'waypoints': len(route.waypoints)}
    try:
        backend.start()
        session = backend.map_session
        (session/'scan_route.json').write_text(json.dumps(route.route,indent=2)+'\n')
        print('SESSION', session, flush=True)
        while backend.running:
            backend.drain()
            fields = backend.telemetry
            # Match the GUI's initial settling/calibration interval before STANDUP.
            if not standup_sent and float(backend.view.get('t', 0)) >= 9:
                backend.command_standup()
                standup_sent = True
            failure = backend.live_scan.failure_reason()
            if failure:
                raise RuntimeError(failure)
            if fields.get('mode') in ('Fault', 'Fallen', 'EmergencyStop'):
                raise RuntimeError('Controller: '+fields.get('fault_reason', fields['mode']))
            settled = (fields.get('mode') == 'Balancing' and
                       fields.get('height_settled') == '1' and fields.get('height_motion') == '0')
            if settled and not lowered:
                backend.command_height(.14305)
                lowered = True
            elif (settled and lowered and not started and
                  abs(float(backend.view.get('body_height', '0'))-.14305) < .005 and
                  fields.get('drive_ready') == '1' and backend.live_scan.ready()):
                if not route.start(True, False):
                    raise RuntimeError(route.status)
                started = True
            if started:
                route.update(True, False)
                current = (route.index, route.phase)
                if current != last_phase:
                    print(route.index+1, route.status, flush=True)
                    last_phase = current
                if not route.active:
                    success = route.status == 'Đã hoàn tất tuyến quét map'
                    if not success:
                        raise RuntimeError(route.status)
                    break
            if time.monotonic()-begin > 1200:
                raise TimeoutError('Headless scan exceeded 20 wall minutes')
            time.sleep(.01)
        if not success:
            raise RuntimeError('Simulator ended before route completion')
        backend.send('STOP')
        # Drain the last inference before freezing the final keyframe sequence.
        end = time.monotonic()+3
        while time.monotonic() < end:
            backend.drain()
            time.sleep(.02)
        result.update(success=True, status=route.status, final_index=route.index,
                      sim_time_s=float(backend.view.get('t', 0)))
    except Exception as exc:
        result.update(success=False, error=str(exc))
        raise
    finally:
        backend.stop()
        result['wall_seconds'] = time.monotonic()-begin
        if backend.map_session:
            (backend.map_session/'headless_result.json').write_text(json.dumps(result, indent=2)+'\n')
    env = os.environ.copy()
    env.pop('PYTHONPATH', None)
    env['MUJOCO_GL'] = 'egl'
    python = '/home/dvt/.venvs/deltar/bin/python'
    subprocess.run([python, str(Path(__file__).with_name('workers.py')), 'finalize', str(session)], env=env, check=True)
    subprocess.run([python, str(ROOT/'technology/scan3d_fusion/reconstruction/render_mesh.py'),
                    str(session/'final_mesh.stl'), '--output', str(session/'result.png'),
                    '--elevation', '-60'], env=env, check=True)
    print('RESULT', session/'result.png', flush=True)


if __name__ == '__main__':
    main()
