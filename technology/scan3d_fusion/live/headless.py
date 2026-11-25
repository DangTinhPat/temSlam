"""Run a fixed RGB-D scan without windows, then export its fusion-only mesh."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'gui'), str(ROOT/'technology/scan3d_fusion/mapping')]
from rb1_gui.backend import NativeBackend
from fixed_scan_route import FixedScanRoute


def loop_closure_check(route, session):
    points = route.get('waypoints', [])
    if len(points) < 2 or math.hypot(points[-1]['x']-points[0]['x'],
                                     points[-1]['y']-points[0]['y']) > .1:
        return {'passed': False, 'reason': 'Tuyến chưa khép vòng'}
    try:
        quality = json.loads((session/'pose_quality.json').read_text())
        frames = json.loads((session/'live_sequence.json').read_text())['frames']
        start, end = frames[0]['camera_to_world'], frames[-1]['camera_to_world']
        translation = math.sqrt(sum((float(start[i][3])-float(end[i][3]))**2 for i in range(3)))
        trace = sum(float(start[j][i])*float(end[j][i]) for i in range(3) for j in range(3))
        rotation = math.degrees(math.acos(max(-1., min(1., (trace-1.)/2.))))
        pgo_events = int(quality.get('pgo_events', 0))
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return {'passed': False, 'reason': 'Thiếu lịch sử pose/PGO cuối'}
    passed = pgo_events > 0 and translation <= .1 and rotation <= 5.
    return {'passed': passed, 'pgo_events': pgo_events,
            'closure_translation_m': translation, 'closure_rotation_deg': rotation,
            'reason': 'Đạt kiểm tra khép vòng' if passed else 'PGO hoặc sai số khép vòng chưa đạt'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', type=Path, default=ROOT/'technology/scan3d_fusion/mapping/native/living_room_scan_route.json')
    parser.add_argument('--speed-factor', type=int, default=1)
    parser.add_argument('--pose-policy',choices=['hybrid','cuvslam'],default='hybrid')
    args = parser.parse_args()
    route_path = args.route.resolve()
    route_spec = json.loads(route_path.read_text())
    model_path = ROOT/route_spec['scene']
    if args.speed_factor < 1:
        parser.error('--speed-factor must be at least 1')
    lock = open('/tmp/rb-live-slam.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    os.environ.update(RB1_HEADLESS='1', RB1_LIVE_SLAM='1', RB1_SIM_SPEED_FACTOR=str(args.speed_factor),
                      RB1_SIM_PORT='37685', RB1_SIM_IMU_UNIT='648',
                      RB1_SLAM_POSE_POLICY=args.pose_policy,
                      RB1_SIM_MODEL=str(model_path))
    backend = NativeBackend(root=ROOT)
    route = FixedScanRoute(backend, route_path)
    begin = time.monotonic()
    started = lowered = standup_sent = False
    last_phase = None
    success = False
    result = {'speed_factor': args.speed_factor, 'headless': True, 'route': str(route_path),
              'waypoints': len(route.waypoints),'pose_policy':args.pose_policy}
    shutdown_signals=(signal.SIGINT,signal.SIGTERM,signal.SIGHUP)
    previous_handlers={}
    def request_shutdown(signum,_frame):
        for caught in shutdown_signals:signal.signal(caught,signal.SIG_IGN)
        raise RuntimeError(f'received {signal.Signals(signum).name}')
    for signum in shutdown_signals:
        previous_handlers[signum]=signal.signal(signum,request_shutdown)
    route_speed=min((float(p.get('speed_m_s',.1)) for p in route.waypoints),default=.1)
    scan_stops=sum(bool(p.get('scan')) for p in route.waypoints)
    budget=max(1200+360*sum(bool(p.get('scan_step_deg')) for p in route.waypoints),
               int(2*(route.route.get('route_length_m', 0)/route_speed+
                       360*scan_stops)/args.speed_factor))
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
            if time.monotonic()-begin > budget:
                raise TimeoutError('Headless scan exceeded its route capture budget')
            time.sleep(.01)
        if not success:
            raise RuntimeError('Simulator ended before route completion')
        backend.send('STOP')
        # Drain the last inference before freezing the final keyframe sequence.
        end = time.monotonic()+3
        while time.monotonic() < end:
            backend.drain()
            time.sleep(.02)
        result.update(route_completed=True, status=route.status, final_index=route.index,
                      sim_time_s=float(backend.view.get('t', 0)))
    except Exception as exc:
        result.update(success=False, error=str(exc))
        raise
    finally:
        try:
            backend.stop()
        finally:
            for signum,handler in previous_handlers.items():signal.signal(signum,handler)
            lock.close()
            result['wall_seconds'] = time.monotonic()-begin
            if backend.map_session:
                (backend.map_session/'headless_result.json').write_text(json.dumps(result, indent=2)+'\n')
    env = os.environ.copy()
    shutdown=json.loads((session/'slam_shutdown.json').read_text())
    if not shutdown['clean_stop']:
        raise RuntimeError('Refuse finalize after incomplete SLAM shutdown: '+shutdown['reason'])
    env.pop('PYTHONPATH', None)
    env['MUJOCO_GL'] = 'egl'
    python = '/home/dvt/.venvs/deltar/bin/python'
    subprocess.run([python, str(Path(__file__).with_name('workers.py')), 'finalize', str(session)], env=env, check=True)
    subprocess.run([python, str(ROOT/'technology/scan3d_fusion/reconstruction/render_mesh.py'),
                    str(session/'final_mesh.stl'), '--output', str(session/'result.png'),
                    '--elevation', '-60'], env=env, check=True)
    check = loop_closure_check(route.route, session)
    result.update(success=check['passed'], loop_closure_check=check)
    if not check['passed']:
        result['error'] = 'Tuyến đã chạy xong nhưng bản đồ chưa được xác nhận: '+check['reason']
    (session/'headless_result.json').write_text(json.dumps(result, indent=2)+'\n')
    print('RESULT', session/'result.png', flush=True)
    if not check['passed']:
        raise RuntimeError(result['error'])


if __name__ == '__main__':
    main()
