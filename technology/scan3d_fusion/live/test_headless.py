import json
import math
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import headless


class HeadlessSignalTests(unittest.TestCase):
    def test_loop_closure_check_requires_closed_route_pgo_and_pose_closure(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            pose = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            sequence = {'frames':[{'camera_to_world':pose},
                                  {'camera_to_world':[row[:] for row in pose]}]}
            (session/'live_sequence.json').write_text(json.dumps(sequence))
            (session/'pose_quality.json').write_text(json.dumps({'pgo_events':1}))
            route = {'waypoints':[{'x':0,'y':0},{'x':0,'y':0}]}
            self.assertTrue(headless.loop_closure_check(route,session)['passed'])
            (session/'pose_quality.json').write_text(json.dumps({'pgo_events':0}))
            self.assertFalse(headless.loop_closure_check(route,session)['passed'])
            (session/'pose_quality.json').write_text(json.dumps({'pgo_events':1}))
            angle = math.radians(10)
            sequence['frames'][-1]['camera_to_world'] = [
                [math.cos(angle),-math.sin(angle),0,0],
                [math.sin(angle), math.cos(angle),0,0],
                [0,0,1,0],[0,0,0,1]]
            (session/'live_sequence.json').write_text(json.dumps(sequence))
            self.assertFalse(headless.loop_closure_check(route,session)['passed'])
            sequence['frames'][-1]['camera_to_world'] = [row[:] for row in pose]
            route['waypoints'][-1]['x']=1
            self.assertFalse(headless.loop_closure_check(route,session)['passed'])
            route['waypoints'][-1]['x']=0
            sequence['frames'][-1]['camera_to_world'][0][3]=.2
            (session/'live_sequence.json').write_text(json.dumps(sequence))
            self.assertFalse(headless.loop_closure_check(route,session)['passed'])

    def test_sigterm_stops_backend_and_records_failed_run(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session'
            session.mkdir()
            route_file = Path(directory) / 'route.json'
            route_file.write_text(json.dumps(dict(
                scene='simulation/scenarios/indoor_tiny_house.xml',
                route_length_m=0,
                waypoints=[])))
            handlers = {}
            old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
            real_signal = signal.signal

            class FakeBackend:
                instance = None

                def __init__(self, root):
                    self.map_session = session
                    self.running = True
                    self.view = {'t': '9', 'body_height': '0.14305'}
                    self.telemetry = dict(mode='Balancing', height_settled='1',
                                          height_motion='0', drive_ready='1')
                    self.live_scan = type('Live', (), {
                        'ready': lambda _: True,
                        'failure_reason': lambda _: None,
                    })()
                    self.stopped = False
                    FakeBackend.instance = self

                def start(self): pass
                def drain(self): pass
                def command_standup(self): pass
                def command_height(self, height): pass

                def stop(self):
                    self.stopped = True
                    (session / 'STOP').touch()
                    (session / 'slam_shutdown.json').write_text('{"clean_stop": true}\n')

            class FakeRoute:
                def __init__(self, backend, route_path):
                    self.route = json.loads(route_file.read_text())
                    self.waypoints = []
                    self.route_length_m = 0
                    self.index = 0
                    self.status = 'running'
                    self.active = True

                def start(self, *args): return True

                def update(self, *args):
                    handler = handlers.get(signal.SIGTERM)
                    if handler is None:
                        raise RuntimeError('SIGTERM handler was not installed')
                    handler(signal.SIGTERM, None)

            def capture_handler(sig, handler):
                if handler not in old_handlers.values():
                    handlers[sig] = handler
                return real_signal(sig, handler)

            with patch.object(headless, 'NativeBackend', FakeBackend), \
                    patch.object(headless, 'FixedScanRoute', FakeRoute), \
                    patch.object(signal, 'signal', side_effect=capture_handler), \
                    patch.object(sys, 'argv', ['headless.py', '--route', str(route_file)]):
                with self.assertRaisesRegex(RuntimeError, '^received SIGTERM$'):
                    headless.main()

            self.assertTrue(FakeBackend.instance.stopped)
            self.assertTrue((session / 'STOP').exists())
            result = json.loads((session / 'headless_result.json').read_text())
            self.assertFalse(result['success'])
            self.assertIn('SIGTERM', result['error'])
            self.assertEqual({sig: signal.getsignal(sig) for sig in old_handlers}, old_handlers)


if __name__ == '__main__':
    unittest.main()
