import unittest
import sys
import math
import time
from pathlib import Path
from types import SimpleNamespace
from fixed_scan_route import FixedScanRoute

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'gui'))
from rb1_gui.dashboard import NativeDashboard

class RouteTest(unittest.TestCase):
    def setUp(self):
        self.commands=[]
        self.backend=SimpleNamespace(root=ROOT,running=True,map_session=None,log=[],
            process=SimpleNamespace(args=['runner','--model',str(ROOT/'simulation/scenarios/indoor_tiny_house.xml')]),
            telemetry={'mode':'Balancing','drive_ready':'1','ground_velocity':'0','pitch_rate':'0'},
            view={'t':'0','x':'0','y':'0','qw':'1','qx':'0','qy':'0','qz':'0','hj':'0'},
            send=self.commands.append,command_drive=lambda v,w:self.commands.append(('drive',v,w)))
        self.route=FixedScanRoute(self.backend,ROOT/'technology/scan3d_fusion/mapping/native/fixed_scan_route.json')
        self.assertTrue(self.route.start(True,False))
        self.route.index=1

    def test_transient_not_ready_waits_and_resumes(self):
        self.backend.telemetry['drive_ready']='0'
        self.route.update(True,False)
        self.assertEqual(self.route.phase,'wait_ready');self.assertEqual(self.route.index,1)
        self.assertIn('STOP',self.commands)
        self.backend.view['t']='1';self.backend.telemetry['drive_ready']='1'
        self.route.update(True,False)
        self.backend.view['t']='1.4';self.route.update(True,False)
        self.assertEqual(self.route.phase,'drive')
        self.assertTrue(any(isinstance(c,tuple) for c in self.commands))

    def test_timeout_message_survives_progress(self):
        self.route.phase='position_head';self.route.phase_started=0
        self.backend.view['t']='9';self.route.update(True,False)
        self.assertFalse(self.route.active)
        self.assertIn('không tới được',self.route.status)
        self.assertEqual(self.route._progress('wrong'),self.route.status)

    def test_resume_requires_same_location_and_ready(self):
        self.backend.view.update(x='0',y='-1')
        self.route.stop('paused')
        self.backend.telemetry['drive_ready']='0'
        self.assertFalse(self.route.start(True,False))
        self.backend.telemetry['drive_ready']='1'
        self.assertTrue(self.route.start(True,False));self.assertEqual(self.route.index,1)
        self.route.stop();self.backend.view['x']='2'
        self.assertFalse(self.route.start(True,False))

    def test_fault_stops_and_head_command_does_not_jump_after_ui_stall(self):
        self.backend.view['t']='5';self.route._ramp_head(-3.14,1,0)
        self.assertLessEqual(abs(self.route.head_commanded),math.radians(8)+1e-6)
        self.backend.telemetry['mode']='Fault';self.route.update(True,False)
        self.assertFalse(self.route.active);self.assertIn('mất trạng thái',self.route.status)

    def test_manual_tick_cannot_overwrite_route_command(self):
        app=NativeDashboard.__new__(NativeDashboard)
        app.backend=self.backend;app.scan_route=self.route
        app.enabled=True;app.estop=False;app._drive_stop_latched=False
        app.drive=(0,0)
        app.motion=SimpleNamespace(command_velocity=lambda v,w:self.commands.append(('manual',v,w)))
        self.route.update(True,False)
        app._update_manual_drive()
        self.assertTrue(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))
        self.assertFalse(any(isinstance(c,tuple) and c[0]=='manual' for c in self.commands))
        self.route.stop()
        app._update_manual_drive()
        self.assertEqual(self.commands[-1],('manual',0,0))

    def test_pass_through_point_does_not_send_stop(self):
        self.route.waypoints=[dict(x=0,y=0),dict(x=1,y=0,scan=True)]
        self.route.index=0
        self.route.update(True,False)
        self.assertEqual(self.route.index,1)
        self.assertNotIn('STOP',self.commands)
        self.assertEqual(self.commands[-1],('drive',.20,0))
        self.backend.view['x']='.95'
        self.route.update(True,False)
        self.assertEqual(self.commands[-1],'STOP')
        self.assertEqual(self.route.phase,'settle_base')

    def test_turn_hysteresis_avoids_stop_go_chatter(self):
        self.route.waypoints=[dict(x=1,y=0,scan=True)];self.route.index=0
        for error,turning in [(.8,True),(.3,True),(.1,False),(.23,False),(.21,False)]:
            self.backend.view.update(qw=str(math.cos(error/2)),qz=str(math.sin(error/2)))
            self.route.update(True,False)
            self.assertEqual(self.route._turning,turning)
            self.assertEqual(self.commands[-1][1]==0,turning)

    def test_narrow_passage_speed_limit(self):
        self.route.waypoints=[dict(x=1,y=0,scan=True,speed_m_s=.12)];self.route.index=0
        self.route.update(True,False)
        self.assertEqual(self.commands[-1],('drive',.12,0))

    def test_no_progress_stops_instead_of_pushing_forever(self):
        self.route.update(True,False)
        self.backend.view['t']='12.1'
        self.route.update(True,False)
        self.assertFalse(self.route.active)
        self.assertIn('không tiến triển',self.route.status)
        self.assertIn('STOP',self.commands)

    def test_failed_slam_stops_route(self):
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:'slam: LỖI tracking')
        self.route.update(True,False)
        self.assertFalse(self.route.active)
        self.assertIn('LỖI tracking',self.route.status)

    def test_dense_sweep_waits_for_processed_head_view(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8)];self.route.index=0
        self.route.phase='sweep_head';self.route.phase_started=0
        self.route._sweep_progress_at=time.monotonic()
        feedback=dict(sensor_time=.1,head_yaw_rad=-math.pi)
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:feedback)
        self.route.head_commanded=-math.pi;self.backend.view.update(hj=str(-math.pi),t='1')
        self.route.update(True,False)
        expected=-math.pi+math.radians(8)
        self.assertAlmostEqual(self.route.head_commanded,expected)
        self.backend.view.update(hj=str(expected),t='2');self.route.update(True,False)
        self.assertAlmostEqual(self.route.head_commanded,expected)
        feedback.update(sensor_time=2,head_yaw_rad=expected)
        self.backend.view['t']='3';self.route.update(True,False)
        self.assertAlmostEqual(self.route.head_commanded,expected+math.radians(8))

    def test_dense_sweep_wait_timeout_is_explicit(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8)];self.route.index=0
        self.route.phase='sweep_head';self.route.phase_started=0
        self.route._sweep_progress_at=time.monotonic()-181
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:None)
        self.route.update(True,False)
        self.assertFalse(self.route.active);self.assertIn('quá hạn',self.route.status)

    def test_dense_sweep_cannot_reuse_a_previous_stops_frame(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8)];self.route.index=0
        self.route.phase='sweep_head';self.route.phase_started=10
        self.route._sweep_progress_at=time.monotonic()
        self.route.head_commanded=-math.pi
        self.backend.view.update(hj=str(-math.pi),t='11')
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,
            scan_feedback=lambda:dict(sensor_time=9,head_yaw_rad=math.pi))
        self.route.update(True,False)
        self.assertEqual(self.route.head_commanded,-math.pi)
        self.assertEqual(self.route.phase,'sweep_head')

    def test_dense_sweep_timeout_tracks_angle_progress_not_total_duration(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8)];self.route.index=0
        self.route.phase='sweep_head';self.route.phase_started=0
        self.route._sweep_progress_at=time.monotonic()-181
        self.route.head_commanded=0;self.backend.view.update(hj='0',t='1')
        feedback=dict(sensor_time=1,head_yaw_rad=0)
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:feedback)
        self.route.update(True,False)
        self.assertTrue(self.route.active)
        self.assertGreater(self.route._sweep_progress_at,time.monotonic()-1)
        self.route._sweep_progress_at=time.monotonic()-181
        feedback['sensor_time']=2;self.backend.view['t']='2'
        self.route.update(True,False)
        self.assertFalse(self.route.active)

if __name__=='__main__':unittest.main()
