import unittest
import sys
import math
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from fixed_scan_route import FixedScanRoute

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'gui'))
from rb1_gui.dashboard import NativeDashboard

class RouteTest(unittest.TestCase):
    def setUp(self):
        self.commands=[]
        self.backend=SimpleNamespace(root=ROOT,running=True,map_session=None,log=[],
            process=SimpleNamespace(args=['runner','--model',str(ROOT/'simulation/scenarios/indoor_tiny_house.xml')]),
            telemetry_received_monotonic_s=time.monotonic(),
            telemetry={'mode':'Balancing','drive_ready':'1','ground_velocity':'0','pitch_rate':'0','height_settled':'1','height_motion':'0'},
            view={'t':'0','x':'0','y':'0','qw':'1','qx':'0','qy':'0','qz':'0','hj':'0'},
            send=self.commands.append,command_drive=lambda v,w:self.commands.append(('drive',v,w)))
        self.route=FixedScanRoute(self.backend,ROOT/'technology/scan3d_fusion/mapping/native/fixed_scan_route.json')
        self.assertTrue(self.route.start(True,False))
        self.route.index=1
        self.route.phase="drive"
        self.commands.clear()

    def test_start_holds_stop_until_all_checks_continuously_settle(self):
        self.backend.telemetry['drive_ready']='0'
        self.assertTrue(self.route.start(True,False));self.assertEqual(self.route.phase,'start_hold')
        self.assertEqual(self.commands[-1],'STOP')
        for field,bad in [('drive_ready','0'),('height_settled','0'),('height_motion','1'),
                          ('ground_velocity','-.025'),('pitch_rate','.12')]:
            with self.subTest(field=field):
                self.assertTrue(self.route.start(True,False))
                self.backend.telemetry.update(drive_ready='1',height_settled='1',height_motion='0',ground_velocity='0',pitch_rate='0')
                t=float(self.backend.view['t'])+1
                self.backend.view['t']=str(t);self.route.update(True,False)
                self.backend.telemetry[field]=bad
                self.backend.view['t']=str(t+.2);self.route.update(True,False)
                self.backend.telemetry.update(drive_ready='1',height_settled='1',height_motion='0',ground_velocity='0',pitch_rate='0')
                self.backend.view['t']=str(t+.4);self.route.update(True,False)
                self.backend.view['t']=str(t+.6);self.route.update(True,False)
                self.assertEqual(self.route.phase,'start_hold')
                self.assertEqual(self.commands[-1],'STOP')
        # Release through drive's waypoint handling, including the initial scan.
        self.backend.view['t']=str(t+.71);self.route.update(True,False)
        self.assertEqual(self.route.phase,'settle_base');self.assertEqual(self.route.index,0)
        self.assertFalse(any(isinstance(c,tuple) for c in self.commands))

    def test_start_hold_times_out_with_stop(self):
        self.assertTrue(self.route.start(True,False))
        self.backend.telemetry['ground_velocity']='-.06'
        self.backend.view['t']='30.01';self.route.update(True,False)
        self.assertFalse(self.route.active);self.assertIn('30',self.route.status)
        self.assertIn('STOP',self.commands);self.assertFalse(any(isinstance(c,tuple) for c in self.commands))

    def test_stale_or_missing_telemetry_resets_start_dwell(self):
        with patch('fixed_scan_route.time.monotonic',return_value=100.):
            self.assertTrue(self.route.start(True,False))
            for missing in (False, True):
                self.backend.telemetry_received_monotonic_s=100.
                self.backend.view['t']='1';self.route.update(True,False)
                if missing:del self.backend.telemetry_received_monotonic_s
                else:self.backend.telemetry_received_monotonic_s=98.9
                self.backend.view['t']='1.31';self.route.update(True,False)
                self.assertEqual(self.route.phase,'start_hold');self.assertIsNone(self.route.still_since)
                self.assertEqual(self.commands[-1],'STOP')
                self.backend.telemetry_received_monotonic_s=100.
                self.backend.view['t']='1.4';self.route.update(True,False)
                self.backend.view['t']='1.6';self.route.update(True,False)
                self.assertEqual(self.route.phase,'start_hold')
        self.assertFalse(any(isinstance(c,tuple) for c in self.commands))

    def test_frozen_simulation_clock_has_wall_timeout(self):
        with patch('fixed_scan_route.time.monotonic',return_value=100.):
            self.assertTrue(self.route.start(True,False))
        self.backend.telemetry_received_monotonic_s=100.
        self.backend.telemetry['ground_velocity']='.06'
        with patch('fixed_scan_route.time.monotonic',return_value=130.01):self.route.update(True,False)
        self.assertFalse(self.route.active);self.assertIn('30',self.route.status)
        self.assertIn('STOP',self.commands);self.assertFalse(any(isinstance(c,tuple) for c in self.commands))

    def test_resume_holds_then_drives_to_current_scan_waypoint(self):
        self.route.index=1;self.route.phase='sweep_head'
        self.backend.view.update(x='-1',y='-1');self.route.stop()
        self.route.waypoints=[dict(x=0,y=0),dict(x=-.8,y=-1,scan=True)]
        self.assertTrue(self.route.start(True,False));self.assertEqual(self.route.phase,'start_hold')
        self.route.update(True,False)
        self.backend.view['t']='.31';self.route.update(True,False)
        self.assertEqual(self.route.phase,'drive');self.assertEqual(self.route.index,1)
        self.assertTrue(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))
        self.backend.view.update(t='.4',x='-.8');self.route.update(True,False)
        self.assertEqual(self.route.phase,'settle_base')

    def test_profile_scan_range_stays_inside_head_limits(self):
        self.route.route["head_range_deg"] = [-180, 180]
        self.route.route["scan_range_deg"] = [-135, 135]
        start, end = self.route._scan_angles()
        self.assertAlmostEqual(start, math.radians(-135))
        self.assertAlmostEqual(end, math.radians(135))

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
        self.backend.view['t']='200';self.route.update(True,False)
        self.assertFalse(self.route.active)
        self.assertIn('không tới được',self.route.status)
        self.assertEqual(self.route._progress('wrong'),self.route.status)

    def test_resume_requires_same_location_and_ready(self):
        self.backend.view.update(x='0',y='-1')
        self.route.stop('paused')
        self.backend.telemetry['drive_ready']='0'
        self.assertTrue(self.route.start(True,False));self.assertEqual(self.route.phase,'start_hold')
        self.route.update(True,False);self.assertEqual(self.route.phase,'start_hold')
        self.backend.telemetry['drive_ready']='1'
        self.assertEqual(self.route.index,1)
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

    def test_reverse_waypoint_retraces_a_line_without_turning(self):
        self.route.waypoints=[dict(x=0,y=0,reverse=True,speed_m_s=.05)]
        self.route.index=0;self.route.phase='drive';self.route._drive_index=None
        self.backend.view.update(x='.5',y='0')
        self.route.update(True,False)
        self.assertEqual(self.commands[-1][0:2],('drive',-.05))
        self.assertAlmostEqual(self.commands[-1][2],0)

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

    def test_stale_slam_feedback_holds_body_motion(self):
        self.backend.view['t']='1.1'
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,
            scan_feedback=lambda:dict(sensor_time=0.0,head_yaw_rad=0.0))
        self.route.update(True,False)
        self.assertTrue(self.route.active)
        self.assertEqual(self.commands[-1],'STOP')
        self.assertFalse(any(isinstance(c,tuple) for c in self.commands))
        self.assertIn('SLAM',self.route.status)

    def test_missing_slam_feedback_holds_body_motion(self):
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None)
        self.route.update(True,False)
        self.assertEqual(self.commands[-1],'STOP')
        self.assertFalse(any(isinstance(c,tuple) for c in self.commands))

    def test_fresh_slam_feedback_allows_body_motion(self):
        self.backend.view['t']='1.0'
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,
            scan_feedback=lambda:dict(sensor_time=.5,head_yaw_rad=0.0))
        self.route.update(True,False)
        self.assertTrue(any(isinstance(c,tuple) and c[0]=='drive' for c in self.commands))

    def test_completion_within_head_tolerance_does_not_snap_to_zero(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True)];self.route.index=0
        self.route.phase='return_head';self.route.phase_started=0
        self.route.head_commanded=.05;self.route.head_command_at=0
        self.backend.view.update(hj='.05',t='.1')
        self.commands.clear();self.route.update(True,False)
        self.assertEqual(self.route.phase,'idle')
        head_commands=[float(c.split()[1]) for c in self.commands if isinstance(c,str) and c.startswith('HEAD ')]
        self.assertEqual(len(head_commands),1)
        self.assertAlmostEqual(head_commands[0],.046)
        self.assertAlmostEqual(self.route.head_commanded,.046)
        self.assertIn('STOP',self.commands)

    def test_body_turn_rate_is_capped_in_both_directions(self):
        for y in (-1,1):
            self.route.waypoints=[dict(x=0,y=y)];self.route.index=0
            self.route.phase='drive';self.route._drive_index=None
            self.route.update(True,False)
            self.assertEqual(self.commands[-1][0],'drive')
            self.assertAlmostEqual(self.commands[-1][2],math.copysign(.04,y))

    def test_all_head_phases_cap_actual_command_rate(self):
        for phase,initial in [('position_head',0.),('sweep_head',-1.),('return_head',1.)]:
            with self.subTest(phase=phase):
                self.route.waypoints=[dict(x=0,y=0,scan=True)];self.route.index=0
                self.route.phase=phase;self.route.phase_started=0
                self.route.head_commanded=initial;self.route.head_command_at=0
                self.backend.view.update(hj=str(initial),t='.1')
                self.route.update(True,False)
                change=abs(self.route.head_commanded-initial)
                self.assertGreater(change,0);self.assertLessEqual(change,.04*.1+1e-12)
                self.backend.view.update(t='60',hj=str(self.route.head_commanded))
                self.route.update(True,False)
                self.assertEqual(self.route.phase,phase)

    def test_dense_position_and_return_wait_for_fresh_processed_steps(self):
        for phase,initial in [('position_head',0.),('return_head',math.radians(65))]:
            with self.subTest(phase=phase):
                self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8,scan_range_deg=[-65,65])]
                self.route.index=0;self.route.phase=phase;self.route.phase_started=10
                self.route.head_commanded=initial;self.route.head_command_at=10
                self.backend.view.update(hj=str(initial),t='11')
                feedback=[None]
                self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:feedback[0])
                self.route.update(True,False)
                self.assertEqual(self.route.head_commanded,initial)
                feedback[0]=dict(sensor_time=9,head_yaw_rad=initial)
                self.backend.view['t']='12';self.route.update(True,False)
                self.assertEqual(self.route.head_commanded,initial)
                feedback[0].update(sensor_time=12,head_yaw_rad=initial)
                for t in range(13,20):
                    self.backend.view.update(t=str(t),hj=str(self.route.head_commanded));self.route.update(True,False)
                expected=initial-math.radians(8)
                self.assertAlmostEqual(self.route.head_commanded,expected)
                self.backend.view.update(hj=str(expected),t='20');self.route.update(True,False)
                self.assertAlmostEqual(self.route.head_commanded,expected)
                feedback[0].update(sensor_time=20,head_yaw_rad=expected)
                for t in range(21,28):
                    self.backend.view.update(t=str(t),hj=str(self.route.head_commanded));self.route.update(True,False)
                self.assertAlmostEqual(self.route.head_commanded,expected-math.radians(8))
                target=math.radians(-65) if phase=='position_head' else 0.
                self.backend.view.update(hj=str(target),t='28')
                feedback[0].update(sensor_time=28,head_yaw_rad=target)
                self.route.update(True,False)
                self.assertEqual(self.route.phase,'sweep_head' if phase=='position_head' else 'idle')

    def test_dense_position_and_return_allow_slow_feedback_but_timeout_stalls(self):
        for phase,initial,target in [('position_head',0.,math.radians(-65)),('return_head',math.radians(65),0.)]:
            with self.subTest(phase=phase):
                self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8,scan_range_deg=[-65,65])]
                self.route.index=0;self.route.phase=phase;self.route.phase_started=0
                self.route.head_commanded=initial;self.route.head_command_at=0
                self.backend.view.update(hj=str(initial),t='20')
                feedback=dict(sensor_time=20,head_yaw_rad=initial)
                self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:feedback)
                with patch('fixed_scan_route.time.monotonic',return_value=100):self.route.update(True,False)
                self.assertTrue(self.route.active)
                # Physical arrival cannot advance the route before a processed target observation.
                self.backend.view.update(hj=str(target),t='21')
                with patch('fixed_scan_route.time.monotonic',return_value=101):self.route.update(True,False)
                self.assertEqual(self.route.phase,phase)
                with patch('fixed_scan_route.time.monotonic',return_value=281):self.route.update(True,False)
                self.assertFalse(self.route.active);self.assertIn('quá hạn',self.route.status)

    def test_dense_sweep_waits_for_processed_head_view(self):
        self.route.waypoints=[dict(x=0,y=0,scan=True,scan_step_deg=8)];self.route.index=0
        self.route.phase='sweep_head';self.route.phase_started=0
        self.route._sweep_progress_at=time.monotonic()
        feedback=dict(sensor_time=.1,head_yaw_rad=-math.pi)
        self.backend.live_scan=SimpleNamespace(failure_reason=lambda:None,scan_feedback=lambda:feedback)
        self.route.head_commanded=-math.pi;self.backend.view.update(hj=str(-math.pi),t='1')
        for t in range(1,8):
            self.backend.view.update(t=str(t),hj=str(self.route.head_commanded));self.route.update(True,False)
        expected=-math.pi+math.radians(8)
        self.assertAlmostEqual(self.route.head_commanded,expected)
        self.backend.view.update(hj=str(expected),t='8');self.route.update(True,False)
        self.assertAlmostEqual(self.route.head_commanded,expected)
        feedback.update(sensor_time=8,head_yaw_rad=expected)
        for t in range(9,16):
            self.backend.view.update(t=str(t),hj=str(self.route.head_commanded));self.route.update(True,False)
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
