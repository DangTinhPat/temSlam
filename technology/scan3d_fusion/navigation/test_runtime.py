import unittest
import tempfile
import json
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from runtime import OdomAnchor, NativeNavigation


def packet(t,x=0,yaw=0):
    return dict(time_s=t,x=x,y=0,yaw=yaw,pitch=.01,roll=0,axle_body_x=.02,
                axle_body_z=-.108,wheel_radius=.035,valid=1,slip=False)


class RuntimeTests(unittest.TestCase):
    def test_explicit_body_anchor_propagates_motion_in_map_without_truth(self):
        anchor=OdomAnchor((2,3,np.pi/2))
        first=anchor.update(packet(1),10)
        self.assertTrue(first['usable'])
        np.testing.assert_allclose(first['xy'],[2,3])
        next_pose=anchor.update(packet(1.1,.2),10.1)
        np.testing.assert_allclose(next_pose['xy'],[2,3.2],atol=1e-8)
        self.assertAlmostEqual(next_pose['yaw'],np.pi/2)
        self.assertEqual(next_pose['source'],'wheel_imu_short_run')

    def test_invalid_slip_reset_and_distance_budget_are_not_usable(self):
        for key,value in [('valid',0),('slip',True)]:
            anchor=OdomAnchor((0,0,0));anchor.update(packet(1),10)
            sample=packet(1.1);sample[key]=value
            self.assertFalse(anchor.update(sample,10.1)['usable'])
        anchor=OdomAnchor((0,0,0));anchor.update(packet(1),10)
        with self.assertRaises(ValueError):anchor.update(packet(.5),10.1)
        self.assertFalse(anchor.update(packet(2,2.1),11)['usable'])

    def test_inference_completion_cannot_refresh_old_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);now=time.monotonic()
            sample=packet(100);sample['slip']=False
            (root/'odometry.jsonl').write_text(json.dumps(sample)+'\n')
            mount=np.eye(4);mount[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];mount[2,3]=.0918
            np.savez(root/'rgbd.npz',t=1.,depth=np.full((480,640),2000,np.uint16),
                depth_source='deltar_tof_fusion',K=[400,400,319.5,239.5],mount=mount,
                odometry=json.dumps(packet(1)))
            runtime=NativeNavigation((0,0,0));runtime.standup=runtime.lowered=True
            runtime.anchor.update(packet(99),now-.1)
            runtime.infer=SimpleNamespace(poll=lambda:None)
            runtime.backend=SimpleNamespace(drain=lambda:None,running=True,map_session=root,
                telemetry_received_monotonic_s=now,telemetry=dict(t='100',mode='Balancing',
                height_settled='1',height_motion='0',drive_ready='1'))
            try:
                pose,guard=runtime.observe(now)
                self.assertIsNotNone(pose)
                self.assertFalse(guard['clear'])
                self.assertIn('stale',guard['reason'])
                self.assertGreater(now-guard['stamp'],90)
            finally:
                if runtime.stream:runtime.stream.close()

    def test_startup_requires_continuous_ready_settling(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);base=time.monotonic()
            runtime=NativeNavigation((0,0,0));runtime.standup=runtime.lowered=True
            fields=dict(t='100',mode='Balancing',height_settled='1',height_motion='0',drive_ready='1')
            runtime.backend=SimpleNamespace(drain=lambda:None,running=True,map_session=root,
                telemetry_received_monotonic_s=base,telemetry=fields)
            runtime.infer=SimpleNamespace(poll=lambda:None)
            try:
                for t,ready in [(100,1),(100.3,0),(100.6,1),(100.8,1),(101,1)]:
                    now=base+t-100;fields.update(t=str(t),drive_ready=str(ready))
                    runtime.backend.telemetry_received_monotonic_s=now
                    with (root/'odometry.jsonl').open('a') as f:f.write(json.dumps(packet(t))+'\n')
                    pose,_=runtime.observe(now)
                    self.assertIsNone(pose)
                    self.assertIsNone(runtime.anchor.correction)
                fields['t']='101.2';runtime.backend.telemetry_received_monotonic_s=base+1.2
                with (root/'odometry.jsonl').open('a') as f:f.write(json.dumps(packet(101.2))+'\n')
                pose,_=runtime.observe(base+1.2)
                self.assertTrue(pose['usable'])
            finally:
                if runtime.stream:runtime.stream.close()

    def test_nonfinite_initial_pose_and_packet_rejected(self):
        with self.assertRaises(ValueError):OdomAnchor((0,float('nan'),0))
        anchor=OdomAnchor((0,0,0))
        with self.assertRaises(ValueError):anchor.update(packet(0,float('nan')),1)

    def test_startup_restarts_settling_after_odometry_gap(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);base=time.monotonic()
            runtime=NativeNavigation((0,0,0));runtime.standup=runtime.lowered=True
            fields=dict(t='100',mode='Balancing',height_settled='1',height_motion='0',drive_ready='1')
            runtime.backend=SimpleNamespace(drain=lambda:None,running=True,map_session=root,
                telemetry_received_monotonic_s=base,telemetry=fields)
            runtime.infer=SimpleNamespace(poll=lambda:None)
            try:
                for t,has_sample in [(100,True),(100.4,False),(100.6,True),(101.2,True)]:
                    now=base+t-100;fields['t']=str(t)
                    runtime.backend.telemetry_received_monotonic_s=now
                    if has_sample:
                        with (root/'odometry.jsonl').open('a') as f:f.write(json.dumps(packet(t))+'\n')
                    pose,_=runtime.observe(now)
                    self.assertIsNone(pose)
                    self.assertIsNone(runtime.anchor.correction)
                for t in [101.4,101.6,101.8]:
                    fields['t']=str(t);runtime.backend.telemetry_received_monotonic_s=base+t-100
                    with (root/'odometry.jsonl').open('a') as f:f.write(json.dumps(packet(t))+'\n')
                    pose,_=runtime.observe(base+t-100)
                self.assertTrue(pose['usable'])
            finally:
                if runtime.stream:runtime.stream.close()


if __name__=='__main__':unittest.main()
