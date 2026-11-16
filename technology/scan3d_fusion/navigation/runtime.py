"""Owned MuJoCo session for short navigation trials using anchored wheel/IMU.

No simulator world pose enters navigation. This baseline is explicitly limited
in time/distance; its uncertainty bound is an experiment assumption, not a
calibrated localization certificate.
"""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/mapping/odometry'),
              str(ROOT/'technology/scan3d_fusion/live'),str(ROOT/'gui')]
from localization import odom_body, rigid
from workers import fusion_depth


class OdomAnchor:
    def __init__(self, initial_pose):
        self.initial=np.asarray(initial_pose,float)
        if self.initial.shape!=(3,) or not np.isfinite(self.initial).all():
            raise ValueError('Require finite initial body X/Y/yaw')
        self.correction=None;self.previous=None;self.last_time=-1.;self.start_time=None;self.travel=0.

    def update(self, packet, received):
        local=odom_body(packet);t=packet['time_s']
        if not np.isfinite(received) or t<=self.last_time:
            raise ValueError('Odometry timestamp reset or invalid receipt time')
        self.last_time=t
        if not packet['valid'] or packet.get('slip',True):
            return dict(xy=[0.,0.],yaw=0.,stamp=received,usable=False,uncertainty_m=.05,
                        source='wheel_imu_short_run',reason='invalid_odometry_or_slip')
        if self.correction is None:
            yaw=self.initial[2]-np.arctan2(local[1,0],local[0,0]);c,s=np.cos(yaw),np.sin(yaw)
            self.correction=np.eye(4);self.correction[:2,:2]=[[c,-s],[s,c]]
            self.correction[:2,3]=self.initial[:2]-(self.correction@local)[:2,3]
            self.start_time=t
        if self.previous is not None:self.travel+=float(np.linalg.norm(local[:2,3]-self.previous[:2,3]))
        self.previous=local.copy()
        mapped=self.correction@local
        valid=self.travel<=2 and t-self.start_time<=60
        return dict(xy=mapped[:2,3].tolist(),yaw=float(np.arctan2(mapped[1,0],mapped[0,0])),
            stamp=received,usable=valid,uncertainty_m=.05,source='wheel_imu_short_run',
            world_from_body=mapped.tolist(),reason='odometry_baseline' if valid else 'odometry_trial_limit',
            travel_m=self.travel)


class NativeNavigation:
    def __init__(self, initial_pose):
        self.anchor=OdomAnchor(initial_pose);self.backend=None;self.infer=None;self.stream=None
        self.log=None;self.lock=None;self.pending='';self.pose=None;self.guard=None;self.last_depth=-1.
        self.stop_event=threading.Event();self.last_tick=time.monotonic();self.watchdog_lost=False
        self.standup=False;self.lowered=False;self.ready=False;self.stable_since=None;self.message='Chưa khởi động RB'
        self.startup_sample_time=None

    def start(self):
        from rb1_gui.backend import NativeBackend
        from rb1_gui.motion_api import MotionAPI, MotionLimits
        self.lock=open('/tmp/rb-live-slam.lock','a')
        try:
            fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            changes=dict(RB1_HEADLESS='1',RB1_LIVE_SLAM='0',RB1_SIM_SPEED_FACTOR='1',RB1_SIM_PORT='37687',
                RB1_SIM_IMU_UNIT='648',RB1_SIM_MODEL=str(ROOT/'simulation/scenarios/indoor_tiny_house_living_room.xml'),
                RB1_SIM_ARGS='--capture-sync')
            previous={k:os.environ.get(k) for k in changes}
            try:
                os.environ.update(changes)
                self.backend=NativeBackend(ROOT,start_key='home');self.backend.start()
            finally:
                for key,value in previous.items():
                    if value is None:os.environ.pop(key,None)
                    else:os.environ[key]=value
            self.motion=MotionAPI(self.backend,MotionLimits(max_linear_m_s=.1,max_yaw_rad_s=.4))
            session=self.backend.map_session
            env=os.environ.copy();env.pop('PYTHONPATH',None)
            env.update(MUJOCO_GL='egl',RB1_SIM_MODEL=changes['RB1_SIM_MODEL'],OMP_NUM_THREADS='2')
            self.log=(session/'nav_infer.log').open('w')
            self.infer=subprocess.Popen(['/home/dvt/.venvs/deltar/bin/python',
                str(ROOT/'technology/scan3d_fusion/live/workers.py'),'infer',str(session)],
                env=env,stdout=self.log,stderr=subprocess.STDOUT)
            (session/'navigation_mode.json').write_text(json.dumps(dict(
                pose_source='wheel_imu_short_run',initial_body_pose_xyyaw_rad=self.anchor.initial.tolist(),
                uncertainty_m_assumed=.05,distance_limit_m=2.,time_limit_s=60.,
                limitation='Simulation only; no scan matching correction or truth control.')))
            self.last_tick=time.monotonic()
            self.watchdog=threading.Thread(target=self._watch,daemon=True);self.watchdog.start()
        except Exception:
            self.close();raise

    def _watch(self):
        while not self.stop_event.wait(.1):
            if time.monotonic()-self.last_tick>.5:
                self.watchdog_lost=True
                self.backend.command_stop()

    def rearm(self):
        self.last_tick=time.monotonic();self.watchdog_lost=False

    def observe(self, now):
        from controller import depth_guard
        self.last_tick=now
        self.backend.drain();fields=self.backend.telemetry
        self.ready=False
        if not self.backend.running or self.infer.poll() is not None:
            self.stable_since=None;self.message='Tiến trình SIM/inference đã dừng';return None,None
        if now-self.backend.telemetry_received_monotonic_s>.5:
            self.stable_since=None;self.message='Telemetry hết hạn';return None,None
        t=float(fields.get('t',0))
        if not self.standup and t>=9:
            self.backend.command_standup();self.standup=True
        settled=(fields.get('mode') in ('Balance','Balancing','Drive') and fields.get('height_settled')=='1'
                 and fields.get('height_motion')=='0')
        if settled and not self.lowered:
            self.backend.command_height(.14305);self.lowered=True;settled=False
        self.ready=settled and self.lowered and fields.get('drive_ready')=='1' and not self.watchdog_lost
        if not self.ready and self.anchor.correction is None:self.stable_since=None
        self.message='RB cân bằng · chờ cảm biến' if self.ready else 'Chờ RB đứng dậy/cân bằng'
        if self.watchdog_lost:self.message='Watchdog dừng RB; chọn lại đích để tiếp tục'
        session=self.backend.map_session
        if self.stream is None and (session/'odometry.jsonl').exists():self.stream=(session/'odometry.jsonl').open()
        if self.stream is not None:
            lines=(self.pending+self.stream.read()).split('\n');self.pending=lines.pop()
            latest=None
            for line in lines:
                if not line:continue
                sample=json.loads(line)
                if 'reset' in sample:
                    raise ValueError('Odometry reset; khởi động lại phiên navigation')
                latest=sample
            if latest is not None and self.ready:
                age=max(0.,time.time()-(session/'odometry.jsonl').stat().st_mtime,t-latest['time_s'])
                if self.anchor.correction is None:
                    if self.startup_sample_time is not None and latest['time_s']-self.startup_sample_time>.3:
                        self.stable_since=None
                    self.startup_sample_time=latest['time_s']
                    height=odom_body(latest)[2,3]
                    stable=(age<=.3 and latest['valid'] and not latest.get('slip',True)
                            and abs(height-.14305)<.005 and abs(float(fields.get('ground_velocity',0)))<.03)
                    if not stable:self.stable_since=None
                    elif self.stable_since is None:self.stable_since=latest['time_s']
                    self.ready=stable and latest['time_s']-self.stable_since>=.5
                if self.ready:self.pose=self.anchor.update(latest,now-age)
            elif self.anchor.correction is None:
                self.ready=False
                if self.startup_sample_time is not None and t-self.startup_sample_time>.3:
                    self.stable_since=None
        path=session/'rgbd.npz'
        try:
            age=max(0.,time.time()-path.stat().st_mtime)
            if age<=.5:
                with np.load(path,allow_pickle=False) as data:batch={k:data[k] for k in data.files}
                fusion_depth(batch);timestamp=float(batch['t'])
                if timestamp<self.last_depth:raise ValueError('RGB–ToF timestamp reset')
                if timestamp>self.last_depth:
                    packet=json.loads(str(batch['odometry']));local=odom_body(packet)
                    if abs(packet['time_s']-timestamp)>.02:raise ValueError('RGB–ToF/odometry lệch timestamp')
                    yaw=packet['yaw'];c,s=np.cos(yaw),np.sin(yaw)
                    level=np.eye(4);level[:3,:3]=np.array([[c,s,0],[-s,c,0],[0,0,1]])@local[:3,:3]
                    mount=level@rigid(batch['mount'])
                    capture_age=max(age,t-timestamp)
                    self.guard=depth_guard(batch['depth'],batch['K'],mount,now-capture_age,now,
                                           body_height_m=local[2,3])
                    self.last_depth=timestamp
        except FileNotFoundError:pass
        if self.pose and self.ready:self.message='Wheel/IMU · giới hạn thử 2 m / 60 s'
        return (self.pose if self.ready else None),self.guard

    def command(self, result):
        if result['state'] in ('driving','aligning') and self.ready and not self.watchdog_lost:
            self.motion.command_velocity(result['v'],result['w'])
        else:self.backend.command_stop()

    def close(self):
        self.stop_event.set()
        if self.backend is not None:self.backend.command_stop()
        if self.infer is not None and self.infer.poll() is None:
            self.infer.terminate()
            try:self.infer.wait(timeout=3)
            except subprocess.TimeoutExpired:self.infer.kill();self.infer.wait()
        if self.stream is not None:self.stream.close()
        if self.backend is not None:self.backend.stop()
        if self.log is not None:self.log.close()
        if self.lock is not None:self.lock.close()
