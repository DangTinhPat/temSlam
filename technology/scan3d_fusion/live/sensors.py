"""Render measured RGB/ToF from a synchronized simulator snapshot."""
import json, math, re, tempfile
import os
from pathlib import Path
import mujoco
import numpy as np
import yaml
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/capture'))
from capture_mujoco import zone_statistics, zone_rectangles

class Sensors:
    def __init__(self):
        source=Path(os.environ.get('RB1_SIM_MODEL',
            str(ROOT/'simulation/scenarios/indoor_tiny_house.xml'))).resolve()
        with tempfile.NamedTemporaryFile('w',suffix='.xml',dir=source.parent,delete=False) as f:
            f.write(re.sub(r'\s+inertia="[^"]+"','',source.read_text())); name=Path(f.name)
        try: self.model=mujoco.MjModel.from_xml_path(str(name))
        finally: name.unlink()
        m=self.model;self.data=mujoco.MjData(m)
        self.body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'body_link')
        self.camera=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'head_camera')
        for i in range(m.ngeom):
            parent=int(m.geom_bodyid[i])
            while parent>0 and parent!=self.body: parent=int(m.body_parentid[parent])
            if parent==self.body: m.geom_group[i]=5
        self.options=mujoco.MjvOption();self.options.geomgroup[3]=1;self.options.geomgroup[5]=0
        self.rgb=mujoco.Renderer(m,height=480,width=640)
        self.tof=mujoco.Renderer(m,height=128,width=128)
        self.sensor=yaml.safe_load((ROOT/'technology/scan3d_fusion/config/tof.yaml').read_text())['sensor']
        self.cal=json.loads((ROOT/'technology/scan3d_fusion/config/sim_calibration.json').read_text())['camera_to_tof']
        focal=480/(2*math.tan(math.radians(m.cam_fovy[self.camera])/2))
        self.K=[focal,focal,319.5,239.5];self.rng=np.random.default_rng(2026)
        self.joints={k:m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n)] for k,n in
            [('lh','left_hip'),('lk','left_knee'),('lw','left_wheel'),('rh','right_hip'),('rk','right_knee'),('rw','right_wheel'),('hj','head_yaw')]}

    def render(self,v):
        m,d=self.model,self.data
        d.qpos[:7]=[float(v[k]) for k in ['x','y','z','qw','qx','qy','qz']]
        for k,adr in self.joints.items(): d.qpos[adr]=float(v[k])
        mujoco.mj_forward(m,d)
        self.rgb.update_scene(d,camera='head_camera',scene_option=self.options)
        rgb=self.rgb.render().copy()
        self.tof.enable_depth_rendering();self.tof.update_scene(d,camera='tof_camera',scene_option=self.options)
        hist,mask=zone_statistics(self.tof.render().copy(),self.sensor,self.rng)
        rect=zone_rectangles(hist,np.array(self.cal['rotation_matrix']),np.array(self.cal['translation_m']),
                            m.cam_fovy[self.camera],640,480,self.sensor['fovy_deg'])
        mount=np.eye(4);bodyR=d.xmat[self.body].reshape(3,3)
        mount[:3,:3]=bodyR.T@d.cam_xmat[self.camera].reshape(3,3)@np.diag([1,-1,-1])
        mount[:3,3]=bodyR.T@(d.cam_xpos[self.camera]-d.xpos[self.body])
        return rgb,hist,rect,mask,mount
