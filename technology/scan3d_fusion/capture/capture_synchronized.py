#!/usr/bin/env python3
"""Render RGB/area-ToF at native simulator sensor timestamps (recorded physics)."""
import argparse
import json
import math
from pathlib import Path
import re
import tempfile
import h5py
import mujoco
import numpy as np
import yaml
from capture_mujoco import ROOT, zone_statistics, zone_rectangles


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('log',type=Path)
    p.add_argument('output',type=Path)
    p.add_argument('--start',type=float,default=20)
    p.add_argument('--end',type=float,default=30)
    a=p.parse_args()
    model_path=ROOT/'simulation/scenarios/indoor_tiny_house.xml'
    xml=re.sub(r'\s+inertia="[^"]+"','',model_path.read_text())
    with tempfile.NamedTemporaryFile('w',suffix='.xml',dir=model_path.parent,delete=False) as f:
        f.write(xml); name=Path(f.name)
    try: model=mujoco.MjModel.from_xml_path(str(name))
    finally: name.unlink()
    data=mujoco.MjData(model)
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'body_link')
    camera=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'head_camera')
    for i in range(model.ngeom):
        parent=int(model.geom_bodyid[i])
        while parent>0 and parent!=body: parent=int(model.body_parentid[parent])
        if parent==body: model.geom_group[i]=5
    options=mujoco.MjvOption();options.geomgroup[3]=1;options.geomgroup[5]=0
    sensor=yaml.safe_load((ROOT/'technology/scan3d_fusion/config/tof.yaml').read_text())['sensor']
    cal=json.loads((ROOT/'technology/scan3d_fusion/config/sim_calibration.json').read_text())['camera_to_tof']
    R=np.array(cal['rotation_matrix']);t=np.array(cal['translation_m'])
    rgb=mujoco.Renderer(model,height=480,width=640)
    tof=mujoco.Renderer(model,height=128,width=128)
    focal=480/(2*math.tan(math.radians(model.cam_fovy[camera])/2))
    sequence=dict(pose_source='simulator ground truth for rendering/evaluation only',image_size=[640,480],
                  intrinsics=[focal,focal,319.5,239.5],frames=[])
    joints={key:model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,joint)]
            for key,joint in [('lh','left_hip'),('lk','left_knee'),('lw','left_wheel'),
                              ('rh','right_hip'),('rk','right_knee'),('rw','right_wheel'),('hj','head_yaw')]}
    rng=np.random.default_rng(2026)
    a.output.mkdir(parents=True,exist_ok=False)
    pending=None
    try:
        for line in a.log.read_text().splitlines():
            if line.startswith('TOFMAP '): pending=line.split()
            if not line.startswith('CAPTURE|'): continue
            v=dict(item.split('=',1) for item in line.split('|')[1:])
            ts=float(v['t'])
            if not a.start<=ts<=a.end: continue
            if pending is None or abs(ts-float(pending[1]))>1e-5:
                raise ValueError('Capture state and sensor timestamp mismatch')
            if pending[2]!='1': raise ValueError(f'Robot not balancing at {ts}')
            data.qpos[:7]=[float(v[k]) for k in ['x','y','z','qw','qx','qy','qz']]
            for k,adr in joints.items(): data.qpos[adr]=float(v[k])
            mujoco.mj_forward(model,data)
            rgb.update_scene(data,camera='head_camera',scene_option=options)
            color=rgb.render().copy()
            rgb.enable_depth_rendering();rgb.update_scene(data,camera='head_camera',scene_option=options)
            depth=rgb.render().copy();rgb.disable_depth_rendering()
            depth[~np.isfinite(depth)|(depth<.02)|(depth>4)]=0
            tof.enable_depth_rendering();tof.update_scene(data,camera='tof_camera',scene_option=options)
            hist,mask=zone_statistics(tof.render().copy(),sensor,rng)
            rect=zone_rectangles(hist,R,t,model.cam_fovy[camera],640,480,sensor['fovy_deg'])
            pose=np.eye(4);pose[:3,:3]=data.cam_xmat[camera].reshape(3,3)@np.diag([1,-1,-1]);pose[:3,3]=data.cam_xpos[camera]
            body_pose=np.eye(4);body_pose[:3,:3]=data.xmat[body].reshape(3,3);body_pose[:3,3]=data.xpos[body]
            filename=f'frame_{len(sequence["frames"]):05d}.h5'
            with h5py.File(a.output/filename,'w') as h:
                for key,value in dict(rgb=color,depth=depth,hist_data=hist,fr=rect,mask=mask).items(): h.create_dataset(key,data=value)
            sequence['frames'].append(dict(file=filename,time_s=ts,camera_to_world=pose.tolist(),
                body_from_camera=(np.linalg.inv(body_pose)@pose).tolist()))
            if len(sequence['frames'])%30==0: print('captured',len(sequence['frames']),flush=True)
    finally: rgb.close();tof.close()
    if not sequence['frames']: raise ValueError('No synchronized frames captured')
    (a.output/'sequence.json').write_text(json.dumps(sequence,indent=2)+'\n')
    (a.output/'data.json').write_text(json.dumps({'train':[],'test':[{'filename':f['file']} for f in sequence['frames']]}))
    print('frames',len(sequence['frames']))

if __name__=='__main__': main()
