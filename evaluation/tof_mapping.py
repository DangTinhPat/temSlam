"""Offline only: compare mapper axle odometry against MuJoCo VIEW snapshots."""
import argparse
import csv
import json
import math
import pathlib
import mujoco
import numpy as np

p=argparse.ArgumentParser();p.add_argument('log');p.add_argument('trajectory');p.add_argument('--model',default='simulation/scenarios/indoor_small_house.xml');args=p.parse_args()
m=mujoco.MjModel.from_xml_path(args.model);d=mujoco.MjData(m)
ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n) for n in ('left_wheel','right_wheel')]
truth=[]
for line in pathlib.Path(args.log).read_text().splitlines():
    if not line.startswith('VIEW|'):continue
    v=dict(s.split('=',1) for s in line.split('|')[1:] if '=' in s)
    d.qpos[:13]=[float(v[k]) for k in ('x','y','z','qw','qx','qy','qz','lh','lk','lw','rh','rk','rw')]
    mujoco.mj_forward(m,d)
    xyz=d.xpos[ids].mean(axis=0)
    w,x,y,z=d.qpos[3:7];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    truth.append([float(v['t']),xyz[0],xyz[1],yaw])
truth=np.asarray(truth);truth[:,3]=np.unwrap(truth[:,3])
with open(args.trajectory) as f:
    rows=list(csv.DictReader(line for line in f if not line.startswith('#')))
a=np.array([[float(r[k]) for k in ('t','x','y','yaw')] for r in rows if r['valid']=='1'])
if len(a)<2:raise SystemExit('No supported trajectory to evaluate')
g=np.array([np.interp(a[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)]).T
theta=g[0,2];xy=g[:,:2]-g[0,:2];c,s=math.cos(theta),math.sin(theta)
xy=xy@np.array([[c,-s],[s,c]])
errors=np.linalg.norm(a[:,1:3]-xy,axis=1);dyaw=a[:,3]-(g[:,2]-theta);dyaw=np.arctan2(np.sin(dyaw),np.cos(dyaw))
print(json.dumps({'samples':len(a),'duration_s':float(a[-1,0]-a[0,0]),'alignment':'initial axle XY/yaw only; no fitted scale',
 'position_rmse_m':float(np.sqrt(np.mean(errors**2))),'position_final_m':float(errors[-1]),'position_max_m':float(errors.max()),
 'yaw_rmse_deg':float(np.degrees(np.sqrt(np.mean(dyaw**2)))),'yaw_final_deg':float(np.degrees(dyaw[-1]))},indent=2))
