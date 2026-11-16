"""Isolated kinematic navigation tests; these are not MuJoCo/SLAM success claims."""
import json
from pathlib import Path
import sys
import math
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/navigation'))
from app import demo_grid
from grid import NavigationGrid,FREE,OCCUPIED,UNKNOWN
from controller import Navigator


def run_case(grid,start,goal,interruption=None):
    nav=Navigator(grid);pose=np.asarray(start,float).copy();dt=.05
    result=nav.set_goal(pose[:2],goal)
    poses=[pose.tolist()];maximum_v=maximum_w=0.;collision=False
    for step in range(5000):
        now=step*dt
        estimate=dict(xy=pose[:2].tolist(),yaw=float(pose[2]),stamp=now,usable=True,uncertainty_m=0.)
        obstacles=dict(stamp=now,clear=True,reason='Kinematic static map fixture')
        if interruption=='obstacle' and step>=200:obstacles.update(clear=False,reason='injected obstacle')
        if interruption=='stale' and step>=200:estimate['stamp']=now-2
        if interruption=='cancel' and step==200:nav.cancel()
        result=nav.update(estimate,now,obstacles,dt)
        maximum_v=max(maximum_v,abs(result['v']));maximum_w=max(maximum_w,abs(result['w']))
        previous=pose[:2].copy();angle=pose[2]+result['w']*dt/2
        pose[:2]+=result['v']*dt*np.array([np.cos(angle),np.sin(angle)]);pose[2]+=result['w']*dt
        if result['v'] and not grid.segment_clear(previous,pose[:2]):collision=True;break
        poses.append(pose.tolist())
        if result['state'] not in ('planning','driving','aligning'):
            for i in range(3):
                hold=nav.update(estimate,now+(i+1)*dt,obstacles,dt)
                assert hold['v']==hold['w']==0,'Terminal commands must remain stopped'
            break
    return dict(state=result['state'],reason=result['reason'],simulated_seconds=(step+1)*dt,
        goal_error_m=float(np.linalg.norm(pose[:2]-goal)),inflated_map_collision=collision,
        max_v_m_s=maximum_v,max_w_rad_s=maximum_w,path=result['path'],poses=poses,
        pose_source='exact kinematic test state; not localization')


def main():
    demo=demo_grid();cases={}
    cases['wall_detour']=run_case(demo,[.8,.8,0],[4.2,3.2])
    assert cases['wall_detour']['state']=='reached' and not cases['wall_detour']['inflated_map_collision']
    for name,want in [('obstacle','blocked'),('stale','lost'),('cancel','cancelled')]:
        cases[name]=run_case(demo,[.8,.8,0],[4.2,3.2],name)
        assert cases[name]['state']==want and not cases[name]['inflated_map_collision']
    cases['blocked_goal']=run_case(demo,[.8,.8,0],[2.5,1.5])
    assert cases['blocked_goal']['state']=='blocked'
    scan=NavigationGrid.load(ROOT/'datasets/processed/scan3d_fusion/navigation-grid-20261114.npz')
    cases['scanned_room']=run_case(scan,[.15,0,0],[.5,0])
    assert cases['scanned_room']['state']=='blocked'
    report=dict(scope='Planner/controller verification, exact kinematic pose, synthetic map except scanned_room',
        scanned_grid=dict(shape=list(scan.cells.shape),resolution_m=scan.resolution,clearance_m=scan.clearance_m,
            free=int((scan.cells==FREE).sum()),occupied=int((scan.cells==OCCUPIED).sum()),unknown=int((scan.cells==UNKNOWN).sum()),
            traversable=int((~scan.blocked).sum())),cases=cases)
    target=ROOT/'evaluation/navigation_results.json';target.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({name:{k:v for k,v in record.items() if k not in ('poses','path')} for name,record in cases.items()},indent=2))


if __name__=='__main__':main()
