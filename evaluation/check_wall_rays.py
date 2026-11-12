"""Check both faces of every wall in the living-room scene; simulator-only test."""
import json,os,sys
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/live'))
from sensors import Sensors


def main():
    os.environ['RB1_SIM_MODEL']=str(ROOT/'simulation/scenarios/indoor_tiny_house_living_room.xml')
    sensor=Sensors();m,d=sensor.model,sensor.data
    checks=[]
    try:
        mujoco.mj_forward(m,d)
        for gid in range(m.ngeom):
            name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,gid) or ''
            if not any(k in name for k in ('wall','partition')):continue
            axis=int(np.argmin(m.geom_size[gid,:2]));other=1-axis
            for along in (-.8,-.4,0,.4,.8):
                for side in (-1,1):
                    origin=m.geom_pos[gid].copy();origin[2]=.235
                    origin[other]+=along*m.geom_size[gid,other]
                    origin[axis]+=side*(m.geom_size[gid,axis]+.3)
                    direction=np.zeros(3);direction[axis]=-side
                    hit=np.array([-1],np.int32)
                    distance=mujoco.mj_ray(m,d,origin,direction,np.array([1,1,1,1,1,0],np.uint8),1,-1,hit)
                    checks.append(dict(wall=name,distance=float(distance),pass_check=bool(0<=distance<=.30001)))
        (ROOT/'evaluation/physical_wall_ray_audit.json').write_text(json.dumps(checks,indent=2)+'\n')
        assert len(checks)==110 and all(c['pass_check'] for c in checks)
        print('PASS: 110 rays blocked by walls or nearer objects')
    finally:
        sensor.rgb.close();sensor.tof.close()


if __name__=='__main__':main()
