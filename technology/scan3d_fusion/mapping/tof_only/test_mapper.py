"""Geometric regression: four walls, floor rejection, invalid data, replay and reset."""
import json
import math
import pathlib
import subprocess
import sys
import tempfile
import yaml
from PIL import Image

root=pathlib.Path(__file__).resolve().parents[4]
c=yaml.safe_load((root/'technology/scan3d_fusion/config/tof.yaml').read_text())['sensor']
right=c['xyaxes_body'][:3];up=c['xyaxes_body'][3:]
f=[-(right[1]*up[2]-right[2]*up[1]),-(right[2]*up[0]-right[0]*up[2]),-(right[0]*up[1]-right[1]*up[0])]
offset=c['board_size_m'][0]/2+c['window_size_m'][0]
origin=[c['position_body_m'][j]+offset*f[j] for j in range(3)];origin[2]+=.185

def packet(t,yaw,empty=False):
    co,si=math.cos(yaw),math.sin(yaw)
    o=[co*origin[0]-si*origin[1],si*origin[0]+co*origin[1],origin[2]]
    ranges=[]
    for i in range(64):
        h=math.tan(math.radians(c['fovy_deg']/2));u=(2*(i%8+.5)/8-1)*h;v=(1-2*(i//8+.5)/8)*h
        q=[f[j]+u*right[j]+v*up[j] for j in range(3)];n=math.sqrt(sum(a*a for a in q));q=[a/n for a in q]
        ray=[co*q[0]-si*q[1],si*q[0]+co*q[1],q[2]]
        hits=[]
        for axis in (0,1):
            if abs(ray[axis])>1e-9:
                hits.append(((2 if ray[axis]>0 else -2)-o[axis])/ray[axis])
        if ray[2]<0: hits.append(-o[2]/ray[2])
        d=min(hits)
        ranges.append(0 if empty or d>4 else d)
    fields=[t,1,-yaw*.2401998/2,yaw*.2401998/2,yaw,0,0,0,-.15,.035,.2401998,*ranges]
    return 'TOFMAP '+' '.join(map(str,fields))+'\n'

with tempfile.TemporaryDirectory() as tmp:
    def run(name,stream):
        path=pathlib.Path(tmp)/name
        subprocess.run([sys.argv[1],str(path)],input=stream,text=True,check=True)
        return json.loads((path/'state.json').read_text()),Image.open(path/'map.pgm').copy()
    stream=''.join(packet(i/15,2*math.pi*i/240) for i in range(241))
    state,im=run('room',stream)
    assert state['occupied']>100,state
    for x,y in ((2,0),(-2,0),(0,2),(0,-2)):
        resolution=state['resolution'];width=state['width'];height=state['height']
        col=int((x-state['origin'][0])/resolution);row=height-1-int((y-state['origin'][1])/resolution)
        assert min(im.crop((col-3,row-3,col+4,row+4)).tobytes())==20,(x,y)
    assert im.getpixel((width//2,height//2-1))!=20,'floor became obstacle'
    state2,im2=run('invalid',stream+packet(17,2*math.pi,True)+'TOFMAP nan\n')
    assert im.tobytes()==im2.tobytes(),'invalid ranges changed map'
    assert state2['rejected']==1
    _,again=run('replay',stream)
    assert im.tobytes()==again.tobytes(),'non deterministic replay'
    biased=[]
    for line in stream.splitlines():
        tokens=line.split();tokens[6]='0.05';tokens[7]='0.04';biased.append(' '.join(tokens)+'\n')
    _,tilted=run('tilt_bias',''.join(biased))
    centre=tilted.crop((140,140,180,180))
    assert centre.tobytes().count(20)<20,'tilt bias produced floor rings'
    reset,_=run('reset',stream+'MAPRESET\n')
    assert reset['occupied']==reset['free']==0
    print('PASS: four-wall geometry, floor, invalid data, reset, deterministic replay',state)
