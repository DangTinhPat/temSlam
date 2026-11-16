"""Replay the navigation controller on the actual scanned 3-D mesh, with RB CAD.

Kinematic visualization only: no native controller, sensor estimation or physics
stepping. The route comes from a demo-only surface projection of this same mesh.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'gui'),str(ROOT/'evaluation')]
from grid import FREE, OCCUPIED, UNKNOWN
from mesh_grid import build_mesh_grid
from evaluate_navigation import run_case

DEFAULT_MESH=ROOT/'datasets/processed/scan3d_fusion/room-floor-preserved-20261112/final_mesh.stl'


def make_scene(mesh, path, start, goal):
    from rb1_gui.cad_viewer import RbModel
    tree=ET.parse(ROOT/'model/dynamic.xml');root=tree.getroot()
    asset,world=root.find('asset'),root.find('worldbody')
    for geom in list(world.findall('geom')):world.remove(geom)
    contact=root.find('contact')
    if contact is not None:root.remove(contact)
    ET.SubElement(asset,'mesh',name='nav_scan',file=str(mesh.resolve()),inertia='shell')
    ET.SubElement(world,'geom',name='env_scanned_room',type='mesh',mesh='nav_scan',
                  rgba='.72 .69 .63 1',contype='0',conaffinity='0')
    for a,b in zip(path,path[1:]):
        if np.linalg.norm(np.asarray(a)-b)<1e-8:continue
        ET.SubElement(world,'geom',type='capsule',size='.012',
            fromto=f'{a[0]} {a[1]} .045 {b[0]} {b[1]} .045',rgba='.85 .36 .10 1',contype='0',conaffinity='0')
    for point,color in [(start,'.15 .58 .37 1'),(goal,'.78 .15 .28 1')]:
        ET.SubElement(world,'geom',type='cylinder',pos=f'{point[0]} {point[1]} .04',
                      size='.065 .015',rgba=color,contype='0',conaffinity='0')
    with tempfile.TemporaryDirectory(prefix='rb-mesh-demo-') as tmp:
        file=Path(tmp)/'scene.xml';tree.write(file)
        rb=RbModel(file,'spawn_extended')
    return rb


def replay_pose(rb, pose, wheel_angle):
    yaw=pose[2]
    rb.apply(dict(x=pose[0],y=pose[1],qw=np.cos(yaw/2),qx=0,qy=0,qz=np.sin(yaw/2),
                  lw=wheel_angle,rw=wheel_angle))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh',type=Path,default=DEFAULT_MESH)
    parser.add_argument('--start',type=float,nargs=2,default=[.7,-1.6],metavar=('X','Y'))
    parser.add_argument('--goal',type=float,nargs=2,default=[.8,.8],metavar=('X','Y'))
    parser.add_argument('--output',type=Path,required=True,help='New directory for grid, trace, 3-D GIF and provenance')
    parser.add_argument('--headless',action='store_true',help='Save the 3-D animation without opening a desktop viewer')
    args=parser.parse_args()
    if not np.isfinite([args.start,args.goal]).all():parser.error('Start/goal must be finite')
    if args.output.exists():parser.error('Output directory already exists; choose a new one')
    if args.mesh.resolve().parent in args.output.resolve().parents or args.output.resolve()==args.mesh.resolve().parent:
        parser.error('Output must be outside the source mesh directory')
    import mujoco
    import trimesh
    from PIL import Image,ImageDraw,ImageFont
    mesh=trimesh.load(args.mesh,force='mesh')
    grid=build_mesh_grid(mesh.triangles)
    result=run_case(grid,[*args.start,0.],args.goal)
    if result['state']!='reached' or result['inflated_map_collision']:
        raise RuntimeError('No successful demo route: '+result['reason'])
    assert result['goal_error_m']<=.025 and grid.demo_only
    args.output.mkdir(parents=True)
    grid.save(args.output/'grid.npz')
    result.update(scope='Kinematic controller replay on scanned mesh; no physical or sensor-based navigation',
        mesh=str(args.mesh.resolve()),mesh_sha256=hashlib.sha256(args.mesh.read_bytes()).hexdigest(),
        start=args.start,goal=args.goal,grid=dict(resolution_m=grid.resolution,clearance_m=grid.clearance_m,
            free=int((grid.cells==FREE).sum()),occupied=int((grid.cells==OCCUPIED).sum()),
            unknown=int((grid.cells==UNKNOWN).sum()),traversable=int((~grid.blocked).sum())))
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    rb=make_scene(args.mesh,result['path'],args.start,args.goal)
    camera=mujoco.MjvCamera();camera.lookat[:]=[*(np.mean([args.start,args.goal],axis=0)),.1]
    camera.distance=5.5;camera.azimuth=110;camera.elevation=-60
    option=mujoco.MjvOption();option.geomgroup[3]=0
    rb.model.vis.global_.offwidth=1000;rb.model.vis.global_.offheight=720
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    poses=np.asarray(result['poses']);distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1))]
    indices=list(range(0,len(poses),5))
    if indices[-1]!=len(poses)-1:indices.append(len(poses)-1)
    frames=[]
    with mujoco.Renderer(rb.model,height=720,width=1000) as renderer:
        for index in indices:
            replay_pose(rb,poses[index],distance[index]/.035)
            renderer.update_scene(rb.data,camera=camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
            picture=Image.fromarray(renderer.render().copy());draw=ImageDraw.Draw(picture)
            draw.rectangle((0,0,1000,77),fill='#e9e8e4')
            draw.text((18,10),'RB trên bản quét 3D của chúng ta · mô phỏng bám đường',font=font,fill='#242729')
            draw.text((18,42),f'{index*.05:.1f} s  |  '+('ĐÃ TỚI ĐÍCH' if index==len(poses)-1 else 'ĐANG DI CHUYỂN')+
                      '  |  Cam: đường đi · Đỏ: đích · GIF phát 2×',font=small,fill='#343a3a')
            draw.rectangle((0,682,1000,720),fill='#e9e8e4')
            draw.text((18,692),'Mesh RGB–ToF đã lưu · Không dựng thêm sàn · Chưa mô phỏng cân bằng/định vị bằng cảm biến',font=small,fill='#343a3a')
            frames.append(picture)
    frames[0].save(args.output/'navigation_3d.gif',save_all=True,append_images=frames[1:],
                   duration=[125]*(len(frames)-1)+[2500],loop=0)
    frames[-1].save(args.output/'arrived.png')
    print(json.dumps({k:v for k,v in result.items() if k not in ('poses','path')},indent=2),flush=True)
    print('Animation:',args.output/'navigation_3d.gif',flush=True)
    if args.headless:return
    import mujoco.viewer
    replay=[time.monotonic()]
    def key_callback(key):
        if key==32:replay[0]=time.monotonic()
    with mujoco.viewer.launch_passive(rb.model,rb.data,key_callback=key_callback,
                                      show_left_ui=False,show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:]=camera.lookat;viewer.cam.distance=camera.distance
            viewer.cam.azimuth=camera.azimuth;viewer.cam.elevation=camera.elevation
            viewer.opt.geomgroup[3]=0
        print('3-D viewer: mouse to orbit/zoom; Space to replay; close window to exit. KINEMATIC DEMO ONLY.',flush=True)
        while viewer.is_running():
            index=min(len(poses)-1,int((time.monotonic()-replay[0])/.05))
            with viewer.lock():replay_pose(rb,poses[index],distance[index]/.035)
            viewer.sync();time.sleep(.02)


if __name__=='__main__':main()
