"""Native mapper process and optional map display. Input is native runner output."""
import argparse
import datetime
import json
import math
import pathlib
import subprocess
import sys
import threading
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[3]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapper', type=pathlib.Path, required=True)
    parser.add_argument('--runner-command', default='[]')
    parser.add_argument('--session', type=pathlib.Path)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    session = args.session or ROOT / 'datasets/raw' / datetime.datetime.now().strftime('tof-%Y%m%d-%H%M%S-%f')
    session.mkdir(parents=True, exist_ok=True)
    (session / 'metadata.json').write_text(json.dumps({
        'schema': 1, 'input': 'TOFMAP', 'runner_command': json.loads(args.runner_command), 'calibration': (ROOT / 'technology/scan3d_fusion/config/tof.yaml').read_text(),
        'mapping': 'encoder/gyro odometry; 2.5cm voxel; observed free slices, not clearance',
        'command': [str(args.mapper), str(session)]}, indent=2))
    errors = open(session / 'mapper.stderr', 'w')
    mapper = subprocess.Popen([str(args.mapper), str(session)], stdin=subprocess.PIPE, text=True, stderr=errors)
    stopped = threading.Event()
    def feed():
        try:
            with open(session / 'sensors.log', 'w', buffering=1) as log:
                for line in sys.stdin:
                    if line.startswith('VIEW|'):
                        view = {key: float(value) for key, value in
                                (part.split('=', 1) for part in line.strip().split('|')[1:] if '=' in part)}
                        tmp = session / 'robot_state.tmp'
                        tmp.write_text(json.dumps(view))
                        tmp.replace(session / 'robot_state.json')
                        continue
                    if line.startswith(('TOFMAP ', 'MAPRESET')):
                        log.write(line)
                        mapper.stdin.write(line)
                        mapper.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass
        finally:
            try: mapper.stdin.close()
            except (OSError, ValueError): pass
            stopped.set()
    if args.headless:
        feed()
        errors.close()
        return
    threading.Thread(target=feed, daemon=True).start()
    root=tk.Tk();root.title('RB · ToF mapping · native / no ROS');root.geometry('+1120+50')
    canvas=tk.Canvas(root,width=720,height=720,bg='#7f7f7f');canvas.pack()
    status=tk.Label(root,anchor='w');status.pack(fill='x')
    tk.Label(root,text=f'Saved: {session}',anchor='w').pack(fill='x')
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize
    from matplotlib.figure import Figure
    view3d=tk.Toplevel(root);view3d.title('RB · ToF occupancy · 3D');view3d.geometry('800x760+400+50')
    figure=Figure(figsize=(8,7),dpi=90);ax=figure.add_subplot(111,projection='3d')
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',xlim=(-6.4,6.4),ylim=(-6.4,6.4),zlim=(0,.8),
           title='ToF 8×8 · occupied voxels, projected map and path')
    ax.set_facecolor('#f3f5f7');ax.set_box_aspect((16,16,5));ax.view_init(elev=38,azim=-55)
    points=ax.scatter([],[],[],s=14,marker='s',c=[],cmap='turbo',norm=Normalize(.05,.8),depthshade=False)
    figure.colorbar(ScalarMappable(norm=Normalize(.05,.8),cmap='turbo'),ax=ax,
                    shrink=.62,pad=.12,label='height (m)')
    trajectory=ax.plot([],[],[],color='#00a844',linewidth=1.8)[0]
    floor=None
    canvas3d=FigureCanvasTkAgg(figure,master=view3d);canvas3d.get_tk_widget().pack(fill='both',expand=True)
    NavigationToolbar2Tk(canvas3d,view3d,pack_toolbar=False).pack(fill='x')
    view3d.attributes('-topmost',True)
    path=[];last_frame=-1;last_t=-1
    def update():
        nonlocal last_frame,last_t,floor
        try:
            state=json.loads((session/'state.json').read_text())
            frame=state['frame']
            if frame!=last_frame or state['t']<last_t:
                if frame<last_frame or state['t']<last_t: path.clear()
                last_frame=frame;last_t=state['t']
                im=Image.open(session/'map.pgm').convert('RGB').resize((720,720),Image.Resampling.NEAREST)
                photo=ImageTk.PhotoImage(im);canvas.delete('all');canvas.create_image(0,0,image=photo,anchor='nw');canvas.image=photo
                map_resolution=state['resolution'];stride=max(1,round(.2/map_resolution))
                grid=np.asarray(Image.open(session/'map.pgm').convert('L'))[::stride,::stride]
                rgba=np.repeat((grid/255.0)[:,:,None],3,axis=2)
                rgba=np.concatenate((rgba,np.where(grid==127,0.0,np.where(grid<100,.85,.22))[:,:,None]),axis=2)
                row,col=np.indices(grid.shape)
                cell=map_resolution*stride
                origin_x,origin_y=state['origin'];map_width=state['width']*map_resolution;map_height=state['height']*map_resolution
                xx=origin_x+(col+.5)*cell; yy=origin_y+map_height-(row+.5)*cell
                if floor is not None: floor.remove()
                floor=ax.plot_surface(xx,yy,np.full_like(xx,.006),facecolors=rgba,
                                      rstride=1,cstride=1,shade=False,linewidth=0,antialiased=False)
                if state.get('occupied_voxels_3d',0):
                    cloud=np.loadtxt(session/'map.ply',skiprows=10,ndmin=2)
                    points._offsets3d=(cloud[:,0],cloud[:,1],cloud[:,2])
                    points.set_array(cloud[:,2])
                    xmin=min(cloud[:,0].min(),state['x'])-.5;xmax=max(cloud[:,0].max(),state['x'])+.5
                    ymin=min(cloud[:,1].min(),state['y'])-.5;ymax=max(cloud[:,1].max(),state['y'])+.5
                    ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax)
                    span=max(2.0,xmax-xmin,ymax-ymin)
                    ax.set_zlim(0,max(.8,float(cloud[:,2].max())+.1))
                    ax.set_box_aspect((span,span,min(5.0,span*.6)))
                def pixel(x,y): return ((x-origin_x)/map_width*720,(origin_y+map_height-y)/map_height*720)
                pt=pixel(state['x'],state['y'])
                if state['valid'] and (not path or math.dist(pt,path[-1])>1): path.append(pt)
                trajectory.set_data_3d([p[0]/(720/map_width)+origin_x for p in path],
                                       [origin_y+map_height-p[1]/(720/map_height) for p in path],
                                       [.025]*len(path))
                canvas3d.draw_idle()
                if len(path)>1: canvas.create_line(*[v for p in path[-10000:] for v in p],fill='#1266dd',width=2)
                px,py=pt;angle=state['yaw']
                canvas.create_oval(px-4,py-4,px+4,py+4,fill='#db2b28',outline='white')
                canvas.create_line(px,py,px+20*math.cos(angle),py-20*math.sin(angle),fill='#db2b28',width=3,arrow='last')
                canvas.create_line(20,695,65,695,width=3);canvas.create_text(42,680,text='1 m')
                mode='Mapping' if state['valid'] else 'Paused: waiting for supported balance / valid sensors'
                status.configure(text=f"{mode} | t={state['t']:.1f}s | {state['returns']}/64 | occupied={state['occupied']} | rejected={state['rejected']}")
        except (OSError, ValueError, KeyError):
            status.configure(text='Waiting for mapper output…')
        if mapper.poll() is not None:
            status.configure(text=f'Mapper stopped ({mapper.returncode}). Map/log saved in session folder.')
            if stopped.is_set(): root.after(1000,root.destroy)
        root.after(200,update)
    try:
        update();root.mainloop()
    finally:
        if mapper.poll() is None:
            mapper.terminate()
            try: mapper.wait(timeout=2)
            except subprocess.TimeoutExpired: mapper.kill();mapper.wait()
        errors.close()

if __name__=='__main__': main()
