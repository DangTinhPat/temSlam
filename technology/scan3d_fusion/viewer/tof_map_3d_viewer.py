"""Interactive 3D view of a native ToF map session (voxel cloud + 2D projection + path)."""
import argparse
import csv
import json
from pathlib import Path
import tkinter as tk

import numpy as np
from PIL import Image
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.figure import Figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True, help='datasets/raw/tof-... session')
    args = parser.parse_args()
    session = args.session.resolve()
    root = tk.Tk();root.title('RB · ToF map · interactive 3D');root.geometry('1100x900+360+60')
    figure=Figure(figsize=(11,8),dpi=95);ax=figure.add_subplot(111,projection='3d')
    ax.set_facecolor('#f3f5f7');ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',
        title='Occupied ToF voxels · height color · gray projected map · green path')
    ax.view_init(elev=38,azim=-55);ax.set_zlim(0,.85);ax.set_box_aspect((1,1,.45))
    colorbar=figure.colorbar(ScalarMappable(norm=Normalize(.025,.8),cmap='turbo'),ax=ax,shrink=.65,pad=.1)
    colorbar.set_label('height (m)')
    canvas=FigureCanvasTkAgg(figure,master=root);canvas.get_tk_widget().pack(fill='both',expand=True)
    NavigationToolbar2Tk(canvas,root,pack_toolbar=True)
    status=tk.Label(root,anchor='w');status.pack(fill='x')
    previous=-1
    def update():
        nonlocal previous
        try:
            state=json.loads((session/'state.json').read_text())
            if state['frame']!=previous:
                previous=state['frame']
                cloud=np.loadtxt(session/'map.ply',skiprows=10,ndmin=2)
                map_resolution=state['resolution'];stride=max(1,round(.4/map_resolution))
                gray=np.asarray(Image.open(session/'map.pgm').convert('L'))[::stride,::stride]
                rgba=np.repeat((gray/255.0)[:,:,None],3,axis=2)
                rgba=np.concatenate((rgba,np.where(gray==127,0.0,np.where(gray<100,.85,.22))[:,:,None]),axis=2)
                row,col=np.indices(gray.shape)
                cell=map_resolution*stride
                origin_x,origin_y=state['origin'];map_height=state['height']*map_resolution
                xx=origin_x+(col+.5)*cell;yy=origin_y+map_height-(row+.5)*cell
                for artist in list(ax.collections):artist.remove()
                ax.plot_surface(xx,yy,np.full_like(xx,.006),facecolors=rgba,shade=False,
                                rstride=1,cstride=1,linewidth=0,antialiased=False)
                if cloud.size:
                    ax.scatter(cloud[:,0],cloud[:,1],cloud[:,2],c=cloud[:,2],cmap='turbo',
                        norm=Normalize(state['voxel_z_m'],.8),s=8,marker='s',depthshade=False,rasterized=True)
                    xmin,xmax=cloud[:,0].min()-.5,cloud[:,0].max()+.5
                    ymin,ymax=cloud[:,1].min()-.5,cloud[:,1].max()+.5
                else:xmin,xmax,ymin,ymax=-2,2,-2,2
                tx=[];ty=[]
                with (session/'trajectory.csv').open() as f:
                    for row in csv.DictReader(line for line in f if not line.startswith('#')):
                        if row.get('valid')=='1':tx.append(float(row['x']));ty.append(float(row['y']))
                if len(tx)>1:
                    stride=max(1,len(tx)//3000)
                    tx,ty=tx[::stride],ty[::stride]
                    ax.plot(tx,ty,np.full(len(tx),.015),color='#00a844',linewidth=1.8)
                    xmin=min(xmin,min(tx)-.5);xmax=max(xmax,max(tx)+.5)
                    ymin=min(ymin,min(ty)-.5);ymax=max(ymax,max(ty)+.5)
                ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax)
                span=max(2,xmax-xmin,ymax-ymin);ax.set_box_aspect((span,span,min(5,span*.6)))
                canvas.draw_idle()
                status.configure(text=f"t={state['t']:.1f}s · {state.get('occupied_voxels_3d',len(cloud)):,} voxels · {state['voxel_z_m']*100:.1f} cm · frame {state['frame']}  |  Drag: rotate · wheel: zoom · toolbar: pan/reset/save")
        except (OSError,ValueError,KeyError,IndexError):
            status.configure(text='Waiting for map files…')
        root.after(2500,update)
    update();root.mainloop()


if __name__=='__main__':main()
