"""Click-to-goal navigation UI. Demo is kinematic; native owns a MuJoCo trial."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from grid import NavigationGrid, FREE, OCCUPIED, UNKNOWN
from controller import Navigator

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_GRID=ROOT/'datasets/processed/scan3d_fusion/navigation-grid-20261114.npz'


def demo_grid():
    cells=np.zeros((100,100),np.int8)
    cells[15:64,48:53]=OCCUPIED
    cells[76:88,12:30]=OCCUPIED
    cells[78:,78:]=UNKNOWN
    return NavigationGrid(cells,[0,0],.05,.25)


class Viewport:
    def __init__(self, grid, width, height):
        self.center=grid.origin+np.array(grid.cells.shape[::-1])*grid.resolution/2
        size=np.array(grid.cells.shape[::-1])*grid.resolution
        self.scale=max(1.,min((width-48)/size[0],(height-48)/size[1]))
        self.mid=np.array([width,height])/2
    def screen(self, xy):return (self.mid+(np.asarray(xy)-self.center)*[self.scale,-self.scale]).tolist()
    def world(self, pixel):return (self.center+(np.asarray(pixel)-self.mid)/[self.scale,-self.scale]).tolist()


class NavigationApp:
    def __init__(self, root, grid, initial_pose, mode='demo', map_label='Bản đồ 2D'):
        import tkinter as tk
        from tkinter import ttk
        self.root,self.grid,self.mode=root,grid,mode
        self.pose=np.asarray(initial_pose,float);self.navigator=Navigator(grid)
        self.preview=[];self.target=None;self.runtime=None;self.log=None;self.message='Bấm một ô đi được để chọn đích.'
        self.placing=False;self.running=False;self.last_tick=time.monotonic();self.closed=False
        self.view=Viewport(grid,760,660)
        if not np.any(~grid.blocked):self.message='Bản quét chưa có vùng đủ khoảng hở để lập đường.'
        root.title('RB · Navigation · Chọn đích trên bản đồ');root.geometry('1100x760');root.minsize(900,650)
        root.configure(bg='#e9e8e4');root.protocol('WM_DELETE_WINDOW',self.close)
        root.bind('<Escape>',lambda event:self.cancel())
        style=ttk.Style();style.configure('TButton',font=('Helvetica',11),padding=6);style.configure('Nav.TFrame',background='#e9e8e4')
        style.configure('Nav.TLabel',background='#e9e8e4',foreground='#242729',font=('Helvetica',10))
        outer=ttk.Frame(root,padding=18,style='Nav.TFrame');outer.pack(fill='both',expand=True)
        ttk.Label(outer,text='Chọn đích · Xem đường · Di chuyển',style='Nav.TLabel',font=('Helvetica',17,'bold')).pack(anchor='w')
        mode_text=('MÔ PHỎNG ĐỘNG HỌC · không điều khiển RB' if mode=='demo' else
                   'RB TRONG MUJOCO · wheel/IMU thử nghiệm · tối đa 2 m / 60 s')
        ttk.Label(outer,text=mode_text+' · '+map_label,style='Nav.TLabel').pack(anchor='w',pady=(7,14))
        body=ttk.Frame(outer,style='Nav.TFrame');body.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(body,bg='#e9e8e4',highlightthickness=0,width=760,height=620)
        self.canvas.pack(side='left',fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda event:self.draw())
        self.canvas.bind('<Button-1>',self.click)
        panel=ttk.Frame(body,padding=(18,0,0,0),width=285,style='Nav.TFrame');panel.pack(side='right',fill='y')
        self.status=tk.StringVar(value='Sẵn sàng');self.details=tk.StringVar()
        ttk.Label(panel,textvariable=self.status,wraplength=260,style='Nav.TLabel',font=('Helvetica',12,'bold')).pack(anchor='w',pady=(0,10))
        ttk.Label(panel,textvariable=self.details,wraplength=260,style='Nav.TLabel').pack(anchor='w',pady=(0,15))
        self.start_button=ttk.Button(panel,text='Đi tới đích',command=self.begin);self.start_button.pack(fill='x',pady=4)
        ttk.Button(panel,text='Dừng / Hủy chuyến đi · Esc',command=self.cancel).pack(fill='x',pady=4)
        self.place_button=ttk.Button(panel,text='Chọn vị trí ban đầu',command=self.place_start);self.place_button.pack(fill='x',pady=4)
        if mode=='native':
            ttk.Button(panel,text='Khởi động RB trong MuJoCo',command=self.start_native).pack(fill='x',pady=(16,4))
        free_label='Trắng ngà: sàn mesh đủ hỗ trợ' if grid.demo_only else 'Trắng ngà: đã quan sát trống'
        ttk.Label(panel,text=free_label+'\nXám đậm: vật cản\nXám nhạt: chưa đủ dữ liệu\nBe: khoảng đệm bị chặn\nCam: đường dự kiến',style='Nav.TLabel',wraplength=260).pack(anchor='w',pady=22)
        ttk.Label(panel,text='Vùng chưa biết bị chặn.\nBộ định vị scan matching hiện chưa đạt; màn hình này không chứng nhận robot tự hành an toàn.',style='Nav.TLabel',wraplength=260).pack(anchor='w')
        self.update_labels();self.root.after(50,self.tick)

    def update_labels(self):
        self.status.set(self.message)
        target='Chưa chọn' if self.target is None else f'{self.target[0]:.2f}, {self.target[1]:.2f} m'
        length=sum(math.dist(a,b) for a,b in zip(self.preview,self.preview[1:]))
        self.details.set(f'Vị trí: {self.pose[0]:.2f}, {self.pose[1]:.2f} m\nHướng: {math.degrees(self.pose[2]):.0f}°\nĐích: {target}\nĐường dự kiến: {length:.2f} m\nKhoảng đệm: {self.grid.clearance_m:.2f} m')
        ready=(self.mode=='demo' or (self.runtime is not None and self.runtime.ready))
        self.start_button.configure(state='normal' if self.preview and ready and not self.running else 'disabled')

    def select_goal(self, xy):
        self.cancel(silent=True);self.target=list(xy);self.preview=[]
        if self.runtime is not None:self.runtime.rearm()
        try:
            self.preview=self.grid.plan(self.pose[:2],xy)
            self.message='Đã có đường. Bấm “Đi tới đích”.'
        except ValueError as exc:self.message='Không thể lập đường: '+str(exc)
        self.update_labels();self.draw()

    def click(self,event):
        xy=self.view.world([event.x,event.y])
        if self.placing:
            if not self.grid.segment_clear(xy,xy):
                self.message='Vị trí ban đầu phải nằm trong vùng đi được.'
            else:
                self.pose[:2]=xy;self.placing=False;self.target=None;self.preview=[]
                self.message='Đã đặt vị trí ban đầu; tiếp tục chọn đích.'
            self.update_labels();self.draw()
        else:self.select_goal(xy)

    def place_start(self):
        if self.runtime is not None:
            self.message='Đóng và mở phiên mới để đổi initial pose của RB.'
        else:
            self.cancel(silent=True);self.placing=True
            self.message='Bấm vị trí thân robot ban đầu trên bản đồ.'
        self.update_labels()

    def begin(self):
        if self.target is None or not self.preview:return
        if self.mode=='native' and (self.runtime is None or not self.runtime.ready):
            self.message='Chờ RB cân bằng và cảm biến sẵn sàng.';self.update_labels();return
        result=self.navigator.set_goal(self.pose[:2],self.target)
        self.running=result['state']=='planning'
        if self.runtime is not None:self.runtime.rearm()
        self.last_tick=time.monotonic();self.message='Đang di chuyển' if self.running else result['reason']
        self.update_labels()

    def cancel(self,silent=False):
        result=self.navigator.cancel();self.running=False
        if self.runtime is not None and self.runtime.backend is not None:self.runtime.command(result)
        if not silent:self.message='Đã hủy chuyến đi.';self.update_labels()

    def start_native(self):
        if self.runtime is not None:return
        if self.grid.demo_only:
            self.message='Grid chiếu từ mesh chỉ dùng demo; chưa có bằng chứng khoảng trống từ cảm biến.'
            self.update_labels();return
        if not self.grid.segment_clear(self.pose[:2],self.pose[:2]):
            self.message='Initial pose bị chặn; kiểm tra map và vị trí trước khi khởi động.';self.update_labels();return
        from runtime import NativeNavigation
        self.runtime=NativeNavigation(self.pose)
        try:
            self.runtime.start()
            self.log=(self.runtime.backend.map_session/'navigation.jsonl').open('w')
            self.place_button.configure(state='disabled');self.message='Đang khởi động RB và cảm biến…'
        except Exception as exc:
            self.runtime.close();self.runtime=None;self.message='Không thể khởi động: '+str(exc)
        self.update_labels()

    def tick(self):
        if self.closed:return
        now=time.monotonic()
        try:
            measured=guard=None
            if self.runtime is not None:
                measured,guard=self.runtime.observe(now)
                if measured is not None and measured['usable']:self.pose[:2]=measured['xy'];self.pose[2]=measured['yaw']
                if not self.running and self.navigator.state=='idle':self.message=self.runtime.message
            elif self.mode=='demo':
                measured=dict(xy=self.pose[:2].tolist(),yaw=float(self.pose[2]),stamp=now,usable=True,uncertainty_m=0.)
                guard=dict(stamp=now,clear=True,reason='Kinematic demo: static grid, no real depth sensor')
            decision_now=time.monotonic();dt=decision_now-self.last_tick;self.last_tick=decision_now
            if self.running:
                result=self.navigator.update(measured,decision_now,guard,dt)
                if self.runtime is not None:
                    self.runtime.command(result)
                    self.log.write(json.dumps(dict(wall_time=time.time(),pose=measured,**result))+'\n');self.log.flush()
                elif self.mode=='demo':
                    yaw=self.pose[2]+result['w']*dt/2
                    self.pose[:2]+=result['v']*dt*np.array([np.cos(yaw),np.sin(yaw)])
                    self.pose[2]=math.atan2(math.sin(self.pose[2]+result['w']*dt),math.cos(self.pose[2]+result['w']*dt))
                self.running=result['state'] in ('planning','driving','aligning')
                labels=dict(reached='Đã tới đích',blocked='Đã dừng: đường/cảm biến bị chặn',lost='Đã dừng: mất pose hoặc hết hạn',driving='Đang di chuyển',aligning='Đang căn hướng',planning='Đang lập đường')
                self.message=labels.get(result['state'],result['state'])
                if result['reason']:self.message+=' · '+result['reason']
            self.update_labels();self.draw()
        except Exception as exc:
            self.cancel(silent=True);self.message='Đã dừng do lỗi: '+str(exc);self.update_labels()
        self.root.after(50,self.tick)

    def draw(self):
        from PIL import Image,ImageTk
        width,height=max(100,self.canvas.winfo_width()),max(100,self.canvas.winfo_height())
        self.view=Viewport(self.grid,width,height);self.canvas.delete('all')
        rgb=np.empty((*self.grid.cells.shape,3),np.uint8);rgb[:]=[199,202,198]
        rgb[self.grid.cells==FREE]=[247,245,239];rgb[self.grid.cells==OCCUPIED]=[73,78,82]
        rgb[(self.grid.cells==FREE)&self.grid.blocked]=[222,210,190]
        image=Image.fromarray(rgb[::-1]);size=np.maximum(1,np.rint(np.array(image.size)*self.grid.resolution*self.view.scale)).astype(int)
        image=image.resize(tuple(size),Image.Resampling.NEAREST);self.photo=ImageTk.PhotoImage(image)
        self.canvas.create_image(width/2,height/2,image=self.photo)
        if len(self.preview)>1:
            coords=[v for xy in self.preview for v in self.view.screen(xy)]
            self.canvas.create_line(*coords,fill='#b6652c',width=3)
        if self.target is not None:
            x,y=self.view.screen(self.target);self.canvas.create_oval(x-6,y-6,x+6,y+6,outline='#b6652c',width=3)
        x,y=self.view.screen(self.pose[:2]);r=max(6,self.grid.clearance_m*self.view.scale)
        color='#31576b' if self.grid.segment_clear(self.pose[:2],self.pose[:2]) else '#ae4039'
        self.canvas.create_oval(x-r,y-r,x+r,y+r,outline=color,width=2)
        self.canvas.create_oval(x-4,y-4,x+4,y+4,fill=color,outline=color)
        tip=self.view.screen(self.pose[:2]+self.grid.clearance_m*np.array([np.cos(self.pose[2]),np.sin(self.pose[2])]))
        self.canvas.create_line(x,y,*tip,fill=color,width=3,arrow='last')
        self.canvas.create_text(24,18,anchor='nw',text='↑ +Y     → +X     đơn vị: mét',fill='#44484b',font=('Helvetica',10))

    def close(self):
        self.closed=True;self.cancel(silent=True)
        if self.runtime is not None:self.runtime.close()
        if self.log is not None:self.log.close()
        self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid',type=Path,default=DEFAULT_GRID)
    parser.add_argument('--example',action='store_true',help='Synthetic test grid; kinematic mode only')
    parser.add_argument('--mode',choices=['demo','native'],default='demo')
    parser.add_argument('--initial-pose',nargs=3,type=float,metavar=('X','Y','YAW_DEG'))
    args=parser.parse_args()
    if args.example and args.mode!='demo':parser.error('Example grid is only for kinematic demo')
    if args.mode=='native' and args.initial_pose is None:parser.error('Native requires explicit --initial-pose X Y YAW_DEG')
    grid=demo_grid() if args.example else NavigationGrid.load(args.grid)
    if args.mode=='native' and grid.demo_only:parser.error('Mesh projection grid is demo-only; native motion is disabled')
    initial=args.initial_pose or ([.8,.8,0] if args.example else [.15,0,0])
    if not np.isfinite(initial).all():parser.error('Initial pose must be finite')
    import tkinter as tk
    root=tk.Tk();NavigationApp(root,grid,[initial[0],initial[1],np.deg2rad(initial[2])],args.mode,
        map_label=('BẢN ĐỒ MINH HỌA' if args.example else 'CHIẾU TỪ MESH · CHỈ DEMO' if grid.demo_only
                   else 'Bản đồ phòng khách từ RGB–ToF'));root.mainloop()


if __name__=='__main__':main()
