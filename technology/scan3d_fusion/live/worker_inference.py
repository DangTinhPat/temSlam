"""RGB + ToF inference worker."""
import json
import sys
import time
from collections import deque

import numpy as np

from worker_common import FUSION_DEPTH_SOURCE, ROOT, status, write_json, write_npz
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/mapping/cuvslam'))
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/mapping/odometry'))
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/reconstruction'))
from ground_segmentation import GroundSegmenter

def inference(session):
    import torch,timm,h5py
    torch.set_num_threads(2)
    from sensors import Sensors
    from depth_support import tof_support
    from localization import odom_body,rigid
    ground=GroundSegmenter()
    bundle=ROOT/'technology/scan3d_fusion/model/deltar'
    sys.path.insert(0,str(bundle));sys.argv=[str(bundle/'evaluate.py'),str(ROOT/'technology/scan3d_fusion/config/deltar_eval.txt')]
    original=timm.create_model
    def create(name,*a,**kw): kw['pretrained']=False;return original(name,*a,**kw)
    timm.create_model=create
    from src.config import args
    from src.models.deltar import Deltar
    from src.dataloader.zjuL5 import ZJUL5
    status(session,'infer','Đang nạp DELTAR trên GPU')
    model=Deltar(n_bins=args.n_bins,min_val=args.min_depth,max_val=args.max_depth,norm='linear').cuda().eval()
    model.load_state_dict(torch.load(str(bundle/'weights/nyu.pt'),map_location='cuda'))
    sensors=Sensors();work=session/'inference';work.mkdir(exist_ok=True)
    write_json(work/'data.json',{'train':[],'test':[{'filename':'frame.h5'}]})
    args.data_path_eval=str(work);args.filenames_file_eval=str(work/'data.json')
    loader=ZJUL5(args,'online_eval').data
    odom=[];pending='';last=-1;warmup=True;captures=deque(maxlen=12000);capture_pending=''
    while not (session/'odometry.jsonl').exists():
        if (session/'STOP').exists(): return
        time.sleep(.05)
    capture_path=session/'live_capture.jsonl'
    while not capture_path.exists():
        if (session/'STOP').exists(): return
        time.sleep(.05)
    status(session,'infer','Đang chờ ảnh và odometry đồng bộ')
    with (session/'odometry.jsonl').open() as stream, capture_path.open() as capture_stream:
        while not (session/'STOP').exists():
            chunk=stream.read()
            lines=(pending+chunk).split('\n');pending=lines.pop()
            for line in lines:
                if not line: continue
                packet=json.loads(line)
                if 'reset' in packet:
                    if last>=0: raise RuntimeError('Simulator đã reset: Stop SIM rồi Start SIM để tạo map mới')
                    odom=[]
                else: odom.append(packet)
            odom=odom[-300:]
            chunk=capture_stream.read()
            lines=(capture_pending+chunk).split('\n');capture_pending=lines.pop()
            captures.extend(json.loads(line) for line in lines if line)
            if not odom or not captures: time.sleep(.02);continue
            v=packet=None
            for candidate in reversed(captures):
                t=float(candidate['t'])
                if t<=last: break
                nearest=min(odom,key=lambda p:abs(p['time_s']-t))
                if abs(nearest['time_s']-t)<=.02:
                    v,packet=candidate,nearest
                    break
            if v is None: time.sleep(.02);continue
            t=float(v['t'])
            last=t
            if not packet['valid']:
                status(session,'infer','Chờ robot đứng cân bằng');time.sleep(.05);continue
            start=time.monotonic()
            rgb,hist,rect,mask,mount=sensors.render(v)
            with h5py.File(work/'frame.h5','w') as h:
                for k,value in dict(rgb=rgb,depth=np.zeros((480,640),np.float32),hist_data=hist,fr=rect,mask=mask).items(): h.create_dataset(k,data=value)
            batch=next(iter(loader));extra=batch['additional']
            with torch.no_grad():
                prediction=model({'rgb':batch['image'].cuda(),'additional':{
                    'hist_data':extra['hist_data'].cuda(),'rect_data':extra['rect_data'].cuda(),
                    'mask':extra['mask'].cuda(),'patch_info':extra['patch_info']}})[1]
                depth=torch.nn.functional.interpolate(prediction,size=(480,640),mode='bilinear',align_corners=True).squeeze().cpu().numpy()
            depth=tof_support(depth,hist,rect,mask,sensors.K,sensors.cal)
            if warmup:
                warmup=False;status(session,'infer','GPU sẵn sàng; bắt đầu lấy ảnh trực tiếp');continue
            measured=np.rint(np.clip(depth,0,65.535)*1000).astype(np.uint16)
            labels=ground.classify(measured.astype(np.float32)/1000,sensors.K,
                                   odom_body(packet)@rigid(mount),t)
            write_npz(session/'rgbd.npz',rgb=rgb,depth=measured,
                      t=t,K=sensors.K,mount=mount,odometry=json.dumps(packet),
                      depth_source=FUSION_DEPTH_SOURCE,
                      tof_hist=hist,tof_rect=rect,tof_valid=mask,**labels)
            status(session,'infer','RGB-D đang chạy',sensor_time=t,inference_ms=1000*(time.monotonic()-start),
                   depth_source=FUSION_DEPTH_SOURCE,ground_pixels=int((labels['ground_labels']==1).sum()),
                   obstacle_pixels=int((labels['ground_labels']==2).sum()))
