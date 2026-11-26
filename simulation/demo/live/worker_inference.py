"""RGB + ToF inference worker."""
import json
import sys
import time
from collections import deque

import numpy as np

from worker_common import FUSION_DEPTH_SOURCE, ROOT, status, write_json, write_npz
sys.path.insert(0, str(ROOT / 'simulation/demo/reconstruction'))
from ground_segmentation import GroundSegmenter

def inference(session):
    from sensors import Sensors
    from vslam.localization import odom_body,rigid
    from obser.fusion import Fusion

    ground=GroundSegmenter()
    status(session,'infer','Đang nạp DELTAR trên GPU')
    sensors=Sensors()
    fusion=Fusion(sensors.K, sensors.cal)
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
            rendered = time.monotonic()
            measured=fusion.depth_mm(rgb, hist, rect, mask)
            fused = time.monotonic()
            if warmup:
                warmup=False;status(session,'infer','GPU sẵn sàng; bắt đầu lấy ảnh trực tiếp');continue
            labels=ground.classify(measured.astype(np.float32)/1000,sensors.K,
                                   odom_body(packet)@rigid(mount),t)
            write_npz(session/'rgbd.npz',rgb=rgb,depth=measured,
                      t=t,K=sensors.K,mount=mount,odometry=json.dumps(packet),
                      depth_source=FUSION_DEPTH_SOURCE,
                      tof_hist=hist,tof_rect=rect,tof_valid=mask,**labels)
            status(session,'infer','RGB-D đang chạy',sensor_time=t,inference_ms=1000*(time.monotonic()-start),
                   render_ms=1000*(rendered-start), fusion_ms=1000*(fused-rendered),
                   label_and_publish_ms=1000*(time.monotonic()-fused),
                   depth_source=FUSION_DEPTH_SOURCE,ground_pixels=int((labels['ground_labels']==1).sum()),
                   obstacle_pixels=int((labels['ground_labels']==2).sum()))
