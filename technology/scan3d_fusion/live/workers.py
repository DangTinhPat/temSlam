"""Separate GPU inference, tracking, and mesh workers; only display uses ROS."""
import argparse,json,os,sys,time,traceback
from collections import deque
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
FUSION_DEPTH_SOURCE='deltar_tof_fusion'
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/mapping/cuvslam'))
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/reconstruction'))
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/mapping/odometry'))
from ground_segmentation import GroundSegmenter,FIELDS,SOURCE,read_labels

def write_json(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value));tmp.replace(path)

def write_npz(destination,**values):
    tmp=destination.with_suffix('.tmp')
    with tmp.open('wb') as f: np.savez(f,**values)
    tmp.replace(destination)

def status(session,stage,message,**values):
    write_json(session/(stage+'_status.json'),dict(message=message,wall_time=time.time(),**values))

def keep_keyframe(pose, frames):
    if len(frames) < 3:
        return True
    previous = np.asarray(frames[-1]['camera_to_world'])
    moved = np.linalg.norm(pose[:3, 3]-previous[:3, 3]) >= .05
    rotated = np.trace(previous[:3, :3].T@pose[:3, :3]) <= 1+2*np.cos(.08)
    return moved or rotated

def cloud_voxel_indices(cloud,labels=None):
    if not len(cloud):return np.empty(0,dtype=np.int64)
    cells=np.floor(cloud/.03).astype(np.int64)
    if labels is not None:
        cells=np.column_stack((cells,labels))
    keys=np.ravel_multi_index((cells-cells.min(0)).T,np.ptp(cells,axis=0)+1)
    return np.unique(keys,return_index=True)[1]

def fusion_depth(sample):
    if ('depth_source' not in sample or str(sample['depth_source'])!=FUSION_DEPTH_SOURCE
            or ('floor_plane' in sample and np.isfinite(sample['floor_plane']).any())):
        raise ValueError('Depth lacks fusion-only provenance or contains floor correction; replay RGB + ToF')
    read_labels(sample,sample['depth'].shape)
    return sample['depth'].astype(np.float32)/1000

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

def tracking(session):
    from track_rgbd import RgbdSlam,pose_matrix,optimized_map_poses,SLAM_MAP_SOURCE,validate_local_tracking
    from localization import odom_body,rigid
    slam=None;last=-1;frames=[];points=[];point_labels=[];mounts=[];packets=[];times=[];visual_frames=[]
    previous_visual=previous_sensor=None
    clean_stop=False
    def refresh_map():
        optimized={p.timestamp_ns:pose_matrix(p.pose) for p in slam.tracker.get_all_slam_poses()}
        mapped=optimized_map_poses(optimized,[f['timestamp_ns'] for f in frames],
                                   odom_body(packets[0])@rigid(mounts[0]))
        cloud=[]
        for i,(frame,corrected) in enumerate(zip(frames,mapped)):
            frame['camera_to_world']=corrected.tolist()
            cloud.append(points[i]@corrected[:3,:3].T+corrected[:3,3])
        cloud=np.concatenate(cloud).astype(np.float32)
        if len(cloud):
            labels=np.concatenate(point_labels)
            indices=cloud_voxel_indices(cloud,labels)
            cloud=cloud[indices]
            write_npz(session/'cloud.npz',xyz=cloud,path=np.array([f['camera_to_world'] for f in frames])[:,:3,3],
                      depth_source=FUSION_DEPTH_SOURCE,labels=labels[indices],ground_source=SOURCE)
            write_json(session/'live_sequence.json',dict(frames=frames,intrinsics=intrinsics,depth_source=FUSION_DEPTH_SOURCE,
                pose_source=SLAM_MAP_SOURCE,bounds_m=[(cloud.min(0)-.1).tolist(),(cloud.max(0)+.1).tolist()]))
        return len(cloud)
    (session/'keyframes').mkdir(exist_ok=True)
    try:
        while not (session/'STOP').exists():
            try:
                with np.load(session/'rgbd.npz',allow_pickle=False) as f: batch={k:f[k] for k in f.files}
            except FileNotFoundError: time.sleep(.05);continue
            t=float(batch['t'])
            if t<=last: time.sleep(.02);continue
            depth=fusion_depth(batch)
            intrinsics=batch['K'].tolist()
            packet=json.loads(str(batch['odometry']))
            # Timestamp gaps alone do not prove tracking loss; cuVSLAM and the pose/odometry gate decide.
            if slam is None:
                status(session,'slam','Đang làm nóng GPU tracking')
                slam=RgbdSlam([640,480],batch['K'].tolist())

            last=t
            ts=round(t*1e9);pose=slam.track(ts,batch['rgb'],batch['depth'])
            if pose is None: raise RuntimeError('Mất tracking: map đã giữ lại. Stop SIM rồi Start SIM để quét lại')
            sensor_pose=odom_body(packet)@rigid(batch['mount'])
            if previous_visual is not None:
                validate_local_tracking(previous_visual,slam.last_odometry_pose,previous_sensor,sensor_pose)
            previous_visual,previous_sensor=slam.last_odometry_pose,sensor_pose
            # Bootstrap readiness, then retain views by motion; idle time must not fill the map.
            if not keep_keyframe(pose, visual_frames):
                metrics=slam.tracker.get_slam_metrics()
                if frames and metrics and (metrics.lc_status or metrics.pgo_status):refresh_map()
                status(session,'slam','Đang theo dõi',sensor_time=t,keyframes=len(frames),head_yaw_rad=packet['head_yaw']);continue
            if len(frames)>=4096: raise RuntimeError('Đã đạt 4096 keyframe: map đã lưu; bắt đầu phiên mới')
            filename=f'keyframes/{len(frames):05d}.npz'
            visual_frames.append({'camera_to_world':pose.copy()})
            write_npz(session/filename,depth=batch['depth'],depth_source=FUSION_DEPTH_SOURCE,
                      tof_hist=batch['tof_hist'],tof_rect=batch['tof_rect'],tof_valid=batch['tof_valid'],
                      **{key:batch[key] for key in FIELDS if key in batch})
            frames.append(dict(file=filename,time_s=t,timestamp_ns=ts,camera_to_world=pose.tolist()))
            mounts.append(batch['mount']);packets.append(packet);times.append(t)
            y,x=np.mgrid[0:480:8,0:640:8];z=depth[::8,::8];fx,fy,cx,cy=batch['K']
            valid=(z>.02)&(z<=4)
            points.append(np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[valid])
            pixel_labels=read_labels(batch,depth.shape)
            point_labels.append(np.zeros(valid.sum(),np.uint8) if pixel_labels is None else pixel_labels[::8,::8][valid])
            count=refresh_map()
            status(session,'slam','Đang vẽ bản đồ',sensor_time=t,keyframes=len(frames),points=count,head_yaw_rad=packet['head_yaw'])
        clean_stop=True
    finally:
        if slam is not None and clean_stop:
            if frames:refresh_map()
            saved=[]
            slam.tracker.save_map(str(session/'cuvslam_map'),saved.append)
            if saved!=[True]:raise RuntimeError('cuVSLAM map save failed')
            status(session,'slam','Đã lưu lịch sử pose cuối và database SLAM',keyframes=len(frames))

def build_surface(session, sequence):
    from reconstruct import TSDF
    bounds=np.array(sequence['bounds_m']);voxel=max(.04,float(np.prod(bounds[1]-bounds[0])/4_000_000)**(1/3))
    volume=TSDF(bounds,voxel=voxel,truncation=6*voxel)
    for f in sequence['frames']:
        with np.load(session/f['file'],allow_pickle=False) as sample: depth=fusion_depth(sample)
        volume.integrate(depth,f['camera_to_world'],sequence['intrinsics'])
    return volume.mesh(min_views=3), voxel


def finalize(session):
    import torch
    torch.set_num_threads(2)
    sequence=json.loads((session/'live_sequence.json').read_text())
    write_json(session/'surface_sequence.json',sequence)
    surface,voxel=build_surface(session,sequence)
    target=session/'final_mesh.stl'
    surface.export(target)
    surface.export(session/'final_mesh.ply')
    write_json(target.with_suffix('.json'),dict(pose_source=sequence['pose_source'],
        frames=len(sequence['frames']),voxel_m=voxel,faces=len(surface.faces),bounds_m=surface.bounds.tolist(),
        depth_source=FUSION_DEPTH_SOURCE))
    write_json(session/'mesh.json',dict(file=str(target),frames=len(sequence['frames']),depth_source=FUSION_DEPTH_SOURCE))
    status(session,'mesh','Mesh cuối từ RGB + ToF fusion đã xuất',faces=len(surface.faces))


def mesh(session):
    import torch
    torch.set_num_threads(2)
    from reconstruct import TSDF
    status(session,'mesh','Đang làm nóng GPU mesh')
    warm=TSDF([[0,0,.1],[.1,.1,.2]],voxel=.04,truncation=.24)
    warm.integrate(np.ones((8,8),np.float32),np.eye(4),[4,4,4,4])
    torch.cuda.synchronize();del warm
    status(session,'mesh','Chờ dữ liệu để dựng bề mặt')
    last=0;last_sequence=None
    while not (session/'STOP').exists():
        time.sleep(1)
        if time.monotonic()-last<5: continue
        try: sequence=json.loads((session/'live_sequence.json').read_text())
        except FileNotFoundError: continue
        last=time.monotonic()
        if len(sequence['frames'])<3:
            status(session,'mesh','Chờ ít nhất 3 keyframe');continue
        if sequence==last_sequence: continue
        last_sequence=sequence
        status(session,'mesh','Đang dựng mesh')
        try: surface,_=build_surface(session,sequence)
        except RuntimeError:
            status(session,'mesh','Chưa đủ góc nhìn cho mesh');continue
        target=session/f'mesh-{time.time_ns()}.stl'
        surface.export(target)
        write_json(session/'mesh.json',dict(file=str(target),frames=len(sequence['frames']),depth_source=FUSION_DEPTH_SOURCE))
        status(session,'mesh','Mesh đã cập nhật',faces=len(surface.faces))
        # RViz receives a unique URI for each revision to invalidate its mesh cache.
        old=sorted(session.glob('mesh-*.stl'))[:-3]
        for path in old: path.unlink(missing_ok=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['infer','slam','mesh','finalize']);p.add_argument('session',type=Path)
    a=p.parse_args();status(a.session,a.stage,'Đang khởi động')
    try: {'infer':inference,'slam':tracking,'mesh':mesh,'finalize':finalize}[a.stage](a.session)
    except Exception as e:
        status(a.session,a.stage,'LỖI: '+str(e));traceback.print_exc();raise
if __name__=='__main__': main()
