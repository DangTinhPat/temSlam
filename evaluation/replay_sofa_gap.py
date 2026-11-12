"""Replay denser recorded RGB/ToF views, with wheel/IMU-anchored estimated poses.

Simulator body poses reproduce sensors only. Map poses never use simulator world transforms.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import h5py
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/live'),str(ROOT/'technology/scan3d_fusion/reconstruction'),
             str(ROOT/'technology/scan3d_fusion/mapping/odometry')]
from sensors import Sensors
from localization import odom_body
from depth_support import tof_support
from workers import FUSION_DEPTH_SOURCE,build_surface,write_json


def anchored_pose(mapped_anchor,local_anchor,local_current):
    return mapped_anchor@np.linalg.inv(local_anchor)@local_current


def steady_sweep(previous,current):
    dt=current['time_s']-previous['time_s']
    if dt<=0 or dt>.2 or not current['valid']:return False
    speed=np.hypot(current['x']-previous['x'],current['y']-previous['y'])/dt
    rate=(current['head_yaw']-previous['head_yaw'])/dt
    return speed<.035 and .02<rate<.5


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw',type=Path);parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--room',action='store_true',help='Use steady forward head sweeps throughout the recorded room')
    args=parser.parse_args();raw=args.raw.resolve();source=args.source.resolve();out=args.output.resolve()
    sequence=json.loads((source/'live_sequence.json').read_text());frames=sequence['frames'];anchor_times=np.array([f['time_s'] for f in frames])
    captures=[json.loads(line) for line in (raw/'live_capture.jsonl').open()];times=np.array([float(v['t']) for v in captures])
    packets=[json.loads(line) for line in (raw/'odometry.jsonl').open() if 'reset' not in json.loads(line)]
    odom_times=np.array([p['time_s'] for p in packets]);cmd=json.loads((raw/'metadata.json').read_text())['runner_command']
    os.environ['RB1_SIM_MODEL']=cmd[cmd.index('--model')+1]
    out.mkdir(parents=True,exist_ok=False)
    for name in ['inputs','prediction','keyframes']:(out/name).mkdir()
    for frame in frames:
        stem=Path(frame['file']).stem
        for folder,suffix in [('inputs','.h5'),('prediction','_depth.png'),('keyframes','.npz')]:
            (out/folder/(stem+suffix)).symlink_to(source/folder/(stem+suffix))
    sensor=Sensors();mount_cache={};extra=[];samples=[];last=None;offsets=[];start=time.monotonic()
    packet_indices={p['time_s']:i for i,p in enumerate(packets)}
    def packet_at(t,required=True):
        i=int(np.searchsorted(odom_times,t));i=min(range(max(0,i-1),min(len(packets),i+1)),key=lambda j:abs(odom_times[j]-t))
        if abs(odom_times[i]-t)>.02 or not packets[i]['valid']:
            if required:raise ValueError('Missing synchronized wheel/IMU pose')
            return None
        offsets.append(float(abs(odom_times[i]-t)))
        return packets[i]
    try:
        for capture in captures:
            t=float(capture['t'])
            if not args.room and not (1680<=t<=1790 or 1918<=t<=1990):continue
            if np.min(abs(anchor_times-t))<.3:continue
            packet=packet_at(t,required=not args.room)
            if packet is None:continue
            angle=packet['yaw']+packet['head_yaw'];xy=np.array([packet['x'],packet['y']])
            if args.room:
                i=packet_indices[packet['time_s']]
                if i==0 or not steady_sweep(packets[i-1],packet):continue
            if last is not None:
                delta=np.arctan2(np.sin(angle-last[0]),np.cos(angle-last[0]))
                if abs(delta)<np.deg2rad(8) and np.linalg.norm(xy-last[1])<.12:continue
            anchor=int(np.argmin(abs(anchor_times-t)))
            if abs(anchor_times[anchor]-t)>12:continue
            if anchor not in mount_cache:
                snapshot=captures[int(np.argmin(abs(times-anchor_times[anchor])))];_,_,_,_,mount=sensor.render(snapshot)
                mount_cache[anchor]=odom_body(packet_at(anchor_times[anchor]))@mount
            rgb,hist,rect,valid,mount=sensor.render(capture)
            pose=anchored_pose(np.asarray(frames[anchor]['camera_to_world']),mount_cache[anchor],odom_body(packet)@mount)
            stem=f'{len(frames)+len(extra):05d}';filename='keyframes/'+stem+'.npz'
            with h5py.File(out/'inputs'/(stem+'.h5'),'w') as h:
                for k,v in dict(rgb=rgb,depth=np.zeros((480,640),np.float32),hist_data=hist,fr=rect,mask=valid).items():
                    h.create_dataset(k,data=v,compression='gzip')
            samples.append(dict(filename=stem+'.h5'));extra.append(dict(file=filename,time_s=t,timestamp_ns=round(t*1e9),
                camera_to_world=pose.tolist(),pose_source='wheel/IMU delta from nearest estimated RGB-D anchor',anchor_file=frames[anchor]['file']))
            last=(angle,xy)
    finally:
        sensor.rgb.close();sensor.tof.close()
    if not extra:raise RuntimeError('No additional recorded views selected')
    write_json(out/'inputs/data.json',dict(train=[],test=samples));print('EXTRA',len(extra),flush=True)
    subprocess.run([sys.executable,str(ROOT/'technology/scan3d_fusion/inference/run_demo.py'),'--max-frames','0',
                    '--data-root',str(out/'inputs'),'--output',str(out/'prediction')],check=True)
    for frame in extra:
        stem=Path(frame['file']).stem;depth=np.array(Image.open(out/'prediction'/(stem+'_depth.png')),np.float32)/1000
        with h5py.File(out/'inputs'/(stem+'.h5'),'r') as h:hist,rect,valid=h['hist_data'][:],h['fr'][:],h['mask'][:]
        depth=tof_support(depth,hist,rect,valid,sequence['intrinsics'],sensor.cal)
        np.savez_compressed(out/frame['file'],depth=np.rint(depth*1000).astype(np.uint16),depth_source=FUSION_DEPTH_SOURCE,
                            tof_hist=hist,tof_rect=rect,tof_valid=valid)
    sequence['frames']=sorted(frames+extra,key=lambda f:f['time_s']);write_json(out/'live_sequence.json',sequence)
    surface,voxel=build_surface(out,sequence);surface.export(out/'final_mesh.stl');surface.export(out/'final_mesh.ply')
    write_json(out/'final_mesh.json',dict(pose_source=sequence.get('pose_source','estimated'),depth_source=FUSION_DEPTH_SOURCE,
        bounds_m=surface.bounds.tolist(),voxel_m=voxel,frames=len(sequence['frames'])))
    write_json(out/'source.json',dict(source=str(source),raw=str(raw),added_frames=len(extra),base_frames=len(frames),
        depth_source=FUSION_DEPTH_SOURCE,base_depth_and_poses_unchanged=True,ground_truth_map_pose_used=False,
        ground_truth_depth_used=False,floor_completion=False,max_sensor_timestamp_offset_s=max(offsets),
        scope='whole room steady head sweeps' if args.room else 'sofa/table close passes',
        pose_method='nearest estimated anchor + synchronized wheel/IMU delta and head encoder mount',wall_seconds=time.monotonic()-start,
        limitation='Extra poses are odometry-propagated, not newly tracked or globally optimized; recorded input views only.'))
    print('RESULT',out,len(extra),flush=True)


if __name__=='__main__':main()
