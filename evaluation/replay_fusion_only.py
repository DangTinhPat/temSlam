"""Replay RGB/ToF sensor inputs through DELTAR; keep estimated mapping poses unchanged.

Simulator snapshots reproduce sensor inputs only, never mapping poses or RGB-D depth.
"""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import h5py
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/live'),str(ROOT/'technology/scan3d_fusion/reconstruction')]
from sensors import Sensors
from depth_support import tof_support
from workers import build_surface,FUSION_DEPTH_SOURCE,write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();source=args.source.resolve();out=args.output.resolve()
    path=source/'surface_sequence.json'
    sequence=json.loads((path if path.exists() else source/'live_sequence.json').read_text())
    out.mkdir(parents=True,exist_ok=False)
    start=time.monotonic()
    cached=(source/'inputs/data.json').is_file() and (source/'prediction').is_dir()
    if cached:
        for name in ('inputs','prediction'):(out/name).symlink_to(source/name)
        inputs=out/'inputs';(out/'keyframes').mkdir()
        cal=json.loads((ROOT/'technology/scan3d_fusion/config/sim_calibration.json').read_text())['camera_to_tof']
        offsets=[json.loads((source/'source.json').read_text())['max_sensor_timestamp_offset_s']]
    else:
        cmd=json.loads((source/'metadata.json').read_text())['runner_command']
        os.environ['RB1_SIM_MODEL']=cmd[cmd.index('--model')+1]
        captures=[json.loads(line) for line in (source/'live_capture.jsonl').open()]
        times=np.array([float(v['t']) for v in captures])
        inputs=out/'inputs';inputs.mkdir();(out/'keyframes').mkdir()
        sensor=Sensors();cal=sensor.cal;samples=[];offsets=[]
        try:
            for i,frame in enumerate(sequence['frames']):
                index=int(np.argmin(abs(times-frame['time_s'])));offset=abs(times[index]-frame['time_s'])
                if offset>.02:raise ValueError('No synchronized sensor snapshot for keyframe')
                rgb,hist,rect,valid,_=sensor.render(captures[index])
                filename=f'{i:05d}.h5';samples.append(dict(filename=filename));offsets.append(float(offset))
                with h5py.File(inputs/filename,'w') as h:
                    for k,v in dict(rgb=rgb,depth=np.zeros((480,640),np.float32),hist_data=hist,fr=rect,mask=valid).items():
                        h.create_dataset(k,data=v,compression='gzip')
                if i%100==0:print('sensor',i,flush=True)
        finally:
            sensor.rgb.close();sensor.tof.close()
        write_json(inputs/'data.json',dict(train=[],test=samples))
        subprocess.run([sys.executable,str(ROOT/'technology/scan3d_fusion/inference/run_demo.py'),
            '--max-frames','0','--data-root',str(inputs),'--output',str(out/'prediction')],check=True)
    fx,fy,cx,cy=sequence['intrinsics'];y,x=np.mgrid[0:480:8,0:640:8];cloud=[];valid_pixels=0
    for i,frame in enumerate(sequence['frames']):
        stem=Path(frame['file']).stem
        depth=np.array(Image.open(out/'prediction'/(stem+'_depth.png')),np.float32)/1000
        with h5py.File(inputs/(stem+'.h5'),'r') as h:
            hist,rect,valid=h['hist_data'][:],h['fr'][:],h['mask'][:]
        depth=tof_support(depth,hist,rect,valid,sequence['intrinsics'],cal)
        valid_pixels+=int((depth>0).sum())
        np.savez_compressed(out/frame['file'],depth=np.rint(depth*1000).astype(np.uint16),
                            depth_source=FUSION_DEPTH_SOURCE,tof_hist=hist,tof_rect=rect,tof_valid=valid)
        z=depth[::8,::8];keep=np.isfinite(z)&(z>.02)&(z<=4)
        points=np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[keep]
        pose=np.asarray(frame['camera_to_world']);cloud.append(points@pose[:3,:3].T+pose[:3,3])
    cloud=np.concatenate(cloud)
    if not len(cloud):raise RuntimeError('Fusion returned no supported depth')
    sequence['depth_source']=FUSION_DEPTH_SOURCE
    if not cached:
        sequence['bounds_m']=[(cloud.min(0)-.1).tolist(),(cloud.max(0)+.1).tolist()]
    write_json(out/'live_sequence.json',sequence)
    surface,voxel=build_surface(out,sequence)
    surface.export(out/'final_mesh.stl');surface.export(out/'final_mesh.ply')
    write_json(out/'final_mesh.json',dict(pose_source=sequence.get('pose_source','estimated'),
        depth_source=FUSION_DEPTH_SOURCE,frames=len(sequence['frames']),bounds_m=surface.bounds.tolist(),voxel_m=voxel))
    write_json(out/'mesh.json',dict(file=str(out/'final_mesh.stl'),depth_source=FUSION_DEPTH_SOURCE))
    report=dict(source=str(source),keyframes=len(sequence['frames']),depth_source=FUSION_DEPTH_SOURCE,
        estimated_poses_unchanged=True,model_predictions_reused=cached,plane_depth_correction=False,floor_completion=False,
        ground_truth_rgbd_depth_used=False,valid_depth_pixels=valid_pixels,
        max_sensor_timestamp_offset_s=max(offsets),wall_seconds=time.monotonic()-start,
        limitation='Sensor replay with frozen estimated poses, not a new end-to-end SLAM run; holes remain unknown.')
    write_json(out/'source.json',report);print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
