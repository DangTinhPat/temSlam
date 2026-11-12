"""Re-render recorded simulator ToF measurements to audit a saved depth map.

Estimated mapping poses are copied unchanged; simulator poses are used only to
reproduce sensor inputs. No CAD depth is substituted into predicted RGB-D.
"""
import argparse,json,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'technology/scan3d_fusion/live'),str(ROOT/'technology/scan3d_fusion/reconstruction')]
from sensors import Sensors
from depth_support import tof_consistency_mask
from workers import build_surface,fusion_depth


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();source=args.source.resolve();out=args.output.resolve()
    meta=json.loads((source/'metadata.json').read_text());cmd=meta['runner_command']
    os.environ['RB1_SIM_MODEL']=cmd[cmd.index('--model')+1]
    captures=[json.loads(l) for l in (source/'live_capture.jsonl').open()]
    times=np.array([float(v['t']) for v in captures])
    path=source/'surface_sequence.json'
    sequence=json.loads((path if path.exists() else source/'live_sequence.json').read_text())
    for frame in sequence['frames']:
        with np.load(source/frame['file'],allow_pickle=False) as data:fusion_depth(data)
    out.mkdir(parents=True,exist_ok=False);(out/'keyframes').mkdir()
    sensor=Sensors();before=after=0
    try:
        for i,frame in enumerate(sequence['frames']):
            _,hist,rect,valid,_=sensor.render(captures[np.argmin(abs(times-frame['time_s']))])
            with np.load(source/frame['file'],allow_pickle=False) as data:batch={k:data[k] for k in data.files}
            depth=batch['depth'].astype(float)/1000
            keep=tof_consistency_mask(depth,hist,valid,sensor.K,sensor.cal,rect)
            before+=int((depth>0).sum());after+=int(keep.sum())
            batch['depth']=np.where(keep,batch['depth'],0).astype(np.uint16)
            np.savez_compressed(out/frame['file'],**batch)
            if i%100==0:print(i,flush=True)
    finally:
        sensor.rgb.close();sensor.tof.close()
    (out/'live_sequence.json').write_text(json.dumps(sequence))
    (out/'source.json').write_text(json.dumps(dict(source=str(source),operation='ToF consistency rejection only; estimated poses unchanged',before_pixels=before,after_pixels=after)))
    surface,voxel=build_surface(out,sequence)
    surface.export(out/'final_mesh.stl');surface.export(out/'final_mesh.ply')
    (out/'final_mesh.json').write_text(json.dumps(dict(pose_source=sequence.get('pose_source','cuVSLAM'),bounds_m=surface.bounds.tolist(),voxel_m=voxel)))


if __name__=='__main__':main()
