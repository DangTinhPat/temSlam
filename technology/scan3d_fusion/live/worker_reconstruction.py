"""Live preview and final TSDF mesh workers."""
import json
import sys
import time

import numpy as np

from worker_common import FUSION_DEPTH_SOURCE, ROOT, fusion_depth, status, write_json
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/reconstruction'))

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
