"""Live preview and final TSDF mesh workers."""
import json
import sys
import time

import numpy as np

from worker_common import FUSION_DEPTH_SOURCE, ROOT, fusion_depth, status, write_json
sys.path.insert(0, str(ROOT / 'simulation/demo/reconstruction'))

def new_volume(bounds):
    from reconstruct import TSDF
    voxel=max(.04,float(np.prod(bounds[1]-bounds[0])/4_000_000)**(1/3))
    return TSDF(bounds,voxel=voxel,truncation=6*voxel)


def integrate(volume, session, frames, intrinsics):
    for f in frames:
        with np.load(session/f['file'],allow_pickle=False) as sample: depth=fusion_depth(sample)
        volume.integrate(depth,f['camera_to_world'],intrinsics)


def build_surface(session, sequence):
    volume = new_volume(np.asarray(sequence['bounds_m']))
    integrate(volume, session, sequence['frames'], sequence['intrinsics'])
    return volume.mesh(min_views=3), volume.voxel


class LiveVolume:
    def __init__(self):
        self.volume = None
        self.frames = []
        self.intrinsics = None

    def update(self, session, sequence):
        bounds = np.asarray(sequence['bounds_m'])
        frames, intrinsics = sequence['frames'], sequence['intrinsics']
        start = len(self.frames)
        if (self.volume is None or intrinsics != self.intrinsics or frames[:start] != self.frames
                or np.any(bounds[0] < self.volume.bounds[0]) or np.any(bounds[1] > self.volume.bounds[1])):
            # Leave space for the next few views so small changes do not resize the live grid.
            padded = np.array([np.floor((bounds[0]-.25)*2)/2, np.ceil((bounds[1]+.25)*2)/2])
            self.volume = None
            self.frames = []
            self.volume = new_volume(padded)
            start = 0
        try:
            integrate(self.volume, session, frames[start:], intrinsics)
        except Exception:
            self.volume = None
            self.frames = []
            raise
        self.frames, self.intrinsics = frames, intrinsics
        return len(frames)-start


def finalize(session):
    import torch
    torch.set_num_threads(2)
    sequence=json.loads((session/'live_sequence.json').read_text())
    if sequence.get('pose_stage') == 'preview':
        raise ValueError('Finalize requires the final cuVSLAM pose snapshot')
    write_json(session/'surface_sequence.json',sequence)
    surface,voxel=build_surface(session,sequence)
    target=session/'final_mesh.stl'
    surface.export(target)
    surface.export(session/'final_mesh.ply')
    pose_metadata = {k: sequence[k] for k in ('pose_source', 'body_pose_constraint') if k in sequence}
    write_json(target.with_suffix('.json'),dict(**pose_metadata,
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
    last=0;last_revision=None
    live = LiveVolume()
    sequence_file = session/'live_sequence.json'
    while not (session/'STOP').exists():
        time.sleep(.1)
        if time.monotonic()-last<1: continue
        try:
            revision = sequence_file.stat().st_mtime_ns
            if revision == last_revision:
                continue
            sequence=json.loads(sequence_file.read_text())
        except FileNotFoundError: continue
        last=time.monotonic()
        last_revision=revision
        if len(sequence['frames'])<3:
            status(session,'mesh','Chờ ít nhất 3 keyframe');continue
        status(session,'mesh','Đang dựng mesh')
        try:
            views_integrated = live.update(session, sequence)
            surface=live.volume.mesh(min_views=3)
        except RuntimeError:
            status(session,'mesh','Chưa đủ góc nhìn cho mesh');continue
        target=session/f'mesh-{time.time_ns()}.stl'
        surface.export(target)
        write_json(session/'mesh.json',dict(file=str(target),frames=len(sequence['frames']),depth_source=FUSION_DEPTH_SOURCE))
        build_ms = 1000*(time.monotonic()-last)
        with (session/'mesh_diagnostics.jsonl').open('a') as log:
            log.write(json.dumps(dict(wall_time=time.time(), sensor_time=sequence['frames'][-1]['time_s'],
                keyframes=len(sequence['frames']), integrated=views_integrated,
                faces=len(surface.faces), build_ms=build_ms))+'\n')
        status(session,'mesh','Mesh đã cập nhật',faces=len(surface.faces),
               keyframes=len(sequence['frames']), integrated=views_integrated,
               build_ms=build_ms)
        # RViz receives a unique URI for each revision to invalidate its mesh cache.
        old=sorted(session.glob('mesh-*.stl'))[:-3]
        for path in old: path.unlink(missing_ok=True)
