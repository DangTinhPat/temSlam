"""Shared live-worker file and map helpers."""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
FUSION_DEPTH_SOURCE = 'deltar_tof_fusion'
MAP_REFRESH_KEYFRAMES = 25
sys.path.insert(0, str(ROOT / 'technology/scan3d_fusion/reconstruction'))
from ground_segmentation import read_labels

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


def startup_tracking_state(previous, still_since, current_time, current_pose, max_interval_s=.5):
    if previous is None:
        return (current_time,current_pose.copy()),current_time,False
    previous_time,previous_pose=previous
    dt=current_time-previous_time
    stable=False
    if np.isfinite(dt) and 0<dt<=max_interval_s:
        delta=np.linalg.inv(previous_pose)@current_pose
        translation_rate=np.linalg.norm(delta[:3,3])/dt
        angle=np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
        stable=translation_rate<.01 and angle/dt<.01
    if stable:
        if still_since is None:still_since=current_time
    else:still_since=None
    ready=still_since is not None and current_time-still_since>=.5
    return (current_time,current_pose.copy()),still_since,ready


def map_refresh_due(keyframes,last_refresh,graph_update):
    return keyframes==1 or graph_update or keyframes-last_refresh>=MAP_REFRESH_KEYFRAMES


def graph_update_kind(metrics):
    if not metrics:return None
    if metrics.pgo_status:return 'pgo'
    if metrics.lc_status:return 'loop_closure'
    return None


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
