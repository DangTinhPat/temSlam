"""Measure the first complete neck sweep; truth is a diagnostic oracle only.

Run with the DELTAR Python environment, PYTHONPATH unset and MUJOCO_GL=egl.
This does not change any saved depth, poses or reconstructed meshes.
"""
import argparse
import json
import os
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/live'))
from sensors import Sensors


def diagnose(session, output):
    sequence = json.loads((session/'live_sequence.json').read_text())
    captures = [json.loads(line) for line in (session/'live_capture.jsonl').open()]
    times = np.array([float(v['t']) for v in captures])
    start = next(i for i,v in enumerate(captures)
                 if float(v['t']) >= sequence['frames'][0]['time_s'] and float(v['hj']) < -2.8)
    positive = next(i for i in range(start, len(captures)) if float(captures[i]['hj']) > 2.8)
    end = next(i for i in range(positive, len(captures)) if abs(float(captures[i]['hj'])) < .01)
    # Include the turn towards the first extreme.
    begin = max(sequence['frames'][0]['time_s'], times[start]-4)
    command = json.loads((session/'metadata.json').read_text())['runner_command']
    os.environ['RB1_SIM_MODEL'] = command[command.index('--model')+1]
    sensors = Sensors()
    fx,fy,cx,cy = sequence['intrinsics']
    y,x = np.mgrid[:480,:640]
    rays = np.stack(((x-cx)/fx,(y-cy)/fy,np.ones_like(x)), -1)
    frames = []; samples = []
    def stats(values):
        return dict(median_cm=float(np.median(values)*100),
                    p95_abs_cm=float(np.quantile(np.abs(values),.95)*100))
    try:
        for frame in sequence['frames']:
            if not begin <= frame['time_s'] <= times[end]:
                continue
            view = captures[np.argmin(np.abs(times-frame['time_s']))]
            m,d = sensors.model,sensors.data
            d.qpos[:7] = [float(view[k]) for k in ('x','y','z','qw','qx','qy','qz')]
            for key,address in sensors.joints.items(): d.qpos[address] = float(view[key])
            mujoco.mj_forward(m,d)
            sensors.rgb.enable_depth_rendering()
            sensors.rgb.update_scene(d,camera='head_camera',scene_option=sensors.options)
            true_depth = sensors.rgb.render().copy()
            rotation = d.cam_xmat[sensors.camera].reshape(3,3)@np.diag([1,-1,-1])
            position = d.cam_xpos[sensors.camera].copy()
            truth = (rays*true_depth[...,None])@rotation.T+position
            with np.load(session/frame['file'], allow_pickle=False) as batch:
                depth = batch['depth'].astype(float)/1000
                supported = bool(np.isfinite(batch['floor_plane']).all())
            mask = (np.abs(truth[...,2]) < .003) & (depth > .02) & (depth < 4) & (true_depth < 4)
            if mask.sum() < 100: continue
            camera = rays*depth[...,None]
            pose = np.asarray(frame['camera_to_world'])
            actual = camera@rotation.T+position
            mapped = camera@pose[:3,:3].T+pose[:3,3]
            a,b = actual[...,2][mask],mapped[...,2][mask]
            frames.append(dict(time_s=frame['time_s'],head_deg=float(view['hj'])*180/np.pi,
                               floor_supported=supported,pixels=int(mask.sum()),
                               depth_true_pose=stats(a),depth_estimated_pose=stats(b)))
            samples.append(np.stack((a[::8],b[::8]),-1))
    finally:
        sensors.rgb.close(); sensors.tof.close()
    if len(frames) < 5: raise RuntimeError('Not enough frames in the first complete sweep')
    points = np.concatenate(samples)
    result = dict(session=str(session.resolve()),source='Saved predicted depth; truth used only to identify floor pixels and evaluate error',
                  window_s=[float(begin),float(times[end])],frames=frames,
                  aggregate=dict(depth_true_pose=stats(points[:,0]),depth_estimated_pose=stats(points[:,1])))
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['aggregate'],indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    diagnose(args.session,args.output)
