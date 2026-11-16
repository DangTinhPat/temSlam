#!/usr/bin/env python3
"""Prepare current-room simulated measurements and SEPARATE supervised RGB-Z labels.

Raw snapshots contain state/joints, not captured RGB/ToF arrays. Newly generated
inputs are replay-rendered simulated sensor measurements with the unchanged
closest-return reducer. Existing validation H5 inputs are copied byte-for-byte.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import h5py
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(value):
    path=Path(value)
    return path if path.is_absolute() else ROOT/path


def validate_manifest(manifest):
    samples=manifest['samples'];ids=[s['sample_id'] for s in samples]
    if not samples or len(ids)!=len(set(ids)):raise ValueError('Require unique sample IDs')
    keys=[(str(resolve(s['capture_file'])),s['capture_line_1based']) for s in samples]
    if len(keys)!=len(set(keys)):raise ValueError('Train/validation snapshot overlap')
    if set(s['split'] for s in samples)!={'train','validation'}:raise ValueError('Require train and validation only')
    if not 100<=sum(s['split']=='train' for s in samples)<=240 or sum(s['split']=='validation' for s in samples)!=48:
        raise ValueError('Require100..240 train and48 validation samples')


def verified_line(path,line,expected):
    data=Path(path).read_bytes().splitlines(keepends=True)[line-1]
    if hashlib.sha256(data).hexdigest()!=expected:raise ValueError('Frozen snapshot/odometry line hash mismatch')
    return json.loads(data)


def verify_frozen_sources(manifest):
    expected=manifest['provenance']['source_file_sha256']
    if not expected:raise ValueError('Frozen source provenance is empty')
    for path,value in expected.items():
        if digest(resolve(path))!=value:raise ValueError(f'Frozen source hash mismatch:{path}')


def prepare(manifest_path,output):
    manifest_path=Path(manifest_path).resolve();output=Path(output).resolve()
    manifest=json.loads(manifest_path.read_text());validate_manifest(manifest);verify_frozen_sources(manifest)
    if output.exists():raise FileExistsError(output)
    for sample in manifest['samples']:
        session=resolve(sample['raw_session'])
        if output==session or session in output.parents:raise ValueError('Output must be outside immutable raw sessions')
    sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/live'))
    from sensors import Sensors
    import mujoco
    hashes={str(p):digest(p) for p in (manifest_path,Path(__file__),ROOT/'technology/scan3d_fusion/live/sensors.py',
         ROOT/'technology/scan3d_fusion/capture/capture_mujoco.py',ROOT/'technology/scan3d_fusion/config/tof.yaml',
         ROOT/'technology/scan3d_fusion/config/sim_calibration.json')}
    prepared=copy.deepcopy(manifest);prepared['frozen_split_sha256']=digest(manifest_path)
    prepared['supervision_scope']='Current-room simulation only; optical-Z targets never enter model inputs'
    prepared['capture_line_hash_includes_newline']=True;prepared['odometry_line_hash_includes_newline']=True
    output.mkdir(parents=True);(output/'inputs').mkdir();(output/'labels').mkdir()
    renderers={};started=time.monotonic();timings=[]
    try:
        for count,sample in enumerate(prepared['samples'],1):
            tick=time.monotonic();sample_id=sample['sample_id']
            if Path(sample_id).name!=sample_id or sample_id in ('.','..'):raise ValueError('Unsafe sample ID')
            capturepath=resolve(sample['capture_file']);odompath=resolve(sample['odometry_file'])
            capture=verified_line(capturepath,sample['capture_line_1based'],sample['capture_line_sha256'])
            odometry=verified_line(odompath,sample['odometry_line_1based'],sample['odometry_line_sha256'])
            if not odometry.get('valid') or odometry.get('slip',False):raise ValueError('Invalid/slipping synchronized odometry')
            if abs(float(capture['t'])-float(sample['time_s']))>1e-8 or abs(float(odometry['time_s'])-float(capture['t']))>.020000001:
                raise ValueError('Snapshot/odometry synchronization mismatch')
            metadata_path=resolve(sample['raw_session'])/'metadata.json';metadata=json.loads(metadata_path.read_text())
            command=metadata['runner_command'];model=resolve(command[command.index('--model')+1])
            for path in (capturepath,odompath,metadata_path,model):
                if str(path) not in hashes:hashes[str(path)]=digest(path)
            if str(model) not in renderers:
                previous=os.environ.get('RB1_SIM_MODEL');os.environ['RB1_SIM_MODEL']=str(model)
                try:sensor=Sensors()
                finally:
                    if previous is None:os.environ.pop('RB1_SIM_MODEL',None)
                    else:os.environ['RB1_SIM_MODEL']=previous
                target_renderer=mujoco.Renderer(sensor.model,height=480,width=640)
                target_renderer.enable_depth_rendering();renderers[str(model)]=(sensor,target_renderer)
                prepared.setdefault('render_provenance',{})[str(model)]=dict(sensor_configuration=sensor.sensor,
                    camera_to_tof=sensor.cal,intrinsics=sensor.K,sensor_rng_seed=2026,
                    visibility='Same Sensors scene options and head_camera for RGB and separate target optical-Z renderer')
            sensor,target_renderer=renderers[str(model)]
            rgb,hist,rect,valid,_=sensor.render(capture)
            inputpath=output/'inputs'/f'{sample_id}.h5'
            if sample.get('input_h5'):
                original=resolve(sample['input_h5'])
                if digest(original)!=sample['input_h5_sha256']:raise ValueError('Existing measured H5 hash mismatch')
                hashes[str(original)]=digest(original)
                with h5py.File(str(original),'r') as h5:
                    if not np.array_equal(h5['rgb'][:],rgb):raise ValueError('Label camera/snapshot RGB differs from saved input')
                    if not np.all(h5['depth'][:]==0):raise ValueError('Measured H5 depth placeholder is not zero')
                shutil.copyfile(original,inputpath)
                if digest(inputpath)!=sample['input_h5_sha256']:raise ValueError('Input copy changed bytes')
                sample['input_provenance']='Byte-preserved previously replay-rendered simulated sensor H5'
            else:
                with h5py.File(str(inputpath),'w') as h5:
                    for key,value in dict(rgb=rgb,depth=np.zeros((480,640),np.float32),hist_data=hist,fr=rect,mask=valid).items():
                        h5.create_dataset(key,data=value,compression='gzip')
                sample['input_provenance']='Deterministic replay-rendered simulated RGB/closest-ToF from frozen native snapshot; not originally recorded RGB/ToF bytes'
            # Separate renderer, same exact RGB camera/visibility; target never
            # influences sensor.render, histogram selection, rectangles or masks.
            target_renderer.update_scene(sensor.data,camera='head_camera',scene_option=sensor.options)
            target=np.asarray(target_renderer.render(),np.float32).copy()
            if target.shape!=(480,640) or np.count_nonzero(np.isfinite(target)&(target>.001)&(target<10))<2:
                raise ValueError('Invalid RGB optical-Z supervision shape/units')
            targetpath=output/'labels'/f'{sample_id}.npy';np.save(str(targetpath),target,allow_pickle=False)
            sample.update(input_h5=str(inputpath),input_h5_sha256=digest(inputpath),target_depth_npy=str(targetpath),
                target_depth_sha256=digest(targetpath),label_status='Separate RGB optical-Z metres; supervision only',
                simulator_model=str(model),intrinsics=sensor.K,render_rng_seed=2026,
                supervision_in_model_input=False)
            timings.append(time.monotonic()-tick)
            if count==20 or count%20==0 or count==len(prepared['samples']):
                print(f'prepare {count}/{len(prepared["samples"])} wall_s={time.monotonic()-started:.2f}',flush=True)
    finally:
        for sensor,renderer in renderers.values():sensor.rgb.close();sensor.tof.close();renderer.close()
    verify_frozen_sources(manifest)
    if any(digest(path)!=value for path,value in hashes.items()):raise RuntimeError('Immutable preparation source changed')
    prepared.update(status='prepared',source_sha256=hashes,source_sha256_verified_after=True,
                    wall_seconds=time.monotonic()-started,first20_wall_seconds=float(sum(timings[:20])),
                    ground_truth_used_as_training_labels_only=True,ground_truth_depth_used_as_model_input=False,map_accepted=False)
    (output/'prepared_manifest.json').write_text(json.dumps(prepared,indent=2,allow_nan=False)+'\n')
    return prepared


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('manifest',type=Path);parser.add_argument('output',type=Path)
    cli=parser.parse_args();existed=cli.output.exists()
    try:prepare(cli.manifest,cli.output)
    except Exception as exc:
        if not existed and cli.output.is_dir():
            (cli.output/'failed_preparation.json').write_text(json.dumps(dict(complete=False,failure=str(exc),
                supervision_in_model_input=False,map_accepted=False),indent=2)+'\n')
        raise
    return 0

if __name__=='__main__':raise SystemExit(main())
