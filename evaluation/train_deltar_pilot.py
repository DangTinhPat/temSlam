#!/usr/bin/env python3
"""Bounded current-room SIMULATION training pilot; labels are never model inputs."""
from pathlib import Path
import hashlib
import sys
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'technology/scan3d_fusion/model/deltar'
CHECKPOINT=MODEL/'weights/nyu.pt'
TAIL=('decoder.up4.','decoder.conv0.','depth_head.','conv_out.')


def model_imports():
    sys.path.insert(0,str(MODEL))
    previous=sys.argv;sys.argv=['evaluate.py',str(ROOT/'technology/scan3d_fusion/config/deltar_eval.txt')]
    try:
        from src.config import args,parser
        configured=parser.parse_args(['@'+str(ROOT/'technology/scan3d_fusion/config/deltar_eval.txt')])
        for key,value in vars(configured).items():setattr(args,key,value)
        from src.models.deltar import Deltar
        from src.dataloader.zjuL5 import DataLoadPreprocess, preprocessing_transforms
        from src.loss import SILogLoss
    finally:sys.argv=previous
    return args,Deltar,DataLoadPreprocess,preprocessing_transforms,SILogLoss


def build_model(checkpoint=CHECKPOINT):
    args,Deltar,_,_,_=model_imports()
    import timm
    import src.models.decoder as decoder
    from deltar_spatial import CalibratedTransformerFusion
    original_create=timm.create_model;original_fusion=decoder.TransformerFusion
    def local_model(*a,**kw):
        kw['pretrained']=False
        return original_create(*a,**kw)
    timm.create_model=local_model;decoder.TransformerFusion=CalibratedTransformerFusion
    try:model=Deltar(n_bins=args.n_bins,min_val=args.min_depth,max_val=args.max_depth,norm='linear')
    finally:timm.create_model=original_create;decoder.TransformerFusion=original_fusion
    model.load_state_dict(torch.load(str(checkpoint),map_location='cpu'),strict=True)
    return model


def freeze_tail(model):
    model.eval()  # Batch1: freeze ALL running statistics, including the tail BN.
    for name,parameter in model.named_parameters():parameter.requires_grad_(name.startswith(TAIL))
    parameters=[p for p in model.parameters() if p.requires_grad]
    if sum(p.numel() for p in parameters)!=431040:raise ValueError('Unexpected trainable scope')
    return parameters


def state_hashes(model):
    return {name:hashlib.sha256(value.detach().cpu().contiguous().numpy().tobytes()).hexdigest()
            for name,value in model.state_dict().items()}


def supervised_loss(prediction,target):
    if not torch.isfinite(prediction).all() or (prediction<=0).any():raise ValueError('Invalid prediction')
    mask=torch.isfinite(target)&(target>.001)&(target<10)
    if mask.sum()<2:raise ValueError('Require at least two finite positive supervised pixels')
    *_,SILogLoss=model_imports()
    loss=SILogLoss()(prediction.float(),target.float(),mask=mask,interpolate=True)
    if not torch.isfinite(loss):raise ValueError('Nonfinite SILog loss')
    return loss


def load_measurements(path):
    """Only runtime RGB + recorded closest-return ToF; never a label argument."""
    import h5py
    model_imports()
    from src.config import args
    from src.dataloader.zjuL5 import ToTensor
    from src.utils.dataloader import sample_point_from_hist_parallel,patch_info_from_rect_data
    from torch.utils.data._utils.collate import default_collate
    with h5py.File(str(path),'r') as sample:
        rgb=sample['rgb'][:];placeholder=sample['depth'][:]
        hist=sample['hist_data'][:];rect=sample['fr'][:];mask=sample['mask'][:]
    if rgb.shape!=(480,640,3) or rgb.dtype!=np.uint8:raise ValueError('Require native480x640 uint8 RGB')
    if placeholder.shape!=(480,640) or not np.all(placeholder==0):raise ValueError('Input depth must be zero runtime placeholder')
    if hist.shape!=(64,2) or rect.shape!=(64,4) or mask.shape!=(64,):raise ValueError('Require measured8x8 ToF records')
    image=ToTensor('test')({'image':rgb.astype(np.float32)/255.,'focal':0.})['image']
    hist=torch.from_numpy(hist);rect=torch.from_numpy(rect).float();mask=torch.from_numpy(mask).bool()
    sampled=sample_point_from_hist_parallel(hist,mask,args).float()
    return default_collate([dict(rgb=image,additional=dict(hist_data=sampled,rect_data=rect,mask=mask,
                                                         patch_info=patch_info_from_rect_data(rect)))])


def load_target(path):
    """Separately rendered RGB optical-Z metres, used exclusively by the loss."""
    target=np.load(str(path),allow_pickle=False)
    if target.shape!=(480,640) or target.dtype!=np.float32:raise ValueError('Target must be480x640 float32 optical-Z metres')
    return torch.from_numpy(target.copy())[None,None]


def training_order(samples):
    training=[i for i,s in enumerate(samples) if s['split']=='train']
    if len(training)<100:raise ValueError('Require at least100 distinct training samples')
    return np.random.default_rng(2026).permutation(training)[:100].tolist()


def check_limits(elapsed,reserved_bytes):
    if elapsed>600:raise RuntimeError('600-second GPU trial limit exceeded')
    if reserved_bytes>9.5*2**30:raise RuntimeError('9.5GiB reserved GPU memory limit exceeded')


def to_device(value,device):
    if isinstance(value,dict):return {k:to_device(v,device) for k,v in value.items()}
    return value.to(device)


def validation(model,samples,folder,check):
    """Fixed48, no gradients or checkpoint selection; labels used for scoring only."""
    from PIL import Image
    folder.mkdir();absolute=squared=relative=0.;pixels=0;losses=[]
    with torch.no_grad():
        for count,sample in enumerate(samples,1):
            check()
            data=to_device(load_measurements(sample['input_h5']),'cuda')
            target=load_target(sample['target_depth_npy']).cuda()
            prediction=model(data)[1]
            loss=supervised_loss(prediction,target);losses.append(float(loss))
            full=torch.nn.functional.interpolate(prediction,size=(480,640),mode='bilinear',align_corners=True)
            mask=torch.isfinite(target)&(target>.001)&(target<10)
            error=(full-target)[mask];truth=target[mask]
            absolute+=float(error.abs().sum());squared+=float(error.square().sum());relative+=float((error.abs()/truth).sum());pixels+=int(mask.sum())
            depth=full[0,0].cpu().numpy().clip(.001,10)
            Image.fromarray((depth*1000).astype(np.uint16)).save(folder/f'{sample["sample_id"]}_depth.png')
            check()
            if count%12==0:print(f'{folder.name} {count}/48',flush=True)
    return dict(frames=len(samples),valid_target_pixels=pixels,mae_m=absolute/pixels,rmse_m=float(np.sqrt(squared/pixels)),
                abs_rel=relative/pixels,mean_silog=float(np.mean(losses)))


def run(manifest_path,output):
    import json
    import random
    import time
    from prepare_deltar_pilot import validate_manifest,verify_frozen_sources,digest
    manifest_path=Path(manifest_path).resolve();output=Path(output).resolve()
    manifest=json.loads(manifest_path.read_text());validate_manifest(manifest)
    if manifest.get('seed')!=2026:raise ValueError('Frozen seed mismatch')
    if manifest.get('status')!='prepared':raise ValueError('Require successfully prepared frozen manifest')
    verify_frozen_sources(manifest)
    fixed=dict(updates=100,batch_size=1,gradient_accumulation=1,trainable_parameter_count=431040,
        trainable_prefixes=list(TAIL),optimizer='AdamW',learning_rate=1e-5,weight_decay=.01,gradient_clip_norm=.1,
        all_batchnorm_running_statistics_frozen=True,precision='float32',max_reserved_gpu_gib=9.5,max_training_gpu_seconds=600)
    if any(manifest['training_spec'].get(k)!=v for k,v in fixed.items()):raise ValueError('Frozen training spec mismatch')
    if output.exists() or MODEL in output.parents or manifest_path.parent in output.parents:raise ValueError('Output must be new and separate from model/data')
    samples=manifest['samples'];order=training_order(samples);heldout=[s for s in samples if s['split']=='validation']
    hashes={str(manifest_path):digest(manifest_path),str(CHECKPOINT):digest(CHECKPOINT)}
    code=[Path(__file__),Path(__file__).with_name('deltar_spatial.py'),Path(__file__).with_name('prepare_deltar_pilot.py'),
          ROOT/'technology/scan3d_fusion/config/deltar_eval.txt',ROOT/'technology/scan3d_fusion/config/sim_calibration.json']
    code+=list((MODEL/'src').rglob('*.py'))
    for path in code:hashes[str(path)]=digest(path)
    for sample in samples:
        for key,hashkey in (('input_h5','input_h5_sha256'),('target_depth_npy','target_depth_sha256')):
            path=Path(sample[key])
            if digest(path)!=sample[hashkey]:raise ValueError('Prepared input/label hash mismatch')
            hashes[str(path)]=sample[hashkey]
        if Path(sample['input_h5']).resolve()==Path(sample['target_depth_npy']).resolve():raise ValueError('Input and supervision must be separate')
    if not torch.cuda.is_available():raise RuntimeError('Approved supervised pilot requires CUDA; no CPU training fallback')
    random.seed(2026);np.random.seed(2026);torch.manual_seed(2026);torch.cuda.manual_seed_all(2026)
    torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
    fraction=min(1.,9.5*2**30/torch.cuda.get_device_properties(0).total_memory)
    torch.cuda.set_per_process_memory_fraction(fraction,0);torch.cuda.reset_peak_memory_stats()
    model=build_model();parameters=freeze_tail(model);initial=state_hashes(model)
    named=dict(model.named_parameters());trainable_names=[n for n,p in named.items() if p.requires_grad]
    output.mkdir(parents=True)
    report=dict(complete=False,scope='Current-room simulation supervised feasibility only',seed=2026,spec=fixed,
        candidate_checkpoint_status='not_written',candidate_eligible_for_fresh_test=False,
        ground_truth_used_as_training_labels_only=True,ground_truth_depth_used_as_model_input=False,
        map_accepted=False,production_defaults_changed=False,source_sha256_verified_after=False,source_sha256=hashes,initial_checkpoint_sha256=digest(CHECKPOINT),
        trainable_parameter_names=trainable_names,trainable_parameters=sum(p.numel() for p in parameters),
        training_sample_ids=[samples[i]['sample_id'] for i in order],validation_sample_ids=[s['sample_id'] for s in heldout],steps=[])
    def persist():
        (output/'training_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    persist();gpu_started=time.monotonic();device_model=None
    def check():
        torch.cuda.synchronize();check_limits(time.monotonic()-gpu_started,torch.cuda.max_memory_reserved())
    try:
        device_model=model.cuda();parameters=freeze_tail(model)
        optimizer=torch.optim.AdamW(parameters,lr=1e-5,weight_decay=.01)
        if {id(p) for group in optimizer.param_groups for p in group['params']}!={id(p) for p in parameters}:raise RuntimeError('Optimizer scope mismatch')
        check()
        report['validation_original']=validation(model,heldout,output/'original_validation_prediction',check);persist()
        for step,index in enumerate(order,1):
            check();torch.cuda.synchronize();tick=time.monotonic()
            data=to_device(load_measurements(samples[index]['input_h5']),'cuda')
            target=load_target(samples[index]['target_depth_npy']).cuda()
            optimizer.zero_grad(set_to_none=True);prediction=model(data)[1];loss=supervised_loss(prediction,target)
            loss.backward()
            if any(p.grad is not None for p in model.parameters() if not p.requires_grad):raise RuntimeError('Frozen parameter received gradient')
            if any(p.grad is None or not torch.isfinite(p.grad).all() for p in parameters):raise RuntimeError('Missing/nonfinite trainable gradient')
            norm=torch.nn.utils.clip_grad_norm_(parameters,.1)
            if not torch.isfinite(norm) or norm<=0:raise RuntimeError('Nonfinite/zero tail gradient')
            optimizer.step();check();elapsed=time.monotonic()-tick
            record=dict(step=step,sample_id=samples[index]['sample_id'],loss=float(loss.detach()),gradient_norm_before_clip=float(norm),
                        seconds=elapsed,peak_reserved_gpu_bytes=torch.cuda.max_memory_reserved())
            report['steps'].append(record);persist()
            print(json.dumps(record),flush=True)
        final=state_hashes(model);changed=[key for key in initial if initial[key]!=final[key]]
        if not changed or any(key not in trainable_names for key in changed):raise RuntimeError('Frozen weight/BN buffer changed or no tail update')
        report.update(changed_parameter_names=changed,frozen_tensors_unchanged=True,
                      frozen_tensor_sha256={key:value for key,value in initial.items() if key not in trainable_names})
        # Plain state dict: same strict968-key interface as nyu.pt, no production overwrite.
        checkpoint=output/'step100_state.pt'
        torch.save({key:value.detach().cpu() for key,value in model.state_dict().items()},checkpoint)
        roundtrip=torch.load(str(checkpoint),map_location='cpu')
        if any(hashlib.sha256(value.contiguous().numpy().tobytes()).hexdigest()!=final[key] for key,value in roundtrip.items()):raise RuntimeError('Checkpoint roundtrip differs')
        torch.save(dict(optimizer=optimizer.state_dict(),torch_rng=torch.get_rng_state(),cuda_rng=torch.cuda.get_rng_state_all(),
                        numpy_rng=np.random.get_state(),python_rng=random.getstate(),updates=100),output/'step100_training_state.pt')
        report['candidate_checkpoint']=str(checkpoint);report['candidate_checkpoint_sha256']=digest(checkpoint)
        report['candidate_checkpoint_status']='diagnostic_pending_completion_and_paired_validation';persist()
        report['validation_step100']=validation(model,heldout,output/'step100_validation_prediction',check)
        if any(digest(path)!=value for path,value in hashes.items()):raise RuntimeError('Immutable source/code/label/checkpoint changed')
        verify_frozen_sources(manifest)
        seconds=[s['seconds'] for s in report['steps']]
        report.update(complete=True,updates_completed=100,source_sha256_verified_after=True,
            candidate_checkpoint_status='completed_step100_pending_external_paired_validation',candidate_eligible_for_fresh_test=False,
            peak_reserved_gpu_bytes=torch.cuda.max_memory_reserved(),gpu_trial_wall_seconds=time.monotonic()-gpu_started,
            step_seconds_median=float(np.median(seconds)),step_seconds_p95=float(np.quantile(seconds,.95)))
        persist()
    except Exception as exc:
        report.update(failure=str(exc),updates_completed=len(report['steps']),map_accepted=False,
                      candidate_checkpoint_status='diagnostic_failed_trial',candidate_eligible_for_fresh_test=False,
                      source_sha256_verified_after=all(digest(path)==value for path,value in hashes.items()),
                      peak_reserved_gpu_bytes=torch.cuda.max_memory_reserved(),gpu_trial_wall_seconds=time.monotonic()-gpu_started)
        # Failed/partial trials remain reviewable and never become step100 candidates.
        if device_model is not None:
            torch.save({key:value.detach().cpu() for key,value in model.state_dict().items()},output/'failed_trial_state.pt')
        persist();raise
    return report


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('prepared_manifest',type=Path);parser.add_argument('output',type=Path)
    cli=parser.parse_args();run(cli.prepared_manifest,cli.output)
    return 0

if __name__=='__main__':raise SystemExit(main())
