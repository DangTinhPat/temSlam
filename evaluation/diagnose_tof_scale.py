"""Held-out diagnostic of one global prediction scale; never changes production support."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import h5py
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/reconstruction'))
from depth_support import tof_support


def project(depth,K,cal):
    depth=np.asarray(depth,float);K=np.asarray(K,float)
    R=np.asarray(cal['rotation_matrix'],float);t=np.asarray(cal['translation_m'],float)
    if (depth.ndim!=2 or K.shape!=(4,) or not np.isfinite(K).all() or min(K[:2])<=0
            or R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all()
            or not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.999):
        raise ValueError('Require depth, calibrated pinhole intrinsics and rigid RGB-to-ToF transform')
    y,x=np.indices(depth.shape)
    rays=np.stack(((x-K[2])/K[0],-(y-K[3])/K[1],-np.ones_like(x)),axis=-1)
    points=(rays*depth[...,None])@R.T+t
    z=-points[...,2];safe=np.where(np.isfinite(z)&(z>.02),z,1.)
    half=np.tan(np.deg2rad(22.5))
    u=(points[...,0]/safe/half+1)*4;v=(1-points[...,1]/safe/half)*4
    inside=np.isfinite(depth)&(depth>.02)&np.isfinite(u)&np.isfinite(v)&(z>.02)&(u>=0)&(u<8)&(v>=0)&(v<8)
    index=np.floor(np.where(inside,v,0)).astype(int)*8+np.floor(np.where(inside,u,0)).astype(int)
    return z,index,inside


def residuals(depth,hist,valid,K,cal):
    z,index,inside=project(depth,K,cal)
    zones=[]
    for zone,(mean,sigma) in enumerate(hist):
        values=np.abs(z[inside&(index==zone)]-mean)
        if valid[zone] and len(values)>=32:
            row,col=divmod(zone,8)
            zones.append(dict(zone=zone,training=(row+col)%2==0,range_m=float(mean),sigma_m=float(sigma),
                              pixels=len(values),median_signed_m=float(np.median(z[inside&(index==zone)]-mean)),median_abs_m=float(np.median(values)),p95_abs_m=float(np.quantile(values,.95))))
    def summary(selected):
        return dict(zones=len(selected),median_abs_m=float(np.median([z['median_abs_m'] for z in selected])) if selected else None,
                    p95_abs_m=float(np.quantile([z['p95_abs_m'] for z in selected],.95)) if selected else None)
    result=dict(valid_pixels=int(np.count_nonzero(np.isfinite(depth)&(depth>.02))),
                valid_fraction=float(np.mean(np.isfinite(depth)&(depth>.02))),zones=zones)
    result['training']=summary([z for z in zones if z['training'] and z['sigma_m']<=.02])
    result['heldout']=summary([z for z in zones if not z['training'] and z['sigma_m']<=.02])
    result['by_range']={name:summary([z for z in zones if lo<=z['range_m']<hi])
                        for name,lo,hi in [('near',0,.5),('middle',.5,2.),('far',2.,float('inf'))]}
    result['heterogeneous']=summary([z for z in zones if z['sigma_m']>.02])
    return result


def diagnose_frame(raw,hist,rect,valid,K,cal):
    raw=np.asarray(raw,float);hist=np.asarray(hist,float);rect=np.asarray(rect,float);valid=np.asarray(valid,bool)
    if (hist.shape!=(64,2) or rect.shape!=(64,4) or valid.shape!=(64,)
            or not np.isfinite(hist[valid]).all() or np.any(hist[valid,1]<0) or not np.isfinite(rect).all()):
        raise ValueError('Require 64 valid ToF distribution/rectangle records')
    z,_,_=project(raw,K,cal)
    original_valid=np.isfinite(raw)&(raw>.02)
    b=-float(cal['translation_m'][2])
    a=z-b
    baseline=tof_support(raw,hist,rect,valid,K,cal)
    baseline[~original_valid]=0
    result=dict(fit_accepted=False,scale=None,training_zones=0,rejection_reasons=[],
                scale_bracket=[.1,10.],original_invalid_pixels=int((~original_valid).sum()),
                raw=residuals(raw,hist,valid,K,cal),baseline=residuals(baseline,hist,valid,K,cal))
    anchors=[];zone_diagnostics=[]
    for zone,((mean,sigma),box,ok) in enumerate(zip(hist,rect,valid)):
        y0,x0,y1,x1=np.rint(box).astype(int)
        y0,y1=np.clip([y0,y1],0,raw.shape[0]);x0,x1=np.clip([x0,x1],0,raw.shape[1])
        patch=a[y0:y1,x0:x1];keep=original_valid[y0:y1,x0:x1]&(patch>0)
        values=patch[keep]
        row,col=divmod(zone,8)
        eligible=False
        if ok and sigma<=.02 and len(values)>=32:
            median=float(np.median(values))
            # Reject competing predicted returns before fitting, in measured-range units.
            spread=float(np.quantile(values,.9)-np.quantile(values,.1))/median*mean
            eligible=spread<=max(.04,4*sigma)
            if eligible and (row+col)%2==0:anchors.append((zone,median,mean))
        raw_patch=z[y0:y1,x0:x1][keep]
        nearby=raw_patch[np.abs(raw_patch-mean)<max(.2,3*sigma)] if ok else np.empty(0)
        delta=float(mean-np.mean(nearby)) if len(nearby)>=32 else None
        zone_diagnostics.append(dict(zone=zone,eligible=bool(eligible),anchor_pixels=len(nearby),
                                     additive_clip=bool(delta is not None and abs(delta)>.15)))
    result['anchor_diagnostics']=zone_diagnostics
    result['additive_anchor_zones']=sum(z['anchor_pixels']>=32 for z in zone_diagnostics)
    result['additive_clipped_fraction']=sum(z['additive_clip'] for z in zone_diagnostics)/max(1,result['additive_anchor_zones'])
    result['raw_positive_depth_quantiles_m']=np.quantile(raw[original_valid],[.1,.5,.9]).tolist() if original_valid.any() else None
    result['training_zones']=len(anchors)
    if (len(anchors)<12 or len({z//8 for z,_,_ in anchors})<3 or len({z%8 for z,_,_ in anchors})<3):
        result['rejection_reasons'].append('Need >=12 single-surface training zones spanning both axes')
        return result,None
    coefficients=np.array([a for _,a,_ in anchors]);targets=np.array([m-b for _,_,m in anchors])
    # Median absolute residual is piecewise linear: its minima occur at zeros or line intersections.
    choices=list(targets/coefficients)
    for i in range(len(coefficients)):
        for j in range(i):
            if coefficients[i]!=coefficients[j]:choices.append((targets[i]-targets[j])/(coefficients[i]-coefficients[j]))
            choices.append((targets[i]+targets[j])/(coefficients[i]+coefficients[j]))
    choices=np.asarray(choices);choices=choices[np.isfinite(choices)&(choices>0)]
    if not len(choices):
        result['rejection_reasons'].append('No finite positive scale')
        return result,None
    loss=np.median(np.abs(choices[:,None]*coefficients-targets),axis=1)
    scale=float(choices[np.argmin(loss)]);result['scale']=scale
    # Fixed diagnostic bracket limits hypothesis search, not a hardware calibration claim.
    if not .1<=scale<=10.:
        result['rejection_reasons'].append('Scale outside fixed diagnostic bracket [0.1, 10]')
        return result,None
    result['training_fit_median_abs_m']=float(loss.min())
    result['additive_only_fit_m']=float(np.median(targets-coefficients))
    result['additive_only_training_median_abs_m']=float(np.median(abs(coefficients+result['additive_only_fit_m']-targets)))
    scaled=np.where(original_valid,raw*scale,0.)
    candidate=tof_support(scaled,hist,rect,valid,K,cal);candidate[~original_valid]=0
    # Held-out residuals are measured BEFORE support can hide a bad scale by dropping pixels.
    result['candidate']=residuals(scaled,hist,valid,K,cal)
    result['candidate_supported']=residuals(candidate,hist,valid,K,cal)
    held=[z for z in result['candidate']['zones'] if not z['training'] and z['sigma_m']<=.02]
    expected={z['zone'] for z in zone_diagnostics if z['eligible'] and sum(divmod(z['zone'],8))%2==1}
    observed={z['zone'] for z in held}
    if len(expected)<12 or not expected.issubset(observed):result['rejection_reasons'].append('Insufficient held-out zone coverage after reprojection')
    if any(z['p95_abs_m']>max(.02,min(.05,3*z['sigma_m'])) for z in held):
        result['rejection_reasons'].append('Held-out per-zone p95 exceeds unchanged measured tolerance')
    before={z['zone']:z for z in result['raw']['zones']}
    common=[z for z in held if z['zone'] in before]
    old=float(np.median([before[z['zone']]['median_abs_m'] for z in common])) if common else None
    new=float(np.median([z['median_abs_m'] for z in common])) if common else None
    result['paired_heldout_median_before_m']=old;result['paired_heldout_median_after_m']=new
    if old is None or new>old*.5+1e-9:result['rejection_reasons'].append('Held-out median did not improve by 50%')
    # Control support must not get worse; a recovered failed frame must gain real supported pixels.
    base_valid=result['baseline']['valid_pixels'];new_valid=result['candidate_supported']['valid_pixels']
    if new_valid<base_valid or (base_valid==0 and new_valid==0):result['rejection_reasons'].append('Supported coverage did not improve or was lost')
    base_zones={z['zone']:z for z in result['baseline']['zones']}
    if any(z['zone'] in base_zones and z['p95_abs_m']>base_zones[z['zone']]['p95_abs_m']+.01
           for z in result['candidate_supported']['zones']):
        result['rejection_reasons'].append('Supported control-zone p95 worsened by >1cm')
    result['fit_accepted']=not result['rejection_reasons']
    return result,candidate if result['fit_accepted'] else None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--indices',required=True,help='Comma-separated frozen frame indices, at most 12')
    args=parser.parse_args();source=args.prepared.resolve();output=args.output.resolve()
    try:indices=[int(v) for v in args.indices.split(',')]
    except ValueError:parser.error('indices must be comma-separated integers')
    sequence=json.loads((source/'localization_input.json').read_text())
    if not indices or len(indices)>12 or len(set(indices))!=len(indices) or min(indices)<0 or max(indices)>=len(sequence['frames']):
        parser.error('Require 1–12 unique existing frame indices')
    if output==source or source in output.parents:parser.error('Output must be outside immutable source')
    output.mkdir(parents=True,exist_ok=False)
    calpath=ROOT/'technology/scan3d_fusion/config/sim_calibration.json'
    cal=json.loads(calpath.read_text())['camera_to_tof']
    reports=[];candidates=[];hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (calpath,source/'localization_input.json')}
    for index in indices:
        frame=sequence['frames'][index];stem=Path(frame['file']).stem
        h5path=source/'inputs'/f'{stem}.h5';prediction=source/'prediction'/f'{stem}_depth.png'
        for path in (h5path,prediction):hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        with h5py.File(h5path) as sample:hist,rect,valid=sample['hist_data'][:],sample['fr'][:],sample['mask'][:]
        raw=np.asarray(Image.open(prediction),float)/1000
        result,depth=diagnose_frame(raw,hist,rect,valid,sequence['intrinsics'],cal)
        result.update(index=index,time_s=frame['time_s']);reports.append(result)
        if depth is not None:candidates.append((frame,depth,hist,rect,valid))
    passed=all(r['fit_accepted'] for r in reports)
    report=dict(source=str(source),indices=indices,frames=reports,all_selected_frames_pass=passed,
                production_change_approved=False,ablation_accepted=False,ground_truth_used=False,source_sha256=hashes,
                required_external_checks=['Held-out true surface-depth bias and foreground-edge retention',
                                          'Continuous tracking repeatability; sparse selected frames do not test this'],
                limits='Diagnostic scale hypothesis only. No held-out true geometry/edge-retention or tracking gate has passed; not an accepted map.')
    if passed:
        (output/'keyframes').mkdir()
        frames=[]
        for i,(frame,depth,hist,rect,valid) in enumerate(candidates):
            filename=f'keyframes/{i:05d}.npz'
            np.savez_compressed(output/filename,depth=np.rint(depth*1000).astype(np.uint16),depth_source='deltar_tof_fusion',
                                diagnostic_depth_transform='one global per-frame scale from checkerboard training zones',
                                tof_hist=hist,tof_rect=rect,tof_valid=valid)
            (output/'inputs').mkdir(exist_ok=True)
            (output/'inputs'/f'{i:05d}.h5').symlink_to(source/'inputs'/f'{Path(frame["file"]).stem}.h5')
            frames.append(dict(frame,file=filename))
        (output/'localization_input.json').write_text(json.dumps(dict(sequence,frames=frames,diagnostic_only=True)))
        report['candidate_scope']='Selected sparse frames only; not suitable for motion tracking without continuous preparation'
    (output/'quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(all_selected_frames_pass=passed,output=str(output),rejections={str(r['index']):r['rejection_reasons'] for r in reports}),indent=2))
    return 0 if passed else 2


if __name__=='__main__':sys.exit(main())
