"""Diagnostic-only nearest-return consistency; no truth, fill, or support changes."""
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/capture'))
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/reconstruction'))
from capture_mujoco import closest_cluster, zone_statistics
from depth_support import tof_consistency_mask, sloping_zone_support, tof_support

SENSOR=dict(min_range_m=.02,max_range_m=4.,noise=dict(near_accuracy_m=.02,far_accuracy_fraction=.05,sigma_fraction_of_bound=0.))
HALF=np.tan(np.deg2rad(22.5))


def project_rays(depth,K,cal):
    """Nearest ToF optical-Z raster, plus winning original RGB pixel per cell."""
    depth=np.asarray(depth,float);K=np.asarray(K,float)
    R=np.asarray(cal['rotation_matrix'],float);t=np.asarray(cal['translation_m'],float)
    if (depth.ndim!=2 or K.shape!=(4,) or not np.isfinite(K).all() or min(K[:2])<=0 or
            R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all() or
            not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.999):
        raise ValueError('Require image, calibrated intrinsics and rigid camera-to-ToF transform')
    y,x=np.indices(depth.shape)
    rays=np.stack(((x-K[2])/K[0],-(y-K[3])/K[1],-np.ones_like(x)),axis=-1)
    points=(rays*depth[...,None])@R.T+t
    z=-points[...,2];derivative=-(rays@R.T)[...,2]
    safe=np.where(np.isfinite(z)&(z>=.02),z,1.)
    u=64*(points[...,0]/safe/HALF+1);v=64*(1-points[...,1]/safe/HALF)
    inside=(np.isfinite(depth)&(depth>.02)&np.isfinite(u)&np.isfinite(v)&
            (z>=.02)&(z<=4)&(u>=0)&(u<128)&(v>=0)&(v<128))
    source=np.flatnonzero(inside)
    cells=np.floor(v[inside]).astype(int)*128+np.floor(u[inside]).astype(int)
    # Sort by cell, nearest Z, then original pixel index for deterministic ties.
    order=np.lexsort((source,z.flat[source],cells));source=source[order];cells=cells[order]
    first=np.r_[True,cells[1:]!=cells[:-1]] if len(cells) else np.zeros(0,bool)
    raster=np.full(128*128,np.nan);owner=np.full(128*128,-1,int)
    raster[cells[first]]=z.flat[source[first]];owner[cells[first]]=source[first]
    cell_by_pixel=np.full(depth.size,-1,int);cell_by_pixel[np.flatnonzero(inside)]=np.floor(v[inside]).astype(int)*128+np.floor(u[inside]).astype(int)
    return dict(raster=raster.reshape(128,128),owner=owner.reshape(128,128),
                z=z,derivative=derivative,u=u/16,v=v/16,cell_by_pixel=cell_by_pixel.reshape(depth.shape))


def zone_view(array,zone):
    row,col=divmod(zone,8)
    return array[row*16:(row+1)*16,col*16:(col+1)*16]


def moments(projected,zone):
    rays=zone_view(projected['raster'],zone)
    selected=closest_cluster(rays,SENSOR)
    finite=np.isfinite(rays)
    coverage=dict(represented_rays=int(finite.sum()),quadrants=[int(finite[y:y+8,x:x+8].sum()) for y in (0,8) for x in (0,8)])
    if selected is not None:
        coverage.update(selected)
        coverage['sigma_m']=max(.005,selected['std_m'])
        coverage['selected_fraction']=selected['selected_count']/selected['valid_count']
        chosen=finite&(rays>=selected['low_m'])&(rays<=selected['high_m'])
        coverage['selected_source_rgb_pixels']=zone_view(projected['owner'],zone)[chosen].tolist()
    return coverage


def fit_original_surface(projected,zone,info):
    """Membership in one ORIGINAL predicted continuous inverse-depth surface.

    Uses all RGB samples, including initially nonwinning ones. The angular span,
    min24, p90<=.025m and pointwise<.03m are the existing slope-fit criteria.
    This is predicted geometry evidence, never evaluator/simulator truth.
    """
    row,col=divmod(zone,8);cells=projected['cell_by_pixel'];z=projected['z']
    selected=((cells>=0)&((cells//128)//16==row)&((cells%128)//16==col)&
              (z>=info['low_m'])&(z<=info['high_m']))
    X=np.stack((projected['u']-col-.5,projected['v']-row-.5,np.ones_like(z)),-1)
    data=X[selected];depth=z[selected]
    if len(depth)<24 or np.any(np.ptp(data[:,:2],axis=0)<.7):return None
    fit=np.linalg.lstsq(data,1/depth,rcond=None)[0];inverse=data@fit
    if np.any(inverse<=0) or np.quantile(np.abs(1/inverse-depth),.9)>.025:return None
    full_inverse=X@fit
    prediction=1/np.maximum(full_inverse,1e-6)
    member=((cells>=0)&(full_inverse>0)&np.isfinite(z)&(np.abs(prediction-z)<.03)&
            (z>=info['low_m'])&(z<=info['high_m']))
    return dict(member=member,fit=fit.tolist(),fit_points=len(depth),
                p90_residual_m=float(np.quantile(np.abs(1/inverse-depth),.9)))


def surface_visibility_reasons(before,after,membership):
    """REJECTED counterfactual:25mm layers are not identifiable under these bounds.

    Retained for the failure regression only; never authorizes candidate depth.
    """
    reasons=[];old=before['raster'];new=after['raster'];a=before['owner'];b=after['owner']
    covered=np.isfinite(old);newcovered=np.isfinite(new)
    if not np.array_equal(covered,newcovered):reasons.append('global angular coverage changed')
    member=np.asarray(membership,bool).ravel()
    oldselected=covered&member[np.maximum(a,0)];newselected=newcovered&member[np.maximum(b,0)]
    if not np.array_equal(oldselected,newselected):reasons.append('original selected-surface visibility changed')
    changed=(a!=b)|~np.isclose(old,new,rtol=0,atol=0,equal_nan=True)
    if np.any(changed&~oldselected):reasons.append('outside-selected source/depth changed')
    changedowners=(a!=b)&covered&newcovered
    if np.any(changedowners&~(member[np.maximum(a,0)]&member[np.maximum(b,0)])):
        reasons.append('changed owner outside original continuous surface')
    reasons.extend(affected_cluster_reasons(before,after))
    return sorted(set(reasons))


def affected_cluster_reasons(before,after):
    reasons=[]
    old=before['raster'];new=after['raster']
    changed=~np.isclose(old,new,rtol=0,atol=0,equal_nan=True)
    zones=np.unique((np.nonzero(changed)[0]//16)*8+np.nonzero(changed)[1]//16)
    for zone in zones:
        prior=zone_view(old,int(zone));proposed=zone_view(new,int(zone))
        p=closest_cluster(prior,SENSOR);q=closest_cluster(proposed,SENSOR)
        if (p is None)!=(q is None):reasons.append('affected-zone selected return changed');continue
        if p is None:continue
        psel=np.isfinite(prior)&(prior>=p['low_m'])&(prior<=p['high_m'])
        qsel=np.isfinite(proposed)&(proposed>=q['low_m'])&(proposed<=q['high_m'])
        if p['significant_clusters']!=q['significant_clusters'] or not np.array_equal(psel,qsel):
            reasons.append('affected-zone cluster association changed')
    return sorted(set(reasons))


def strict_visibility_reasons(before,after,selected_pixels=None):
    """Exact global source identity, selected-source locality and cluster membership."""
    reasons=[]
    if not np.array_equal(before['owner'],after['owner']):reasons.append('occlusion/source ownership changed')
    if selected_pixels is not None:
        owners=before['owner'];covered=owners>=0
        allowed=covered&selected_pixels.ravel()[np.maximum(owners,0)]
        changed=~np.isclose(before['raster'],after['raster'],rtol=0,atol=0,equal_nan=True)
        if np.any(changed&~allowed):reasons.append('outside-selected source/depth changed')
    reasons.extend(affected_cluster_reasons(before,after))
    return sorted(set(reasons))


def diagnose_frame(raw,hist,rect,valid,K,cal,*,bias_enabled=False,selected_slope=False,owner_policy='strict'):
    if owner_policy!='strict':raise ValueError('Surface-owner exception rejected:25mm layers not identifiable')
    raw=np.asarray(raw,float);hist=np.asarray(hist,float);rect=np.asarray(rect,float);valid=np.asarray(valid,bool)
    if hist.shape!=(64,2) or rect.shape!=(64,4) or valid.shape!=(64,) or not np.isfinite(hist[valid]).all() or np.any(hist[valid,1]<0):
        raise ValueError('Require 64 measured histogram, rectangle and validity records')
    if not np.isfinite(rect[valid]).all() or np.any(rect[valid,2:]<=rect[valid,:2]):
        raise ValueError('Valid rectangles must be finite and have positive area')
    initial=project_rays(raw,K,cal);candidate=raw.copy();reports=[]
    # Nonoverlapping source ownership follows z-buffer cells, not projected boxes.
    for zone,(observed,ok) in enumerate(zip(hist,valid)):
        info=moments(initial,zone);reasons=[];bias=0.;accepted=False;corrected=0
        if not ok:reasons.append('invalid measured zone')
        if info['represented_rays']!=256:reasons.append('incomplete angular coverage')
        if 'mean_m' not in info:reasons.append('no significant predicted return')
        if 'mean_m' in info:
            mu,sd=observed
            info['selected_mean_residual_m']=float(info['mean_m']-mu) if ok else None
            info['selected_sigma_residual_m']=float(info['sigma_m']-sd) if ok else None
            if info['significant_clusters']!=1:reasons.append('mixed or ambiguous predicted clusters')
            if ok and abs(info['sigma_m']-sd)>max(.025,.35*sd):reasons.append('inconsistent selected spread')
            if not reasons:
                residual=mu-info['mean_m']
                # <=1mm is the frozen PNG quantization deadband, not floor tuning.
                if bias_enabled and abs(residual)>.001:
                    bias=float(np.clip(residual,-.15,.15))
                    if abs(residual)>.15:reasons.append('bias exceeds fixed 0.15m bound')
        reports.append(dict(zone=zone,training_zone=(sum(divmod(zone,8))%2==0),observed_mean_m=float(observed[0]),
                            observed_sigma_m=float(observed[1]),baseline=info,proposed_bias_m=bias,
                            candidate_accepted=accepted,corrected_pixels=corrected,rejection_reasons=reasons))
        if reasons or bias==0:continue
        row,col=divmod(zone,8);cells=initial['cell_by_pixel'];z=initial['z']
        zone_pixels=(cells>=0)&((cells//128)//16==row)&((cells%128)//16==col)
        # Only points in the identified selected return, never the far tail.
        selected=zone_pixels&(z>=info['low_m'])&(z<=info['high_m'])&(initial['derivative']>.1)
        before=project_rays(candidate,K,cal)
        proposal=candidate.copy();proposal[selected]+=bias/initial['derivative'][selected]
        newproj=project_rays(proposal,K,cal);after=moments(newproj,zone)
        oldowners=zone_view(initial['owner'],zone);newowners=zone_view(newproj['owner'],zone)
        oldselected=(zone_view(initial['raster'],zone)>=info['low_m'])&(zone_view(initial['raster'],zone)<=info['high_m'])
        newselected=(zone_view(newproj['raster'],zone)>=after.get('low_m',np.inf))&(zone_view(newproj['raster'],zone)<=after.get('high_m',-np.inf))
        if after['represented_rays']!=256:reasons.append('proposal lost angular coverage')
        if after.get('significant_clusters')!=1 or not np.array_equal(oldselected,newselected):reasons.append('selected cluster association changed')
        reasons.extend(strict_visibility_reasons(before,newproj,selected))
        if 'mean_m' not in after or abs(after['mean_m']-observed[0])>=abs(info['mean_m']-observed[0]):reasons.append('selected residual did not improve')
        if 'sigma_m' in after and abs(after['sigma_m']-observed[1])>abs(info['sigma_m']-observed[1])+.001:reasons.append('selected spread residual worsened')
        reports[-1]['proposal']=after
        if not reasons:
            candidate=proposal;reports[-1]['candidate_accepted']=True;reports[-1]['corrected_pixels']=int(selected.sum())
    final=project_rays(candidate,K,cal)
    for report in reports:
        zone=report['zone'];after=moments(final,zone);report['final']=after
        if valid[zone] and 'mean_m' in after:after['selected_mean_residual_m']=float(after['mean_m']-hist[zone,0])
    # Current consistency/nearest-return veto is unchanged; no second mean bias.
    safe_rect=np.where(valid[:,None],rect,0.)
    support=diagnostic_consistency_mask(candidate,hist,valid,K,cal,safe_rect,selected_slope=selected_slope)
    original_valid=np.isfinite(raw)&(raw>.02)
    supported=np.where(support&original_valid,candidate,0).astype(np.float32)
    assert np.array_equal(np.isfinite(candidate)&(candidate>.02),original_valid)
    result=dict(zones=reports,corrected_pixels=int(np.count_nonzero(candidate[original_valid]!=raw[original_valid])),
                original_valid_pixels=int(original_valid.sum()),candidate_supported_pixels=int(np.count_nonzero(supported)),
                candidate_gate_rejected_pixels=int(np.count_nonzero(original_valid&~support)),
                calibration_scope='Independent per-zone in-sample calibration only; no held-out-zone validation claim',
                diagnostic_only=True,selected_bias_enabled=bias_enabled,selected_slope_enabled=selected_slope,owner_policy=owner_policy,surface_owner_exception_approved=False,ground_truth_used=False,map_accepted=False)
    return result,candidate


def selected_return_slope_support(z, u, v, index, inside, hist, valid):
    """Diagnostic: same fitted-plane gates, comparing the selected return moments."""
    coefficients=np.zeros((64,3));verified=np.zeros(64,bool)
    row,col=np.divmod(index,8)
    x=u-col-.5;y=v-row-.5
    design=np.stack((x[::4,::4],y[::4,::4],np.ones_like(x[::4,::4])),-1)
    sampled_z=z[::4,::4];sampled_index=index[::4,::4];sampled_inside=inside[::4,::4]
    axis=(np.arange(16)+.5)/16-.5
    gy,gx=np.meshgrid(axis,axis,indexing='ij')
    rays=np.stack((gx.ravel(),gy.ravel(),np.ones(256)),-1)
    for zone,(mean,sigma) in enumerate(hist):
        if not valid[zone] or not np.isfinite([mean,sigma]).all() or sigma<=.05:
            continue
        keep=sampled_inside&(sampled_index==zone)&np.isfinite(sampled_z)&(sampled_z>.02)&(sampled_z<=4)
        X=design[keep];depth=sampled_z[keep]
        if len(depth)<24 or np.any(np.ptp(X[:,:2],axis=0)<.7):
            continue
        # Inverse optical depth is affine in ray direction for any plane, without an IMU/floor prior.
        fit=np.linalg.lstsq(X,1/depth,rcond=None)[0]
        predicted_inverse=X@fit
        if np.any(predicted_inverse<=0):
            continue
        residual=np.abs(1/predicted_inverse-depth)
        if np.quantile(residual,.9)>.025:
            continue
        inverse=rays@fit
        if np.any(inverse<=0):
            continue
        distribution=1/inverse
        selected=closest_cluster(distribution,SENSOR)
        if (distribution.min()<.02 or distribution.max()>4 or distribution.std()<.03 or selected is None
                or abs(selected['mean_m']-mean)>.08
                or abs(max(.005,selected['std_m'])-sigma)>max(.025,.35*sigma)):
            continue
        coefficients[zone]=fit;verified[zone]=True
    inverse=coefficients[index,0]*x+coefficients[index,1]*y+coefficients[index,2]
    predicted=1/np.maximum(inverse,1e-6)
    # This only validates existing fusion pixels; it never supplies replacement depth or fills zeros.
    return (inside & verified[index] & (z<=4) & (np.abs(z-predicted)<.03)
            & (np.abs(z-hist[index,0])<=np.maximum(.08,3*hist[index,1])))


def diagnostic_consistency_mask(depth, hist, valid, K, camera_to_tof, rect=None, *, selected_slope=False):
    """Associate each predicted 3-D point with its actual ToF zone, not overlapping boxes."""
    depth=np.asarray(depth)
    hist=np.asarray(hist);valid=np.asarray(valid,bool)
    if depth.ndim!=2 or hist.shape!=(64,2) or valid.shape!=(64,):
        raise ValueError('Expected depth image and 64 ToF distributions')
    fx,fy,cx,cy=K
    if not np.isfinite(K).all() or min(fx,fy)<=0:
        raise ValueError('Invalid intrinsics')
    y,x=np.indices(depth.shape)
    gl=np.stack(((x-cx)*depth/fx,-(y-cy)*depth/fy,-depth),-1)
    points=gl@np.asarray(camera_to_tof['rotation_matrix']).T+np.asarray(camera_to_tof['translation_m'])
    z=-points[...,2]
    half=np.tan(np.deg2rad(45)/2)  # The calibrated RB ToF has a 45-degree square FoV.
    safe_z=np.where(np.isfinite(z)&(z>.02),z,1.)
    u=(points[...,0]/safe_z/half+1)*4
    v=(1-points[...,1]/safe_z/half)*4
    inside=np.isfinite(u)&np.isfinite(v)&(u>=0)&(u<8)&(v>=0)&(v<8)&(z>.02)
    col=np.floor(np.where(inside,u,0)).astype(int)
    row=np.floor(np.where(inside,v,0)).astype(int)
    index=row*8+col
    mean,sigma=hist[index,0],hist[index,1]
    # Broad mixed returns cannot justify arbitrarily far predictions behind the closest surface.
    tolerance=np.maximum(.08,np.minimum(3*sigma,.15))
    slope_rule=selected_return_slope_support if selected_slope else sloping_zone_support
    sloping=slope_rule(z,u,v,index,inside,hist,valid)
    support=(inside & valid[index] & np.isfinite(depth) & (depth>.02) & np.isfinite(mean)
             & np.isfinite(sigma) & (sigma>=0) & ((np.abs(z-mean)<=tolerance)|sloping))
    if rect is not None:
        # Near projected returns occlude far ones; overlapping rectangles must not be OR-ed.
        nearest=np.full(depth.shape,np.inf)
        h,w=depth.shape
        for (mu,sd),box,ok in zip(hist,rect,valid):
            if not ok or not np.isfinite([mu,sd]).all() or sd<0:
                continue
            y0,x0,y1,x1=np.rint(box).astype(int)
            y0,y1=np.clip([y0,y1],0,h);x0,x1=np.clip([x0,x1],0,w)
            nearest[y0:y1,x0:x1]=np.minimum(nearest[y0:y1,x0:x1],mu+max(.08,min(3*sd,.15)))
        support &= (z<=nearest)|sloping
    return support


def main():
    """Moment/coverage evidence ONLY. No real-data bias fit or candidate output."""
    import argparse
    import hashlib
    import json
    import h5py
    from PIL import Image
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path)
    parser.add_argument('predictions',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--indices',default='7,22,37,52,67,82,97,112,127,142')
    cli=parser.parse_args()
    indices=[int(v) for v in cli.indices.split(',')]
    if not indices or len(set(indices))!=len(indices) or min(indices)<0:parser.error('Require unique nonnegative frame indices')
    source=cli.prepared.resolve();prediction=cli.predictions.resolve();output=cli.output.resolve()
    if output==source or source in output.parents or output==prediction or prediction in output.parents:
        parser.error('Output must be separate from immutable inputs')
    manifest=source/'localization_input.json'
    sequence=json.loads(manifest.read_text())
    if max(indices)>=len(sequence['frames']):parser.error('Frame index outside source')
    calpath=ROOT/'technology/scan3d_fusion/config/sim_calibration.json'
    cal=json.loads(calpath.read_text())['camera_to_tof']
    def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    hashes={str(p):digest(p) for p in (manifest,calpath,Path(__file__),ROOT/'technology/scan3d_fusion/capture/capture_mujoco.py')}
    frames=[]
    for index in indices:
        frame=sequence['frames'][index];stem=Path(frame['file']).stem
        h5path=source/'inputs'/f'{stem}.h5';png=prediction/f'{stem}_depth.png'
        hashes.update({str(p):digest(p) for p in (h5path,png)})
        with h5py.File(str(h5path),'r') as sample:
            hist=sample['hist_data'][:];rect=sample['fr'][:];valid=sample['mask'][:]
        raw=np.asarray(Image.open(png),float)/1000
        result,unchanged=diagnose_frame(raw,hist,rect,valid,sequence['intrinsics'],cal)
        np.testing.assert_array_equal(raw,unchanged)
        role='development' if index in (7,37,67,97,127) else 'heldout' if index in (22,52,82,112,142) else 'control'
        result.update(index=index,role=role,raw_prediction_sha256=digest(png))
        frames.append(result)
    for path,value in hashes.items():
        if digest(Path(path))!=value:raise RuntimeError('Immutable source changed during diagnostic')
    def summarize(selected):
        zones=[z for f in selected for z in f['zones']]
        measured=[z for z in zones if z['baseline'].get('selected_mean_residual_m') is not None]
        residual=[abs(z['baseline']['selected_mean_residual_m']) for z in measured]
        return dict(frames=len(selected),total_zones=len(zones),moment_comparable_zones=len(measured),
                    fully_covered_valid_zones=sum(z['baseline']['represented_rays']==256 for z in measured),
                    selected_mean_abs_median_m=float(np.median(residual)) if residual else None,
                    selected_mean_abs_p95_m=float(np.quantile(residual,.95)) if residual else None)
    report=dict(mode='MOMENT_COVERAGE_ONLY_NO_BIAS',source=str(source),predictions=str(prediction),
                indices=indices,frames=frames,summary={role:summarize([f for f in frames if f['role']==role]) for role in ('development','heldout','control')},
                source_sha256=hashes,ground_truth_used=False,diagnostic_only=True,map_accepted=False,
                production_change_approved=False,real_data_bias_fitted=False,candidate_artifacts_written=False,
                coverage_rule='256 of256 nearest ToF raster cells; missing cells never filled',
                fitting_scope='None; checkerboard flags document zone IDs only, no shared-bias or heldout-zone claim',
                limits='Moment agreement cannot certify dense geometry. External frozen floor/obstacle scoring remains necessary.')
    output.mkdir(parents=True,exist_ok=False)
    (output/'quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['summary'],indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
