"""Read-only CPU floor scoring of two identically timed prepared label sequences.

Truth is read only by this evaluator. No renderer, tracker, classification,
reconstruction, simulator stepping, or PLY export is invoked.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/reconstruction'))
sys.path.insert(0, str(ROOT/'technology/scan3d_fusion/mapping/odometry'))
from ground_segmentation import read_labels
from localization import odom_body

FRAME_INDICES = (7,22,37,52,67,82,97,112,127,142)
STRIDE = 8


def validate_frame_indices(frame_indices, expected_frame_count):
    if (isinstance(expected_frame_count, bool) or not isinstance(expected_frame_count, int)
            or expected_frame_count <= 0):
        raise ValueError('Require positive expected frame count')
    indices = FRAME_INDICES if frame_indices is None else tuple(frame_indices)
    if (not indices or any(isinstance(i, bool) or not isinstance(i, (int, np.integer)) for i in indices)
            or len(set(indices)) != len(indices) or min(indices) < 0 or max(indices) >= expected_frame_count):
        raise ValueError('Require unique integer frame indices within expected source bounds')
    return indices


def fixed_obstacle_counts(reference, support, labels):
    return dict(baseline_valid_visible_obstacle_ge_025=int(reference.sum()),
        candidate_valid=int((reference & support).sum()),
        candidate_non_ground_blocked=int((reference & support & (labels != 1)).sum()),
        candidate_obstacle_only=int((reference & support & (labels == 2)).sum()),
        candidate_uncertain=int((reference & support & (labels == 0)).sum()),
        candidate_invalid=int((reference & ~support).sum()),
        candidate_ground_removed=int((reference & support & (labels == 1)).sum()))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frame_path(folder, frame):
    relative = Path(frame['file'])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Require relative prepared frame paths')
    return folder/relative


def count_labels(labels, depth, geom, optical_z, hit_height, floor):
    valid=(depth>20)&(depth<=4000)
    visible=(geom>=0)&(optical_z>.02)&(optical_z<=4.)
    eligible=valid&visible
    hitfloor=geom==floor
    obstacle=(geom!=floor)&(hit_height>=.025)
    g,o,u=labels==1,labels==2,labels==0
    return dict(sampled_pixels=int(len(labels)),valid_measured_pixels=int(valid.sum()),
        visible_truth_in_range=int(visible.sum()),valid_with_unscored_truth=int((valid&~visible).sum()),
        invalid_measured_pixels=int((~valid).sum()),visible_floor_all_pixels=int((visible&hitfloor).sum()),
        visible_floor_valid_measured=int((eligible&hitfloor).sum()),removed_ground_eligible=int((eligible&g).sum()),
        removed_ground_true_floor=int((eligible&g&hitfloor).sum()),true_floor_labeled_ground=int((eligible&hitfloor&g).sum()),
        true_floor_labeled_uncertain=int((eligible&hitfloor&u).sum()),true_floor_labeled_obstacle=int((eligible&hitfloor&o).sum()),
        visible_obstacle_ge_025_valid=int((eligible&obstacle).sum()),obstacle_ge_025_not_ground=int((eligible&obstacle&~g).sum()),
        obstacle_ge_025_labeled_obstacle=int((eligible&obstacle&o).sum()),obstacle_ge_025_labeled_uncertain=int((eligible&obstacle&u).sum()),
        obstacle_ge_025_labeled_ground=int((eligible&obstacle&g).sum()),valid_eligible_uncertain=int((eligible&u).sum()),
        valid_eligible=int(eligible.sum()))


def fractions(counts):
    ratio=lambda a,b:counts[a]/counts[b] if counts[b] else None
    return dict(removed_floor_precision=ratio('removed_ground_true_floor','removed_ground_eligible'),
        all_valid_floor_recall=ratio('true_floor_labeled_ground','visible_floor_valid_measured'),
        floor_observed_depth_completeness=ratio('visible_floor_valid_measured','visible_floor_all_pixels'),
        obstacle_ge_025_retained_blocked_fraction=ratio('obstacle_ge_025_not_ground','visible_obstacle_ge_025_valid'),
        obstacle_ge_025_retained_obstacle_only_fraction=ratio('obstacle_ge_025_labeled_obstacle','visible_obstacle_ge_025_valid'),
        floor_leak_fraction_among_valid_floor=ratio('true_floor_labeled_obstacle','visible_floor_valid_measured'),
        uncertainty_fraction=ratio('valid_eligible_uncertain','valid_eligible'))


def distribution(values):
    values=np.asarray(values)
    return (dict(n=len(values),median=float(np.median(values)),p05=float(np.quantile(values,.05)),
        p95=float(np.quantile(values,.95)),p95_absolute=float(np.quantile(abs(values),.95))) if len(values) else dict(n=0))


class TruthRays:
    def __init__(self, prepared, manifest):
        source=json.loads((prepared/'source.json').read_text())
        self.model_path=Path(source['simulator_model'])
        self.raw_path=Path(source['source'])/'live_capture.jsonl'
        self.snapshots={float(v['t']):v for v in map(json.loads,self.raw_path.open())}
        with tempfile.NamedTemporaryFile('w',suffix='.xml',dir=self.model_path.parent,delete=False) as stream:
            stream.write(re.sub(r'\s+inertia="[^"]+"','',self.model_path.read_text()))
            temporary=Path(stream.name)
        try:self.model=mujoco.MjModel.from_xml_path(str(temporary))
        finally:temporary.unlink()
        m=self.model;self.data=mujoco.MjData(m)
        self.body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'body_link')
        self.camera=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'head_camera')
        self.floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'ground')
        if min(self.body,self.camera,self.floor)<0:raise ValueError('Required scene body/camera/floor absent')
        for i in range(m.ngeom):
            parent=int(m.geom_bodyid[i])
            while parent>0 and parent!=self.body:parent=int(m.body_parentid[parent])
            if parent==self.body:m.geom_group[i]=5
        self.groups=np.array([1,1,1,1,0,0],np.uint8)
        self.joints={key:m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)]
            for key,name in [('lh','left_hip'),('lk','left_knee'),('lw','left_wheel'),('rh','right_hip'),
                             ('rk','right_knee'),('rw','right_wheel'),('hj','head_yaw')]}
        self.reference={f['time_s']:f for f in json.loads((prepared/'evaluator_truth.json').read_text())['frames']}
        self.K=manifest['intrinsics']

    def sample(self, frame, shape):
        m,d=self.model,self.data;v=self.snapshots[frame['time_s']]
        d.qpos[:7]=[float(v[k]) for k in ('x','y','z','qw','qx','qy','qz')]
        for key,address in self.joints.items():d.qpos[address]=float(v[key])
        mujoco.mj_forward(m,d)
        R=d.cam_xmat[self.camera].reshape(3,3)@np.diag([1,-1,-1]);origin=d.cam_xpos[self.camera].copy()
        pose=np.eye(4);pose[:3,:3]=R;pose[:3,3]=origin
        expected=np.asarray(self.reference[frame['time_s']]['simulator_from_body'])@np.asarray(frame['body_from_camera'])
        if not np.allclose(pose,expected,rtol=0,atol=1e-10):raise ValueError('Snapshot camera differs from isolated prepared truth')
        y,x=np.indices(shape);y=y[::STRIDE,::STRIDE].ravel();x=x[::STRIDE,::STRIDE].ravel()
        fx,fy,cx,cy=self.K;rays=np.column_stack(((x-cx)/fx,(y-cy)/fy,np.ones(len(x))))
        norms=np.linalg.norm(rays,axis=1);directions=rays@R.T/norms[:,None]
        geom=np.full(len(x),-1,np.int32);z=np.full(len(x),np.nan);height=np.full(len(x),np.nan);gid=np.array([-1],np.int32)
        for i,direction in enumerate(directions):
            distance=mujoco.mj_ray(m,d,origin,direction,self.groups,1,-1,gid);geom[i]=gid[0]
            if distance>=0:z[i]=distance/norms[i];height[i]=origin[2]+distance*direction[2]
        return geom,z,height,rays


def evaluate(folders, truth_source, frame_indices=None, expected_frame_count=150):
    frame_indices = validate_frame_indices(frame_indices, expected_frame_count)
    truth_source=Path(truth_source).resolve();folders=[Path(p).resolve() for p in folders]
    original=json.loads((truth_source/'localization_input.json').read_text())
    if len(original['frames'])!=expected_frame_count:raise ValueError('Source does not match expected frame count')
    manifests=[];watched={truth_source/'localization_input.json',truth_source/'source.json',truth_source/'evaluator_truth.json'}
    for folder in folders:
        manifest=json.loads((folder/'localization_input.json').read_text());manifests.append(manifest)
        if manifest['intrinsics']!=original['intrinsics'] or len(manifest['frames'])!=expected_frame_count:
            raise ValueError('Require unchanged K and complete expected source history')
        for old,new in zip(original['frames'],manifest['frames']):
            if any(new[key]!=old[key] for key in ('file','time_s','odometry','body_from_camera')):
                raise ValueError('Candidate measurement identities/times/odometry/mount differ')
        watched.add(folder/'localization_input.json')
        watched.update(folder/'prediction'/f'{i:05d}_depth.png' for i in frame_indices)
        watched.update(frame_path(folder,manifest['frames'][i]) for i in frame_indices)
    truth=TruthRays(truth_source,original);watched.update([truth.model_path,truth.raw_path])
    before={str(p):digest(p) for p in watched};results=[]
    cache={};sample_cache={}
    for folder,manifest in zip(folders,manifests):
        total={};records=[];depth_error=[];floor_height=[]
        raw_values={k:[] for k in ['all_valid_depth_error','true_floor_depth_error','true_floor_physical_height','on_own_supported_floor_depth_error','on_own_supported_floor_physical_height','supported_floor_depth_error','supported_minus_raw_floor_depth']}
        raw_counts=dict(sampled_pixels=0,valid_measured_pixels=0,visible_floor_all_pixels=0,visible_floor_valid_raw=0)
        for index in frame_indices:
            frame=manifest['frames'][index]
            with np.load(frame_path(folder,frame),allow_pickle=False) as a:
                depth=a['depth'];labels=read_labels(a,depth.shape)
                if labels is None or depth.dtype!=np.uint16 or depth.shape!=(480,640) or str(a['depth_source'])!='deltar_tof_fusion':
                    raise ValueError('Require original-size uint16 fusion depth and complete existing physical labels')
                confirmations=int(a['ground_confirmations'])
                with Image.open(folder/'prediction'/f'{index:05d}_depth.png') as image:raw_depth=np.asarray(image,dtype=np.float64)[::STRIDE,::STRIDE].ravel()/1000
                sampled_depth=depth[::STRIDE,::STRIDE].ravel();sampled_labels=labels[::STRIDE,::STRIDE].ravel()
            if index not in cache:cache[index]=truth.sample(original['frames'][index],depth.shape)
            geom,z,height,rays=cache[index]
            counts=count_labels(sampled_labels,sampled_depth,geom,z,height,truth.floor)
            for key,value in counts.items():total[key]=total.get(key,0)+value
            eligible=(sampled_depth>20)&(sampled_depth<=4000)&(geom>=0)&(z>.02)&(z<=4.)
            depth_error.extend((sampled_depth[eligible]/1000-z[eligible]).tolist())
            physical=odom_body(frame['odometry'])@np.asarray(frame['body_from_camera']);up=physical[:3,:3].T@np.array([0,0,1.])
            h=sampled_depth/1000*(rays@up)+physical[2,3];floor_height.extend(h[eligible&(geom==truth.floor)].tolist())
            visible=(geom>=0)&(z>.02)&(z<=4.);truefloor=visible&(geom==truth.floor)
            raw_valid=np.isfinite(raw_depth)&(raw_depth>.02)&(raw_depth<=4.)
            raw_h=raw_depth*(rays@up)+physical[2,3];own_floor=eligible&truefloor;raw_floor=raw_valid&truefloor
            for key,value in [('sampled_pixels',len(raw_depth)),('valid_measured_pixels',raw_valid.sum()),('visible_floor_all_pixels',truefloor.sum()),('visible_floor_valid_raw',raw_floor.sum())]:raw_counts[key]+=int(value)
            for key,v,select in [('all_valid_depth_error',raw_depth-z,raw_valid&visible),('true_floor_depth_error',raw_depth-z,raw_floor),('true_floor_physical_height',raw_h,raw_floor),('on_own_supported_floor_depth_error',raw_depth-z,own_floor),('on_own_supported_floor_physical_height',raw_h,own_floor),('supported_floor_depth_error',sampled_depth/1000-z,own_floor),('supported_minus_raw_floor_depth',sampled_depth/1000-raw_depth,own_floor)]:raw_values[key].extend(v[select].tolist())
            sample_cache[(str(folder),index)]=dict(support=eligible,truefloor=truefloor,
                true_obstacle_ge_025=visible&(geom!=truth.floor)&(height>=.025),
                z=z,raw=raw_depth,raw_height=raw_h,supported=sampled_depth/1000,supported_height=h,labels=sampled_labels)
            records.append(dict(index=index,time_s=frame['time_s'],ground_confirmations=confirmations,counts=counts,metrics=fractions(counts)))
        metric=fractions(total)
        results.append(dict(source=str(folder),counts=total,metrics=metric,per_frame=records,
            measured_minus_truth_optical_z_m=distribution(depth_error),true_floor_measured_physical_height_m=distribution(floor_height),
            partial_gate_checks=dict(removed_floor_precision_0995=metric['removed_floor_precision'] is not None and metric['removed_floor_precision']>=.995,
                all_valid_floor_recall_090=metric['all_valid_floor_recall'] is not None and metric['all_valid_floor_recall']>=.9,
                observed_surface_obstacle_retention_099=metric['obstacle_ge_025_retained_blocked_fraction'] is not None and metric['obstacle_ge_025_retained_blocked_fraction']>=.99),
            raw_prediction=dict(counts=raw_counts,distributions_m={key:distribution(value) for key,value in raw_values.items()}),
            full_gate_c_accepted=False,map_accepted=False))
    for result,folder in zip(results,folders):
        values={key:[] for key in ['raw_depth_error','raw_physical_height','supported_depth_error','supported_physical_height']}
        fixed=dict(baseline_valid_visible_floor=0,candidate_valid_depth_on_baseline_floor=0,candidate_ground_on_baseline_floor=0,candidate_obstacle_on_baseline_floor=0,candidate_uncertain_including_invalid_on_baseline_floor=0)
        obstacles={}
        for index in frame_indices:
            a=sample_cache[(str(folder),index)];common=a['truefloor'].copy()
            baseline=sample_cache[(str(folders[0]),index)];reference_floor=baseline['support']&baseline['truefloor']
            fixed['baseline_valid_visible_floor']+=int(reference_floor.sum())
            fixed['candidate_valid_depth_on_baseline_floor']+=int((reference_floor&a['support']).sum())
            fixed['candidate_ground_on_baseline_floor']+=int((reference_floor&a['support']&(a['labels']==1)).sum())
            fixed['candidate_obstacle_on_baseline_floor']+=int((reference_floor&a['support']&(a['labels']==2)).sum())
            fixed['candidate_uncertain_including_invalid_on_baseline_floor']+=int((reference_floor&(~a['support']|(a['labels']==0))).sum())
            reference_obstacle=baseline['support']&baseline['true_obstacle_ge_025']
            for key,value in fixed_obstacle_counts(reference_obstacle,a['support'],a['labels']).items():
                obstacles[key]=obstacles.get(key,0)+value
            for other in folders:common &= sample_cache[(str(other),index)]['support']
            for key,data in [('raw_depth_error',a['raw']-a['z']),('raw_physical_height',a['raw_height']),('supported_depth_error',a['supported']-a['z']),('supported_physical_height',a['supported_height'])]:values[key].extend(data[common].tolist())
        result['common_supported_true_floor_distributions_m']={key:distribution(value) for key,value in values.items()}
        denominator=fixed['baseline_valid_visible_floor']
        fixed['fixed_baseline_floor_recall']=fixed['candidate_ground_on_baseline_floor']/denominator if denominator else None
        fixed['candidate_depth_retention_on_baseline_floor']=fixed['candidate_valid_depth_on_baseline_floor']/denominator if denominator else None
        result['fixed_baseline_valid_floor']=fixed
        obstacle_denominator=obstacles['baseline_valid_visible_obstacle_ge_025']
        obstacles['valid_blocked_fraction_fixed_baseline']=obstacles['candidate_non_ground_blocked']/obstacle_denominator if obstacle_denominator else None
        obstacles['obstacle_only_fraction_fixed_baseline']=obstacles['candidate_obstacle_only']/obstacle_denominator if obstacle_denominator else None
        obstacles['invalid_is_unknown_not_free']=True
        result['fixed_baseline_obstacle_ge_025']=obstacles
        result['partial_gate_checks']['own_valid_floor_recall_090']=result['partial_gate_checks']['all_valid_floor_recall_090']
        result['partial_gate_checks']['all_valid_floor_recall_090']=fixed['fixed_baseline_floor_recall'] is not None and fixed['fixed_baseline_floor_recall']>=.9
    if before!={str(p):digest(p) for p in watched}:raise RuntimeError('Read-only source changed while scoring')
    return dict(status='Evaluator-only comparison; no accepted map',truth_source=str(truth_source),evaluator_only=True,
        ground_truth_used_for_estimation=False,labels_depth_poses_modified=False,frame_indices=list(frame_indices),expected_frame_count=expected_frame_count,pixel_stride=STRIDE,
        fixed_floor_recall_denominator='All valid visible physical-floor pixels in the FIRST prepared folder; candidate invalid/uncertain pixels remain failures',
        pixel_grid_origin_yx=[0,0],truth_range_m=[.02,4.],measured_valid_mm=[20,4000],depth_agreement_exclusion=False,
        visibility='CPU mj_ray first visible scene hit, sensor groups[1,1,1,1,0,0], robot excluded; no renderer/stepping',
        obstacle_ge_025_definition='Visible nonfloor surface hit at least .025m above physical floor; not object-instance recall',
        results=results,read_only_file_sha256=before,scorer_sha256=digest(__file__),
        limitations=[f'Frozen{len(frame_indices)} frames stride8, not full observed-area or object-instance coverage.',
            'All valid visible floor remains in recall denominator, including uncertain and obstacle labels.',
            'Separate completeness includes visible floor with invalid measured depth; unscored true range/nohit counts reported.',
            'No PLY, map trajectory, seam or navigation validation; this scorer does not approve GateC.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',nargs=2,type=Path)
    parser.add_argument('--truth-source',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    if args.output.exists():parser.error('output must be a new evidence file')
    result=evaluate(args.prepared,args.truth_source)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(source=r['source'],metrics=r['metrics'],counts=r['counts']) for r in result['results']],indent=2))


if __name__=='__main__':main()
