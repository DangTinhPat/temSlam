"""Offline diagnostic SE(2) graph from wheel/IMU and independently checked depth revisits.

No live-worker imports or saved SLAM poses. A failed loop gate emits diagnostics only.
shortcut: fixed gates are not calibrated hardware covariances; tune from synchronized real RGB-D and wheel/IMU logs before hardware use.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion'))
from mapping.odometry.localization import odom_body
from reconstruction.surface_matching import vertical_points


def se2(v):
    x,y,a=np.asarray(v,float);c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1.]])


def vector(t):
    return np.array([t[0,3],t[1,3],np.arctan2(t[1,0],t[0,0])])


def relative(a,b):return np.linalg.inv(a)@b


def preserve_body(raw,planar):
    # Left multiplication changes only world XY/yaw, preserving physical tilt/height.
    return planar@relative(se2(vector(raw)),raw)


def error(a,b):return vector(relative(a,b))


def require_finished(source):
    path=source/'slam_shutdown.json'
    if not (source/'STOP').exists() or not path.exists() or not json.loads(path.read_text()).get('clean_stop'):
        raise ValueError('Require finished session with STOP and clean SLAM shutdown')
    result_path=source/'headless_result.json';route_path=source/'scan_route.json'
    if not result_path.exists() or not route_path.exists():
        raise ValueError('Require completed full-house route')
    result=json.loads(result_path.read_text());route=json.loads(route_path.read_text())
    total=len(route.get('waypoints',[]))
    if (not total or result.get('success') is not True or result.get('waypoints')!=total
            or result.get('final_index') not in (total-1,total)):
        raise ValueError('Require completed full-house route')


def require_loops(loops):
    if len(loops)<2 or not all(e.get('cycle_verified') for e in loops):
        raise ValueError('Require at least two independently corroborated, observable revisit edges')


def correspondences(points,normals,target,tn,tree,transform):
    q=points@transform[:3,:3].T+transform[:3,3];qn=normals@transform[:3,:3].T
    distance,index=tree.query(q);n=tn[index];r=np.sum(n*(q-target[index]),axis=1)
    valid=(distance<.2)&(np.abs(r)<.1)&(np.abs(np.sum(n*qn,axis=1))>.85)
    q,n,r=q[valid],n[valid],r[valid]
    j=np.column_stack((n[:,:2],-n[:,0]*q[:,1]+n[:,1]*q[:,0]))
    return j,r,n,int(valid.sum()),float(valid.mean())


def icp(points,normals,target,tn,initial):
    report=dict(accepted=False,reason='insufficient_points',matches=0)
    if min(len(points),len(target))<100:return initial,report
    tree=cKDTree(target);transform=initial.copy()
    for _ in range(25):
        j,r,n,count,overlap=correspondences(points,normals,target,tn,tree,transform)
        report.update(matches=count,overlap=overlap)
        if count<100:report['reason']='insufficient_matches';return initial,report
        eigen=np.linalg.eigvalsh(j.T@j/count)
        normal_min=float(np.linalg.eigvalsh(n[:,:2].T@n[:,:2]/count)[0])
        report.update(normal_min_eigenvalue=normal_min,jacobian_eigenvalues=eigen.tolist())
        if normal_min<.025 or eigen[0]<1e-3 or eigen[-1]/eigen[0]>1e4:
            report['reason']='unobservable';return initial,report
        w=np.minimum(1,.025/np.maximum(np.abs(r),1e-9))
        step=np.linalg.lstsq(j*np.sqrt(w[:,None]),-r*np.sqrt(w),rcond=None)[0]
        transform=se2(step)@transform
        change=error(initial,transform)
        if np.linalg.norm(change[:2])>.5 or abs(change[2])>.35:
            report['reason']='outside_local_capture_range';return initial,report
        if np.linalg.norm(step)<1e-5:break
    else:
        report['reason']='icp_not_converged';return initial,report
    report.update(accepted=True,reason='observable_fit',delta=error(initial,transform).tolist())
    return transform,report


def heldout(fixed,moving,transform):
    if min(len(fixed[0]),len(moving[0]))<100:return dict(accepted=False,reason='insufficient_heldout_points')
    j,r,n,count,overlap=correspondences(*moving,*fixed,cKDTree(fixed[0]),transform)
    if count<100:return dict(accepted=False,reason='insufficient_heldout_matches',matches=count,overlap=overlap)
    eigen=np.linalg.eigvalsh(j.T@j/count)
    normal_min=float(np.linalg.eigvalsh(n[:,:2].T@n[:,:2]/count)[0])
    if normal_min<.025 or eigen[0]<1e-3 or eigen[-1]/eigen[0]>1e4:
        return dict(accepted=False,reason='unobservable_heldout',matches=count,overlap=overlap,
                    normal_min_eigenvalue=normal_min,jacobian_eigenvalues=eigen.tolist())
    # Count out-of-gate points in overlap; tails are conditional on correspondence gate.
    median=float(np.median(np.abs(r))) if count else None
    p90=float(np.percentile(np.abs(r),90)) if count else None
    return dict(accepted=bool(count>=100 and overlap>=.25 and p90<=.06 and median<=.025),
                matches=count,overlap=overlap,median_abs_m=median,p90_abs_m=p90,
                normal_min_eigenvalue=normal_min,jacobian_eigenvalues=eigen.tolist())


def verify_revisit(fixed,moving,fixed_hold,moving_hold,initial):
    forward,fr=icp(*moving,*fixed,initial)
    reverse,rr=icp(*fixed,*moving,np.linalg.inv(initial))
    cycle=vector(forward@reverse)
    fh=heldout(fixed_hold,moving_hold,forward);rh=heldout(moving_hold,fixed_hold,reverse)
    accepted=bool(fr['accepted'] and rr['accepted'] and fh['accepted'] and rh['accepted']
                  and np.linalg.norm(cycle[:2])<=.03 and abs(cycle[2])<=.035)
    return forward,dict(accepted=accepted,forward=fr,reverse=rr,heldout_forward=fh,
                       heldout_reverse=rh,two_way_error=cycle.tolist())


def corroborate(loops,poses,times):
    result=[]
    for e in loops:
        partners=[]
        for f in loops:
            if e is f or set(e['support'])&set(f['support']):continue
            if min(abs(times[e['i']]-times[f['i']]),abs(times[e['j']]-times[f['j']]))<5:continue
            predicted=relative(poses[e['i']],poses[f['i']])@f['z']@relative(poses[f['j']],poses[e['j']])
            residual=error(e['z'],predicted)
            if np.linalg.norm(residual[:2])<=.05 and abs(residual[2])<=.03:partners.append([f['i'],f['j']])
        if partners:result.append(dict(e,cycle_verified=True,cycle_partners=partners))
    return result


def solve_graph(poses,edges):
    start=np.array([vector(p) for p in poses]);anchor=poses[0].copy()
    def solve(active,initial):
        sparse=lil_matrix((3*len(active),3*(len(poses)-1)),dtype=int)
        for k,e in enumerate(active):
            for index in (e['i'],e['j']):
                if index:sparse[3*k:3*k+3,3*(index-1):3*index]=1
        def residual(x):
            state=np.concatenate([start[:1],x.reshape(-1,3)])
            output=[]
            for e in active:
                r=error(e['z'],relative(se2(state[e['i']]),se2(state[e['j']])))
                sigma=np.array([.03,.03,.02]) if e['kind']=='loop' else np.array([.02,.02,.01])
                output.extend(r/sigma)
            return output
        loop_rows=np.repeat([e['kind']=='loop' for e in active],3)
        def loss(z):
            # Odometry stays quadratic even when a false loop tries to tear the chain.
            rho=np.array([z.copy(),np.ones_like(z),np.zeros_like(z)])
            q=1+z[loop_rows]
            rho[:,loop_rows]=np.array([2*(np.sqrt(q)-1),1/np.sqrt(q),-.5/q**1.5])
            return rho
        fit=least_squares(residual,initial[1:].ravel(),jac_sparsity=sparse.tocsr(),tr_options={'atol':1e-10,'btol':1e-10},loss=loss,f_scale=1.,max_nfev=500,ftol=1e-6,xtol=1e-6,gtol=1e-6)
        if not fit.success:raise ValueError('Sparse pose graph did not converge: '+fit.message)
        return np.concatenate([start[:1],fit.x.reshape(-1,3)]),float(fit.cost)
    state,cost=solve(edges,start)
    rejected=[];active=[]
    for e in edges:
        r=error(e['z'],relative(se2(state[e['i']]),se2(state[e['j']])))
        if e['kind']=='loop' and (np.linalg.norm(r[:2])>.09 or abs(r[2])>.06):rejected.append(e)
        else:active.append(e)
    if rejected:state,cost=solve(active,start)
    result=[se2(v) for v in state];result[0]=anchor
    retained=[e for e in active if e['kind']=='loop']
    adjacent=[];loop_errors=[]
    # Re-evaluate the final state, including every loop surviving the re-solve.
    for e in active:
        r=error(e['z'],relative(result[e['i']],result[e['j']]))
        translation,angle=float(np.linalg.norm(r[:2])),float(abs(r[2]))
        if e['kind']=='odom':
            adjacent.append((translation,angle))
            if translation>.06 or angle>.03:
                raise ValueError('Final adjacent odometry strain exceeds 0.06 m / 0.03 rad')
        else:
            loop_errors.append(dict(i=e['i'],j=e['j'],translation_m=translation,angle_rad=angle))
            if translation>.09 or angle>.06:
                raise ValueError('Final retained loop residual exceeds 0.09 m / 0.06 rad after re-solve')
    return result,dict(cost=cost,odometry_loss='quadratic; robust loss applies only to loops',
                       final_adjacent_translation_max_m=max((r[0] for r in adjacent),default=0.),
                       final_adjacent_angle_max_rad=max((r[1] for r in adjacent),default=0.),
                       final_loop_residuals=loop_errors,rejected_loops=len(rejected),retained_loops=len(retained),
                       retained_pairs=[[e['i'],e['j']] for e in retained],
                       rejected_pairs=[[e['i'],e['j']] for e in rejected],
                       uncertainty='Fixed diagnostic edge scales; not calibrated pose uncertainty')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def link_keyframe(source,destination):
    Path(destination).symlink_to(Path(source).resolve())


def nearest(records,times,t):
    i=int(np.searchsorted(times,t));indices=[k for k in (i-1,i) if 0<=k<len(records)]
    k=min(indices,key=lambda k:abs(times[k]-t))
    if abs(times[k]-t)>.02+1e-9:raise ValueError('No synchronized source sample within 20 ms')
    return records[k]


class JointMount:
    """Forward kinematics only: global simulator pose is never read or assigned."""
    def __init__(self,model_path):
        import mujoco
        self.mj=mujoco
        with tempfile.NamedTemporaryFile('w',suffix='.xml',dir=model_path.parent,delete=False) as f:
            f.write(re.sub(r'\s+inertia="[^"]+"','',model_path.read_text()));name=Path(f.name)
        try:self.model=mujoco.MjModel.from_xml_path(str(name))
        finally:name.unlink()
        m=self.model;self.data=mujoco.MjData(m)
        self.body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'body_link')
        self.camera=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'head_camera')
        joint_ids={k:mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for k,n in
            [('lh','left_hip'),('lk','left_knee'),('lw','left_wheel'),('rh','right_hip'),('rk','right_knee'),('rw','right_wheel'),('hj','head_yaw')]}
        if min(joint_ids.values())<0:raise ValueError('Missing measured joint in physical model')
        self.joints={k:m.jnt_qposadr[index] for k,index in joint_ids.items()}
        if min(self.body,self.camera)<0:raise ValueError('Missing physical body/camera model')

    def __call__(self,sample):
        m,d=self.model,self.data
        self.mj.mj_resetData(m,d)
        # Canonical free joint; only measured joint angles influence the relative mount.
        d.qpos[:7]=[0,0,0,1,0,0,0]
        for k,index in self.joints.items():d.qpos[index]=float(sample[k])
        if not np.isfinite(d.qpos).all():raise ValueError('Invalid measured joints')
        self.mj.mj_forward(m,d);r=d.xmat[self.body].reshape(3,3)
        mount=np.eye(4)
        mount[:3,:3]=r.T@d.cam_xmat[self.camera].reshape(3,3)@np.diag([1,-1,-1])
        mount[:3,3]=r.T@(d.cam_xpos[self.camera]-d.xpos[self.body])
        return mount


def read_source(source):
    require_finished(source)
    names=['live_sequence.json','metadata.json','live_capture.jsonl','odometry.jsonl','slam_shutdown.json',
           'headless_result.json','scan_route.json','STOP']
    hashes={str(source/n):sha(source/n) for n in names}
    sequence=json.loads((source/'live_sequence.json').read_text())
    for name in ('source.json','tracking.json'):
        if (source/name).exists():
            hashes[str(source/name)]=sha(source/name)
            record=json.loads((source/name).read_text())
            if any(record.get(k) is True for k in ('ground_truth_depth_used','ground_truth_pose_used_by_tracker','not_for_mapping')):
                raise ValueError('Refuse oracle/tracking-only source')
    if any(sequence.get(k) is True for k in ('ground_truth_depth_used','ground_truth_pose_used_by_tracker','not_for_mapping')):
        raise ValueError('Refuse oracle/tracking-only source')
    frames=sequence['frames'];times=np.array([float(f['time_s']) for f in frames])
    if len(frames)<10 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):raise ValueError('Require increasing keyframe timeline')
    k=np.asarray(sequence['intrinsics'],float)
    if k.shape!=(4,) or not np.isfinite(k).all() or min(k[:2])<=0:raise ValueError('Invalid intrinsics')
    command=json.loads((source/'metadata.json').read_text())['runner_command']
    model_path=Path(command[command.index('--model')+1]).resolve()
    # Include files can change kinematics without changing the top-level XML.
    import xml.etree.ElementTree as ET
    def model_hashes(path):
        hashes[str(path)]=sha(path)
        for node in ET.parse(path).iter('include'):
            child=(path.parent/node.attrib['file']).resolve()
            if str(child) not in hashes:model_hashes(child)
    model_hashes(model_path);mount_for=JointMount(model_path)
    # Discard world pose fields immediately; they cannot enter any mapping calculation.
    joints=['t','lh','lk','lw','rh','rk','rw','hj']
    captures=[{k:r[k] for k in joints} for r in map(json.loads,(source/'live_capture.jsonl').read_text().splitlines())]
    packets=list(map(json.loads,(source/'odometry.jsonl').read_text().splitlines()))
    capture_times=np.array([float(v['t']) for v in captures]);packet_times=np.array([float(v['time_s']) for v in packets])
    if np.any(np.diff(capture_times)<=0) or np.any(np.diff(packet_times)<=0):raise ValueError('Nonincreasing raw source timeline')
    bodies=[];mounts=[];matched=[]
    for f,t in zip(frames,times):
        path=Path(f['file'])
        if path.is_absolute() or '..' in path.parts:raise ValueError('Unsafe keyframe path')
        path=source/path;hashes[str(path)]=sha(path)
        packet=nearest(packets,packet_times,t);capture=nearest(captures,capture_times,t)
        if not packet['valid'] or packet['slip']:raise ValueError('Invalid/slipping synchronized odometry: cannot constrain graph')
        with np.load(path,allow_pickle=False) as sample:
            depth=sample['depth']
            if (str(sample['depth_source'])!='deltar_tof_fusion' or depth.dtype!=np.uint16
                    or depth.ndim!=2 or not np.any(depth>0)):raise ValueError('Require nonempty immutable measured-fusion uint16 depth')
        bodies.append(odom_body(packet));mounts.append(mount_for(capture));matched.append(packet)
    return sequence,times,bodies,mounts,matched,hashes


def submap(source,frames,intrinsics,bodies,mounts,poses,index,offsets):
    points=[];normals=[];anchor=np.linalg.inv(poses[index])
    for offset in offsets:
        j=index+offset
        with np.load(source/frames[j]['file'],allow_pickle=False) as sample:
            p,n=vertical_points(sample['depth'],intrinsics,anchor@bodies[j]@mounts[j])
        points.append(p);normals.append(n)
    p,n=np.concatenate(points),np.concatenate(normals)
    if len(p):
        _,ids=np.unique(np.floor(p/.05).astype(np.int64),axis=0,return_index=True)
        ids=ids[np.linspace(0,len(ids)-1,min(6000,len(ids))).astype(int)];p,n=p[ids],n[ids]
    return p,n


def find_loops(source,sequence,times,bodies,mounts,poses,max_candidates):
    # shortcut: geometry-only loops can alias repeated rooms; verify appearance before deploying on real robots.
    from functools import lru_cache
    @lru_cache(maxsize=64)
    def local(i):
        return (submap(source,sequence['frames'],sequence['intrinsics'],bodies,mounts,poses,i,[-2,0,2]),
                submap(source,sequence['frames'],sequence['intrinsics'],bodies,mounts,poses,i,[-1,1]))
    candidates=[];anchors=list(range(2,len(poses)-2,5))
    xy=np.array([p[:2,3] for p in poses])
    for a,i in enumerate(anchors):
        for j in anchors[a+1:]:
            distance=float(np.linalg.norm(xy[i]-xy[j]))
            if times[j]-times[i]>=30 and distance<=1.:candidates.append((distance,i,j))
    candidates.sort();records=[];loops=[]
    for number,(_,i,j) in enumerate(candidates[:max_candidates]):
        fi,hi=local(i);fj,hj=local(j)
        z,record=verify_revisit(fi,fj,hi,hj,relative(poses[i],poses[j]))
        records.append(dict(i=i,j=j,**record))
        if record['accepted']:
            loops.append(dict(i=i,j=j,z=z,kind='loop',support=list(range(i-2,i+3))+list(range(j-2,j+3))))
        if number%10==0:print(json.dumps(dict(candidate=number+1,total=min(len(candidates),max_candidates),geometrically_verified=len(loops))),flush=True)
    verified=corroborate(loops,poses,times)
    return verified,dict(candidate_count=len(candidates),tested_candidates=len(records),candidate_limit=max_candidates,
                         geometric_matches=len(loops),corroborated_loops=len(verified),candidates=records)


def unchanged(hashes):
    if any(sha(p)!=h for p,h in hashes.items()):raise ValueError('Source changed during offline processing; discard output')


def write_cloud(source,sequence,path):
    fx,fy,cx,cy=sequence['intrinsics'];blocks=[]
    for f in sequence['frames']:
        with np.load(source/f['file'],allow_pickle=False) as sample:z=sample['depth'][::8,::8].astype(float)/1000
        y,x=np.mgrid[:z.shape[0],:z.shape[1]]*8
        p=np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[(z>.02)&(z<=4)]
        t=np.asarray(f['camera_to_world']);blocks.append(p@t[:3,:3].T+t[:3,3])
    p=np.concatenate(blocks).astype('<f4')
    if not len(p) or not np.isfinite(p).all():raise ValueError('No finite measured points to export')
    _,ids=np.unique(np.floor(p/.025).astype(np.int64),axis=0,return_index=True);p=p[ids]
    header=f'ply\nformat binary_little_endian 1.0\nelement vertex {len(p)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
    with path.open('xb') as stream:stream.write(header.encode('ascii'));stream.write(p.tobytes())
    return [(p.min(0)-.1).tolist(),(p.max(0)+.1).tolist()],len(p)


def run(source,output,max_candidates=500):
    source,output=Path(source).resolve(),Path(output).resolve()
    if output.exists() or output==source or source in output.parents:raise ValueError('Require new output outside raw session')
    if max_candidates<1:raise ValueError('Positive candidate limit required')
    sequence,times,bodies,mounts,packets,hashes=read_source(source)
    for path in (Path(__file__).resolve(),ROOT/'technology/scan3d_fusion/mapping/odometry/localization.py',
                 ROOT/'technology/scan3d_fusion/reconstruction/surface_matching.py'):
        hashes[str(path)]=sha(path)
    poses=[se2(vector(b)) for b in bodies]
    loops,search=find_loops(source,sequence,times,bodies,mounts,poses,max_candidates)
    report=dict(status='rejected_no_export',map_quality_status='diagnostic/unvalidated',source=str(source),
        ground_truth_depth_used=False,ground_truth_pose_used_by_tracker=False,
        pose_source='anchored wheel/IMU body SE2 + independently corroborated local depth revisit edges',
        camera_mount_source='synchronized measured joints; canonical-body forward kinematics, no simulator global pose',
        source_sha256=hashes,loop_search=search,
        observation_coverage=dict(keyframes=len(times),duration_s=float(times[-1]-times[0]),
            keyframe_interval_median_s=float(np.median(np.diff(times))),
            keyframe_interval_p95_s=float(np.percentile(np.diff(times),95)),
            keyframe_interval_max_s=float(np.max(np.diff(times)))),
        limitations=['Geometric room aliasing remains possible; no semantic identity verification.',
                     'Only saved keyframe depth is available; missing observations are not reconstructed.',
                     'Outside revisit-constrained spans, odometry distortion remains possible.',
                     'Sparse-cloud export is sampled at 8-pixel stride, not a complete surface or navigation map.',
                     'Capture metadata does not pin model XML bytes; hashes bind this run to the current kinematic model.'])
    unchanged(hashes);output.mkdir(parents=True,exist_ok=False)
    try:
        require_loops(loops)
        edges=[dict(i=i,j=i+1,z=relative(poses[i],poses[i+1]),kind='odom') for i in range(len(poses)-1)]+loops
        optimized,solved=solve_graph(poses,edges);report['solve']=solved
        pairs=set(map(tuple,solved['retained_pairs']))
        retained=corroborate([e for e in loops if (e['i'],e['j']) in pairs],poses,times);require_loops(retained)
        if len(retained)!=len(pairs):raise ValueError('Robust rejection broke independent cycle support')
        report['revisit_constrained_time_envelope_s']=[float(times[min(e['i'] for e in retained)]),
                                                       float(times[max(e['j'] for e in retained)])]
        report['unconstrained_tail_frames']=int(min(e['i'] for e in retained)+len(poses)-1-max(e['j'] for e in retained))
        corrected=[preserve_body(b,a) for b,a in zip(bodies,optimized)]
        result=dict(intrinsics=sequence['intrinsics'],frames=[],pose_source=report['pose_source'],
                    map_quality_status=report['map_quality_status'],ground_truth_depth_used=False,
                    ground_truth_pose_used_by_tracker=False,depth_source='deltar_tof_fusion')
        (output/'keyframes').mkdir()
        for i,(f,body,mount,packet) in enumerate(zip(sequence['frames'],corrected,mounts,packets)):
            filename=f'keyframes/{i:05d}.npz';link_keyframe(source/f['file'],output/filename)
            if sha(output/filename)!=hashes[str(source/f['file'])]:raise ValueError('Depth copy changed')
            result['frames'].append(dict(file=filename,time_s=float(times[i]),camera_to_world=(body@mount).tolist(),
                body_from_camera=mount.tolist(),odometry=packet,optimized_body=body.tolist()))
        for name in ('metadata.json','live_capture.jsonl','headless_result.json','scan_route.json'):
            shutil.copyfile(source/name,output/name)
        result['bounds_m'],report['points']=write_cloud(output,result,output/'map.ply')
        unchanged(hashes)
        (output/'live_sequence.json').write_text(json.dumps(result,indent=2)+'\n')
        report.update(status='diagnostic_export_complete',frames=len(bodies),
                      loop_edges=[dict(i=e['i'],j=e['j'],measurement=e['z'].tolist(),cycle_partners=e['cycle_partners']) for e in retained],
                      maximum_xy_correction_m=float(max(np.linalg.norm(a[:2,3]-b[:2,3]) for a,b in zip(poses,optimized))))
    except Exception as exc:
        # A failed run must never look like a usable scan, even after partial copying.
        for name in ('map.ply','live_sequence.json'):
            if (output/name).exists():(output/name).unlink()
        if (output/'keyframes').exists():shutil.rmtree(output/'keyframes')
        report['rejection_reason']=str(exc)
        (output/'pose_graph_report.json').write_text(json.dumps(report,indent=2)+'\n')
        raise
    (output/'pose_graph_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--max-candidates',type=int,default=500)
    args=parser.parse_args()
    try:
        report=run(args.source,args.output,args.max_candidates)
        print(json.dumps(dict(status=report['status'],frames=report['frames'],output=str(args.output))),flush=True)
        return 0
    except (ValueError,RuntimeError) as exc:
        print(str(exc),file=sys.stderr);return 2


if __name__=='__main__':sys.exit(main())
