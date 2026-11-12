"""Planar point-to-plane refinement of measured RGB-D surfaces, initialized by odometry."""
from copy import deepcopy
import numpy as np
from scipy.spatial import cKDTree


def vertical_points(depth, intrinsics, pose):
    depth = np.asarray(depth)[::8, ::8].astype(float)/1000
    fx, fy, cx, cy = intrinsics
    if not np.isfinite(intrinsics).all() or min(fx, fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    y,x = np.mgrid[:depth.shape[0], :depth.shape[1]]*8
    points = np.stack(((x-cx)*depth/fx, (y-cy)*depth/fy, depth), -1)
    dy,dx = np.gradient(points, axis=(0,1))
    normals = np.cross(dx,dy)
    lengths = np.linalg.norm(normals,axis=2)
    normals /= np.maximum(lengths[...,None],1e-9)
    points = points@pose[:3,:3].T+pose[:3,3]
    normals = normals@pose[:3,:3].T
    valid = ((depth>.1)&(depth<3.5)&(lengths>1e-7)&(np.abs(normals[...,2])<.4)
             &(points[...,2]>.15)&(points[...,2]<1.6))
    # Depth discontinuities have unreliable normals, especially at object silhouettes.
    for axis in (0,1):
        valid &= ((np.abs(np.roll(depth,1,axis)-depth)<.15)
                  &(np.abs(np.roll(depth,-1,axis)-depth)<.15))
    valid[[0,-1],:] = False
    valid[:,[0,-1]] = False
    return points[valid], normals[valid]


def planar_change(delta, origin):
    c,s = np.cos(delta[2]),np.sin(delta[2])
    change = np.eye(4)
    change[:3,:3] = [[c,-s,0],[s,c,0],[0,0,1]]
    change[:3,3] = origin-change[:3,:3]@origin+[delta[0],delta[1],0]
    return change


def match_surface(points, normals, target, target_normals, tree, origin):
    delta = np.zeros(3)
    accepted = False
    count = 0
    if len(points) >= 100 and tree is not None:
        for _ in range(12):
            change = planar_change(delta,origin)
            q = points@change[:3,:3].T+change[:3,3]
            qn = normals@change[:3,:3].T
            distances,indices = tree.query(q)
            n = target_normals[indices]
            residual = np.sum(n*(q-target[indices]),axis=1)
            valid = ((distances<.2)&(np.abs(residual)<.1)
                     &(np.abs(np.sum(n*qn,axis=1))>.85))
            count = int(valid.sum())
            if count < 100:
                break
            n,r,offset = n[valid],residual[valid],q[valid]-origin
            # A single wall cannot constrain translation along itself.
            if np.linalg.eigvalsh(n[:,:2].T@n[:,:2]/count)[0] < .025:
                break
            jacobian = np.column_stack((n[:,:2],-n[:,0]*offset[:,1]+n[:,1]*offset[:,0]))
            weights = np.minimum(1,.025/np.maximum(np.abs(r),1e-9))
            # shortcut: fixed odometry regularization; use calibrated covariances for real hardware.
            prior = np.diag([20.,20.,50.])
            step = np.linalg.solve(jacobian.T@(weights[:,None]*jacobian)+prior,
                                   -jacobian.T@(weights*r)-prior@delta)
            delta += step
            if np.linalg.norm(delta[:2])>.2 or abs(delta[2])>.12:
                accepted = False
                break
            accepted = True
            if np.linalg.norm(step)<1e-4:
                break
    if not accepted:
        delta[:] = 0
    return planar_change(delta,origin), dict(accepted=accepted, matches=count, delta=delta.tolist())


def refine_sequence(session, sequence):
    result = deepcopy(sequence)
    correction = np.eye(4)
    target = target_normals = np.empty((0,3))
    tree = None
    records = []
    for frame in result['frames']:
        raw = np.asarray(frame['camera_to_world'],float)
        if (raw.shape!=(4,4) or not np.isfinite(raw).all() or
                not np.allclose(raw[3],[0,0,0,1]) or
                not np.allclose(raw[:3,:3].T@raw[:3,:3],np.eye(3),atol=1e-5)):
            raise ValueError('Invalid camera pose')
        initial = correction@raw
        with np.load(session/frame['file'],allow_pickle=False) as data:
            points,normals = vertical_points(data['depth'],result['intrinsics'],initial)
        change,record = match_surface(points,normals,target,target_normals,tree,initial[:3,3])
        pose = change@initial
        correction = pose@np.linalg.inv(raw)
        frame['camera_to_world'] = pose.tolist()
        records.append(dict(time_s=frame['time_s'],**record))
        if len(points):
            target = np.concatenate((target,points@change[:3,:3].T+change[:3,3]))
            target_normals = np.concatenate((target_normals,normals@change[:3,:3].T))
            _,indices = np.unique(np.floor(target/.04).astype(np.int64),axis=0,return_index=True)
            target,target_normals = target[indices],target_normals[indices]
            tree = cKDTree(target)
    # Recompute bounds from all measured depths, including floor and furniture.
    low,high = np.full(3,np.inf),np.full(3,-np.inf)
    fx,fy,cx,cy = result['intrinsics']
    for frame in result['frames']:
        with np.load(session/frame['file'],allow_pickle=False) as data:
            z = data['depth'][::8,::8].astype(float)/1000
        y,x = np.mgrid[:z.shape[0],:z.shape[1]]*8
        points = np.stack(((x-cx)*z/fx,(y-cy)*z/fy,z),-1)[(z>.02)&(z<=4)]
        if len(points):
            pose = np.asarray(frame['camera_to_world'])
            points = points@pose[:3,:3].T+pose[:3,3]
            low,high = np.minimum(low,points.min(0)),np.maximum(high,points.max(0))
    if not np.isfinite([low,high]).all():
        raise ValueError('No valid depth observations')
    result['bounds_m'] = [(low-.1).tolist(),(high+.1).tolist()]
    result['pose_source'] = 'wheel/IMU + gated visual poses + planar surface matching'
    return result, dict(method='robust point-to-plane XY/yaw, measured vertical surfaces',
                        accepted_frames=sum(r['accepted'] for r in records),frames=records,
                        limitation='Sequential refinement, not a global pose graph; preserves inertial height/tilt; unobserved regions remain unknown.')
