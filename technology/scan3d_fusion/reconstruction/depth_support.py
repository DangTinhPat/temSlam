"""Align DELTAR depth to measured ToF zones and reject unsupported pixels."""
import numpy as np
from scipy.ndimage import gaussian_filter


def tof_consistency_mask(depth, hist, valid, K, camera_to_tof, rect=None):
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
    support=(inside & valid[index] & np.isfinite(depth) & (depth>.02) & np.isfinite(mean)
             & np.isfinite(sigma) & (sigma>=0) & (np.abs(z-mean)<=tolerance))
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
        support &= z<=nearest
    return support


def tof_support(depth, hist, rect, valid, K, camera_to_tof):
    """Anchor local depth bias to ToF and reject inconsistent predictions."""
    h, w = depth.shape
    original_valid = np.isfinite(depth) & (depth > .02)
    y, x = np.indices((h, w))
    fx, fy, cx, cy = K
    cv = np.stack(((x-cx)*depth/fx, (y-cy)*depth/fy, depth), -1)
    # Calibration file uses OpenGL camera coordinates.
    gl = cv * [1, -1, -1]
    R = np.asarray(camera_to_tof["rotation_matrix"])
    t = np.asarray(camera_to_tof["translation_m"])
    tof_z = -(gl@R.T+t)[..., 2]
    correction, confidence = np.zeros_like(depth), np.zeros_like(depth)
    for (mean, sigma), box, ok in zip(hist, rect, valid):
        if not ok:
            continue
        y0, x0, y1, x1 = np.rint(box).astype(int)
        y0, y1 = np.clip([y0, y1], 0, h)
        x0, x1 = np.clip([x0, x1], 0, w)
        patch = tof_z[y0:y1, x0:x1]
        nearby = patch[np.isfinite(patch) & (np.abs(patch-mean) < max(.2, 3*sigma))]
        if nearby.size < 32:
            continue
        # shortcut: local mean alignment approximates a ToF loss; replace with joint scene optimization.
        delta = np.clip(mean-np.mean(nearby), -.15, .15)
        weight = 1/max(.02, sigma)**2
        correction[y0:y1, x0:x1] += delta*weight
        confidence[y0:y1, x0:x1] += weight
    bias = gaussian_filter(correction, 12)/np.maximum(gaussian_filter(confidence, 12), 1e-6)
    dz = -((x-cx)/fx*R[2, 0] - (y-cy)/fy*R[2, 1] - R[2, 2])
    depth = depth + bias/np.maximum(dz, .1)
    support = tof_consistency_mask(depth,hist,valid,K,camera_to_tof,rect)
    return np.where(support & original_valid & np.isfinite(depth), depth, 0).astype(np.float32)

