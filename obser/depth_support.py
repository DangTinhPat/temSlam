"""Align DELTAR depth to measured ToF zones and reject unsupported pixels."""
import numpy as np
from scipy.ndimage import gaussian_filter


def sloping_zone_support(z, u, v, index, inside, hist, valid):
    """Allow a continuous predicted slope only when its whole-zone ToF statistics agree."""
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
        if (distribution.min()<.02 or distribution.max()>4 or distribution.std()<.03
                or abs(distribution.mean()-mean)>.08
                or abs(distribution.std()-sigma)>max(.025,.35*sigma)):
            continue
        coefficients[zone]=fit;verified[zone]=True
    inverse=coefficients[index,0]*x+coefficients[index,1]*y+coefficients[index,2]
    predicted=1/np.maximum(inverse,1e-6)
    # This only validates existing fusion pixels; it never supplies replacement depth or fills zeros.
    return (inside & verified[index] & (z<=4) & (np.abs(z-predicted)<.03)
            & (np.abs(z-hist[index,0])<=np.maximum(.08,3*hist[index,1])))


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
    sloping=sloping_zone_support(z,u,v,index,inside,hist,valid)
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

