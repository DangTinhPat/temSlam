"""Integrate sensor depth and camera poses into a GPU TSDF mesh."""
import numpy as np
from scipy.ndimage import gaussian_filter, binary_dilation
from skimage.measure import marching_cubes
import torch
import trimesh


class TSDF:
    def __init__(self, bounds, voxel=.01, truncation=.06, device="cuda"):
        self.bounds = np.asarray(bounds, dtype=np.float32)
        if (self.bounds.shape != (2, 3) or not np.isfinite(self.bounds).all() or
                np.any(self.bounds[1] <= self.bounds[0]) or not np.isfinite([voxel, truncation]).all()
                or voxel <= 0 or truncation < voxel):
            raise ValueError("invalid volume bounds, voxel size or truncation")
        self.shape = tuple(np.ceil((self.bounds[1]-self.bounds[0])/voxel).astype(int)+1)
        if np.prod(self.shape) > 20_000_000:
            raise ValueError("volume exceeds 20 million voxels; use a smaller ROI or larger voxels")
        self.voxel, self.truncation = voxel, truncation
        axes = [torch.arange(n, device=device)*voxel+float(lo)
                for n, lo in zip(self.shape, self.bounds[0])]
        self.points = torch.stack(torch.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        self.sums = torch.zeros(len(self.points), device=device)
        self.weights = torch.zeros_like(self.sums)

    @torch.no_grad()
    def integrate(self, depth, camera_to_world, intrinsics):
        depth = np.asarray(depth, dtype=np.float32)
        pose = np.asarray(camera_to_world, dtype=np.float32)
        if (depth.ndim != 2 or pose.shape != (4, 4) or not np.isfinite(pose).all() or
                not np.allclose(pose[3], [0, 0, 0, 1]) or
                not np.allclose(pose[:3, :3].T@pose[:3, :3], np.eye(3), atol=1e-4) or
                np.linalg.det(pose[:3, :3]) < .99):
            raise ValueError("invalid depth image or rigid camera pose")
        fx, fy, cx, cy = intrinsics
        if not np.isfinite(intrinsics).all() or min(fx, fy) <= 0:
            raise ValueError("invalid camera intrinsics")
        dev = self.points.device
        T = torch.as_tensor(pose, device=dev)
        camera = (self.points-T[:3, 3])@T[:3, :3]
        z = camera[:, 2]
        u = torch.round(camera[:, 0]/z.clamp_min(.001)*fx+cx).long()
        v = torch.round(camera[:, 1]/z.clamp_min(.001)*fy+cy).long()
        h, w = depth.shape
        visible = (z>.02) & (u>=0) & (u<w) & (v>=0) & (v<h)
        ids = torch.nonzero(visible).flatten()
        measured = torch.as_tensor(depth, device=dev)[v[ids], u[ids]]
        sdf = measured-z[ids]
        valid = torch.isfinite(measured) & (measured>.02) & (sdf>=-self.truncation)
        ids, sdf = ids[valid], sdf[valid]
        self.sums[ids] += (sdf/self.truncation).clamp(-1, 1)
        self.weights[ids] += 1

    def mesh(self, min_views=6):
        weight = self.weights.cpu().numpy().reshape(self.shape)
        sums = self.sums.cpu().numpy().reshape(self.shape)
        raw = sums/np.maximum(weight, 1)
        field = gaussian_filter(sums, 1)/np.maximum(gaussian_filter(weight, 1), 1e-6)
        # Lewiner checks mask at the high corner: every corner of that cube must have real observations.
        supported = weight >= min_views
        cubes = np.ones(tuple(n-1 for n in self.shape),bool)
        recovery_support=np.ones_like(cubes)
        raw_negative=np.zeros_like(cubes);raw_positive=np.zeros_like(cubes)
        smooth_negative=np.zeros_like(cubes);smooth_positive=np.zeros_like(cubes)
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    corner=(slice(x,x+cubes.shape[0]),slice(y,y+cubes.shape[1]),slice(z,z+cubes.shape[2]))
                    cubes &= supported[corner]
                    recovery_support &= weight[corner]>=2*min_views
                    raw_negative |= raw[corner]<0;raw_positive |= raw[corner]>=0
                    smooth_negative |= field[corner]<0;smooth_positive |= field[corner]>=0
        gradient=np.gradient(raw)
        norm_squared=sum(g*g for g in gradient)
        horizontal=(norm_squared>1e-10) & (gradient[2]**2>=.81*norm_squared)
        # Preserve well-observed near-horizontal surfaces in the Z-up map; leave steep wall speckle smoothed.
        lost=(cubes & recovery_support & raw_negative & raw_positive & horizontal[1:,1:,1:]
              & ~binary_dilation(cubes & smooth_negative & smooth_positive))
        restore=np.zeros(self.shape,bool)
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    restore[x:x+cubes.shape[0],y:y+cubes.shape[1],z:z+cubes.shape[2]] |= lost
        field[restore]=raw[restore]
        observed=np.zeros(self.shape,bool);observed[1:,1:,1:]=cubes
        if not (lost.any() or np.any(cubes & smooth_negative & smooth_positive)):
            raise RuntimeError("insufficient observed zero crossings for a surface")
        vertices, faces, _, _ = marching_cubes(field, 0, spacing=(self.voxel,)*3,
                                              mask=observed, allow_degenerate=False)
        mesh = trimesh.Trimesh(vertices+self.bounds[0], faces, process=True)
        parts = [p for p in mesh.split(only_watertight=False) if p.area >= .01]
        if not parts:
            raise RuntimeError("no supported surface components")
        mesh = trimesh.util.concatenate(parts)
        trimesh.smoothing.filter_taubin(mesh, iterations=6)
        mesh.fix_normals()
        return mesh
